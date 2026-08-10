// SPDX-License-Identifier: GPL-2.0
/*
 * Fairphone 3 / 3+ swappable module detection
 *
 * Copyright (c) 2026, Marc Lainez <marc.lainez@gmail.com>
 *
 * The Fairphone 3 is built to be taken apart: the rear camera, the front
 * camera (inside the "top" module) and the loudspeaker are all
 * user-replaceable with a #00 screwdriver, and the Fairphone 3+ upgrade
 * kit swaps them for different silicon:
 *
 *	slot		Fairphone 3		Fairphone 3+
 *	rear camera	Sony IMX363		Samsung S5KGM1SP @0x10
 *	front camera	Samsung S5K4H7YX @0x10	Samsung S5K3P9SP @0x10
 *	loudspeaker	Awinic AW8898 @0x34	TI TAS2557 @0x4c
 *
 * Addresses are not even reliable within one model: rear IMX363 modules
 * have been seen at both 0x1a and 0x10, and both front sensors share 0x10
 * outright, so a module has to be identified by reading its ID register
 * rather than by which address answers.
 *
 * The mainboard is identical either way - both variants report
 * qcom,msm-id = <349 0> and qcom,board-id = <8 0x10000> - so no amount of
 * board identification can tell them apart, and because the modules are
 * sold separately a single phone can carry any mix of the two.  Shipping
 * one device tree per variant therefore cannot be made correct: there is
 * no variant, only a set of independently swappable modules.
 *
 * So describe the *slot* in the base device tree instead - the parts that
 * really are soldered down: which I2C bus the module hangs off, its
 * regulators, its reference clock and its enable line - and let this
 * driver work out what is plugged in.  For each slot it powers the module
 * up, reads its ID register, applies the device tree overlay that
 * describes that particular chip, and powers back down.  The stock,
 * unmodified sensor and codec drivers then bind against a device tree
 * that finally describes the hardware which is actually present.
 *
 * This follows the pattern established by drivers/misc/lan966x_pci.c: the
 * driver that owns a connector describes what it found using an overlay
 * linked into the kernel image.  Note that this is the in-kernel overlay
 * API, not the out-of-tree configfs interface - nothing here is applied
 * from userspace.
 *
 * Ordering falls out of the device tree.  CAMSS and the sound card are
 * left status = "disabled" in the base DT, so neither probes until an
 * overlay flips it; of_reconfig_get_state_change() turns that flip into
 * an OF_RECONFIG_CHANGE_ADD and the platform device is created then.  By
 * the time CAMSS parses its graph, every endpoint has a remote.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

/*
 * Overlays are compiled from the .dtso files next to this driver and
 * linked into the image; see the Makefile.  scripts/Makefile.dtbs names
 * the symbols after the file, with dashes turned into underscores.
 */
#define FP3_DECLARE_DTBO(sym)					\
	extern char __dtbo_##sym##_begin[];			\
	extern char __dtbo_##sym##_end[]

FP3_DECLARE_DTBO(fp3_slot_cam_rear_imx363_1a);
FP3_DECLARE_DTBO(fp3_slot_cam_rear_imx363_10);
FP3_DECLARE_DTBO(fp3_slot_cam_rear_s5kgm1sp);
FP3_DECLARE_DTBO(fp3_slot_cam_front_s5k4h7yx);
FP3_DECLARE_DTBO(fp3_slot_cam_front_s5k3p9sp);
FP3_DECLARE_DTBO(fp3_slot_spk_aw8898);
FP3_DECLARE_DTBO(fp3_slot_spk_tas2557);
FP3_DECLARE_DTBO(fp3_slot_camss);

struct fp3_dtbo {
	const char *name;
	char *begin;
	char *end;
};

#define FP3_DTBO(sym) {						\
	.name  = #sym,						\
	.begin = __dtbo_##sym##_begin,				\
	.end   = __dtbo_##sym##_end,				\
}

static const struct fp3_dtbo fp3_dtbo_cam_rear_imx363_1a =
	FP3_DTBO(fp3_slot_cam_rear_imx363_1a);
static const struct fp3_dtbo fp3_dtbo_cam_rear_imx363_10 =
	FP3_DTBO(fp3_slot_cam_rear_imx363_10);
static const struct fp3_dtbo fp3_dtbo_cam_rear_s5kgm1sp =
	FP3_DTBO(fp3_slot_cam_rear_s5kgm1sp);
static const struct fp3_dtbo fp3_dtbo_cam_front_s5k4h7yx =
	FP3_DTBO(fp3_slot_cam_front_s5k4h7yx);
static const struct fp3_dtbo fp3_dtbo_cam_front_s5k3p9sp =
	FP3_DTBO(fp3_slot_cam_front_s5k3p9sp);
static const struct fp3_dtbo fp3_dtbo_spk_aw8898 =
	FP3_DTBO(fp3_slot_spk_aw8898);
static const struct fp3_dtbo fp3_dtbo_spk_tas2557 =
	FP3_DTBO(fp3_slot_spk_tas2557);
static const struct fp3_dtbo fp3_dtbo_camss =
	FP3_DTBO(fp3_slot_camss);

/*
 * How to recognise a chip.  The camera sensors all speak the SMIA-style
 * 16-bit register / 16-bit value protocol and must be told apart by their
 * model ID, because both front sensors answer to the same address.  The
 * two loudspeaker amplifiers sit at different addresses and use
 * incompatible register maps (TAS2557 is book/page paged), so for those a
 * plain "does anything ACK here" test is both sufficient and safer.
 */
/* Longest candidate list any slot may declare. */
#define FP3_MAX_CHIPS 4

enum fp3_probe_mode {
	FP3_PROBE_MODEL_ID,	/* read @id_reg, expect @id */
	FP3_PROBE_ACK,		/* address responds to a one byte read */
};

struct fp3_chip {
	const char *name;
	u16 addr;
	enum fp3_probe_mode mode;
	u16 id_reg;
	u16 id;
	/*
	 * Level to drive the slot's enable line while probing this chip.
	 *
	 * The slot node describes the line as GPIO_ACTIVE_HIGH and this is
	 * the raw level, because the bindings disagree about polarity even
	 * though the silicon does not.  Every module here runs with the pin
	 * physically high: the camera sensors and the AW8898 call it an
	 * active-low reset and release it with logical 0
	 * (snd-soc-aw8898.c), while the TAS2557 calls the very same pin an
	 * active-high "shutdown" and runs at logical 1 (tas2557.c).  Drive
	 * it low for either amplifier and the chip stays asleep and never
	 * acknowledges.
	 */
	int enable_level;
	const struct fp3_dtbo *dtbo;
};

struct fp3_slot_type {
	const char *name;
	const char * const *supplies;
	unsigned int num_supplies;
	bool has_clk;
	unsigned long clk_rate;
	const struct fp3_chip *chips;
	unsigned int num_chips;
	bool is_camera;
};

static const char * const fp3_camera_supplies[] = { "vana", "vdig", "vif" };
static const char * const fp3_speaker_supplies[] = { "dvdd", "vddio", "vdd" };

static const struct fp3_chip fp3_rear_camera_chips[] = {
	{
		.name = "Samsung S5KGM1SP (48MP, Fairphone 3+)",
		.addr = 0x10,
		.mode = FP3_PROBE_MODEL_ID,
		.id_reg = 0x0000,
		.id = 0x08d1,
		.enable_level = 1,
		.dtbo = &fp3_dtbo_cam_rear_s5kgm1sp,
	}, {
		/*
		 * Some Fairphone 3 rear modules strap the IMX363 to 0x10
		 * instead of the 0x1a the mainline device tree assumed, so
		 * both are tried. The Samsung above shares 0x10 but keeps
		 * its ID in a different register, so there is no ambiguity.
		 */
		.name = "Sony IMX363 (12MP, Fairphone 3) @0x10",
		.addr = 0x10,
		.mode = FP3_PROBE_MODEL_ID,
		.id_reg = 0x0016,
		.id = 0x0363,
		.enable_level = 1,
		.dtbo = &fp3_dtbo_cam_rear_imx363_10,
	}, {
		.name = "Sony IMX363 (12MP, Fairphone 3) @0x1a",
		.addr = 0x1a,
		.mode = FP3_PROBE_MODEL_ID,
		.id_reg = 0x0016,
		.id = 0x0363,
		.enable_level = 1,
		.dtbo = &fp3_dtbo_cam_rear_imx363_1a,
	},
};

static const struct fp3_chip fp3_front_camera_chips[] = {
	{
		.name = "Samsung S5K3P9SP (16MP, Fairphone 3+)",
		.addr = 0x10,
		.mode = FP3_PROBE_MODEL_ID,
		.id_reg = 0x0000,
		.id = 0x3109,
		.enable_level = 1,
		.dtbo = &fp3_dtbo_cam_front_s5k3p9sp,
	}, {
		.name = "Samsung S5K4H7YX (8MP, Fairphone 3)",
		.addr = 0x10,
		.mode = FP3_PROBE_MODEL_ID,
		.id_reg = 0x0000,
		.id = 0x487b,
		.enable_level = 1,
		.dtbo = &fp3_dtbo_cam_front_s5k4h7yx,
	},
};

static const struct fp3_chip fp3_speaker_chips[] = {
	{
		.name = "TI TAS2557 (Fairphone 3+)",
		.addr = 0x4c,
		.mode = FP3_PROBE_ACK,
		.enable_level = 1,
		.dtbo = &fp3_dtbo_spk_tas2557,
	}, {
		.name = "Awinic AW8898 (Fairphone 3)",
		.addr = 0x34,
		.mode = FP3_PROBE_ACK,
		.enable_level = 1,
		.dtbo = &fp3_dtbo_spk_aw8898,
	},
};

static const struct fp3_slot_type fp3_slot_types[] = {
	{
		.name = "rear-camera",
		.supplies = fp3_camera_supplies,
		.num_supplies = ARRAY_SIZE(fp3_camera_supplies),
		.has_clk = true,
		.clk_rate = 24000000,
		.chips = fp3_rear_camera_chips,
		.num_chips = ARRAY_SIZE(fp3_rear_camera_chips),
		.is_camera = true,
	}, {
		.name = "front-camera",
		.supplies = fp3_camera_supplies,
		.num_supplies = ARRAY_SIZE(fp3_camera_supplies),
		.has_clk = true,
		.clk_rate = 24000000,
		.chips = fp3_front_camera_chips,
		.num_chips = ARRAY_SIZE(fp3_front_camera_chips),
		.is_camera = true,
	}, {
		.name = "speaker-amp",
		.supplies = fp3_speaker_supplies,
		.num_supplies = ARRAY_SIZE(fp3_speaker_supplies),
		.chips = fp3_speaker_chips,
		.num_chips = ARRAY_SIZE(fp3_speaker_chips),
	},
};

/* Runtime state for one slot, filled in during the resource pass. */
struct fp3_slot {
	const struct fp3_slot_type *type;
	struct device_node *np;
	struct i2c_adapter *adap;
	struct regulator_bulk_data supplies[ARRAY_SIZE(fp3_camera_supplies)];
	struct clk *clk;
	struct gpio_desc *enable;
};

struct fp3_module_slots {
	struct device *dev;
	struct fp3_slot *slots;
	unsigned int num_slots;
};

static const struct fp3_slot_type *fp3_slot_type_of(struct device_node *np)
{
	const char *name;
	unsigned int i;

	if (of_property_read_string(np, "fairphone,slot", &name))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(fp3_slot_types); i++)
		if (!strcmp(fp3_slot_types[i].name, name))
			return &fp3_slot_types[i];

	return NULL;
}

static void fp3_slot_release(struct fp3_slot *slot)
{
	unsigned int i;

	if (!IS_ERR_OR_NULL(slot->enable))
		gpiod_put(slot->enable);
	slot->enable = NULL;

	if (!IS_ERR_OR_NULL(slot->clk))
		clk_put(slot->clk);
	slot->clk = NULL;

	for (i = 0; i < slot->type->num_supplies; i++) {
		if (!IS_ERR_OR_NULL(slot->supplies[i].consumer))
			regulator_put(slot->supplies[i].consumer);
		slot->supplies[i].consumer = NULL;
	}

	if (slot->adap) {
		i2c_put_adapter(slot->adap);
		slot->adap = NULL;
	}
}

/*
 * Take the slot's resources for the duration of a detection pass only.
 *
 * Nothing here may be held past the point where the overlay is applied.
 * of_overlay_fdt_apply() runs its reconfiguration notifiers inline, so the
 * sensor or codec driver probes *inside* that call and immediately asks
 * for the very same reset line — and unlike clocks and regulators, a GPIO
 * has exactly one consumer.  Holding on would hand the real driver
 * -EBUSY.
 */
static int fp3_slot_acquire(struct fp3_module_slots *priv,
			    struct fp3_slot *slot)
{
	struct device *dev = priv->dev;
	struct device_node *np = slot->np;
	struct device_node *bus;
	unsigned int i;

	bus = of_parse_phandle(np, "i2c-bus", 0);
	if (!bus)
		return dev_err_probe(dev, -EINVAL,
				     "%pOFn: missing i2c-bus phandle\n", np);

	slot->adap = of_get_i2c_adapter_by_node(bus);
	of_node_put(bus);
	if (!slot->adap)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "%pOFn: I2C bus not ready\n", np);

	for (i = 0; i < slot->type->num_supplies; i++) {
		struct regulator *reg;

		reg = of_regulator_get_optional(dev, np,
						slot->type->supplies[i]);
		if (IS_ERR(reg)) {
			slot->supplies[i].consumer = NULL;
			fp3_slot_release(slot);
			return dev_err_probe(dev, PTR_ERR(reg),
					     "%pOFn: %s-supply\n", np,
					     slot->type->supplies[i]);
		}

		slot->supplies[i].supply = slot->type->supplies[i];
		slot->supplies[i].consumer = reg;
	}

	if (slot->type->has_clk) {
		slot->clk = of_clk_get(np, 0);
		if (IS_ERR(slot->clk)) {
			int ret = PTR_ERR(slot->clk);

			slot->clk = NULL;
			fp3_slot_release(slot);
			return dev_err_probe(dev, ret, "%pOFn: clock\n", np);
		}
	}

	slot->enable = fwnode_gpiod_get_index(of_fwnode_handle(np), "enable", 0,
					      GPIOD_OUT_LOW, "fp3-slot-enable");
	if (IS_ERR(slot->enable)) {
		int ret = PTR_ERR(slot->enable);

		slot->enable = NULL;
		fp3_slot_release(slot);
		return dev_err_probe(dev, ret, "%pOFn: enable-gpios\n", np);
	}

	return 0;
}

static int fp3_slot_power_on(struct fp3_slot *slot, int enable_level)
{
	int ret;

	ret = regulator_bulk_enable(slot->type->num_supplies, slot->supplies);
	if (ret)
		return ret;

	/* Mirrors the settle time the sensor drivers use after the rails. */
	fsleep(1000);

	if (slot->clk) {
		ret = clk_set_rate(slot->clk, slot->type->clk_rate);
		if (ret)
			goto err_regulators;

		ret = clk_prepare_enable(slot->clk);
		if (ret)
			goto err_regulators;
	}

	gpiod_set_value_cansleep(slot->enable, enable_level);

	/*
	 * 12 ms is what s5kgm1sp_power_on() waits after releasing reset;
	 * the other three sensors are quicker and the amplifiers need
	 * only a few hundred microseconds.
	 */
	fsleep(12000);

	return 0;

err_regulators:
	regulator_bulk_disable(slot->type->num_supplies, slot->supplies);
	return ret;
}

static void fp3_slot_power_off(struct fp3_slot *slot)
{
	gpiod_set_value_cansleep(slot->enable, 0);

	if (slot->clk)
		clk_disable_unprepare(slot->clk);

	regulator_bulk_disable(slot->type->num_supplies, slot->supplies);
}

/*
 * Walking a candidate list means deliberately addressing chips that are not
 * there, and on the CCI controller the NAK that comes back leaves the bus
 * unhappy enough that the *next* transfer fails too — which on a Fairphone 3
 * meant the IMX363 at 0x1a was missed purely because 0x10 had been tried
 * first.  One retry after a short pause is enough to let the controller
 * settle; a chip that really is absent just NAKs twice.
 */
#define FP3_I2C_TRIES 2

static int fp3_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int n)
{
	int ret = -EIO;
	int i;

	for (i = 0; i < FP3_I2C_TRIES; i++) {
		if (i)
			fsleep(2000);

		ret = i2c_transfer(adap, msgs, n);
		if (ret == n)
			return 0;
	}

	return ret < 0 ? ret : -EIO;
}

static int fp3_read_model_id(struct i2c_adapter *adap, u16 addr, u16 reg,
			     u16 *val)
{
	u8 wbuf[2] = { reg >> 8, reg & 0xff };
	u8 rbuf[2];
	struct i2c_msg msgs[2] = {
		{ .addr = addr, .flags = 0,	    .len = 2, .buf = wbuf },
		{ .addr = addr, .flags = I2C_M_RD,  .len = 2, .buf = rbuf },
	};
	int ret;

	ret = fp3_i2c_xfer(adap, msgs, ARRAY_SIZE(msgs));
	if (ret)
		return ret;

	*val = get_unaligned_be16(rbuf);
	return 0;
}

static bool fp3_chip_acks(struct i2c_adapter *adap, u16 addr)
{
	u8 buf;
	struct i2c_msg msg = {
		.addr = addr,
		.flags = I2C_M_RD,
		.len = 1,
		.buf = &buf,
	};

	return fp3_i2c_xfer(adap, &msg, 1) == 0;
}

/*
 * Returns true when @chip is the module in this slot.  @why is filled in
 * either way: when a slot turns up empty its whole candidate list gets
 * reported, and "0x4c: no ack" versus "0x4c: id 0x1234" is the difference
 * between a module that is absent and one that is present but unexpected.
 */
static bool fp3_chip_present(struct fp3_slot *slot,
			     const struct fp3_chip *chip,
			     char *why, size_t why_len)
{
	u16 val;
	int ret;

	if (chip->mode == FP3_PROBE_ACK) {
		if (fp3_chip_acks(slot->adap, chip->addr))
			return true;

		scnprintf(why, why_len, "0x%02x: no ack", chip->addr);
		return false;
	}

	ret = fp3_read_model_id(slot->adap, chip->addr, chip->id_reg, &val);
	if (ret) {
		scnprintf(why, why_len, "0x%02x: no ack (%d)", chip->addr, ret);
		return false;
	}

	if (val != chip->id) {
		scnprintf(why, why_len, "0x%02x: id 0x%04x, wanted 0x%04x",
			  chip->addr, val, chip->id);
		return false;
	}

	return true;
}

/*
 * Last resort when nothing in the candidate list answered: walk the bus and
 * report whatever is out there, so an unknown or re-addressed module shows
 * up in dmesg instead of just being missing.
 */
static void fp3_scan_bus(struct device *dev, struct fp3_slot *slot)
{
	/*
	 * Report the two register locations image sensors keep their model
	 * ID in — Samsung and the SMIA-alikes at 0x0000, Sony at 0x0016 —
	 * so an unrecognised module identifies itself in dmesg instead of
	 * needing a kernel rebuild to find out what it is.
	 */
	static const u16 id_regs[] = { 0x0000, 0x0016 };
	bool empty = true;
	u16 addr;

	for (addr = 0x08; addr <= 0x77; addr++) {
		char ids[64];
		size_t n = 0;
		unsigned int i;

		if (!fp3_chip_acks(slot->adap, addr))
			continue;

		empty = false;
		ids[0] = '\0';

		for (i = 0; i < ARRAY_SIZE(id_regs); i++) {
			u16 val;

			if (fp3_read_model_id(slot->adap, addr, id_regs[i],
					      &val))
				n += scnprintf(ids + n, sizeof(ids) - n,
					       " [%04x]=??", id_regs[i]);
			else
				n += scnprintf(ids + n, sizeof(ids) - n,
					       " [%04x]=0x%04x", id_regs[i],
					       val);
		}

		dev_warn(dev, "%s slot: bus scan: 0x%02x acks,%s\n",
			 slot->type->name, addr, ids);
	}

	if (empty)
		dev_warn(dev, "%s slot: bus scan found nothing\n",
			 slot->type->name);
}

static int fp3_apply_overlay(struct device *dev, const struct fp3_dtbo *dtbo)
{
	int ovcs_id = 0;
	int ret;

	ret = of_overlay_fdt_apply(dtbo->begin, dtbo->end - dtbo->begin,
				   &ovcs_id, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "failed to apply overlay %s\n",
				     dtbo->name);

	return 0;
}

/*
 * Detection pass for one slot.  A slot we cannot identify is not fatal -
 * the module may simply be missing, and the phone should still boot with
 * everything else working.
 */
static int fp3_slot_detect(struct fp3_module_slots *priv, struct fp3_slot *slot)
{
	const struct fp3_dtbo *found_dtbo = NULL;
	struct device *dev = priv->dev;
	const char *found_name = NULL;
	char why[FP3_MAX_CHIPS][48] = { };
	unsigned int i;
	int err = 0;
	int ret;

	ret = fp3_slot_acquire(priv, slot);
	if (ret)
		return ret;

	for (i = 0; i < slot->type->num_chips && !found_dtbo; i++) {
		const struct fp3_chip *chip = &slot->type->chips[i];

		err = fp3_slot_power_on(slot, chip->enable_level);
		if (err) {
			dev_err(dev, "%pOFn: failed to power up: %d\n",
				slot->np, err);
			break;
		}

		if (fp3_chip_present(slot, chip,
				     why[i], sizeof(why[0]))) {
			found_dtbo = chip->dtbo;
			found_name = chip->name;
		}

		fp3_slot_power_off(slot);
	}

	if (!found_dtbo && !err) {
		/* Powered up once more so the scan sees a live bus. */
		if (!fp3_slot_power_on(slot, 1)) {
			fp3_scan_bus(dev, slot);
			fp3_slot_power_off(slot);
		}
	}

	/*
	 * Hand the reset line back before the overlay goes live — the real
	 * driver probes inside of_overlay_fdt_apply() and needs it.
	 */
	fp3_slot_release(slot);

	if (!found_dtbo) {
		if (err)
			return err;

		for (i = 0; i < slot->type->num_chips; i++)
			dev_warn(dev, "%s slot: %s (%s)\n", slot->type->name,
				 slot->type->chips[i].name,
				 why[i]);

		dev_warn(dev, "%s slot: no known module found\n",
			 slot->type->name);
		return 0;
	}

	dev_info(dev, "%s slot: %s\n", slot->type->name, found_name);

	ret = fp3_apply_overlay(dev, found_dtbo);
	if (ret)
		return ret;

	return 1;
}

static int fp3_module_slots_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fp3_module_slots *priv;
	struct device_node *child;
	unsigned int cameras = 0;
	unsigned int n = 0;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->num_slots = of_get_available_child_count(dev->of_node);
	if (!priv->num_slots)
		return dev_err_probe(dev, -EINVAL, "no module slots described\n");

	priv->slots = devm_kcalloc(dev, priv->num_slots, sizeof(*priv->slots),
				   GFP_KERNEL);
	if (!priv->slots)
		return -ENOMEM;

	for_each_available_child_of_node(dev->of_node, child) {
		struct fp3_slot *slot = &priv->slots[n];

		slot->type = fp3_slot_type_of(child);
		if (!slot->type) {
			dev_warn(dev, "%pOFn: unknown fairphone,slot, skipping\n",
				 child);
			continue;
		}

		/*
		 * for_each_available_child_of_node() drops its reference on
		 * every iteration, and the slot outlives the loop.  The
		 * driver is built in and never unbinds, so these are held
		 * for the life of the system.
		 */
		slot->np = of_node_get(child);
		n++;
	}
	priv->num_slots = n;

	/*
	 * Dry run.  Take and immediately drop every slot's resources so that
	 * anything which is going to return -EPROBE_DEFER does so now, while
	 * the device tree is still untouched.  Without this a late deferral
	 * would re-run probe with some overlays already applied, and their
	 * nodes would be added a second time.
	 */
	for (n = 0; n < priv->num_slots; n++) {
		ret = fp3_slot_acquire(priv, &priv->slots[n]);
		if (ret)
			return ret;

		fp3_slot_release(&priv->slots[n]);
	}

	/* Identify each module and describe it.  Overlays go live here. */
	for (n = 0; n < priv->num_slots; n++) {
		struct fp3_slot *slot = &priv->slots[n];

		ret = fp3_slot_detect(priv, slot);
		if (ret < 0)
			return ret;

		if (ret && slot->type->is_camera)
			cameras++;
	}

	/*
	 * CAMSS is disabled in the base device tree and each camera
	 * overlay contributes only its own port, so it is safe to bring up
	 * even when just one of the two modules is fitted.  Enable it last,
	 * once every endpoint that exists has a remote to point at.
	 */
	if (cameras) {
		ret = fp3_apply_overlay(dev, &fp3_dtbo_camss);
		if (ret)
			return ret;
	} else {
		dev_warn(dev, "no camera modules detected, leaving CAMSS off\n");
	}

	return 0;
}

static const struct of_device_id fp3_module_slots_of_match[] = {
	{ .compatible = "fairphone,fp3-module-slots" },
	{ }
};
MODULE_DEVICE_TABLE(of, fp3_module_slots_of_match);

static struct platform_driver fp3_module_slots_driver = {
	.probe = fp3_module_slots_probe,
	.driver = {
		.name = "fp3-module-slots",
		.of_match_table = fp3_module_slots_of_match,
		/* Overlays are applied for the life of the system. */
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(fp3_module_slots_driver);
