// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017, Linaro Ltd.

#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/slimbus.h>
#include <linux/module.h>

#include "internal.h"

/*
 * WCD9335/9340 on Qualcomm NGD controllers can NACK the first VE read
 * attempt after an idle period (seen on MSM8953 + WCD9326). Downstream
 * works around this by retrying up to 3 times with a 5 ms gap between
 * attempts (see techpack wcd9xxx-core.c:WCD9XXX_SLIM_RW_MAX_TRIES). Do
 * the same here so the problem doesn't leak into every codec driver.
 */
#define REGMAP_SLIMBUS_MAX_TRIES		3
#define REGMAP_SLIMBUS_PAGED_TRIES	3

/*
 * Debug knobs for isolating which DAPM register writes to the WCD9335 PGD
 * (LA=200) cause AFE DEVICE_START to return ADSP_EFAILED.
 *
 * skip_codec_writes=1 : blanket NO-OP all LA=200 writes (original knob).
 *   Confirmed: DEVICE_START succeeds when set.  PORT_STATUS becomes 0x82
 *   (overflow) because codec analog path is disabled.  Not a permanent fix.
 *
 * skip_above=<addr>   : NO-OP writes to LA=200 where addr >= skip_above.
 *   Use this to binary-search which register address range contains the
 *   offending write(s).  Default 0x10000 = pass everything through.
 *
 * skip_below=<addr>   : NO-OP writes to LA=200 where addr < skip_below.
 *   Use together with skip_above to isolate a window.  Default 0 = no-op.
 *
 * Typical bisection workflow (no reflash needed, survives reboot):
 *   Boot → trigger playback → EFAILED.  Then:
 *   (1) echo 0x1000 > /sys/module/regmap_slimbus/parameters/skip_above
 *       Reboot+play: if success → offending write is in [0x1000, 0xFFFF]
 *       if still EFAILED → in [0x0000, 0x1000)
 *   (2) Repeat, halving the range each time.
 *   (3) Once a 16-register range is isolated, examine wcd9335.c for which
 *       DAPM widget writes those registers and compare values with downstream.
 *
 * Both knobs filter ONLY when sdev->laddr == 200 (PGD codec proper), so
 * boot-time writes via wcd9335_bring_up are also suppressed during the
 * test window — be aware of this if the bring-up window matters.
 * Leave skip_above at 0x10000 (default) to pass all writes through.
 * IMPORTANT: these params are evaluated at write time, so they affect
 * any write that happens AFTER you set them, including bring_up writes
 * if set before the codec is probed.  Set them after boot, before play.
 */
static bool skip_codec_writes;
module_param(skip_codec_writes, bool, 0644);
MODULE_PARM_DESC(skip_codec_writes,
	"Drop ALL writes to PGD codec (LA=200) — DAPM-clash diagnostic");

static uint skip_above = 0x10000;
module_param(skip_above, uint, 0644);
MODULE_PARM_DESC(skip_above,
	"Drop LA=200 writes where addr >= this value (bisection, default=0x10000=off)");

static uint skip_below;
module_param(skip_below, uint, 0644);
MODULE_PARM_DESC(skip_below,
	"Drop LA=200 writes where addr < this value (bisection, default=0=off)");

static int regmap_slimbus_write(void *context, const void *data, size_t count)
{
	struct slim_device *sdev = context;
	int tries = REGMAP_SLIMBUS_MAX_TRIES;
	u16 addr = *(u16 *)data;
	int ret;

	if (sdev->laddr == 200) {
		if (skip_codec_writes)
			return 0;
		if (addr >= skip_above || addr < skip_below)
			return 0;
	}

	do {
		ret = slim_write(sdev, addr, count - 2, (u8 *)data + 2);
		if (!ret || --tries == 0)
			break;
		usleep_range(5000, 5100);
	} while (1);

	return ret;
}

static int regmap_slimbus_read(void *context, const void *reg, size_t reg_size,
			       void *val, size_t val_size)
{
	struct slim_device *sdev = context;
	int tries = REGMAP_SLIMBUS_MAX_TRIES;
	u16 addr = *(u16 *)reg;
	int ret;

	do {
		ret = slim_read(sdev, addr, val_size, val);
		if (!ret || --tries == 0)
			break;
		usleep_range(5000, 5100);
	} while (1);

	return ret;
}

static const struct regmap_bus regmap_slimbus_bus = {
	.write = regmap_slimbus_write,
	.read = regmap_slimbus_read,
	.reg_format_endian_default = REGMAP_ENDIAN_LITTLE,
	.val_format_endian_default = REGMAP_ENDIAN_LITTLE,
};

/*
 * Flat register-window bus for WCD9335 (and similar Qualcomm SLIMbus codecs).
 *
 * The WCD9335 codec PGD does NOT have a page selector. Its register bank is
 * exposed at SLIMbus value-element offset 0x800 — i.e. codec register `reg`
 * is accessed at SLIMbus addr 0x800 + reg, for the full 16-bit register
 * space. This matches downstream's TASHA_REGISTER_START_OFFSET = 0x800 in
 * techpack/audio/asoc/codecs/wcd9xxx-core.c (slim_change_val_element /
 * slim_request_val_element pass start_offset = 0x800 + reg).
 *
 * Earlier mainline forks tried regmap_range_cfg page-RMW and a
 * "manual paging" bus that wrote a page selector at 0x800 then accessed
 * 0x800+offset — both wrong: only registers with high byte == 0 happened
 * to land at the right SLIMbus address; everything in pages 0x06, 0x0a,
 * 0x0d, ... silently aliased to low-page registers and the codec NACKed.
 *
 * Bus used only when the codec driver requests it via
 * regmap_init_slimbus_paged() (kept name for source compatibility).
 * Drivers using the standard regmap_init_slimbus() see no change.
 */
struct regmap_slimbus_paged_ctx {
	struct slim_device *sdev;
};

#define REGMAP_SLIMBUS_WCD_BASE	0x0800

static int regmap_slimbus_paged_write(void *context, const void *data,
				      size_t count)
{
	struct regmap_slimbus_paged_ctx *ctx = context;
	struct slim_device *sdev = ctx->sdev;
	int tries = REGMAP_SLIMBUS_PAGED_TRIES;
	u16 vaddr = *(u16 *)data;
	u16 hw_addr;
	int ret;

	if (sdev->laddr == 200) {
		if (skip_codec_writes)
			return 0;
		if (vaddr >= skip_above || vaddr < skip_below)
			return 0;
	}

	/*
	 * Flat addressing: codec register `vaddr` is at SLIMbus
	 * value-element offset 0x800 + vaddr. The codec has no page
	 * register; what mainline called the "page selector" at 0x800
	 * is just the first byte of the flat register window.
	 */
	hw_addr = REGMAP_SLIMBUS_WCD_BASE + vaddr;
	do {
		ret = slim_write(sdev, hw_addr, count - 2, (u8 *)data + 2);
		if (!ret || --tries == 0)
			break;
		usleep_range(5000, 5100);
	} while (1);

	return ret;
}

static int regmap_slimbus_paged_read(void *context, const void *reg,
				     size_t reg_size, void *val,
				     size_t val_size)
{
	struct regmap_slimbus_paged_ctx *ctx = context;
	struct slim_device *sdev = ctx->sdev;
	int tries = REGMAP_SLIMBUS_PAGED_TRIES;
	u16 vaddr = *(u16 *)reg;
	u16 hw_addr;
	int ret;

	hw_addr = REGMAP_SLIMBUS_WCD_BASE + vaddr;
	do {
		ret = slim_read(sdev, hw_addr, val_size, val);
		if (!ret || --tries == 0)
			break;
		usleep_range(5000, 5100);
	} while (1);

	return ret;
}

static void regmap_slimbus_paged_free_ctx(void *context)
{
	kfree(context);
}

static const struct regmap_bus regmap_slimbus_paged_bus = {
	.write = regmap_slimbus_paged_write,
	.read = regmap_slimbus_paged_read,
	.free_context = regmap_slimbus_paged_free_ctx,
	.reg_format_endian_default = REGMAP_ENDIAN_LITTLE,
	.val_format_endian_default = REGMAP_ENDIAN_LITTLE,
};

struct regmap *__regmap_init_slimbus_paged(struct slim_device *slimbus,
					   const struct regmap_config *config,
					   struct lock_class_key *lock_key,
					   const char *lock_name)
{
	struct regmap_slimbus_paged_ctx *ctx;

	if (config->val_bits != 8 || config->reg_bits != 16)
		return ERR_PTR(-ENOTSUPP);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->sdev = slimbus;

	return __regmap_init(&slimbus->dev, &regmap_slimbus_paged_bus, ctx,
			     config, lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__regmap_init_slimbus_paged);

static const struct regmap_bus *regmap_get_slimbus(struct slim_device *slim,
					const struct regmap_config *config)
{
	if (config->val_bits == 8 && config->reg_bits == 16)
		return &regmap_slimbus_bus;

	return ERR_PTR(-ENOTSUPP);
}

struct regmap *__regmap_init_slimbus(struct slim_device *slimbus,
				     const struct regmap_config *config,
				     struct lock_class_key *lock_key,
				     const char *lock_name)
{
	const struct regmap_bus *bus = regmap_get_slimbus(slimbus, config);

	if (IS_ERR(bus))
		return ERR_CAST(bus);

	return __regmap_init(&slimbus->dev, bus, slimbus, config, lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__regmap_init_slimbus);

struct regmap *__devm_regmap_init_slimbus(struct slim_device *slimbus,
					  const struct regmap_config *config,
					  struct lock_class_key *lock_key,
					  const char *lock_name)
{
	const struct regmap_bus *bus = regmap_get_slimbus(slimbus, config);

	if (IS_ERR(bus))
		return ERR_CAST(bus);

	return __devm_regmap_init(&slimbus->dev, bus, slimbus, config, lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__devm_regmap_init_slimbus);

MODULE_DESCRIPTION("Register map access API - SLIMbus support");
MODULE_LICENSE("GPL v2");
