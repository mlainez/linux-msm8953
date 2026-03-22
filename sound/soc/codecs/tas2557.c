// SPDX-License-Identifier: GPL-2.0
//
// Driver for the Texas Instruments TAS2557 Audio Amplifier
//
// Minimal mainline driver based on TI downstream tas2557-android-driver.
// Implements book/page register access, firmware loading from
// tas2557_uCDSP.bin, and ASoC codec registration.
//
// Architecture matches downstream: firmware is loaded asynchronously
// right after i2c_probe via request_firmware_nowait.  The fw_ready
// callback initializes the chip fully (hw_reset, SW_RESET, default
// registers, program blocks, PLL, config), then shuts down into idle.
// At mute_stream(unmute), we just run startup + unmute sequences.
// This avoids the chip dying during the idle gap between probe and
// first playback.

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/firmware.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "tas2557.h"

#define TAS2557_FORMATS                                      \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
	 SNDRV_PCM_FMTBIT_S32_LE)

#define TAS2557_NUM_SUPPLIES 3

static const char *const tas2557_supply_names[TAS2557_NUM_SUPPLIES] = {
	"dvdd",
	"vddio",
	"vdd",
};

struct tas2557_data {
	struct snd_soc_component *component;
	struct gpio_desc *reset_gpio;
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[TAS2557_NUM_SUPPLIES];
	u8 *fw_data;
	size_t fw_size;
	int cur_book;
	int cur_page;
	bool powered;
	bool fw_loaded;
};

/* --- Book/Page register access --- */

static int tas2557_change_book_page(struct tas2557_data *tas2557, int book,
				    int page)
{
	int ret;

	if (tas2557->cur_page != page || tas2557->cur_book != book) {
		if (tas2557->cur_book != book) {
			/* Must go to page 0 first to access book register */
			ret = regmap_write(tas2557->regmap, TAS2557_PAGE_REG,
					   0);
			if (ret < 0)
				return ret;

			ret = regmap_write(tas2557->regmap, TAS2557_BOOK_REG,
					   book);
			if (ret < 0)
				return ret;

			tas2557->cur_book = book;
		}

		ret = regmap_write(tas2557->regmap, TAS2557_PAGE_REG, page);
		if (ret < 0)
			return ret;

		tas2557->cur_page = page;
	}

	return 0;
}

static int tas2557_reg_write(struct tas2557_data *tas2557, unsigned int reg,
			     unsigned int value)
{
	int book = reg / (256 * 128);
	int page = (reg % (256 * 128)) / 128;
	int offset = reg % 128;
	int ret;

	ret = tas2557_change_book_page(tas2557, book, page);
	if (ret < 0)
		return ret;

	return regmap_write(tas2557->regmap, offset, value);
}

static int tas2557_reg_read(struct tas2557_data *tas2557, unsigned int reg,
			    unsigned int *value)
{
	int book = reg / (256 * 128);
	int page = (reg % (256 * 128)) / 128;
	int offset = reg % 128;
	int ret;

	ret = tas2557_change_book_page(tas2557, book, page);
	if (ret < 0)
		return ret;

	return regmap_read(tas2557->regmap, offset, value);
}

static int tas2557_reg_bulk_write(struct tas2557_data *tas2557,
				  unsigned int reg, const u8 *data,
				  unsigned int len)
{
	int book = reg / (256 * 128);
	int page = (reg % (256 * 128)) / 128;
	int offset = reg % 128;
	int ret;

	ret = tas2557_change_book_page(tas2557, book, page);
	if (ret < 0)
		return ret;

	return regmap_bulk_write(tas2557->regmap, offset, data, len);
}

static int tas2557_reg_update_bits(struct tas2557_data *tas2557,
				   unsigned int reg, unsigned int mask,
				   unsigned int val)
{
	int book = reg / (256 * 128);
	int page = (reg % (256 * 128)) / 128;
	int offset = reg % 128;
	int ret;

	ret = tas2557_change_book_page(tas2557, book, page);
	if (ret < 0)
		return ret;

	return regmap_update_bits(tas2557->regmap, offset, mask, val);
}

/* --- Firmware parsing --- */

static inline u32 tas2557_fw_get_u32(const u8 *data)
{
	return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
}

static inline u16 tas2557_fw_get_u16(const u8 *data)
{
	return (data[0] << 8) | data[1];
}

/*
 * Load a firmware block: sequence of 4-byte commands.
 * Command format per downstream tas2557_load_block():
 *   offset 0x00-0x7F: register write {book, page, offset, data}
 *   offset 0x81:      sleep — msleep((book << 8) + page)
 *   offset 0x85:      bulk write — length=(book<<8)+page, next cmd has
 *                      {book, page, offset, data[0]}, rest packed in
 *                      subsequent 4-byte slots
 */
static int tas2557_load_block(struct tas2557_data *tas2557, const u8 *data,
			      unsigned int num_cmds)
{
	unsigned int i = 0;
	int ret;

	/*
	 * Downstream tas2557_load_block() (tas2557-core.c line 1851) does NOT
	 * check return values of individual write() and bulk_write() calls.
	 * Only the CRC/checksum is verified after all commands complete.
	 * We match this behavior: log errors as warnings but keep going.
	 */
	while (i < num_cmds) {
		unsigned int book = data[i * 4];
		unsigned int page = data[i * 4 + 1];
		unsigned int offset = data[i * 4 + 2];
		unsigned int val = data[i * 4 + 3];

		i++;

		if (offset <= 0x7F) {
			ret = tas2557_reg_write(
				tas2557, TAS2557_REG(book, page, offset), val);
			if (ret < 0)
				dev_warn(
					tas2557->dev,
					"reg write warn at cmd %u/%u: B%u P%u R%u V%u ret=%d\n",
					i, num_cmds, book, page, offset, val,
					ret);
		} else if (offset == 0x81) {
			unsigned int sleep_ms = (book << 8) + page;

			msleep(sleep_ms);
		} else if (offset == 0x85) {
			unsigned int length = (book << 8) + page;
			const u8 *bulk_cmd;

			if (i >= num_cmds)
				return -EINVAL;

			bulk_cmd = data + i * 4;
			book = bulk_cmd[0];
			page = bulk_cmd[1];
			offset = bulk_cmd[2];

			if (length > 1) {
				ret = tas2557_reg_bulk_write(
					tas2557,
					TAS2557_REG(book, page, offset),
					bulk_cmd + 3, length);
			} else {
				ret = tas2557_reg_write(tas2557,
							TAS2557_REG(book, page,
								    offset),
							bulk_cmd[3]);
			}
			if (ret < 0)
				dev_warn(
					tas2557->dev,
					"bulk write warn at cmd %u/%u: B%u P%u R%u len=%u ret=%d\n",
					i, num_cmds, book, page, offset, length,
					ret);

			i++;
			if (length >= 2)
				i += ((length - 2) / 4) + 1;
		}
	}

	return 0;
}

/*
 * Skip a variable-length string in the firmware: reads until null
 * terminator. Returns the number of bytes consumed (including null).
 */
static int tas2557_fw_skip_string(const u8 *data, unsigned int remaining)
{
	unsigned int i;

	for (i = 0; i < remaining; i++) {
		if (data[i] == 0)
			return i + 1;
	}

	return -EINVAL;
}

static int tas2557_fw_parse_block(struct tas2557_data *tas2557, const u8 *data,
				  unsigned int remaining,
				  unsigned int driver_ver, u32 load_type)
{
	unsigned int off = 0;
	unsigned int block_type;
	unsigned int num_cmds;
	int ret;

	if (remaining < 4)
		return -EINVAL;

	block_type = tas2557_fw_get_u32(data + off);
	off += 4;

	if (driver_ver >= 0x200) {
		if (remaining < off + 4)
			return -EINVAL;
		off += 4;
	}

	if (remaining < off + 4)
		return -EINVAL;

	num_cmds = tas2557_fw_get_u32(data + off);
	off += 4;

	if (remaining < off + num_cmds * 4)
		return -EINVAL;

	dev_dbg(tas2557->dev, "block type=0x%x cmds=%u load=%s\n", block_type,
		num_cmds,
		(load_type != U32_MAX && block_type == load_type) ? "yes" :
								    "skip");

	if (load_type != U32_MAX && block_type == load_type) {
		ret = tas2557_load_block(tas2557, data + off, num_cmds);
		if (ret < 0)
			return ret;
	}

	off += num_cmds * 4;

	return off;
}

static int tas2557_fw_parse_data(struct tas2557_data *tas2557, const u8 *data,
				 unsigned int remaining,
				 unsigned int driver_ver, u32 load_type)
{
	unsigned int off = 0;
	unsigned int num_blocks;
	unsigned int i;
	int ret;

	if (remaining < 64)
		return -EINVAL;
	off += 64;

	ret = tas2557_fw_skip_string(data + off, remaining - off);
	if (ret < 0)
		return ret;
	off += ret;

	if (remaining < off + 2)
		return -EINVAL;
	num_blocks = tas2557_fw_get_u16(data + off);
	off += 2;

	dev_dbg(tas2557->dev, "  data section: %u blocks\n", num_blocks);

	for (i = 0; i < num_blocks; i++) {
		ret = tas2557_fw_parse_block(tas2557, data + off,
					     remaining - off, driver_ver,
					     load_type);
		if (ret < 0)
			return ret;
		off += ret;
	}

	return off;
}

/*
 * Walk the firmware to find the start offset and size of a specific
 * program's or configuration's data section.  This allows us to
 * re-scan only the blocks we need without storing parsed structures.
 *
 * The firmware layout after the header is:
 *   PLLs → Programs → Configurations → Calibrations
 *
 * We need to skip to the right section and index to find the data.
 */

static int tas2557_fw_skip_block(const u8 *data, unsigned int remaining,
				 unsigned int driver_ver)
{
	unsigned int off = 0;
	unsigned int num_cmds;

	if (remaining < 4)
		return -EINVAL;
	off += 4; /* block_type */

	if (driver_ver >= 0x200) {
		if (remaining < off + 4)
			return -EINVAL;
		off += 4; /* checksums */
	}

	if (remaining < off + 4)
		return -EINVAL;
	num_cmds = (data[off] << 24) | (data[off + 1] << 16) |
		   (data[off + 2] << 8) | data[off + 3];
	off += 4;

	if (remaining < off + num_cmds * 4)
		return -EINVAL;
	off += num_cmds * 4;

	return off;
}

static int tas2557_fw_skip_data(const u8 *data, unsigned int remaining,
				unsigned int driver_ver)
{
	unsigned int off = 0;
	unsigned int num_blocks, i;
	int ret;

	if (remaining < 64)
		return -EINVAL;
	off += 64; /* name */

	ret = tas2557_fw_skip_string(data + off, remaining - off);
	if (ret < 0)
		return ret;
	off += ret; /* description */

	if (remaining < off + 2)
		return -EINVAL;
	num_blocks = (data[off] << 8) | data[off + 1];
	off += 2;

	for (i = 0; i < num_blocks; i++) {
		ret = tas2557_fw_skip_block(data + off, remaining - off,
					    driver_ver);
		if (ret < 0)
			return ret;
		off += ret;
	}

	return off;
}

static int tas2557_load_firmware(struct tas2557_data *tas2557, const u8 *data,
				 unsigned int size)
{
	unsigned int offset = 0;
	unsigned int driver_ver;
	unsigned int device;
	unsigned int num_plls, num_programs, num_configs, num_calibrations;
	unsigned int prog0_data_off = 0, cfg0_data_off = 0;
	unsigned int pll0_off = 0;
	unsigned int i;
	int ret;

	if (size < 4)
		return -EINVAL;

	if (data[0] != 0x35 || data[1] != 0x35 || data[2] != 0x35 ||
	    data[3] != 0x32) {
		dev_err(tas2557->dev, "invalid firmware magic\n");
		return -EINVAL;
	}
	offset += 4;

	if (size < offset + 24)
		return -EINVAL;

	offset += 4; /* fw_size */
	offset += 4; /* checksum */
	offset += 4; /* ppc_ver */
	offset += 4; /* fw_ver */
	driver_ver = tas2557_fw_get_u32(data + offset);
	offset += 4; /* driver_ver */
	offset += 4; /* timestamp */

	dev_dbg(tas2557->dev, "firmware driver version: 0x%x\n", driver_ver);

	if (size < offset + 64)
		return -EINVAL;
	offset += 64; /* DDC name */

	ret = tas2557_fw_skip_string(data + offset, size - offset);
	if (ret < 0)
		return ret;
	offset += ret; /* description */

	if (size < offset + 8)
		return -EINVAL;

	offset += 4; /* device_family */
	device = tas2557_fw_get_u32(data + offset);
	offset += 4;

	if (device != TAS2557_DEVICE_MONO) {
		dev_err(tas2557->dev,
			"firmware device mismatch: expected %d, got %d\n",
			TAS2557_DEVICE_MONO, device);
		return -EINVAL;
	}

	/* --- PLLs: skip all, remember first PLL offset --- */
	if (size < offset + 2)
		return -EINVAL;
	num_plls = tas2557_fw_get_u16(data + offset);
	offset += 2;

	for (i = 0; i < num_plls; i++) {
		if (size < offset + 64)
			return -EINVAL;
		if (i == 0)
			pll0_off = offset + 64; /* after name, will adjust */
		offset += 64; /* name */

		ret = tas2557_fw_skip_string(data + offset, size - offset);
		if (ret < 0)
			return ret;
		if (i == 0)
			pll0_off = offset + ret; /* points to block_data */
		offset += ret; /* description */

		ret = tas2557_fw_skip_block(data + offset, size - offset,
					    driver_ver);
		if (ret < 0)
			return ret;
		offset += ret;
	}

	/* --- Programs: skip all, remember program 0 data section offset --- */
	if (size < offset + 2)
		return -EINVAL;
	num_programs = tas2557_fw_get_u16(data + offset);
	offset += 2;

	for (i = 0; i < num_programs; i++) {
		if (size < offset + 64)
			return -EINVAL;
		offset += 64; /* name */

		ret = tas2557_fw_skip_string(data + offset, size - offset);
		if (ret < 0)
			return ret;
		offset += ret; /* description */

		if (size < offset + 3)
			return -EINVAL;
		offset += 3; /* app_mode(1) + boost(u16) */

		if (i == 0)
			prog0_data_off = offset;

		ret = tas2557_fw_skip_data(data + offset, size - offset,
					   driver_ver);
		if (ret < 0)
			return ret;
		offset += ret;
	}

	/* --- Configurations: skip all, remember config 0 data section offset --- */
	if (size < offset + 2)
		return -EINVAL;
	num_configs = tas2557_fw_get_u16(data + offset);
	offset += 2;

	for (i = 0; i < num_configs; i++) {
		unsigned int config_hdr_size;

		if (size < offset + 64)
			return -EINVAL;
		offset += 64; /* name */

		ret = tas2557_fw_skip_string(data + offset, size - offset);
		if (ret < 0)
			return ret;
		offset += ret; /* description */

		config_hdr_size = 0;
		if (driver_ver >= 0x300)
			config_hdr_size += 2;
		config_hdr_size += 1 + 1 + 4;
		if (driver_ver >= 0x400)
			config_hdr_size += 1 + 4;

		if (size < offset + config_hdr_size)
			return -EINVAL;
		offset += config_hdr_size;

		if (i == 0)
			cfg0_data_off = offset;

		ret = tas2557_fw_skip_data(data + offset, size - offset,
					   driver_ver);
		if (ret < 0)
			return ret;
		offset += ret;
	}

	/* --- Calibrations: skip --- */
	if (size >= offset + 2) {
		num_calibrations = tas2557_fw_get_u16(data + offset);
		offset += 2;

		for (i = 0; i < num_calibrations; i++) {
			if (size < offset + 64)
				return -EINVAL;
			offset += 64; /* name */

			ret = tas2557_fw_skip_string(data + offset,
						     size - offset);
			if (ret < 0)
				return ret;
			offset += ret; /* description */

			if (size < offset + 2)
				return -EINVAL;
			offset += 2; /* program(1) + configuration(1) */

			ret = tas2557_fw_skip_data(data + offset, size - offset,
						   driver_ver);
			if (ret < 0)
				return ret;
			offset += ret;
		}
	}

	/*
	 * Load sequence matching downstream tas2557_set_program() +
	 * tas2557_load_coefficient():
	 *   1. DEV_A             (set_program: tas2557_load_data PGM_DEV_A)
	 *   2. DAC gain read     (set_program: tas2557_get_DAC_gain)
	 *   3. PLL               (load_coefficient)
	 *   4. PRE, COEFF        (load_coefficient)
	 */
	if (num_programs > 0 && prog0_data_off) {
		ret = tas2557_fw_parse_data(tas2557, data + prog0_data_off,
					    size - prog0_data_off, driver_ver,
					    TAS2557_BLOCK_BASE_MAIN);
		if (ret < 0) {
			dev_err(tas2557->dev, "program 0 load failed: %d\n",
				ret);
			return ret;
		}

		{
			unsigned int spk_ctrl = 0;

			tas2557_reg_read(tas2557, TAS2557_SPK_CTRL_REG,
					 &spk_ctrl);
			dev_info(
				tas2557->dev,
				"DAC gain after program load: SPK_CTRL=0x%02x gain=%u dB\n",
				spk_ctrl, (spk_ctrl >> 3) & 0xf);
		}
	}

	if (num_plls > 0 && pll0_off) {
		ret = tas2557_fw_parse_block(tas2557, data + pll0_off,
					     size - pll0_off, driver_ver,
					     TAS2557_BLOCK_PLL);
		if (ret < 0) {
			dev_err(tas2557->dev, "PLL 0 load failed: %d\n", ret);
			return ret;
		}
	}

	if (num_configs > 0 && cfg0_data_off) {
		ret = tas2557_fw_parse_data(tas2557, data + cfg0_data_off,
					    size - cfg0_data_off, driver_ver,
					    TAS2557_BLOCK_CONF_PRE);
		if (ret < 0) {
			dev_err(tas2557->dev, "config 0 PRE load failed: %d\n",
				ret);
			return ret;
		}

		ret = tas2557_fw_parse_data(tas2557, data + cfg0_data_off,
					    size - cfg0_data_off, driver_ver,
					    TAS2557_BLOCK_CONF_COEFF);
		if (ret < 0) {
			dev_err(tas2557->dev,
				"config 0 COEFF load failed: %d\n", ret);
			return ret;
		}
	}

	dev_info(tas2557->dev, "firmware loaded successfully\n");

	return 0;
}

/* --- Hardware init / startup / shutdown --- */

static int tas2557_hw_reset(struct tas2557_data *tas2557)
{
	if (tas2557->reset_gpio) {
		gpiod_set_value_cansleep(tas2557->reset_gpio, 0);
		msleep(5);
		gpiod_set_value_cansleep(tas2557->reset_gpio, 1);
		msleep(2);
	}

	tas2557->cur_book = 0;
	tas2557->cur_page = 0;

	return 0;
}

/*
 * Default init sequence matching downstream p_tas2557_default_data[] +
 * load_platdata().  Called after SW_RESET when clocks are running.
 */
static int tas2557_default_init(struct tas2557_data *tas2557)
{
	unsigned int broadcast_val;
	int ret;

	ret = tas2557_reg_write(tas2557, TAS2557_TEST_MODE_REG, 0x0d);
	if (ret < 0)
		return ret;

	ret = tas2557_reg_read(tas2557, TAS2557_BROADCAST_REG, &broadcast_val);
	if (ret < 0)
		return ret;
	dev_dbg(tas2557->dev, "broadcast trim=0x%x\n", broadcast_val);

	/* Downstream p_tas2557_default_data[] */
	ret = tas2557_reg_write(tas2557, TAS2557_SAR_ADC2_REG, 0x05);
	if (ret < 0)
		return ret;

	ret = tas2557_reg_write(tas2557, TAS2557_CLK_ERR2_REG, 0x21);
	if (ret < 0)
		return ret;

	ret = tas2557_reg_write(tas2557, TAS2557_CLK_ERR3_REG, 0x21);
	if (ret < 0)
		return ret;

	ret = tas2557_reg_write(tas2557, TAS2557_SAFE_GUARD_REG,
				TAS2557_SAFE_GUARD_PATTERN);
	if (ret < 0)
		return ret;

	/* load_platdata: ASI1 DAC format bit 0 */
	ret = tas2557_reg_update_bits(tas2557, TAS2557_ASI1_DAC_FORMAT_REG,
				      0x01, 0x01);
	if (ret < 0)
		return ret;

	/* load_platdata: configIRQ — GPIO HIZ for interrupt pin */
	ret = tas2557_reg_update_bits(tas2557, TAS2557_GPIO_HIZ_CTRL2_REG, 0x30,
				      0x30);
	if (ret < 0)
		return ret;

	dev_dbg(tas2557->dev, "default_init: complete\n");
	return 0;
}

/*
 * Startup sequence from TI downstream driver p_tas2557_startup_data[]:
 *   1. Enable GPI pins (DIN, MCLK, CCI)
 *   2. Enable BCLK, WCLK on GPIO1/GPIO2
 *   3. Power up Class-D, Boost, IV sense
 *   4. Power up PLL, DSP, clock dividers
 *   5. Enable clock error detection
 */
static int tas2557_startup(struct tas2557_data *tas2557)
{
	int ret;

	/* GPI pin enable: DIN, MCLK, CCI */
	ret = tas2557_reg_write(tas2557, TAS2557_GPI_PIN_REG, 0x15);
	if (ret < 0)
		return ret;

	/* GPIO1 = BCLK, GPIO2 = WCLK */
	ret = tas2557_reg_write(tas2557, TAS2557_GPIO1_PIN_REG, 0x01);
	if (ret < 0)
		return ret;

	ret = tas2557_reg_write(tas2557, TAS2557_GPIO2_PIN_REG, 0x01);
	if (ret < 0)
		return ret;

	/* Power up Class-D + Boost */
	ret = tas2557_reg_write(tas2557, TAS2557_POWER_CTRL2_REG, 0xa0);
	if (ret < 0)
		return ret;

	/* + IV sense power up */
	ret = tas2557_reg_write(tas2557, TAS2557_POWER_CTRL2_REG, 0xa3);
	if (ret < 0)
		return ret;

	/* PLL + DSP + clock dividers power up */
	ret = tas2557_reg_write(tas2557, TAS2557_POWER_CTRL1_REG, 0xf8);
	if (ret < 0)
		return ret;

	usleep_range(2000, 3000);

	/* Enable clock error detection — clocks are running at this point */
	ret = tas2557_reg_write(tas2557, TAS2557_CLK_ERR1_REG, 0x2b);
	if (ret < 0)
		return ret;

	return 0;
}

static void tas2557_unmute(struct tas2557_data *tas2557)
{
	/* Downstream p_tas2557_unmute_data[] */
	tas2557_reg_write(tas2557, TAS2557_MUTE_REG, 0x00);
	tas2557_reg_write(tas2557, TAS2557_DSP_MUTE_REG, 0x00);
}

/*
 * Shutdown sequence from TI downstream p_tas2557_shutdown_data[]:
 * Disable clock error → soft mute → mute → power down → GPIO off
 */
static void tas2557_do_shutdown(struct tas2557_data *tas2557)
{
	tas2557_reg_write(tas2557, TAS2557_CLK_ERR1_REG, 0x00);
	tas2557_reg_write(tas2557, TAS2557_DSP_MUTE_REG, 0x01);
	usleep_range(10000, 12000);
	tas2557_reg_write(tas2557, TAS2557_MUTE_REG, 0x03);
	tas2557_reg_write(tas2557, TAS2557_POWER_CTRL1_REG, 0x60);
	usleep_range(2000, 3000);
	tas2557_reg_write(tas2557, TAS2557_POWER_CTRL2_REG, 0x00);
	tas2557_reg_write(tas2557, TAS2557_POWER_CTRL1_REG, 0x00);
	tas2557_reg_write(tas2557, TAS2557_GPIO1_PIN_REG, 0x00);
	tas2557_reg_write(tas2557, TAS2557_GPIO2_PIN_REG, 0x00);
	tas2557_reg_write(tas2557, TAS2557_GPI_PIN_REG, 0x00);
}

/*
 * Firmware ready callback — called from request_firmware_nowait context.
 *
 * Matching downstream flow:
 *   i2c_probe → request_firmware_nowait(tas2557_fw_ready)
 *   tas2557_fw_ready → tas2557_set_program(0):
 *     hw_reset → SW_RESET → default_init → load program → PLL → config
 *   Then if !powered → stay in shutdown state (DSP off, but chip alive)
 *
 * The chip is freshly reset from i2c_probe, but we do another hw_reset +
 * SW_RESET here matching downstream tas2557_set_program() which always
 * resets before loading a new program.
 */
static void tas2557_fw_ready(const struct firmware *fw, void *context)
{
	struct tas2557_data *tas2557 = context;
	unsigned int sg_val = 0;
	int ret;

	if (!fw) {
		dev_err(tas2557->dev, "firmware request failed — no %s found\n",
			TAS2557_FW_NAME);
		return;
	}

	dev_info(tas2557->dev,
		 "firmware %s loaded (%zu bytes), programming chip\n",
		 TAS2557_FW_NAME, fw->size);

	/*
	 * Full chip init matching downstream tas2557_set_program():
	 * hw_reset → SW_RESET → default_init → load firmware blocks
	 */
	tas2557_hw_reset(tas2557);

	ret = tas2557_reg_write(tas2557, TAS2557_SW_RESET_REG,
				TAS2557_SW_RESET);
	if (ret < 0) {
		dev_err(tas2557->dev, "SW_RESET failed in fw_ready: %d\n", ret);
		goto out;
	}
	msleep(1);
	tas2557->cur_book = 0;
	tas2557->cur_page = 0;

	ret = tas2557_default_init(tas2557);
	if (ret < 0) {
		dev_err(tas2557->dev, "default_init failed in fw_ready: %d\n",
			ret);
		goto out;
	}

	/* Load firmware blocks (program 0 + PLL 0 + config 0) */
	ret = tas2557_load_firmware(tas2557, fw->data, fw->size);

	if (ret < 0) {
		dev_err(tas2557->dev, "firmware programming failed: %d\n", ret);
		goto out;
	}

	/* Verify safe guard pattern survived firmware loading */
	ret = tas2557_reg_read(tas2557, TAS2557_SAFE_GUARD_REG, &sg_val);
	if (ret < 0 || sg_val != TAS2557_SAFE_GUARD_PATTERN) {
		dev_warn(
			tas2557->dev,
			"safe guard check after fw load: ret=%d val=0x%x (expected 0x%x)\n",
			ret, sg_val, TAS2557_SAFE_GUARD_PATTERN);
	}

	/*
	 * Keep a copy of the firmware data so we can re-program the chip
	 * later if it dies during idle (safe guard check failure).
	 */
	if (!tas2557->fw_data) {
		tas2557->fw_data = devm_kmemdup(tas2557->dev, fw->data,
						fw->size, GFP_KERNEL);
		if (tas2557->fw_data)
			tas2557->fw_size = fw->size;
		else
			dev_warn(
				tas2557->dev,
				"failed to save firmware copy — recovery will not work\n");
	}

	/*
	 * Chip is now fully initialized with firmware but not playing.
	 * Use the full shutdown sequence to reach a safe idle state.
	 * This disables clock error detection, mutes, powers down,
	 * and disables GPIO pins — matching downstream shutdown.
	 * tas2557_startup() will re-enable everything when playback begins.
	 */
	tas2557_do_shutdown(tas2557);

	tas2557->fw_loaded = true;
	dev_info(tas2557->dev, "chip programmed and ready for playback\n");

out:
	release_firmware(fw);
}

/* --- ASoC codec --- */

static int tas2557_power_cycle(struct tas2557_data *tas2557)
{
	int ret;

	if (tas2557->reset_gpio)
		gpiod_set_value_cansleep(tas2557->reset_gpio, 0);

	regulator_bulk_disable(TAS2557_NUM_SUPPLIES, tas2557->supplies);
	msleep(50);

	ret = regulator_bulk_enable(TAS2557_NUM_SUPPLIES, tas2557->supplies);
	if (ret < 0) {
		dev_err(tas2557->dev, "regulator re-enable failed: %d\n", ret);
		return ret;
	}
	msleep(10);

	if (tas2557->reset_gpio) {
		gpiod_set_value_cansleep(tas2557->reset_gpio, 1);
		msleep(2);
	}

	tas2557->cur_book = 0;
	tas2557->cur_page = 0;

	return 0;
}

static int tas2557_reinit(struct tas2557_data *tas2557)
{
	unsigned int rev_id = 0;
	int ret;

	if (!tas2557->fw_data || !tas2557->fw_size) {
		dev_err(tas2557->dev,
			"no firmware copy available for reinit\n");
		return -ENOENT;
	}

	dev_info(tas2557->dev,
		 "reinitializing chip (recovery from idle death)\n");

	tas2557_hw_reset(tas2557);
	i2c_recover_bus(tas2557->client->adapter);

	ret = tas2557_reg_read(tas2557, TAS2557_REV_PGID_REG, &rev_id);
	dev_info(tas2557->dev, "post-reset revision read: ret=%d val=0x%x\n",
		 ret, rev_id);
	if (ret < 0) {
		dev_info(tas2557->dev,
			 "chip unresponsive after hw_reset, power cycling\n");
		ret = tas2557_power_cycle(tas2557);
		if (ret < 0)
			return ret;
		i2c_recover_bus(tas2557->client->adapter);

		ret = tas2557_reg_read(tas2557, TAS2557_REV_PGID_REG, &rev_id);
		dev_info(tas2557->dev,
			 "post-power-cycle revision read: ret=%d val=0x%x\n",
			 ret, rev_id);
		if (ret < 0)
			return ret;
	}

	ret = tas2557_reg_write(tas2557, TAS2557_SW_RESET_REG,
				TAS2557_SW_RESET);
	if (ret < 0)
		return ret;
	msleep(1);
	tas2557->cur_book = 0;
	tas2557->cur_page = 0;

	ret = tas2557_default_init(tas2557);
	if (ret < 0)
		return ret;

	ret = tas2557_load_firmware(tas2557, tas2557->fw_data,
				    tas2557->fw_size);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * Enable/disable the amplifier.
 *
 * Called from mute_stream.  At this point BCLK/MCLK are guaranteed
 * running from the SoC MI2S interface.
 *
 * On unmute: check that firmware was loaded → startup → unmute
 * On mute:   shutdown
 *
 * NO firmware reload, NO hw_reset, NO regulator cycling.
 * The chip was already programmed by tas2557_fw_ready().
 */
static int tas2557_enable(struct tas2557_data *tas2557, bool enable)
{
	unsigned int sg_val = 0;
	int ret;

	if (enable) {
		if (!tas2557->fw_loaded) {
			dev_err(tas2557->dev,
				"cannot enable: firmware not loaded yet\n");
			return -ENODEV;
		}

		/* Verify safe guard — chip is still alive and programmed */
		ret = tas2557_reg_read(tas2557, TAS2557_SAFE_GUARD_REG,
				       &sg_val);
		if (ret < 0 || sg_val != TAS2557_SAFE_GUARD_PATTERN) {
			dev_warn(
				tas2557->dev,
				"safe guard failed (ret=%d val=0x%x), attempting reinit\n",
				ret, sg_val);
			ret = tas2557_reinit(tas2557);
			if (ret < 0) {
				dev_err(tas2557->dev, "reinit failed: %d\n",
					ret);
				return ret;
			}
		}

		ret = tas2557_startup(tas2557);
		if (ret < 0) {
			dev_err(tas2557->dev, "startup failed: %d\n", ret);
			return ret;
		}

		tas2557_unmute(tas2557);
		tas2557->powered = true;

		{
			unsigned int spk_ctrl = 0, mute_val = 0, pwr1 = 0;
			unsigned int pwr2 = 0, clk_err1 = 0;

			tas2557_reg_read(tas2557, TAS2557_SPK_CTRL_REG,
					 &spk_ctrl);
			tas2557_reg_read(tas2557, TAS2557_MUTE_REG, &mute_val);
			tas2557_reg_read(tas2557, TAS2557_POWER_CTRL1_REG,
					 &pwr1);
			tas2557_reg_read(tas2557, TAS2557_POWER_CTRL2_REG,
					 &pwr2);
			tas2557_reg_read(tas2557, TAS2557_CLK_ERR1_REG,
					 &clk_err1);

			dev_info(
				tas2557->dev,
				"post-unmute regs: SPK_CTRL=0x%02x MUTE=0x%02x PWR1=0x%02x PWR2=0x%02x CLK_ERR1=0x%02x\n",
				spk_ctrl, mute_val, pwr1, pwr2, clk_err1);
		}

		dev_info(tas2557->dev, "amplifier enabled\n");
	} else {
		if (tas2557->powered) {
			tas2557_do_shutdown(tas2557);
			tas2557->powered = false;
			dev_info(tas2557->dev, "amplifier disabled\n");
		}
	}

	return 0;
}

static int tas2557_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct tas2557_data *tas2557 =
		snd_soc_component_get_drvdata(dai->component);

	return tas2557_enable(tas2557, !mute);
}

static int tas2557_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	/*
	 * The TAS2557 sample rate and format are configured by the
	 * firmware/DSP. No per-stream register configuration needed
	 * for basic playback — the firmware configures the I2S
	 * interface during the configuration block loading.
	 */
	return 0;
}

static int tas2557_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* Firmware configures I2S format. Accept I2S + NB_NF. */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops tas2557_dai_ops = {
	.hw_params = tas2557_hw_params,
	.set_fmt = tas2557_set_dai_fmt,
	.mute_stream = tas2557_mute_stream,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver tas2557_dai[] = {
	{
		.name = "tas2557-amplifier",
		.id = 0,
		.playback = {
			.stream_name	= "ASI1 Playback",
			.channels_min	= 1,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= TAS2557_FORMATS,
		},
		.ops = &tas2557_dai_ops,
	},
};

static const struct snd_soc_dapm_widget tas2557_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route tas2557_audio_map[] = {
	{ "DAC", NULL, "ASI1" },
	{ "OUT", NULL, "DAC" },
};

/*
 * Codec probe: minimal — just store component pointer.
 * All chip init happens in i2c_probe + fw_ready callback.
 */
static int tas2557_codec_probe(struct snd_soc_component *component)
{
	struct tas2557_data *tas2557 = snd_soc_component_get_drvdata(component);

	tas2557->component = component;
	return 0;
}

static void tas2557_codec_remove(struct snd_soc_component *component)
{
	struct tas2557_data *tas2557 = snd_soc_component_get_drvdata(component);

	if (tas2557->powered)
		tas2557_do_shutdown(tas2557);
}

static const struct snd_soc_component_driver soc_component_dev_tas2557 = {
	.probe = tas2557_codec_probe,
	.remove = tas2557_codec_remove,
	.dapm_widgets = tas2557_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tas2557_dapm_widgets),
	.dapm_routes = tas2557_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tas2557_audio_map),
	.idle_bias_on = 1,
	.use_pmdown_time = 1,
	.endianness = 1,
};

/* --- I2C probe --- */

static const struct regmap_config tas2557_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 128,
	.cache_type = REGCACHE_NONE,
};

static void tas2557_regulator_disable(void *data)
{
	struct tas2557_data *tas2557 = data;

	regulator_bulk_disable(TAS2557_NUM_SUPPLIES, tas2557->supplies);
}

static int tas2557_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct tas2557_data *tas2557;
	unsigned int rev_id;
	int ret, i;

	tas2557 = devm_kzalloc(dev, sizeof(*tas2557), GFP_KERNEL);
	if (!tas2557)
		return -ENOMEM;

	tas2557->client = client;
	tas2557->dev = dev;

	for (i = 0; i < TAS2557_NUM_SUPPLIES; i++)
		tas2557->supplies[i].supply = tas2557_supply_names[i];

	ret = devm_regulator_bulk_get(dev, TAS2557_NUM_SUPPLIES,
				      tas2557->supplies);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get regulator supplies\n");

	ret = regulator_bulk_enable(TAS2557_NUM_SUPPLIES, tas2557->supplies);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to enable regulator supplies\n");

	ret = devm_add_action_or_reset(dev, tas2557_regulator_disable, tas2557);
	if (ret)
		return ret;

	tas2557->reset_gpio =
		devm_gpiod_get_optional(dev, "shutdown", GPIOD_OUT_HIGH);
	if (IS_ERR(tas2557->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(tas2557->reset_gpio),
				     "failed to get shutdown GPIO\n");

	/*
	 * Initial hw_reset matching downstream i2c_probe:
	 * GPIO low for 5ms → high → 2ms wait
	 */
	tas2557_hw_reset(tas2557);

	tas2557->regmap = devm_regmap_init_i2c(client, &tas2557_regmap_config);
	if (IS_ERR(tas2557->regmap))
		return dev_err_probe(dev, PTR_ERR(tas2557->regmap),
				     "failed to allocate register map\n");

	/*
	 * SW_RESET + revision read, matching downstream i2c_probe:
	 *   tas2557_dev_write(SW_RESET, 0x01)
	 *   msleep(1)
	 *   tas2557_dev_read(REV_PGID, &revision)
	 */
	ret = tas2557_reg_write(tas2557, TAS2557_SW_RESET_REG,
				TAS2557_SW_RESET);
	if (ret < 0)
		return dev_err_probe(dev, ret, "SW_RESET failed\n");
	msleep(1);

	ret = tas2557_reg_read(tas2557, TAS2557_REV_PGID_REG, &rev_id);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read REV_PGID\n");

	dev_info(dev, "TAS2557 revision: 0x%02x\n", rev_id);

	dev_set_drvdata(dev, tas2557);

	/*
	 * Request firmware asynchronously, matching downstream:
	 *   request_firmware_nowait(THIS_MODULE, ..., TAS2557_FW_NAME,
	 *                           dev, GFP_KERNEL, pTAS2557,
	 *                           tas2557_fw_ready)
	 *
	 * The callback will init the chip with firmware immediately,
	 * before the chip can die from idle timeout.
	 */
	ret = request_firmware_nowait(THIS_MODULE, true, TAS2557_FW_NAME, dev,
				      GFP_KERNEL, tas2557, tas2557_fw_ready);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "request_firmware_nowait failed\n");

	return devm_snd_soc_register_component(dev, &soc_component_dev_tas2557,
					       tas2557_dai,
					       ARRAY_SIZE(tas2557_dai));
}

static const struct of_device_id tas2557_of_match[] = {
	{
		.compatible = "ti,tas2557",
	},
	{},
};
MODULE_DEVICE_TABLE(of, tas2557_of_match);

static const struct i2c_device_id tas2557_id[] = { { "tas2557", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, tas2557_id);

static struct i2c_driver tas2557_i2c_driver = {
	.driver = {
		.name = "tas2557",
		.of_match_table = of_match_ptr(tas2557_of_match),
	},
	.probe = tas2557_i2c_probe,
	.id_table = tas2557_id,
};

module_i2c_driver(tas2557_i2c_driver);

MODULE_DESCRIPTION("TAS2557 Audio Amplifier driver");
MODULE_FIRMWARE(TAS2557_FW_NAME);
MODULE_LICENSE("GPL");
