// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX363 sensor driver
 *
 * Copyright (C) 2024 Marc Lainez <marc.lainez@gmail.com>
 *
 * Based on the sdm670-mainline IMX363 driver and the mainline IMX258 driver.
 * Register tables reverse-engineered from Fairphone 3 vendor firmware.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

/* Streaming mode */
#define IMX363_REG_MODE_SELECT CCI_REG8(0x0100)
#define IMX363_MODE_STANDBY 0x00
#define IMX363_MODE_STREAMING 0x01

/* Software reset */
#define IMX363_REG_RESET CCI_REG8(0x0103)

/* Chip ID */
#define IMX363_REG_CHIP_ID CCI_REG16(0x0016)
#define IMX363_CHIP_ID 0x0363

/* Group hold */
#define IMX363_REG_GROUP_HOLD CCI_REG8(0x0104)

/* Frame timing */
#define IMX363_REG_FRM_LENGTH_LINES CCI_REG16(0x0340)
#define IMX363_REG_LINE_LENGTH_PCK CCI_REG16(0x0342)

/* Exposure control */
#define IMX363_REG_EXPOSURE CCI_REG16(0x0202)
#define IMX363_EXPOSURE_MIN 4
#define IMX363_EXPOSURE_STEP 1
#define IMX363_EXPOSURE_DEFAULT 0x0640
#define IMX363_EXPOSURE_OFFSET 10

/* Analog gain control */
#define IMX363_REG_ANALOG_GAIN CCI_REG16(0x0204)
#define IMX363_ANA_GAIN_MIN 0
#define IMX363_ANA_GAIN_MAX 480
#define IMX363_ANA_GAIN_STEP 1
#define IMX363_ANA_GAIN_DEFAULT 0

/* Digital gain control */
#define IMX363_REG_GR_DIGITAL_GAIN CCI_REG16(0x020e)
#define IMX363_REG_R_DIGITAL_GAIN CCI_REG16(0x0210)
#define IMX363_REG_B_DIGITAL_GAIN CCI_REG16(0x0212)
#define IMX363_REG_GB_DIGITAL_GAIN CCI_REG16(0x0214)
#define IMX363_DGTL_GAIN_MIN 0
#define IMX363_DGTL_GAIN_MAX 4096
#define IMX363_DGTL_GAIN_DEFAULT 1024
#define IMX363_DGTL_GAIN_STEP 1

/* HDR control */
#define IMX363_REG_HDR CCI_REG8(0x0220)
#define IMX363_HDR_ON BIT(0)
#define IMX363_REG_HDR_RATIO CCI_REG8(0x0222)
#define IMX363_HDR_RATIO_MIN 0
#define IMX363_HDR_RATIO_MAX 5

/* Test pattern */
#define IMX363_REG_TEST_PATTERN CCI_REG16(0x0600)

/* Clock lane mode */
#define IMX363_CLK_BLANK_STOP CCI_REG8(0x4040)

/* Orientation */
#define IMX363_REG_ORIENTATION CCI_REG8(0x0101)
#define IMX363_ORIENT_HFLIP BIT(0)
#define IMX363_ORIENT_VFLIP BIT(1)

/* PLL registers */
#define IMX363_REG_IVTPXCK_DIV CCI_REG8(0x0301)
#define IMX363_REG_IVTSYCK_DIV CCI_REG8(0x0303)
#define IMX363_REG_PREPLLCK_VT_DIV CCI_REG8(0x0305)
#define IMX363_REG_PLL_IVT_MPY CCI_REG16(0x0306)
#define IMX363_REG_IOPPXCK_DIV CCI_REG8(0x0309)
#define IMX363_REG_IOPSYCK_DIV CCI_REG8(0x030b)
#define IMX363_REG_PREPLLCK_OP_DIV CCI_REG8(0x030d)
#define IMX363_REG_PLL_IOP_MPY CCI_REG16(0x030e)
#define IMX363_REG_PLL_MULT_DRIV CCI_REG8(0x0310)
#define IMX363_REG_CSI_LANE_MODE CCI_REG8(0x0114)
#define IMX363_REG_EXCK_FREQ CCI_REG16(0x0136)
#define IMX363_REG_REQ_LINK_BIT_RATE_H CCI_REG16(0x0820)
#define IMX363_REG_REQ_LINK_BIT_RATE_L CCI_REG16(0x0822)

/* Pixel array */
#define IMX363_NATIVE_WIDTH 4048U
#define IMX363_NATIVE_HEIGHT 3168U
#define IMX363_PIXEL_ARRAY_LEFT 8U
#define IMX363_PIXEL_ARRAY_TOP 24U
#define IMX363_PIXEL_ARRAY_WIDTH 4032U
#define IMX363_PIXEL_ARRAY_HEIGHT 3024U

#define IMX363_VTS_MAX 65525

/* Regulator supplies */
static const char *const imx363_supply_name[] = {
	"vana", /* Analog (2.8V) supply */
	"vdig", /* Digital Core (1.2V) supply */
	"vif", /* IF (1.8V) supply */
};

#define IMX363_NUM_SUPPLIES ARRAY_SIZE(imx363_supply_name)

struct imx363_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

/* No separate PLL table struct needed; PLL regs are in each mode table */

struct imx363_mode {
	u32 width;
	u32 height;
	u32 vts_def;
	u32 vts_min;
	u32 llp; /* Line length in pixels (for hblank) */
	u32 link_freq_index;
	struct imx363_reg_list reg_list;
	struct v4l2_rect crop;
};

/*
 * Bayer order table for flip combinations:
 *  [0] no flip, [1] h flip, [2] v flip, [3] h+v flip
 */
static const u32 imx363_mbus_codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const char *const imx363_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

/*
 * Link frequencies supported by the FP3 (4-lane MIPI CSI-2).
 * Values from vendor camera_config.xml, confirmed by PLL register analysis.
 */
enum {
	IMX363_LINK_FREQ_620MHZ,
	IMX363_LINK_FREQ_350MHZ,
	IMX363_LINK_FREQ_510MHZ,
};

static const s64 imx363_link_freq_menu[] = {
	620000000LL,
	350000000LL,
	510000000LL,
};

#define REGS(_list)                               \
	{                                         \
		.num_of_regs = ARRAY_SIZE(_list), \
		.regs = _list,                    \
	}

/*
 * Pixel rate calculation: link_freq * 2 (DDR) * num_lanes / bits_per_pixel
 * For 4 lanes, 10bpp: pixel_rate = link_freq * 2 * 4 / 10
 */
static u64 imx363_link_freq_to_pixel_rate(u64 link_freq)
{
	return div_u64(link_freq * 2 * 4, 10);
}

/* --- Common (global init) registers from vendor blob --- */
static const struct cci_reg_sequence imx363_common_regs[] = {
	{ CCI_REG8(0x0136), 0x18 }, /* EXCK_FREQ[15:8] = 24 MHz (BCD) */
	{ CCI_REG8(0x0137), 0x00 }, /* EXCK_FREQ[7:0] */
	{ CCI_REG8(0x31a3), 0x00 }, { CCI_REG8(0x64d4), 0x01 },
	{ CCI_REG8(0x64d5), 0xaa }, { CCI_REG8(0x64d6), 0x01 },
	{ CCI_REG8(0x64d7), 0xa9 }, { CCI_REG8(0x64d8), 0x01 },
	{ CCI_REG8(0x64d9), 0xa5 }, { CCI_REG8(0x64da), 0x01 },
	{ CCI_REG8(0x64db), 0xa1 }, { CCI_REG8(0x720a), 0x24 },
	{ CCI_REG8(0x720b), 0x89 }, { CCI_REG8(0x720c), 0x85 },
	{ CCI_REG8(0x720d), 0xa1 }, { CCI_REG8(0x720e), 0x6e },
	{ CCI_REG8(0x729c), 0x59 }, { CCI_REG8(0x817c), 0xff },
	{ CCI_REG8(0x817d), 0x80 }, { CCI_REG8(0x9348), 0x96 },
	{ CCI_REG8(0x934b), 0x8c }, { CCI_REG8(0x934c), 0x82 },
	{ CCI_REG8(0x9353), 0xaa }, { CCI_REG8(0x9354), 0xaa },
};

/* --- Mode register tables --- */

/* Mode 0: 4032x3024 @30fps - full resolution, no binning */
static const struct cci_reg_sequence imx363_mode_4032x3024_regs[] = {
	{ CCI_REG8(0x0112), 0x0a }, /* CSI_DT_FMT[15:8] = 10-bit RAW */
	{ CCI_REG8(0x0113), 0x0a }, /* CSI_DT_FMT[7:0] */
	{ CCI_REG8(0x0114), 0x03 }, /* CSI_LANE_MODE = 4 lanes */
	{ CCI_REG8(0x0220), 0x00 }, /* HDR mode */
	{ CCI_REG8(0x0221), 0x11 },
	{ CCI_REG8(0x0340), 0x0c }, /* FLL = 3116 */
	{ CCI_REG8(0x0341), 0x2c },
	{ CCI_REG8(0x0342), 0x14 }, /* LLP = 5304 */
	{ CCI_REG8(0x0343), 0xb8 },
	{ CCI_REG8(0x0381), 0x01 }, /* X_EVN_INC */
	{ CCI_REG8(0x0383), 0x01 }, /* X_ODD_INC */
	{ CCI_REG8(0x0385), 0x01 }, /* Y_EVN_INC */
	{ CCI_REG8(0x0387), 0x01 }, /* Y_ODD_INC */
	{ CCI_REG8(0x0900), 0x00 }, /* BINNING_MODE = off */
	{ CCI_REG8(0x0901), 0x11 }, /* BINNING_TYPE */
	{ CCI_REG8(0x30f4), 0x02 },
	{ CCI_REG8(0x30f5), 0xbc },
	{ CCI_REG8(0x30f6), 0x01 },
	{ CCI_REG8(0x30f7), 0xb8 },
	{ CCI_REG8(0x31a0), 0x02 },
	{ CCI_REG8(0x31a5), 0x00 },
	{ CCI_REG8(0x31a6), 0x00 },
	{ CCI_REG8(0x560f), 0xc8 },
	{ CCI_REG8(0x5856), 0x04 },
	{ CCI_REG8(0x58d0), 0x0e },
	{ CCI_REG8(0x734a), 0x23 },
	{ CCI_REG8(0x734f), 0x64 },
	{ CCI_REG8(0x7441), 0x5a },
	{ CCI_REG8(0x7914), 0x02 },
	{ CCI_REG8(0x7928), 0x08 },
	{ CCI_REG8(0x7929), 0x08 },
	{ CCI_REG8(0x793f), 0x02 },
	{ CCI_REG8(0xbc7b), 0x2c },
	{ CCI_REG8(0x0344), 0x00 }, /* X_ADD_STA = 0 */
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 }, /* Y_ADD_STA = 0 */
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x0f }, /* X_ADD_END = 4031 */
	{ CCI_REG8(0x0349), 0xbf },
	{ CCI_REG8(0x034a), 0x0b }, /* Y_ADD_END = 3023 */
	{ CCI_REG8(0x034b), 0xcf },
	{ CCI_REG8(0x034c), 0x0f }, /* X_OUT_SIZE = 4032 */
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x0b }, /* Y_OUT_SIZE = 3024 */
	{ CCI_REG8(0x034f), 0xd0 },
	{ CCI_REG8(0x0408), 0x00 }, /* DIG_CROP_X_OFFSET = 0 */
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 }, /* DIG_CROP_Y_OFFSET = 0 */
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x0f }, /* DIG_CROP_WIDTH = 4032 */
	{ CCI_REG8(0x040d), 0xc0 },
	{ CCI_REG8(0x040e), 0x0b }, /* DIG_CROP_HEIGHT = 3024 */
	{ CCI_REG8(0x040f), 0xd0 },
	{ CCI_REG8(0x0301), 0x03 }, /* VT_PIX_CLK_DIV */
	{ CCI_REG8(0x0303), 0x02 }, /* VT_SYS_CLK_DIV */
	{ CCI_REG8(0x0305), 0x04 }, /* PREPLLCK_VT_DIV */
	{ CCI_REG8(0x0306), 0x00 }, /* PLL_IVT_MPY = 124 */
	{ CCI_REG8(0x0307), 0x7c },
	{ CCI_REG8(0x0309), 0x0a }, /* OP_PIX_CLK_DIV = 10 */
	{ CCI_REG8(0x030b), 0x01 }, /* OP_SYS_CLK_DIV = 1 */
	{ CCI_REG8(0x030d), 0x04 }, /* PREPLLCK_OP_DIV */
	{ CCI_REG8(0x030e), 0x00 }, /* PLL_IOP_MPY = 174 */
	{ CCI_REG8(0x030f), 0xae },
	{ CCI_REG8(0x0310), 0x01 }, /* PLL_MULT_DRIV */
	{ CCI_REG8(0x0202), 0x0c }, /* COARSE_INTEG_TIME = 0x0c1e */
	{ CCI_REG8(0x0203), 0x1e },
	{ CCI_REG8(0x0224), 0x01 }, /* HDR short exposure */
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x0204), 0x00 }, /* ANA_GAIN = 0 */
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x0216), 0x00 }, /* HDR gain */
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x020e), 0x01 }, /* DIG_GAIN_GR = 0x0100 (1x) */
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0226), 0x01 }, /* HDR digital gain */
	{ CCI_REG8(0x0227), 0x00 },
};

/* Mode 1: 2016x1512 @30fps - 2x2 binning */
static const struct cci_reg_sequence imx363_mode_2016x1512_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0220), 0x00 },
	{ CCI_REG8(0x0221), 0x11 },
	{ CCI_REG8(0x0340), 0x0b }, /* FLL = 2968 */
	{ CCI_REG8(0x0341), 0x98 },
	{ CCI_REG8(0x0342), 0x0c }, /* LLP = 3144 */
	{ CCI_REG8(0x0343), 0x48 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0900), 0x01 }, /* BINNING_MODE = on */
	{ CCI_REG8(0x0901), 0x22 }, /* BINNING_TYPE = 2x2 */
	{ CCI_REG8(0x30f4), 0x02 },
	{ CCI_REG8(0x30f5), 0x58 },
	{ CCI_REG8(0x30f6), 0x00 },
	{ CCI_REG8(0x30f7), 0x14 },
	{ CCI_REG8(0x31a0), 0x00 },
	{ CCI_REG8(0x31a5), 0x00 },
	{ CCI_REG8(0x31a6), 0x00 },
	{ CCI_REG8(0x560f), 0xff },
	{ CCI_REG8(0x5856), 0x08 },
	{ CCI_REG8(0x58d0), 0x10 },
	{ CCI_REG8(0x734a), 0x01 },
	{ CCI_REG8(0x734f), 0x2b },
	{ CCI_REG8(0x7441), 0x55 },
	{ CCI_REG8(0x7914), 0x03 },
	{ CCI_REG8(0x7928), 0x04 },
	{ CCI_REG8(0x7929), 0x04 },
	{ CCI_REG8(0x793f), 0x03 },
	{ CCI_REG8(0xbc7b), 0x18 },
	{ CCI_REG8(0x0344), 0x00 }, /* X_ADD_STA = 0 */
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 }, /* Y_ADD_STA = 0 */
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x0f }, /* X_ADD_END = 4031 */
	{ CCI_REG8(0x0349), 0xbf },
	{ CCI_REG8(0x034a), 0x0b }, /* Y_ADD_END = 3023 */
	{ CCI_REG8(0x034b), 0xcf },
	{ CCI_REG8(0x034c), 0x07 }, /* X_OUT_SIZE = 2016 */
	{ CCI_REG8(0x034d), 0xe0 },
	{ CCI_REG8(0x034e), 0x05 }, /* Y_OUT_SIZE = 1512 */
	{ CCI_REG8(0x034f), 0xe8 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x07 }, /* DIG_CROP_WIDTH = 2016 */
	{ CCI_REG8(0x040d), 0xe0 },
	{ CCI_REG8(0x040e), 0x05 }, /* DIG_CROP_HEIGHT = 1512 */
	{ CCI_REG8(0x040f), 0xe8 },
	{ CCI_REG8(0x0301), 0x03 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x04 },
	{ CCI_REG8(0x0306), 0x00 }, /* PLL_IVT_MPY = 70 */
	{ CCI_REG8(0x0307), 0x46 },
	{ CCI_REG8(0x0309), 0x0a },
	{ CCI_REG8(0x030b), 0x02 }, /* OP_SYS_CLK_DIV = 2 */
	{ CCI_REG8(0x030d), 0x04 },
	{ CCI_REG8(0x030e), 0x00 }, /* PLL_IOP_MPY = 225 */
	{ CCI_REG8(0x030f), 0xe1 },
	{ CCI_REG8(0x0310), 0x01 },
	{ CCI_REG8(0x0202), 0x0b }, /* COARSE_INTEG_TIME = 0x0b88 */
	{ CCI_REG8(0x0203), 0x88 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0226), 0x01 },
	{ CCI_REG8(0x0227), 0x00 },
};

/* Mode 2: 1920x1080 @60fps - 2x2 binning, center crop */
static const struct cci_reg_sequence imx363_mode_1920x1080_60_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0220), 0x00 },
	{ CCI_REG8(0x0221), 0x11 },
	{ CCI_REG8(0x0340), 0x09 }, /* FLL = 2448 */
	{ CCI_REG8(0x0341), 0x90 },
	{ CCI_REG8(0x0342), 0x0a }, /* LLP = 2776 */
	{ CCI_REG8(0x0343), 0xd8 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0900), 0x01 }, /* BINNING_MODE = on */
	{ CCI_REG8(0x0901), 0x22 }, /* BINNING_TYPE = 2x2 */
	{ CCI_REG8(0x30f4), 0x01 },
	{ CCI_REG8(0x30f5), 0xcc },
	{ CCI_REG8(0x30f6), 0x01 },
	{ CCI_REG8(0x30f7), 0xea },
	{ CCI_REG8(0x31a0), 0x02 },
	{ CCI_REG8(0x31a5), 0x00 },
	{ CCI_REG8(0x31a6), 0x00 },
	{ CCI_REG8(0x560f), 0xc8 },
	{ CCI_REG8(0x5856), 0x04 },
	{ CCI_REG8(0x58d0), 0x0e },
	{ CCI_REG8(0x734a), 0x23 },
	{ CCI_REG8(0x734f), 0x64 },
	{ CCI_REG8(0x7441), 0x5a },
	{ CCI_REG8(0x7914), 0x02 },
	{ CCI_REG8(0x7928), 0x08 },
	{ CCI_REG8(0x7929), 0x08 },
	{ CCI_REG8(0x793f), 0x02 },
	{ CCI_REG8(0xbc7b), 0x2c },
	{ CCI_REG8(0x0344), 0x00 }, /* X_ADD_STA = 0 */
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x01 }, /* Y_ADD_STA = 376 */
	{ CCI_REG8(0x0347), 0x78 },
	{ CCI_REG8(0x0348), 0x0f }, /* X_ADD_END = 4031 */
	{ CCI_REG8(0x0349), 0xbf },
	{ CCI_REG8(0x034a), 0x0a }, /* Y_ADD_END = 2647 */
	{ CCI_REG8(0x034b), 0x57 },
	{ CCI_REG8(0x034c), 0x07 }, /* X_OUT_SIZE = 1920 */
	{ CCI_REG8(0x034d), 0x80 },
	{ CCI_REG8(0x034e), 0x04 }, /* Y_OUT_SIZE = 1080 */
	{ CCI_REG8(0x034f), 0x38 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x07 }, /* DIG_CROP_WIDTH = 2016 */
	{ CCI_REG8(0x040d), 0xe0 },
	{ CCI_REG8(0x040e), 0x04 }, /* DIG_CROP_HEIGHT = 1136 */
	{ CCI_REG8(0x040f), 0x70 },
	{ CCI_REG8(0x0301), 0x03 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x04 },
	{ CCI_REG8(0x0306), 0x00 }, /* PLL_IVT_MPY = 102 */
	{ CCI_REG8(0x0307), 0x66 },
	{ CCI_REG8(0x0309), 0x0a },
	{ CCI_REG8(0x030b), 0x02 }, /* OP_SYS_CLK_DIV = 2 */
	{ CCI_REG8(0x030d), 0x04 },
	{ CCI_REG8(0x030e), 0x01 }, /* PLL_IOP_MPY = 287 */
	{ CCI_REG8(0x030f), 0x1f },
	{ CCI_REG8(0x0310), 0x01 },
	{ CCI_REG8(0x0202), 0x09 }, /* COARSE_INTEG_TIME = 0x0982 */
	{ CCI_REG8(0x0203), 0x82 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0226), 0x01 },
	{ CCI_REG8(0x0227), 0x00 },
};

/* Mode 3: 1920x1080 @90fps - same as mode 2 but FLL=1632 */
static const struct cci_reg_sequence imx363_mode_1920x1080_90_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0220), 0x00 },
	{ CCI_REG8(0x0221), 0x11 },
	{ CCI_REG8(0x0340), 0x06 }, /* FLL = 1632 */
	{ CCI_REG8(0x0341), 0x60 },
	{ CCI_REG8(0x0342), 0x0a }, /* LLP = 2776 */
	{ CCI_REG8(0x0343), 0xd8 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x30f4), 0x01 },
	{ CCI_REG8(0x30f5), 0xcc },
	{ CCI_REG8(0x30f6), 0x01 },
	{ CCI_REG8(0x30f7), 0xea },
	{ CCI_REG8(0x31a0), 0x02 },
	{ CCI_REG8(0x31a5), 0x00 },
	{ CCI_REG8(0x31a6), 0x00 },
	{ CCI_REG8(0x560f), 0xc8 },
	{ CCI_REG8(0x5856), 0x04 },
	{ CCI_REG8(0x58d0), 0x0e },
	{ CCI_REG8(0x734a), 0x23 },
	{ CCI_REG8(0x734f), 0x64 },
	{ CCI_REG8(0x7441), 0x5a },
	{ CCI_REG8(0x7914), 0x02 },
	{ CCI_REG8(0x7928), 0x08 },
	{ CCI_REG8(0x7929), 0x08 },
	{ CCI_REG8(0x793f), 0x02 },
	{ CCI_REG8(0xbc7b), 0x2c },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x01 }, /* Y_ADD_STA = 376 */
	{ CCI_REG8(0x0347), 0x78 },
	{ CCI_REG8(0x0348), 0x0f },
	{ CCI_REG8(0x0349), 0xbf },
	{ CCI_REG8(0x034a), 0x0a }, /* Y_ADD_END = 2647 */
	{ CCI_REG8(0x034b), 0x57 },
	{ CCI_REG8(0x034c), 0x07 }, /* X_OUT = 1920 */
	{ CCI_REG8(0x034d), 0x80 },
	{ CCI_REG8(0x034e), 0x04 }, /* Y_OUT = 1080 */
	{ CCI_REG8(0x034f), 0x38 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x07 },
	{ CCI_REG8(0x040d), 0xe0 },
	{ CCI_REG8(0x040e), 0x04 },
	{ CCI_REG8(0x040f), 0x70 },
	{ CCI_REG8(0x0301), 0x03 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x04 },
	{ CCI_REG8(0x0306), 0x00 }, /* PLL_IVT_MPY = 102 */
	{ CCI_REG8(0x0307), 0x66 },
	{ CCI_REG8(0x0309), 0x0a },
	{ CCI_REG8(0x030b), 0x02 },
	{ CCI_REG8(0x030d), 0x04 },
	{ CCI_REG8(0x030e), 0x01 }, /* PLL_IOP_MPY = 287 */
	{ CCI_REG8(0x030f), 0x1f },
	{ CCI_REG8(0x0310), 0x01 },
	{ CCI_REG8(0x0202), 0x06 }, /* COARSE_INTEG_TIME = 0x0652 */
	{ CCI_REG8(0x0203), 0x52 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0226), 0x01 },
	{ CCI_REG8(0x0227), 0x00 },
};

/* Mode 4: 1920x1080 @120fps - same as mode 2 but FLL=1224 */
static const struct cci_reg_sequence imx363_mode_1920x1080_120_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0220), 0x00 },
	{ CCI_REG8(0x0221), 0x11 },
	{ CCI_REG8(0x0340), 0x04 }, /* FLL = 1224 */
	{ CCI_REG8(0x0341), 0xc8 },
	{ CCI_REG8(0x0342), 0x0a }, /* LLP = 2776 */
	{ CCI_REG8(0x0343), 0xd8 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x30f4), 0x01 },
	{ CCI_REG8(0x30f5), 0xcc },
	{ CCI_REG8(0x30f6), 0x01 },
	{ CCI_REG8(0x30f7), 0xea },
	{ CCI_REG8(0x31a0), 0x02 },
	{ CCI_REG8(0x31a5), 0x00 },
	{ CCI_REG8(0x31a6), 0x00 },
	{ CCI_REG8(0x560f), 0xc8 },
	{ CCI_REG8(0x5856), 0x04 },
	{ CCI_REG8(0x58d0), 0x0e },
	{ CCI_REG8(0x734a), 0x23 },
	{ CCI_REG8(0x734f), 0x64 },
	{ CCI_REG8(0x7441), 0x5a },
	{ CCI_REG8(0x7914), 0x02 },
	{ CCI_REG8(0x7928), 0x08 },
	{ CCI_REG8(0x7929), 0x08 },
	{ CCI_REG8(0x793f), 0x02 },
	{ CCI_REG8(0xbc7b), 0x2c },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x01 }, /* Y_ADD_STA = 376 */
	{ CCI_REG8(0x0347), 0x78 },
	{ CCI_REG8(0x0348), 0x0f },
	{ CCI_REG8(0x0349), 0xbf },
	{ CCI_REG8(0x034a), 0x0a }, /* Y_ADD_END = 2647 */
	{ CCI_REG8(0x034b), 0x57 },
	{ CCI_REG8(0x034c), 0x07 }, /* X_OUT = 1920 */
	{ CCI_REG8(0x034d), 0x80 },
	{ CCI_REG8(0x034e), 0x04 }, /* Y_OUT = 1080 */
	{ CCI_REG8(0x034f), 0x38 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x07 },
	{ CCI_REG8(0x040d), 0xe0 },
	{ CCI_REG8(0x040e), 0x04 },
	{ CCI_REG8(0x040f), 0x70 },
	{ CCI_REG8(0x0301), 0x03 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x04 },
	{ CCI_REG8(0x0306), 0x00 }, /* PLL_IVT_MPY = 102 */
	{ CCI_REG8(0x0307), 0x66 },
	{ CCI_REG8(0x0309), 0x0a },
	{ CCI_REG8(0x030b), 0x02 },
	{ CCI_REG8(0x030d), 0x04 },
	{ CCI_REG8(0x030e), 0x01 }, /* PLL_IOP_MPY = 287 */
	{ CCI_REG8(0x030f), 0x1f },
	{ CCI_REG8(0x0310), 0x01 },
	{ CCI_REG8(0x0202), 0x04 }, /* COARSE_INTEG_TIME = 0x04ba */
	{ CCI_REG8(0x0203), 0xba },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0226), 0x01 },
	{ CCI_REG8(0x0227), 0x00 },
};

/* --- Mode definitions --- */
static const struct imx363_mode imx363_supported_modes[] = {
	{
		/* Mode 0: 4032x3024 @30fps - full resolution */
		.width = 4032,
		.height = 3024,
		.vts_def = 3116,
		.vts_min = 3116,
		.llp = 5304,
		.link_freq_index = IMX363_LINK_FREQ_620MHZ,
		.reg_list = REGS(imx363_mode_4032x3024_regs),
		.crop = {
			.left = IMX363_PIXEL_ARRAY_LEFT,
			.top = IMX363_PIXEL_ARRAY_TOP,
			.width = 4032,
			.height = 3024,
		},
	},
	{
		/* Mode 1: 2016x1512 @30fps - 2x2 binning */
		.width = 2016,
		.height = 1512,
		.vts_def = 2968,
		.vts_min = 2968,
		.llp = 3144,
		.link_freq_index = IMX363_LINK_FREQ_350MHZ,
		.reg_list = REGS(imx363_mode_2016x1512_regs),
		.crop = {
			.left = IMX363_PIXEL_ARRAY_LEFT,
			.top = IMX363_PIXEL_ARRAY_TOP,
			.width = 4032,
			.height = 3024,
		},
	},
	{
		/* Mode 2: 1920x1080 @60fps - 2x2 binning + crop */
		.width = 1920,
		.height = 1080,
		.vts_def = 2448,
		.vts_min = 2448,
		.llp = 2776,
		.link_freq_index = IMX363_LINK_FREQ_510MHZ,
		.reg_list = REGS(imx363_mode_1920x1080_60_regs),
		.crop = {
			.left = IMX363_PIXEL_ARRAY_LEFT,
			.top = IMX363_PIXEL_ARRAY_TOP + 376,
			.width = 4032,
			.height = 2272,
		},
	},
	{
		/* Mode 3: 1920x1080 @90fps */
		.width = 1920,
		.height = 1080,
		.vts_def = 1632,
		.vts_min = 1632,
		.llp = 2776,
		.link_freq_index = IMX363_LINK_FREQ_510MHZ,
		.reg_list = REGS(imx363_mode_1920x1080_90_regs),
		.crop = {
			.left = IMX363_PIXEL_ARRAY_LEFT,
			.top = IMX363_PIXEL_ARRAY_TOP + 376,
			.width = 4032,
			.height = 2272,
		},
	},
	{
		/* Mode 4: 1920x1080 @120fps */
		.width = 1920,
		.height = 1080,
		.vts_def = 1224,
		.vts_min = 1224,
		.llp = 2776,
		.link_freq_index = IMX363_LINK_FREQ_510MHZ,
		.reg_list = REGS(imx363_mode_1920x1080_120_regs),
		.crop = {
			.left = IMX363_PIXEL_ARRAY_LEFT,
			.top = IMX363_PIXEL_ARRAY_TOP + 376,
			.width = 4032,
			.height = 2272,
		},
	},
};

struct imx363 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	const struct imx363_mode *cur_mode;

	unsigned long link_freq_bitmap;
	unsigned int csi2_flags;

	struct gpio_desc *reset_gpio;
	struct mutex mutex;
	struct clk *clk;
	struct regulator_bulk_data supplies[IMX363_NUM_SUPPLIES];
};

static inline struct imx363 *to_imx363(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx363, sd);
}

/* --- V4L2 helper functions --- */

static u32 imx363_get_format_code(const struct imx363 *imx363)
{
	unsigned int i;

	lockdep_assert_held(&imx363->mutex);

	i = (imx363->vflip->val ? 2 : 0) | (imx363->hflip->val ? 1 : 0);

	return imx363_mbus_codes[i];
}

static int imx363_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx363 *imx363 = to_imx363(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(fh->state, 0);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx363->mutex);

	try_fmt->width = imx363_supported_modes[0].width;
	try_fmt->height = imx363_supported_modes[0].height;
	try_fmt->code = imx363_get_format_code(imx363);
	try_fmt->field = V4L2_FIELD_NONE;

	try_crop = v4l2_subdev_state_get_crop(fh->state, 0);
	try_crop->left = IMX363_PIXEL_ARRAY_LEFT;
	try_crop->top = IMX363_PIXEL_ARRAY_TOP;
	try_crop->width = IMX363_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX363_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx363->mutex);

	return 0;
}

/* --- Digital gain helper --- */
static int imx363_update_digital_gain(struct imx363 *imx363, u32 val)
{
	int ret = 0;

	cci_write(imx363->regmap, IMX363_REG_GR_DIGITAL_GAIN, val, &ret);
	cci_write(imx363->regmap, IMX363_REG_GB_DIGITAL_GAIN, val, &ret);
	cci_write(imx363->regmap, IMX363_REG_R_DIGITAL_GAIN, val, &ret);
	cci_write(imx363->regmap, IMX363_REG_B_DIGITAL_GAIN, val, &ret);

	return ret;
}

/* --- Exposure range adjustment --- */
static void imx363_adjust_exposure_range(struct imx363 *imx363)
{
	int exposure_max, exposure_def;

	exposure_max = imx363->cur_mode->height + imx363->vblank->val -
		       IMX363_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, imx363->exposure->val);
	__v4l2_ctrl_modify_range(imx363->exposure, imx363->exposure->minimum,
				 exposure_max, imx363->exposure->step,
				 exposure_def);
}

/* --- V4L2 control operations --- */
static int imx363_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx363 *imx363 =
		container_of(ctrl->handler, struct imx363, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx363->sd);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK)
		imx363_adjust_exposure_range(imx363);

	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(imx363->regmap, IMX363_REG_ANALOG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(imx363->regmap, IMX363_REG_EXPOSURE, ctrl->val,
				NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx363_update_digital_gain(imx363, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(imx363->regmap, IMX363_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		if (!ctrl->val) {
			ret = cci_write(imx363->regmap, IMX363_REG_HDR,
					IMX363_HDR_RATIO_MIN, NULL);
		} else {
			ret = cci_write(imx363->regmap, IMX363_REG_HDR,
					IMX363_HDR_ON, NULL);
			if (ret)
				break;
			ret = cci_write(imx363->regmap, IMX363_REG_HDR_RATIO,
					BIT(IMX363_HDR_RATIO_MAX), NULL);
		}
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(imx363->regmap, IMX363_REG_FRM_LENGTH_LINES,
				imx363->cur_mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		ret = cci_write(
			imx363->regmap, IMX363_REG_ORIENTATION,
			(imx363->hflip->val ? IMX363_ORIENT_HFLIP : 0) |
				(imx363->vflip->val ? IMX363_ORIENT_VFLIP : 0),
			NULL);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n", ctrl->id,
			 ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx363_ctrl_ops = {
	.s_ctrl = imx363_set_ctrl,
};

/* --- Pad operations --- */
static int imx363_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx363 *imx363 = to_imx363(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = imx363_get_format_code(imx363);

	return 0;
}

static int imx363_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx363 *imx363 = to_imx363(sd);

	if (fse->index >= ARRAY_SIZE(imx363_supported_modes))
		return -EINVAL;

	if (fse->code != imx363_get_format_code(imx363))
		return -EINVAL;

	fse->min_width = imx363_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx363_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx363_update_pad_format(struct imx363 *imx363,
				     const struct imx363_mode *mode,
				     struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = imx363_get_format_code(imx363);
	fmt->format.field = V4L2_FIELD_NONE;
}

static int __imx363_get_pad_format(struct imx363 *imx363,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state, fmt->pad);
	else
		imx363_update_pad_format(imx363, imx363->cur_mode, fmt);

	return 0;
}

static int imx363_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx363 *imx363 = to_imx363(sd);
	int ret;

	mutex_lock(&imx363->mutex);
	ret = __imx363_get_pad_format(imx363, sd_state, fmt);
	mutex_unlock(&imx363->mutex);

	return ret;
}

static int imx363_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx363 *imx363 = to_imx363(sd);
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx363_mode *mode;
	s32 vblank_def;
	s32 vblank_min;
	s64 h_blank;
	s64 pixel_rate;
	s64 link_freq;

	mutex_lock(&imx363->mutex);

	fmt->format.code = imx363_get_format_code(imx363);

	mode = v4l2_find_nearest_size(imx363_supported_modes,
				      ARRAY_SIZE(imx363_supported_modes), width,
				      height, fmt->format.width,
				      fmt->format.height);
	imx363_update_pad_format(imx363, mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		imx363->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(imx363->link_freq, mode->link_freq_index);

		link_freq = imx363_link_freq_menu[mode->link_freq_index];
		pixel_rate = imx363_link_freq_to_pixel_rate(link_freq);
		__v4l2_ctrl_modify_range(imx363->pixel_rate, pixel_rate,
					 pixel_rate, 1, pixel_rate);
		/* Update limits and set FPS to default */
		vblank_def = mode->vts_def - mode->height;
		vblank_min = mode->vts_min - mode->height;
		__v4l2_ctrl_modify_range(imx363->vblank, vblank_min,
					 IMX363_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(imx363->vblank, vblank_def);
		h_blank = mode->llp - mode->width;
		__v4l2_ctrl_modify_range(imx363->hblank, h_blank, h_blank, 1,
					 h_blank);
	}

	mutex_unlock(&imx363->mutex);

	return 0;
}

static const struct v4l2_rect *
__imx363_get_pad_crop(struct imx363 *imx363, struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx363->cur_mode->crop;
	}

	return NULL;
}

static int imx363_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx363 *imx363 = to_imx363(sd);

		mutex_lock(&imx363->mutex);
		sel->r = *__imx363_get_pad_crop(imx363, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx363->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX363_NATIVE_WIDTH;
		sel->r.height = IMX363_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX363_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX363_PIXEL_ARRAY_TOP;
		sel->r.width = IMX363_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX363_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/* --- Streaming --- */
static int imx363_start_streaming(struct imx363 *imx363)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx363->sd);
	const struct imx363_reg_list *reg_list;
	int ret;

	/* Software reset */
	ret = cci_write(imx363->regmap, IMX363_REG_RESET, 0x01, NULL);
	if (ret) {
		dev_err(&client->dev, "%s failed to reset sensor\n", __func__);
		return ret;
	}

	/* 12ms required from poweron to standby */
	fsleep(12000);

	/* Apply common (global init) registers */
	ret = cci_multi_reg_write(imx363->regmap, imx363_common_regs,
				  ARRAY_SIZE(imx363_common_regs), NULL);
	if (ret) {
		dev_err(&client->dev, "%s failed to set common regs\n",
			__func__);
		return ret;
	}

	/* Set clock lane mode */
	ret = cci_write(imx363->regmap, IMX363_CLK_BLANK_STOP,
			!!(imx363->csi2_flags &
			   V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK),
			NULL);
	if (ret) {
		dev_err(&client->dev, "%s failed to set clock lane mode\n",
			__func__);
		return ret;
	}

	/* Apply mode registers (includes PLL settings) */
	reg_list = &imx363->cur_mode->reg_list;
	ret = cci_multi_reg_write(imx363->regmap, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(imx363->sd.ctrl_handler);
	if (ret)
		return ret;

	/* Start streaming */
	return cci_write(imx363->regmap, IMX363_REG_MODE_SELECT,
			 IMX363_MODE_STREAMING, NULL);
}

static int imx363_stop_streaming(struct imx363 *imx363)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx363->sd);
	int ret;

	ret = cci_write(imx363->regmap, IMX363_REG_MODE_SELECT,
			IMX363_MODE_STANDBY, NULL);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	return 0;
}

static int imx363_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx363 *imx363 = to_imx363(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx363->mutex);

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto err_unlock;

		ret = imx363_start_streaming(imx363);
		if (ret)
			goto err_rpm_put;
	} else {
		imx363_stop_streaming(imx363);
		pm_runtime_put(&client->dev);
	}

	mutex_unlock(&imx363->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx363->mutex);

	return ret;
}

/* --- Power management --- */
static int imx363_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx363 *imx363 = to_imx363(sd);
	int ret;

	ret = regulator_bulk_enable(IMX363_NUM_SUPPLIES, imx363->supplies);
	if (ret) {
		dev_err(dev, "%s: failed to enable regulators\n", __func__);
		return ret;
	}

	/* Let regulators stabilize before starting clock */
	fsleep(1000);

	ret = clk_prepare_enable(imx363->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		regulator_bulk_disable(IMX363_NUM_SUPPLIES, imx363->supplies);
		return ret;
	}

	/* Deassert reset after clock is running */
	gpiod_set_value_cansleep(imx363->reset_gpio, 0);

	/* 12ms required from power-on to standby per datasheet */
	fsleep(12000);

	return 0;
}

static int imx363_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx363 *imx363 = to_imx363(sd);

	/* Assert reset before stopping clock */
	gpiod_set_value_cansleep(imx363->reset_gpio, 1);

	clk_disable_unprepare(imx363->clk);

	regulator_bulk_disable(IMX363_NUM_SUPPLIES, imx363->supplies);

	return 0;
}

/* --- Chip identification --- */
static int imx363_identify_module(struct imx363 *imx363)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx363->sd);
	int ret;
	u64 val;

	ret = cci_read(imx363->regmap, IMX363_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x\n",
			IMX363_CHIP_ID);
		return ret;
	}

	if (val != IMX363_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%llx\n",
			IMX363_CHIP_ID, val);
		return -EIO;
	}

	return 0;
}

/* --- V4L2 subdev ops --- */
static const struct v4l2_subdev_video_ops imx363_video_ops = {
	.s_stream = imx363_set_stream,
};

static const struct v4l2_subdev_pad_ops imx363_pad_ops = {
	.enum_mbus_code = imx363_enum_mbus_code,
	.get_fmt = imx363_get_pad_format,
	.set_fmt = imx363_set_pad_format,
	.enum_frame_size = imx363_enum_frame_size,
	.get_selection = imx363_get_selection,
};

static const struct v4l2_subdev_ops imx363_subdev_ops = {
	.video = &imx363_video_ops,
	.pad = &imx363_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx363_internal_ops = {
	.open = imx363_open,
};

/* --- Controls init --- */
static int imx363_init_controls(struct imx363 *imx363)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx363->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 vblank_def;
	s64 vblank_min;
	s64 pixel_rate;
	s64 h_blank;
	int ret;

	ctrl_hdlr = &imx363->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 13);
	if (ret)
		return ret;

	mutex_init(&imx363->mutex);
	ctrl_hdlr->lock = &imx363->mutex;

	imx363->link_freq = v4l2_ctrl_new_int_menu(
		ctrl_hdlr, &imx363_ctrl_ops, V4L2_CID_LINK_FREQ,
		ARRAY_SIZE(imx363_link_freq_menu) - 1, 0,
		imx363_link_freq_menu);
	if (imx363->link_freq)
		imx363->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx363->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx363->hflip)
		imx363->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx363->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx363->vflip)
		imx363->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	pixel_rate = imx363_link_freq_to_pixel_rate(
		imx363_link_freq_menu[imx363->cur_mode->link_freq_index]);

	imx363->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, pixel_rate,
					       pixel_rate, 1, pixel_rate);

	vblank_def = imx363->cur_mode->vts_def - imx363->cur_mode->height;
	vblank_min = imx363->cur_mode->vts_min - imx363->cur_mode->height;
	imx363->vblank = v4l2_ctrl_new_std(
		ctrl_hdlr, &imx363_ctrl_ops, V4L2_CID_VBLANK, vblank_min,
		IMX363_VTS_MAX - imx363->cur_mode->height, 1, vblank_def);

	/* hblank is read-only, derived from LLP in mode struct */
	h_blank = imx363->cur_mode->llp - imx363->cur_mode->width;
	imx363->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
					   V4L2_CID_HBLANK, h_blank, h_blank, 1,
					   h_blank);
	if (imx363->hblank)
		imx363->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx363->exposure = v4l2_ctrl_new_std(
		ctrl_hdlr, &imx363_ctrl_ops, V4L2_CID_EXPOSURE,
		IMX363_EXPOSURE_MIN, IMX363_VTS_MAX - IMX363_EXPOSURE_OFFSET,
		IMX363_EXPOSURE_STEP, IMX363_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX363_ANA_GAIN_MIN, IMX363_ANA_GAIN_MAX,
			  IMX363_ANA_GAIN_STEP, IMX363_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX363_DGTL_GAIN_MIN, IMX363_DGTL_GAIN_MAX,
			  IMX363_DGTL_GAIN_STEP, IMX363_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
			  V4L2_CID_WIDE_DYNAMIC_RANGE, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx363_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx363_test_pattern_menu) - 1,
				     0, 0, imx363_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n", __func__,
			ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx363_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx363->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx363->mutex);

	return ret;
}

static void imx363_free_controls(struct imx363 *imx363)
{
	v4l2_ctrl_handler_free(imx363->sd.ctrl_handler);
	mutex_destroy(&imx363->mutex);
}

/* --- Regulators --- */
static int imx363_get_regulators(struct imx363 *imx363,
				 struct i2c_client *client)
{
	unsigned int i;

	for (i = 0; i < IMX363_NUM_SUPPLIES; i++)
		imx363->supplies[i].supply = imx363_supply_name[i];

	return devm_regulator_bulk_get(&client->dev, IMX363_NUM_SUPPLIES,
				       imx363->supplies);
}

/* --- Probe / Remove --- */
static int imx363_probe(struct i2c_client *client)
{
	struct imx363 *imx363;
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	int ret;
	u32 val = 0;

	imx363 = devm_kzalloc(&client->dev, sizeof(*imx363), GFP_KERNEL);
	if (!imx363)
		return -ENOMEM;

	imx363->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx363->regmap)) {
		ret = PTR_ERR(imx363->regmap);
		dev_err(&client->dev, "failed to initialize CCI: %d\n", ret);
		return ret;
	}

	ret = imx363_get_regulators(imx363, client);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to get regulators\n");

	/* Reset GPIO (active low) */
	imx363->reset_gpio =
		devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx363->reset_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(imx363->reset_gpio),
				     "failed to get reset GPIO\n");

	imx363->clk = devm_clk_get_optional(&client->dev, NULL);
	if (IS_ERR(imx363->clk))
		return dev_err_probe(&client->dev, PTR_ERR(imx363->clk),
				     "error getting clock\n");
	if (!imx363->clk) {
		dev_dbg(&client->dev,
			"no clock provided, using clock-frequency property\n");
		device_property_read_u32(&client->dev, "clock-frequency", &val);
	} else {
		val = clk_get_rate(imx363->clk);
	}

	if (val != 24000000) {
		dev_err(&client->dev,
			"input clock frequency of %u not supported (expected 24 MHz)\n",
			val);
		return -EINVAL;
	}

	endpoint =
		fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
	if (!endpoint) {
		dev_err(&client->dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(&client->dev, "Parsing endpoint node failed\n");
		return ret;
	}

	ret = v4l2_link_freq_to_bitmap(&client->dev, ep.link_frequencies,
				       ep.nr_of_link_frequencies,
				       imx363_link_freq_menu,
				       ARRAY_SIZE(imx363_link_freq_menu),
				       &imx363->link_freq_bitmap);
	if (ret) {
		dev_err(&client->dev, "Link frequency not supported\n");
		goto error_endpoint_free;
	}

	if (ep.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(&client->dev, "Only 4 data lanes supported, got %u\n",
			ep.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto error_endpoint_free;
	}

	imx363->csi2_flags = ep.bus.mipi_csi2.flags;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&imx363->sd, client, &imx363_subdev_ops);

	/* Will be powered off via pm_runtime_idle */
	ret = imx363_power_on(&client->dev);
	if (ret)
		goto error_endpoint_free;

	/* Check module identity */
	ret = imx363_identify_module(imx363);
	if (ret)
		goto error_identify;

	/* Set default mode to max resolution */
	imx363->cur_mode = &imx363_supported_modes[0];

	ret = imx363_init_controls(imx363);
	if (ret)
		goto error_identify;

	/* Initialize subdev */
	imx363->sd.internal_ops = &imx363_internal_ops;
	imx363->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx363->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	imx363->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx363->sd.entity, 1, &imx363->pad);
	if (ret)
		goto error_handler_free;

	ret = v4l2_async_register_subdev_sensor(&imx363->sd);
	if (ret < 0)
		goto error_media_entity;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);
	v4l2_fwnode_endpoint_free(&ep);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx363->sd.entity);

error_handler_free:
	imx363_free_controls(imx363);

error_identify:
	imx363_power_off(&client->dev);

error_endpoint_free:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

static void imx363_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx363 *imx363 = to_imx363(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx363_free_controls(imx363);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx363_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

/* --- Module boilerplate --- */
static const struct dev_pm_ops imx363_pm_ops = { SET_RUNTIME_PM_OPS(
	imx363_power_off, imx363_power_on, NULL) };

static const struct of_device_id imx363_dt_ids[] = { { .compatible =
							       "sony,imx363" },
						     { /* sentinel */ } };
MODULE_DEVICE_TABLE(of, imx363_dt_ids);

static struct i2c_driver imx363_i2c_driver = {
	.driver = {
		.name = "imx363",
		.pm = &imx363_pm_ops,
		.of_match_table = imx363_dt_ids,
	},
	.probe = imx363_probe,
	.remove = imx363_remove,
};

module_i2c_driver(imx363_i2c_driver);

MODULE_AUTHOR("Marc Lainez <marc.lainez@gmail.com>");
MODULE_DESCRIPTION("Sony IMX363 sensor driver");
MODULE_LICENSE("GPL");
