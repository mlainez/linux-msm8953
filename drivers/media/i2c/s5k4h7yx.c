// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K4H7YX sensor driver
 *
 * Copyright (C) 2024 Marc Lainez <marc.lainez@gmail.com>
 *
 * Based on the mainline IMX258/IMX363 drivers.
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
#define S5K4H7YX_REG_MODE_SELECT	CCI_REG8(0x0100)
#define S5K4H7YX_MODE_STANDBY		0x00
#define S5K4H7YX_MODE_STREAMING		0x01

/* Chip ID */
#define S5K4H7YX_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K4H7YX_CHIP_ID		0x487B

/* Group hold */
#define S5K4H7YX_REG_GROUP_HOLD		CCI_REG8(0x0104)

/* Frame timing */
#define S5K4H7YX_REG_FRM_LENGTH_LINES	CCI_REG16(0x0340)
#define S5K4H7YX_REG_LINE_LENGTH_PCK	CCI_REG16(0x0342)

/* Exposure control */
#define S5K4H7YX_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K4H7YX_EXPOSURE_MIN		4
#define S5K4H7YX_EXPOSURE_STEP		1
#define S5K4H7YX_EXPOSURE_DEFAULT	0x0208
#define S5K4H7YX_EXPOSURE_OFFSET	8

/* Analog gain control */
#define S5K4H7YX_REG_ANALOG_GAIN	CCI_REG16(0x0204)
#define S5K4H7YX_ANA_GAIN_MIN		32
#define S5K4H7YX_ANA_GAIN_MAX		512
#define S5K4H7YX_ANA_GAIN_STEP		1
#define S5K4H7YX_ANA_GAIN_DEFAULT	32

/* Digital gain control */
#define S5K4H7YX_REG_GR_DIGITAL_GAIN	CCI_REG16(0x020e)
#define S5K4H7YX_REG_R_DIGITAL_GAIN	CCI_REG16(0x0210)
#define S5K4H7YX_REG_B_DIGITAL_GAIN	CCI_REG16(0x0212)
#define S5K4H7YX_REG_GB_DIGITAL_GAIN	CCI_REG16(0x0214)
#define S5K4H7YX_DGTL_GAIN_MIN		0x0100
#define S5K4H7YX_DGTL_GAIN_MAX		0x1000
#define S5K4H7YX_DGTL_GAIN_DEFAULT	0x0100
#define S5K4H7YX_DGTL_GAIN_STEP	1

/* Test pattern */
#define S5K4H7YX_REG_TEST_PATTERN	CCI_REG16(0x0600)

/* Orientation */
#define S5K4H7YX_REG_ORIENTATION	CCI_REG8(0x0101)
#define S5K4H7YX_ORIENT_HFLIP		BIT(0)
#define S5K4H7YX_ORIENT_VFLIP		BIT(1)

/* Pixel array */
#define S5K4H7YX_NATIVE_WIDTH		3280U
#define S5K4H7YX_NATIVE_HEIGHT		2464U
#define S5K4H7YX_PIXEL_ARRAY_LEFT	8U
#define S5K4H7YX_PIXEL_ARRAY_TOP	8U
#define S5K4H7YX_PIXEL_ARRAY_WIDTH	3264U
#define S5K4H7YX_PIXEL_ARRAY_HEIGHT	2448U

#define S5K4H7YX_VTS_MAX		65535

/* Regulator supplies */
static const char * const s5k4h7yx_supply_name[] = {
	"vana",		/* Analog (2.8V) supply */
	"vdig",		/* Digital Core (1.2V) supply */
	"vif",		/* IF (1.8V) supply */
};

#define S5K4H7YX_NUM_SUPPLIES ARRAY_SIZE(s5k4h7yx_supply_name)

struct s5k4h7yx_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct s5k4h7yx_mode {
	u32 width;
	u32 height;
	u32 vts_def;
	u32 vts_min;
	u32 llp;
	u32 link_freq_index;
	struct s5k4h7yx_reg_list reg_list;
	struct v4l2_rect crop;
};

/*
 * Bayer order table for flip combinations:
 *  [0] no flip, [1] h flip, [2] v flip, [3] h+v flip
 * S5K4H7YX native order is GRBG.
 */
static const u32 s5k4h7yx_mbus_codes[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

static const char * const s5k4h7yx_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Colour Bars",
	"Colour Bars With Fade to Grey",
	"PN9",
};

/* Single link frequency: 350 MHz */
static const s64 s5k4h7yx_link_freq_menu[] = {
	350000000LL,
};

#define REGS(_list) { .num_of_regs = ARRAY_SIZE(_list), .regs = _list, }

static u64 s5k4h7yx_link_freq_to_pixel_rate(u64 link_freq)
{
	/* link_freq * 2 (DDR) * 4 lanes / 10 bpp */
	return div_u64(link_freq * 2 * 4, 10);
}

/* --- Common registers (PLL, CSI, MIPI config — same across all modes) --- */
static const struct cci_reg_sequence s5k4h7yx_common_regs[] = {
	/* PLL settings */
	{ CCI_REG8(0x0301), 0x04 },	/* VT_PIX_CLK_DIV */
	{ CCI_REG8(0x0303), 0x01 },	/* VT_SYS_CLK_DIV */
	{ CCI_REG8(0x0305), 0x06 },	/* PREPLLCK_VT_DIV */
	{ CCI_REG8(0x0306), 0x00 },	/* PLL_IVT_MPY[15:8] */
	{ CCI_REG8(0x0307), 0x8C },	/* PLL_IVT_MPY[7:0] = 140 */
	{ CCI_REG8(0x0309), 0x0A },	/* OP_PIX_CLK_DIV = 10 */
	{ CCI_REG8(0x030B), 0x01 },	/* OP_SYS_CLK_DIV */
	{ CCI_REG8(0x030D), 0x06 },	/* PREPLLCK_OP_DIV */
	{ CCI_REG8(0x030F), 0xAF },	/* PLL_IOP_MPY = 175 */

	/* CSI data format: 10-bit RAW */
	{ CCI_REG8(0x0112), 0x0A },	/* CSI_DT_FMT[15:8] */
	{ CCI_REG8(0x0113), 0x0A },	/* CSI_DT_FMT[7:0] */
	{ CCI_REG8(0x0114), 0x03 },	/* CSI_LANE_MODE = 4 lanes */

	/* MIPI data rate: 700 Mbps */
	{ CCI_REG8(0x0820), 0x02 },	/* REQ_LINK_BIT_RATE[31:24] */
	{ CCI_REG8(0x0821), 0xBC },	/* REQ_LINK_BIT_RATE[23:16] = 0x02BC = 700 */

	/* External clock frequency: 24 MHz */
	{ CCI_REG8(0x0136), 0x18 },	/* EXCK_FREQ[15:8] */
	{ CCI_REG8(0x0137), 0x00 },	/* EXCK_FREQ[7:0] */

	/* Fine integration time */
	{ CCI_REG8(0x0200), 0x0D },
	{ CCI_REG8(0x0201), 0xD8 },

	/* Default coarse integration time */
	{ CCI_REG8(0x0202), 0x02 },
	{ CCI_REG8(0x0203), 0x08 },

	/* Vendor-specific registers */
	{ CCI_REG8(0x3C1F), 0x00 },
	{ CCI_REG8(0x3C17), 0x00 },
	{ CCI_REG8(0x3C1C), 0x05 },
	{ CCI_REG8(0x3C1D), 0x15 },
	{ CCI_REG8(0x0404), 0x00 },
	{ CCI_REG8(0x0405), 0x10 },
	{ CCI_REG8(0x0B06), 0x01 },
	{ CCI_REG8(0x3230), 0x49 },
	{ CCI_REG8(0x3C34), 0xEF },
	{ CCI_REG8(0x3C35), 0x0F },
	{ CCI_REG8(0x3C36), 0x7F },
};

/* --- Mode definitions placeholder --- */

/* Mode 0: 3264x2448 @30fps — full resolution, no binning */
static const struct cci_reg_sequence s5k4h7yx_mode_3264x2448_regs[] = {
	{ CCI_REG8(0x0344), 0x00 },	/* X_ADD_STA[15:8] = 8 */
	{ CCI_REG8(0x0345), 0x08 },	/* X_ADD_STA[7:0] */
	{ CCI_REG8(0x0346), 0x00 },	/* Y_ADD_STA[15:8] = 8 */
	{ CCI_REG8(0x0347), 0x08 },	/* Y_ADD_STA[7:0] */
	{ CCI_REG8(0x0348), 0x0C },	/* X_ADD_END[15:8] = 3271 */
	{ CCI_REG8(0x0349), 0xC7 },	/* X_ADD_END[7:0] */
	{ CCI_REG8(0x034A), 0x09 },	/* Y_ADD_END[15:8] = 2455 */
	{ CCI_REG8(0x034B), 0x97 },	/* Y_ADD_END[7:0] */
	{ CCI_REG8(0x034C), 0x0C },	/* X_OUT_SIZE[15:8] = 3264 */
	{ CCI_REG8(0x034D), 0xC0 },	/* X_OUT_SIZE[7:0] */
	{ CCI_REG8(0x034E), 0x09 },	/* Y_OUT_SIZE[15:8] = 2448 */
	{ CCI_REG8(0x034F), 0x90 },	/* Y_OUT_SIZE[7:0] */
	{ CCI_REG8(0x0340), 0x09 },	/* FLL[15:8] = 2530 */
	{ CCI_REG8(0x0341), 0xE2 },	/* FLL[7:0] */
	{ CCI_REG8(0x0900), 0x00 },	/* BINNING_MODE = off */
	{ CCI_REG8(0x0901), 0x11 },	/* BINNING_TYPE = 1x1 */
	{ CCI_REG8(0x0387), 0x01 },	/* Y_ODD_INC */
	{ CCI_REG8(0x3906), 0x7E },	/* Vendor */
};

/* Mode 2: 1440x1080 @60fps — 2x2 binning, center crop */
static const struct cci_reg_sequence s5k4h7yx_mode_1440x1080_regs[] = {
	{ CCI_REG8(0x0344), 0x00 },	/* X_ADD_STA = 200 */
	{ CCI_REG8(0x0345), 0xC8 },
	{ CCI_REG8(0x0346), 0x00 },	/* Y_ADD_STA = 152 */
	{ CCI_REG8(0x0347), 0x98 },
	{ CCI_REG8(0x0348), 0x0C },	/* X_ADD_END = 3079 */
	{ CCI_REG8(0x0349), 0x07 },
	{ CCI_REG8(0x034A), 0x09 },	/* Y_ADD_END = 2311 */
	{ CCI_REG8(0x034B), 0x07 },
	{ CCI_REG8(0x034C), 0x05 },	/* X_OUT_SIZE = 1440 */
	{ CCI_REG8(0x034D), 0xA0 },
	{ CCI_REG8(0x034E), 0x04 },	/* Y_OUT_SIZE = 1080 */
	{ CCI_REG8(0x034F), 0x38 },
	{ CCI_REG8(0x0340), 0x04 },	/* FLL = 1265 */
	{ CCI_REG8(0x0341), 0xF1 },
	{ CCI_REG8(0x0900), 0x01 },	/* BINNING_MODE = on */
	{ CCI_REG8(0x0901), 0x22 },	/* BINNING_TYPE = 2x2 */
	{ CCI_REG8(0x0387), 0x03 },	/* Y_ODD_INC */
	{ CCI_REG8(0x3906), 0x7E },	/* Vendor */
};

/* Mode 3: 816x612 @90fps — 4x4 binning */
static const struct cci_reg_sequence s5k4h7yx_mode_816x612_regs[] = {
	{ CCI_REG8(0x0344), 0x00 },	/* X_ADD_STA = 8 */
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x00 },	/* Y_ADD_STA = 8 */
	{ CCI_REG8(0x0347), 0x08 },
	{ CCI_REG8(0x0348), 0x0C },	/* X_ADD_END = 3271 */
	{ CCI_REG8(0x0349), 0xC7 },
	{ CCI_REG8(0x034A), 0x09 },	/* Y_ADD_END = 2455 */
	{ CCI_REG8(0x034B), 0x97 },
	{ CCI_REG8(0x034C), 0x03 },	/* X_OUT_SIZE = 816 */
	{ CCI_REG8(0x034D), 0x30 },
	{ CCI_REG8(0x034E), 0x02 },	/* Y_OUT_SIZE = 612 */
	{ CCI_REG8(0x034F), 0x64 },
	{ CCI_REG8(0x0340), 0x03 },	/* FLL = 843 */
	{ CCI_REG8(0x0341), 0x4B },
	{ CCI_REG8(0x0900), 0x01 },	/* BINNING_MODE = on */
	{ CCI_REG8(0x0901), 0x44 },	/* BINNING_TYPE = 4x4 */
	{ CCI_REG8(0x0387), 0x07 },	/* Y_ODD_INC */
	{ CCI_REG8(0x3906), 0x7E },	/* Vendor */
};

/* Mode 4: 752x564 @120fps — 4x4 binning, center crop */
static const struct cci_reg_sequence s5k4h7yx_mode_752x564_regs[] = {
	{ CCI_REG8(0x0344), 0x00 },	/* X_ADD_STA = 136 */
	{ CCI_REG8(0x0345), 0x88 },
	{ CCI_REG8(0x0346), 0x00 },	/* Y_ADD_STA = 104 */
	{ CCI_REG8(0x0347), 0x68 },
	{ CCI_REG8(0x0348), 0x0C },	/* X_ADD_END = 3143 */
	{ CCI_REG8(0x0349), 0x47 },
	{ CCI_REG8(0x034A), 0x09 },	/* Y_ADD_END = 2359 */
	{ CCI_REG8(0x034B), 0x37 },
	{ CCI_REG8(0x034C), 0x02 },	/* X_OUT_SIZE = 752 */
	{ CCI_REG8(0x034D), 0xF0 },
	{ CCI_REG8(0x034E), 0x02 },	/* Y_OUT_SIZE = 564 */
	{ CCI_REG8(0x034F), 0x34 },
	{ CCI_REG8(0x0340), 0x02 },	/* FLL = 632 */
	{ CCI_REG8(0x0341), 0x78 },
	{ CCI_REG8(0x0900), 0x01 },	/* BINNING_MODE = on */
	{ CCI_REG8(0x0901), 0x44 },	/* BINNING_TYPE = 4x4 */
	{ CCI_REG8(0x0387), 0x07 },	/* Y_ODD_INC */
	{ CCI_REG8(0x3906), 0x7E },	/* Vendor */
};

/* --- Mode definitions --- */
static const struct s5k4h7yx_mode s5k4h7yx_supported_modes[] = {
	{
		/* Mode 0: 3264x2448 @30fps — full resolution */
		.width = 3264,
		.height = 2448,
		.vts_def = 2530,
		.vts_min = 2530,
		.llp = 3688,
		.link_freq_index = 0,
		.reg_list = REGS(s5k4h7yx_mode_3264x2448_regs),
		.crop = {
			.left = S5K4H7YX_PIXEL_ARRAY_LEFT,
			.top = S5K4H7YX_PIXEL_ARRAY_TOP,
			.width = 3264,
			.height = 2448,
		},
	},
	{
		/* Mode 2: 1440x1080 @60fps — 2x2 binning + crop */
		.width = 1440,
		.height = 1080,
		.vts_def = 1265,
		.vts_min = 1265,
		.llp = 3688,
		.link_freq_index = 0,
		.reg_list = REGS(s5k4h7yx_mode_1440x1080_regs),
		.crop = {
			.left = 200,
			.top = 152,
			.width = 2880,
			.height = 2160,
		},
	},
	{
		/* Mode 3: 816x612 @90fps — 4x4 binning */
		.width = 816,
		.height = 612,
		.vts_def = 843,
		.vts_min = 843,
		.llp = 3688,
		.link_freq_index = 0,
		.reg_list = REGS(s5k4h7yx_mode_816x612_regs),
		.crop = {
			.left = S5K4H7YX_PIXEL_ARRAY_LEFT,
			.top = S5K4H7YX_PIXEL_ARRAY_TOP,
			.width = 3264,
			.height = 2448,
		},
	},
	{
		/* Mode 4: 752x564 @120fps — 4x4 binning + crop */
		.width = 752,
		.height = 564,
		.vts_def = 632,
		.vts_min = 632,
		.llp = 3688,
		.link_freq_index = 0,
		.reg_list = REGS(s5k4h7yx_mode_752x564_regs),
		.crop = {
			.left = 136,
			.top = 104,
			.width = 3008,
			.height = 2256,
		},
	},
};

/* --- Driver state --- */
struct s5k4h7yx {
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

	const struct s5k4h7yx_mode *cur_mode;

	unsigned long link_freq_bitmap;

	struct gpio_desc *reset_gpio;
	struct mutex mutex;
	struct clk *clk;
	struct regulator_bulk_data supplies[S5K4H7YX_NUM_SUPPLIES];
};

static inline struct s5k4h7yx *to_s5k4h7yx(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct s5k4h7yx, sd);
}

/* --- Function stubs (to be filled) --- */

/* --- V4L2 helper functions --- */

static u32 s5k4h7yx_get_format_code(const struct s5k4h7yx *s5k4h7yx)
{
	unsigned int i;

	lockdep_assert_held(&s5k4h7yx->mutex);

	i = (s5k4h7yx->vflip->val ? 2 : 0) |
	    (s5k4h7yx->hflip->val ? 1 : 0);

	return s5k4h7yx_mbus_codes[i];
}

static int s5k4h7yx_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(fh->state, 0);
	struct v4l2_rect *try_crop;

	mutex_lock(&s5k4h7yx->mutex);

	try_fmt->width = s5k4h7yx_supported_modes[0].width;
	try_fmt->height = s5k4h7yx_supported_modes[0].height;
	try_fmt->code = s5k4h7yx_get_format_code(s5k4h7yx);
	try_fmt->field = V4L2_FIELD_NONE;

	try_crop = v4l2_subdev_state_get_crop(fh->state, 0);
	try_crop->left = S5K4H7YX_PIXEL_ARRAY_LEFT;
	try_crop->top = S5K4H7YX_PIXEL_ARRAY_TOP;
	try_crop->width = S5K4H7YX_PIXEL_ARRAY_WIDTH;
	try_crop->height = S5K4H7YX_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&s5k4h7yx->mutex);

	return 0;
}

static int s5k4h7yx_update_digital_gain(struct s5k4h7yx *s5k4h7yx, u32 val)
{
	int ret = 0;

	cci_write(s5k4h7yx->regmap, S5K4H7YX_REG_GR_DIGITAL_GAIN, val, &ret);
	cci_write(s5k4h7yx->regmap, S5K4H7YX_REG_GB_DIGITAL_GAIN, val, &ret);
	cci_write(s5k4h7yx->regmap, S5K4H7YX_REG_R_DIGITAL_GAIN, val, &ret);
	cci_write(s5k4h7yx->regmap, S5K4H7YX_REG_B_DIGITAL_GAIN, val, &ret);

	return ret;
}

static void s5k4h7yx_adjust_exposure_range(struct s5k4h7yx *s5k4h7yx)
{
	int exposure_max, exposure_def;

	exposure_max = s5k4h7yx->cur_mode->height + s5k4h7yx->vblank->val -
		       S5K4H7YX_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, s5k4h7yx->exposure->val);
	__v4l2_ctrl_modify_range(s5k4h7yx->exposure,
				 s5k4h7yx->exposure->minimum,
				 exposure_max, s5k4h7yx->exposure->step,
				 exposure_def);
}

/* --- V4L2 control operations --- */
static int s5k4h7yx_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4h7yx *s5k4h7yx =
		container_of(ctrl->handler, struct s5k4h7yx, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&s5k4h7yx->sd);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK)
		s5k4h7yx_adjust_exposure_range(s5k4h7yx);

	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k4h7yx->regmap, S5K4H7YX_REG_ANALOG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k4h7yx->regmap, S5K4H7YX_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = s5k4h7yx_update_digital_gain(s5k4h7yx, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k4h7yx->regmap, S5K4H7YX_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k4h7yx->regmap,
				S5K4H7YX_REG_FRM_LENGTH_LINES,
				s5k4h7yx->cur_mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		ret = cci_write(s5k4h7yx->regmap, S5K4H7YX_REG_ORIENTATION,
				(s5k4h7yx->hflip->val ?
				 S5K4H7YX_ORIENT_HFLIP : 0) |
				(s5k4h7yx->vflip->val ?
				 S5K4H7YX_ORIENT_VFLIP : 0),
				NULL);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k4h7yx_ctrl_ops = {
	.s_ctrl = s5k4h7yx_set_ctrl,
};

/* --- Pad operations --- */
static int s5k4h7yx_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = s5k4h7yx_get_format_code(s5k4h7yx);

	return 0;
}

static int s5k4h7yx_enum_frame_size(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

	if (fse->index >= ARRAY_SIZE(s5k4h7yx_supported_modes))
		return -EINVAL;

	if (fse->code != s5k4h7yx_get_format_code(s5k4h7yx))
		return -EINVAL;

	fse->min_width = s5k4h7yx_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k4h7yx_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void s5k4h7yx_update_pad_format(struct s5k4h7yx *s5k4h7yx,
					const struct s5k4h7yx_mode *mode,
					struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = s5k4h7yx_get_format_code(s5k4h7yx);
	fmt->format.field = V4L2_FIELD_NONE;
}

static int __s5k4h7yx_get_pad_format(struct s5k4h7yx *s5k4h7yx,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_format *fmt)
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state,
							    fmt->pad);
	else
		s5k4h7yx_update_pad_format(s5k4h7yx, s5k4h7yx->cur_mode, fmt);

	return 0;
}

static int s5k4h7yx_get_pad_format(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_format *fmt)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	int ret;

	mutex_lock(&s5k4h7yx->mutex);
	ret = __s5k4h7yx_get_pad_format(s5k4h7yx, sd_state, fmt);
	mutex_unlock(&s5k4h7yx->mutex);

	return ret;
}

static int s5k4h7yx_set_pad_format(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_format *fmt)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	struct v4l2_mbus_framefmt *framefmt;
	const struct s5k4h7yx_mode *mode;
	s32 vblank_def;
	s32 vblank_min;
	s64 h_blank;
	s64 pixel_rate;
	s64 link_freq;

	mutex_lock(&s5k4h7yx->mutex);

	fmt->format.code = s5k4h7yx_get_format_code(s5k4h7yx);

	mode = v4l2_find_nearest_size(s5k4h7yx_supported_modes,
		ARRAY_SIZE(s5k4h7yx_supported_modes), width, height,
		fmt->format.width, fmt->format.height);
	s5k4h7yx_update_pad_format(s5k4h7yx, mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		s5k4h7yx->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(s5k4h7yx->link_freq,
				   mode->link_freq_index);

		link_freq = s5k4h7yx_link_freq_menu[mode->link_freq_index];
		pixel_rate = s5k4h7yx_link_freq_to_pixel_rate(link_freq);
		__v4l2_ctrl_modify_range(s5k4h7yx->pixel_rate, pixel_rate,
					 pixel_rate, 1, pixel_rate);

		vblank_def = mode->vts_def - mode->height;
		vblank_min = mode->vts_min - mode->height;
		__v4l2_ctrl_modify_range(
			s5k4h7yx->vblank, vblank_min,
			S5K4H7YX_VTS_MAX - mode->height, 1,
			vblank_def);
		__v4l2_ctrl_s_ctrl(s5k4h7yx->vblank, vblank_def);
		h_blank = mode->llp - mode->width;
		__v4l2_ctrl_modify_range(s5k4h7yx->hblank, h_blank,
					 h_blank, 1, h_blank);
	}

	mutex_unlock(&s5k4h7yx->mutex);

	return 0;
}

static const struct v4l2_rect *
__s5k4h7yx_get_pad_crop(struct s5k4h7yx *s5k4h7yx,
			struct v4l2_subdev_state *sd_state,
			unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &s5k4h7yx->cur_mode->crop;
	}

	return NULL;
}

static int s5k4h7yx_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

		mutex_lock(&s5k4h7yx->mutex);
		sel->r = *__s5k4h7yx_get_pad_crop(s5k4h7yx, sd_state,
						   sel->pad, sel->which);
		mutex_unlock(&s5k4h7yx->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K4H7YX_NATIVE_WIDTH;
		sel->r.height = S5K4H7YX_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = S5K4H7YX_PIXEL_ARRAY_LEFT;
		sel->r.top = S5K4H7YX_PIXEL_ARRAY_TOP;
		sel->r.width = S5K4H7YX_PIXEL_ARRAY_WIDTH;
		sel->r.height = S5K4H7YX_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/* --- Streaming --- */
static int s5k4h7yx_start_streaming(struct s5k4h7yx *s5k4h7yx)
{
	struct i2c_client *client = v4l2_get_subdevdata(&s5k4h7yx->sd);
	const struct s5k4h7yx_reg_list *reg_list;
	int ret;

	/* Apply common (PLL, CSI, vendor) registers */
	ret = cci_multi_reg_write(s5k4h7yx->regmap, s5k4h7yx_common_regs,
				  ARRAY_SIZE(s5k4h7yx_common_regs), NULL);
	if (ret) {
		dev_err(&client->dev, "%s failed to set common regs\n",
			__func__);
		return ret;
	}

	/* Apply mode registers */
	reg_list = &s5k4h7yx->cur_mode->reg_list;
	ret = cci_multi_reg_write(s5k4h7yx->regmap, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(s5k4h7yx->sd.ctrl_handler);
	if (ret)
		return ret;

	/* Start streaming */
	return cci_write(s5k4h7yx->regmap, S5K4H7YX_REG_MODE_SELECT,
			 S5K4H7YX_MODE_STREAMING, NULL);
}

static int s5k4h7yx_stop_streaming(struct s5k4h7yx *s5k4h7yx)
{
	struct i2c_client *client = v4l2_get_subdevdata(&s5k4h7yx->sd);
	int ret;

	ret = cci_write(s5k4h7yx->regmap, S5K4H7YX_REG_MODE_SELECT,
			S5K4H7YX_MODE_STANDBY, NULL);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	return 0;
}

static int s5k4h7yx_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&s5k4h7yx->mutex);

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto err_unlock;

		ret = s5k4h7yx_start_streaming(s5k4h7yx);
		if (ret)
			goto err_rpm_put;
	} else {
		s5k4h7yx_stop_streaming(s5k4h7yx);
		pm_runtime_put(&client->dev);
	}

	mutex_unlock(&s5k4h7yx->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&s5k4h7yx->mutex);

	return ret;
}

/* --- Power management --- */
static int s5k4h7yx_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	int ret;

	ret = regulator_bulk_enable(S5K4H7YX_NUM_SUPPLIES,
				    s5k4h7yx->supplies);
	if (ret) {
		dev_err(dev, "%s: failed to enable regulators\n", __func__);
		return ret;
	}

	/* Deassert reset */
	gpiod_set_value_cansleep(s5k4h7yx->reset_gpio, 0);

	ret = clk_prepare_enable(s5k4h7yx->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		gpiod_set_value_cansleep(s5k4h7yx->reset_gpio, 1);
		regulator_bulk_disable(S5K4H7YX_NUM_SUPPLIES,
				       s5k4h7yx->supplies);
		return ret;
	}

	/* Sensor needs time after clock enable before first I2C */
	fsleep(1000);

	return 0;
}

static int s5k4h7yx_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

	clk_disable_unprepare(s5k4h7yx->clk);

	/* Assert reset */
	gpiod_set_value_cansleep(s5k4h7yx->reset_gpio, 1);

	regulator_bulk_disable(S5K4H7YX_NUM_SUPPLIES, s5k4h7yx->supplies);

	return 0;
}

/* --- Chip identification --- */
static int s5k4h7yx_identify_module(struct s5k4h7yx *s5k4h7yx)
{
	struct i2c_client *client = v4l2_get_subdevdata(&s5k4h7yx->sd);
	int ret;
	u64 val;

	ret = cci_read(s5k4h7yx->regmap, S5K4H7YX_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x\n",
			S5K4H7YX_CHIP_ID);
		return ret;
	}

	if (val != S5K4H7YX_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%llx\n",
			S5K4H7YX_CHIP_ID, val);
		return -EIO;
	}

	return 0;
}

/* --- V4L2 subdev ops --- */
static const struct v4l2_subdev_video_ops s5k4h7yx_video_ops = {
	.s_stream = s5k4h7yx_set_stream,
};

static const struct v4l2_subdev_pad_ops s5k4h7yx_pad_ops = {
	.enum_mbus_code = s5k4h7yx_enum_mbus_code,
	.get_fmt = s5k4h7yx_get_pad_format,
	.set_fmt = s5k4h7yx_set_pad_format,
	.enum_frame_size = s5k4h7yx_enum_frame_size,
	.get_selection = s5k4h7yx_get_selection,
};

static const struct v4l2_subdev_ops s5k4h7yx_subdev_ops = {
	.video = &s5k4h7yx_video_ops,
	.pad = &s5k4h7yx_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k4h7yx_internal_ops = {
	.open = s5k4h7yx_open,
};

static const struct dev_pm_ops s5k4h7yx_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k4h7yx_power_off, s5k4h7yx_power_on, NULL)
};

/* --- Controls init --- */
static int s5k4h7yx_init_controls(struct s5k4h7yx *s5k4h7yx)
{
	struct i2c_client *client = v4l2_get_subdevdata(&s5k4h7yx->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 vblank_def;
	s64 vblank_min;
	s64 pixel_rate;
	s64 h_blank;
	int ret;

	ctrl_hdlr = &s5k4h7yx->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 11);
	if (ret)
		return ret;

	mutex_init(&s5k4h7yx->mutex);
	ctrl_hdlr->lock = &s5k4h7yx->mutex;

	s5k4h7yx->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
				&s5k4h7yx_ctrl_ops,
				V4L2_CID_LINK_FREQ,
				ARRAY_SIZE(s5k4h7yx_link_freq_menu) - 1,
				0, s5k4h7yx_link_freq_menu);
	if (s5k4h7yx->link_freq)
		s5k4h7yx->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k4h7yx->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7yx_ctrl_ops,
					     V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5k4h7yx->hflip)
		s5k4h7yx->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5k4h7yx->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7yx_ctrl_ops,
					     V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5k4h7yx->vflip)
		s5k4h7yx->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	pixel_rate = s5k4h7yx_link_freq_to_pixel_rate(
			s5k4h7yx_link_freq_menu[s5k4h7yx->cur_mode->link_freq_index]);

	s5k4h7yx->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr,
				&s5k4h7yx_ctrl_ops,
				V4L2_CID_PIXEL_RATE,
				pixel_rate, pixel_rate, 1, pixel_rate);

	vblank_def = s5k4h7yx->cur_mode->vts_def - s5k4h7yx->cur_mode->height;
	vblank_min = s5k4h7yx->cur_mode->vts_min - s5k4h7yx->cur_mode->height;
	s5k4h7yx->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7yx_ctrl_ops,
				V4L2_CID_VBLANK, vblank_min,
				S5K4H7YX_VTS_MAX - s5k4h7yx->cur_mode->height,
				1, vblank_def);

	h_blank = s5k4h7yx->cur_mode->llp - s5k4h7yx->cur_mode->width;
	s5k4h7yx->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7yx_ctrl_ops,
				V4L2_CID_HBLANK, h_blank, h_blank, 1, h_blank);
	if (s5k4h7yx->hblank)
		s5k4h7yx->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k4h7yx->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7yx_ctrl_ops,
				V4L2_CID_EXPOSURE, S5K4H7YX_EXPOSURE_MIN,
				S5K4H7YX_VTS_MAX - S5K4H7YX_EXPOSURE_OFFSET,
				S5K4H7YX_EXPOSURE_STEP,
				S5K4H7YX_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7yx_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN,
			  S5K4H7YX_ANA_GAIN_MIN, S5K4H7YX_ANA_GAIN_MAX,
			  S5K4H7YX_ANA_GAIN_STEP, S5K4H7YX_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7yx_ctrl_ops,
			  V4L2_CID_DIGITAL_GAIN,
			  S5K4H7YX_DGTL_GAIN_MIN, S5K4H7YX_DGTL_GAIN_MAX,
			  S5K4H7YX_DGTL_GAIN_STEP,
			  S5K4H7YX_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k4h7yx_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(s5k4h7yx_test_pattern_menu) - 1,
				0, 0, s5k4h7yx_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k4h7yx_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	s5k4h7yx->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&s5k4h7yx->mutex);

	return ret;
}

static void s5k4h7yx_free_controls(struct s5k4h7yx *s5k4h7yx)
{
	v4l2_ctrl_handler_free(s5k4h7yx->sd.ctrl_handler);
	mutex_destroy(&s5k4h7yx->mutex);
}

/* --- Regulators --- */
static int s5k4h7yx_get_regulators(struct s5k4h7yx *s5k4h7yx,
				    struct i2c_client *client)
{
	unsigned int i;

	for (i = 0; i < S5K4H7YX_NUM_SUPPLIES; i++)
		s5k4h7yx->supplies[i].supply = s5k4h7yx_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       S5K4H7YX_NUM_SUPPLIES,
				       s5k4h7yx->supplies);
}

/* --- Probe / Remove --- */
static int s5k4h7yx_probe(struct i2c_client *client)
{
	struct s5k4h7yx *s5k4h7yx;
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret;
	u32 val = 0;

	s5k4h7yx = devm_kzalloc(&client->dev, sizeof(*s5k4h7yx), GFP_KERNEL);
	if (!s5k4h7yx)
		return -ENOMEM;

	s5k4h7yx->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k4h7yx->regmap)) {
		ret = PTR_ERR(s5k4h7yx->regmap);
		dev_err(&client->dev, "failed to initialize CCI: %d\n", ret);
		return ret;
	}

	ret = s5k4h7yx_get_regulators(s5k4h7yx, client);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to get regulators\n");

	/* Reset GPIO (active low) */
	s5k4h7yx->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
							GPIOD_OUT_HIGH);
	if (IS_ERR(s5k4h7yx->reset_gpio))
		return dev_err_probe(&client->dev,
				     PTR_ERR(s5k4h7yx->reset_gpio),
				     "failed to get reset GPIO\n");

	s5k4h7yx->clk = devm_clk_get_optional(&client->dev, NULL);
	if (IS_ERR(s5k4h7yx->clk))
		return dev_err_probe(&client->dev, PTR_ERR(s5k4h7yx->clk),
				     "error getting clock\n");
	if (!s5k4h7yx->clk) {
		dev_dbg(&client->dev,
			"no clock provided, using clock-frequency property\n");
		device_property_read_u32(&client->dev, "clock-frequency", &val);
	} else {
		val = clk_get_rate(s5k4h7yx->clk);
	}

	if (val != 24000000) {
		dev_err(&client->dev,
			"input clock frequency of %u not supported (expected 24 MHz)\n",
			val);
		return -EINVAL;
	}

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
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

	ret = v4l2_link_freq_to_bitmap(&client->dev,
				       ep.link_frequencies,
				       ep.nr_of_link_frequencies,
				       s5k4h7yx_link_freq_menu,
				       ARRAY_SIZE(s5k4h7yx_link_freq_menu),
				       &s5k4h7yx->link_freq_bitmap);
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

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&s5k4h7yx->sd, client, &s5k4h7yx_subdev_ops);

	/* Will be powered off via pm_runtime_idle */
	ret = s5k4h7yx_power_on(&client->dev);
	if (ret)
		goto error_endpoint_free;

	/* Check module identity */
	ret = s5k4h7yx_identify_module(s5k4h7yx);
	if (ret)
		goto error_identify;

	/* Set default mode to max resolution */
	s5k4h7yx->cur_mode = &s5k4h7yx_supported_modes[0];

	ret = s5k4h7yx_init_controls(s5k4h7yx);
	if (ret)
		goto error_identify;

	/* Initialize subdev */
	s5k4h7yx->sd.internal_ops = &s5k4h7yx_internal_ops;
	s5k4h7yx->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k4h7yx->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	s5k4h7yx->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k4h7yx->sd.entity, 1,
				     &s5k4h7yx->pad);
	if (ret)
		goto error_handler_free;

	ret = v4l2_async_register_subdev_sensor(&s5k4h7yx->sd);
	if (ret < 0)
		goto error_media_entity;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);
	v4l2_fwnode_endpoint_free(&ep);

	return 0;

error_media_entity:
	media_entity_cleanup(&s5k4h7yx->sd.entity);

error_handler_free:
	s5k4h7yx_free_controls(s5k4h7yx);

error_identify:
	s5k4h7yx_power_off(&client->dev);

error_endpoint_free:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

static void s5k4h7yx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	s5k4h7yx_free_controls(s5k4h7yx);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		s5k4h7yx_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id s5k4h7yx_dt_ids[] = {
	{ .compatible = "samsung,s5k4h7yx" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k4h7yx_dt_ids);

static struct i2c_driver s5k4h7yx_i2c_driver = {
	.driver = {
		.name = "s5k4h7yx",
		.pm = &s5k4h7yx_pm_ops,
		.of_match_table = s5k4h7yx_dt_ids,
	},
	.probe = s5k4h7yx_probe,
	.remove = s5k4h7yx_remove,
};

module_i2c_driver(s5k4h7yx_i2c_driver);

MODULE_AUTHOR("Marc Lainez <marc.lainez@gmail.com>");
MODULE_DESCRIPTION("Samsung S5K4H7YX sensor driver");
MODULE_LICENSE("GPL");
