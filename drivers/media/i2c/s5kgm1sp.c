// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5KGM1SP sensor driver
 *
 * Copyright (C) 2024 Marc Lainez <marc.lainez@gmail.com>
 *
 * Driver for the Samsung S5KGM1SP 48MP ISOCELL sensor found in the
 * Fairphone 3+ rear camera. Register tables reverse-engineered from
 * vendor firmware (libmmcamera_s5kgm1sp.so).
 *
 * Based on the S5K4H7YX driver.
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

/* Chip ID */
#define S5KGM1SP_REG_CHIP_ID CCI_REG16(0x0000)
#define S5KGM1SP_CHIP_ID 0x08D1

/* Streaming control */
#define S5KGM1SP_REG_MODE_SELECT CCI_REG8(0x0100)
#define S5KGM1SP_MODE_STANDBY 0x00
#define S5KGM1SP_MODE_STREAMING 0x01

/* Group hold */
#define S5KGM1SP_REG_GROUP_HOLD CCI_REG16(0x0104)

/* Frame timing */
#define S5KGM1SP_REG_FRM_LENGTH_LINES CCI_REG16(0x0340)
#define S5KGM1SP_REG_LINE_LENGTH_PCK CCI_REG16(0x0342)

/* Exposure control */
#define S5KGM1SP_REG_EXPOSURE CCI_REG16(0x0202)
#define S5KGM1SP_EXPOSURE_MIN 4
#define S5KGM1SP_EXPOSURE_STEP 1
#define S5KGM1SP_EXPOSURE_DEFAULT 0x0648
#define S5KGM1SP_EXPOSURE_OFFSET 22

/* Analog gain control */
#define S5KGM1SP_REG_ANALOG_GAIN CCI_REG16(0x0204)
#define S5KGM1SP_ANA_GAIN_MIN 0
#define S5KGM1SP_ANA_GAIN_MAX 978
#define S5KGM1SP_ANA_GAIN_STEP 1
#define S5KGM1SP_ANA_GAIN_DEFAULT 0

/* Digital gain control */
#define S5KGM1SP_REG_GR_DIGITAL_GAIN CCI_REG16(0x020e)
#define S5KGM1SP_REG_R_DIGITAL_GAIN CCI_REG16(0x0210)
#define S5KGM1SP_REG_B_DIGITAL_GAIN CCI_REG16(0x0212)
#define S5KGM1SP_REG_GB_DIGITAL_GAIN CCI_REG16(0x0214)
#define S5KGM1SP_DGTL_GAIN_MIN 0x0100
#define S5KGM1SP_DGTL_GAIN_MAX 0x1000
#define S5KGM1SP_DGTL_GAIN_DEFAULT 0x0100
#define S5KGM1SP_DGTL_GAIN_STEP 1

/* Test pattern */
#define S5KGM1SP_REG_TEST_PATTERN CCI_REG16(0x0600)

/* Pixel array (48MP native: 8000x6000, active crop 7992x7992) */
#define S5KGM1SP_NATIVE_WIDTH 8000U
#define S5KGM1SP_NATIVE_HEIGHT 6000U
#define S5KGM1SP_PIXEL_ARRAY_LEFT 8U
#define S5KGM1SP_PIXEL_ARRAY_TOP 8U
#define S5KGM1SP_PIXEL_ARRAY_WIDTH 7992U
#define S5KGM1SP_PIXEL_ARRAY_HEIGHT 5992U

#define S5KGM1SP_VTS_MAX 65535

/* Regulator supplies */
static const char *const s5kgm1sp_supply_name[] = {
	"vana", /* Analog (2.8V) supply */
	"vdig", /* Digital Core (1.175V) supply */
	"vif", /* IF (1.8V) supply */
};

#define S5KGM1SP_NUM_SUPPLIES ARRAY_SIZE(s5kgm1sp_supply_name)

struct s5kgm1sp_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct s5kgm1sp_mode {
	u32 width;
	u32 height;
	u32 vts_def;
	u32 vts_min;
	u32 llp;
	u32 link_freq_index;
	struct s5kgm1sp_reg_list reg_list;
	struct v4l2_rect crop;
};

/* Bayer order: GRBG (Samsung default, 10-bit) */
#define S5KGM1SP_MBUS_CODE MEDIA_BUS_FMT_SGRBG10_1X10

static const char *const s5kgm1sp_test_pattern_menu[] = {
	"Disabled",    "Solid Colour",
	"Colour Bars", "Colour Bars With Fade to Grey",
	"PN9",
};

static const s64 s5kgm1sp_link_freq_menu[] = {
	576000000LL, /* 24 * 144 / 3 / 2 — standard modes */
	596000000LL, /* 24 * 149 / 3 / 2 — y-bin modes */
};

#define REGS(_list)                               \
	{                                         \
		.num_of_regs = ARRAY_SIZE(_list), \
		.regs = _list,                    \
	}

static u64 s5kgm1sp_link_freq_to_pixel_rate(u64 link_freq)
{
	/* link_freq * 2 (DDR) * 4 lanes / 10 bpp */
	return div_u64(link_freq * 2 * 4, 10);
}

/*
 * Register tables: init sequence (firmware upload) and mode tables.
 * These are large arrays extracted from vendor firmware — placed in
 * a separate header to keep this file manageable.
 */
#include "s5kgm1sp_regs.h"

static const struct s5kgm1sp_mode s5kgm1sp_supported_modes[] = {
	{
		.width = 4000,
		.height = 3000,
		.vts_def = 3194,
		.vts_min = 3194,
		.llp = 5024,
		.link_freq_index = 0,
		.reg_list = REGS(s5kgm1sp_mode_4000x3000_regs),
		.crop = {
			.left = 8,
			.top = 8,
			.width = 8000,
			.height = 6000,
		},
	},
	{
		.width = 2000,
		.height = 1500,
		.vts_def = 3196,
		.vts_min = 3196,
		.llp = 5024,
		.link_freq_index = 0,
		.reg_list = REGS(s5kgm1sp_mode_2000x1500_30fps_regs),
		.crop = {
			.left = 8,
			.top = 8,
			.width = 8000,
			.height = 6000,
		},
	},
	{
		.width = 2000,
		.height = 1500,
		.vts_def = 1598,
		.vts_min = 1598,
		.llp = 5024,
		.link_freq_index = 0,
		.reg_list = REGS(s5kgm1sp_mode_2000x1500_60fps_regs),
		.crop = {
			.left = 8,
			.top = 8,
			.width = 8000,
			.height = 6000,
		},
	},
	{
		.width = 2000,
		.height = 1500,
		.vts_def = 2132,
		.vts_min = 2132,
		.llp = 2512,
		.link_freq_index = 1,
		.reg_list = REGS(s5kgm1sp_mode_2000x1500_ybin_30fps_regs),
		.crop = {
			.left = 8,
			.top = 8,
			.width = 8000,
			.height = 6000,
		},
	},
	{
		.width = 2000,
		.height = 1500,
		.vts_def = 1600,
		.vts_min = 1600,
		.llp = 2512,
		.link_freq_index = 1,
		.reg_list = REGS(s5kgm1sp_mode_2000x1500_ybin_60fps_regs),
		.crop = {
			.left = 8,
			.top = 8,
			.width = 8000,
			.height = 6000,
		},
	},
};

struct s5kgm1sp {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *analog_gain;

	const struct s5kgm1sp_mode *cur_mode;

	unsigned long link_freq_bitmap;

	struct gpio_desc *reset_gpio;
	struct mutex mutex;
	struct clk *clk;
	struct regulator_bulk_data supplies[S5KGM1SP_NUM_SUPPLIES];
};

static inline struct s5kgm1sp *to_s5kgm1sp(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct s5kgm1sp, sd);
}

static int s5kgm1sp_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5kgm1sp *sensor = to_s5kgm1sp(sd);
	int ret;

	ret = regulator_bulk_enable(S5KGM1SP_NUM_SUPPLIES, sensor->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	fsleep(1000);

	ret = clk_prepare_enable(sensor->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		regulator_bulk_disable(S5KGM1SP_NUM_SUPPLIES, sensor->supplies);
		return ret;
	}

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	fsleep(12000);

	return 0;
}

static int s5kgm1sp_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5kgm1sp *sensor = to_s5kgm1sp(sd);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	clk_disable_unprepare(sensor->clk);
	regulator_bulk_disable(S5KGM1SP_NUM_SUPPLIES, sensor->supplies);

	return 0;
}

static int s5kgm1sp_identify(struct s5kgm1sp *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	u64 val;
	int ret;

	ret = cci_read(sensor->regmap, S5KGM1SP_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5KGM1SP_CHIP_ID) {
		dev_err(&client->dev,
			"chip id mismatch: expected 0x%04x, got 0x%04llx\n",
			S5KGM1SP_CHIP_ID, val);
		return -ENXIO;
	}

	dev_info(&client->dev, "S5KGM1SP chip ID confirmed: 0x%04llx\n", val);
	return 0;
}

static int s5kgm1sp_update_digital_gain(struct s5kgm1sp *sensor, u32 val)
{
	int ret = 0;

	cci_write(sensor->regmap, S5KGM1SP_REG_GR_DIGITAL_GAIN, val, &ret);
	cci_write(sensor->regmap, S5KGM1SP_REG_GB_DIGITAL_GAIN, val, &ret);
	cci_write(sensor->regmap, S5KGM1SP_REG_R_DIGITAL_GAIN, val, &ret);
	cci_write(sensor->regmap, S5KGM1SP_REG_B_DIGITAL_GAIN, val, &ret);

	return ret;
}

static void s5kgm1sp_adjust_exposure_range(struct s5kgm1sp *sensor)
{
	int exposure_max, exposure_def;

	exposure_max = sensor->cur_mode->height + sensor->vblank->val -
		       S5KGM1SP_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, sensor->exposure->val);
	__v4l2_ctrl_modify_range(sensor->exposure, sensor->exposure->minimum,
				 exposure_max, sensor->exposure->step,
				 exposure_def);
}

static int s5kgm1sp_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5kgm1sp *sensor =
		container_of(ctrl->handler, struct s5kgm1sp, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK)
		s5kgm1sp_adjust_exposure_range(sensor);

	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, S5KGM1SP_REG_GROUP_HOLD, 1, &ret);
		cci_write(sensor->regmap, S5KGM1SP_REG_FRM_LENGTH_LINES,
			  sensor->cur_mode->height + sensor->vblank->val, &ret);
		cci_write(sensor->regmap, S5KGM1SP_REG_EXPOSURE, ctrl->val,
			  &ret);
		cci_write(sensor->regmap, S5KGM1SP_REG_ANALOG_GAIN,
			  sensor->analog_gain->val, &ret);
		cci_write(sensor->regmap, S5KGM1SP_REG_GROUP_HOLD, 0, &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = s5kgm1sp_update_digital_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(sensor->regmap, S5KGM1SP_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(sensor->regmap, S5KGM1SP_REG_FRM_LENGTH_LINES,
				sensor->cur_mode->height + ctrl->val, NULL);
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

static const struct v4l2_ctrl_ops s5kgm1sp_ctrl_ops = {
	.s_ctrl = s5kgm1sp_set_ctrl,
};

static int s5kgm1sp_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(fh->state, 0);
	struct v4l2_rect *try_crop;

	try_fmt->width = s5kgm1sp_supported_modes[0].width;
	try_fmt->height = s5kgm1sp_supported_modes[0].height;
	try_fmt->code = S5KGM1SP_MBUS_CODE;
	try_fmt->field = V4L2_FIELD_NONE;

	try_crop = v4l2_subdev_state_get_crop(fh->state, 0);
	try_crop->left = S5KGM1SP_PIXEL_ARRAY_LEFT;
	try_crop->top = S5KGM1SP_PIXEL_ARRAY_TOP;
	try_crop->width = S5KGM1SP_PIXEL_ARRAY_WIDTH;
	try_crop->height = S5KGM1SP_PIXEL_ARRAY_HEIGHT;

	return 0;
}

static int s5kgm1sp_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = S5KGM1SP_MBUS_CODE;
	return 0;
}

static int s5kgm1sp_enum_frame_size(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5kgm1sp_supported_modes))
		return -EINVAL;

	if (fse->code != S5KGM1SP_MBUS_CODE)
		return -EINVAL;

	fse->min_width = s5kgm1sp_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5kgm1sp_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void s5kgm1sp_update_pad_format(const struct s5kgm1sp_mode *mode,
				       struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = S5KGM1SP_MBUS_CODE;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int s5kgm1sp_get_pad_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_format *fmt)
{
	struct s5kgm1sp *sensor = to_s5kgm1sp(sd);

	mutex_lock(&sensor->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(state, fmt->pad);
	else
		s5kgm1sp_update_pad_format(sensor->cur_mode, fmt);
	mutex_unlock(&sensor->mutex);

	return 0;
}

static int s5kgm1sp_set_pad_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_format *fmt)
{
	struct s5kgm1sp *sensor = to_s5kgm1sp(sd);
	struct v4l2_mbus_framefmt *framefmt;
	const struct s5kgm1sp_mode *mode;
	s32 vblank_def, vblank_min;
	s64 h_blank, pixel_rate, link_freq;

	mutex_lock(&sensor->mutex);

	fmt->format.code = S5KGM1SP_MBUS_CODE;

	mode = v4l2_find_nearest_size(s5kgm1sp_supported_modes,
				      ARRAY_SIZE(s5kgm1sp_supported_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);
	s5kgm1sp_update_pad_format(mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		sensor->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(sensor->link_freq, mode->link_freq_index);

		link_freq = s5kgm1sp_link_freq_menu[mode->link_freq_index];
		pixel_rate = s5kgm1sp_link_freq_to_pixel_rate(link_freq);
		__v4l2_ctrl_modify_range(sensor->pixel_rate, pixel_rate,
					 pixel_rate, 1, pixel_rate);

		vblank_def = mode->vts_def - mode->height;
		vblank_min = mode->vts_min - mode->height;
		__v4l2_ctrl_modify_range(sensor->vblank, vblank_min,
					 S5KGM1SP_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(sensor->vblank, vblank_def);

		h_blank = mode->llp - mode->width;
		__v4l2_ctrl_modify_range(sensor->hblank, h_blank, h_blank, 1,
					 h_blank);
	}

	mutex_unlock(&sensor->mutex);

	return 0;
}

static const struct v4l2_rect *
__s5kgm1sp_get_pad_crop(struct s5kgm1sp *sensor,
			struct v4l2_subdev_state *state, unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->cur_mode->crop;
	}

	return NULL;
}

static int s5kgm1sp_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct s5kgm1sp *sensor = to_s5kgm1sp(sd);

		mutex_lock(&sensor->mutex);
		sel->r = *__s5kgm1sp_get_pad_crop(sensor, state, sel->pad,
						  sel->which);
		mutex_unlock(&sensor->mutex);
		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5KGM1SP_NATIVE_WIDTH;
		sel->r.height = S5KGM1SP_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = S5KGM1SP_PIXEL_ARRAY_LEFT;
		sel->r.top = S5KGM1SP_PIXEL_ARRAY_TOP;
		sel->r.width = S5KGM1SP_PIXEL_ARRAY_WIDTH;
		sel->r.height = S5KGM1SP_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int s5kgm1sp_start_streaming(struct s5kgm1sp *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	const struct s5kgm1sp_reg_list *reg_list;
	int ret;

	/* SW reset (first init entry) — needs 4ms settling time */
	ret = cci_multi_reg_write(sensor->regmap, s5kgm1sp_init_regs, 1, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to write SW reset\n");
		return ret;
	}
	fsleep(4000);

	/* Remaining init registers (firmware upload + config) */
	ret = cci_multi_reg_write(sensor->regmap, &s5kgm1sp_init_regs[1],
				  ARRAY_SIZE(s5kgm1sp_init_regs) - 1, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to write init regs\n");
		return ret;
	}

	/* Mode tables start with a 1ms delay-only entry — wait here */
	fsleep(1000);

	reg_list = &sensor->cur_mode->reg_list;
	ret = cci_multi_reg_write(sensor->regmap, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to write mode regs\n");
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(sensor->sd.ctrl_handler);
	if (ret)
		return ret;

	return cci_write(sensor->regmap, S5KGM1SP_REG_MODE_SELECT,
			 S5KGM1SP_MODE_STREAMING, NULL);
}

static int s5kgm1sp_stop_streaming(struct s5kgm1sp *sensor)
{
	return cci_write(sensor->regmap, S5KGM1SP_REG_MODE_SELECT,
			 S5KGM1SP_MODE_STANDBY, NULL);
}

static int s5kgm1sp_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct s5kgm1sp *sensor = to_s5kgm1sp(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&sensor->mutex);

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto err_unlock;

		ret = s5kgm1sp_start_streaming(sensor);
		if (ret)
			goto err_rpm_put;
	} else {
		s5kgm1sp_stop_streaming(sensor);
		pm_runtime_put(&client->dev);
	}

	mutex_unlock(&sensor->mutex);
	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&sensor->mutex);
	return ret;
}

static const struct v4l2_subdev_video_ops s5kgm1sp_video_ops = {
	.s_stream = s5kgm1sp_set_stream,
};

static const struct v4l2_subdev_pad_ops s5kgm1sp_pad_ops = {
	.enum_mbus_code = s5kgm1sp_enum_mbus_code,
	.enum_frame_size = s5kgm1sp_enum_frame_size,
	.get_fmt = s5kgm1sp_get_pad_format,
	.set_fmt = s5kgm1sp_set_pad_format,
	.get_selection = s5kgm1sp_get_selection,
};

static const struct v4l2_subdev_ops s5kgm1sp_subdev_ops = {
	.video = &s5kgm1sp_video_ops,
	.pad = &s5kgm1sp_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5kgm1sp_internal_ops = {
	.open = s5kgm1sp_open,
};

static const struct dev_pm_ops s5kgm1sp_pm_ops = { SET_RUNTIME_PM_OPS(
	s5kgm1sp_power_off, s5kgm1sp_power_on, NULL) };

static int s5kgm1sp_init_controls(struct s5kgm1sp *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 vblank_def, vblank_min;
	s64 pixel_rate, h_blank;
	int ret;

	ctrl_hdlr = &sensor->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 10);
	if (ret)
		return ret;

	mutex_init(&sensor->mutex);
	ctrl_hdlr->lock = &sensor->mutex;

	sensor->link_freq = v4l2_ctrl_new_int_menu(
		ctrl_hdlr, &s5kgm1sp_ctrl_ops, V4L2_CID_LINK_FREQ,
		ARRAY_SIZE(s5kgm1sp_link_freq_menu) - 1, 0,
		s5kgm1sp_link_freq_menu);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = s5kgm1sp_link_freq_to_pixel_rate(
		s5kgm1sp_link_freq_menu[sensor->cur_mode->link_freq_index]);

	sensor->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgm1sp_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, pixel_rate,
					       pixel_rate, 1, pixel_rate);

	vblank_def = sensor->cur_mode->vts_def - sensor->cur_mode->height;
	vblank_min = sensor->cur_mode->vts_min - sensor->cur_mode->height;
	sensor->vblank = v4l2_ctrl_new_std(
		ctrl_hdlr, &s5kgm1sp_ctrl_ops, V4L2_CID_VBLANK, vblank_min,
		S5KGM1SP_VTS_MAX - sensor->cur_mode->height, 1, vblank_def);

	h_blank = sensor->cur_mode->llp - sensor->cur_mode->width;
	sensor->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5kgm1sp_ctrl_ops,
					   V4L2_CID_HBLANK, h_blank, h_blank, 1,
					   h_blank);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->exposure = v4l2_ctrl_new_std(
		ctrl_hdlr, &s5kgm1sp_ctrl_ops, V4L2_CID_EXPOSURE,
		S5KGM1SP_EXPOSURE_MIN,
		S5KGM1SP_VTS_MAX - S5KGM1SP_EXPOSURE_OFFSET,
		S5KGM1SP_EXPOSURE_STEP, S5KGM1SP_EXPOSURE_DEFAULT);

	sensor->analog_gain = v4l2_ctrl_new_std(
		ctrl_hdlr, &s5kgm1sp_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
		S5KGM1SP_ANA_GAIN_MIN, S5KGM1SP_ANA_GAIN_MAX,
		S5KGM1SP_ANA_GAIN_STEP, S5KGM1SP_ANA_GAIN_DEFAULT);

	v4l2_ctrl_cluster(2, &sensor->exposure);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5kgm1sp_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  S5KGM1SP_DGTL_GAIN_MIN, S5KGM1SP_DGTL_GAIN_MAX,
			  S5KGM1SP_DGTL_GAIN_STEP, S5KGM1SP_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5kgm1sp_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5kgm1sp_test_pattern_menu) - 1,
				     0, 0, s5kgm1sp_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "control init failed (%d)\n", ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5kgm1sp_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	sensor->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&sensor->mutex);
	return ret;
}

static void s5kgm1sp_free_controls(struct s5kgm1sp *sensor)
{
	v4l2_ctrl_handler_free(sensor->sd.ctrl_handler);
	mutex_destroy(&sensor->mutex);
}

static int s5kgm1sp_get_regulators(struct s5kgm1sp *sensor,
				   struct i2c_client *client)
{
	unsigned int i;

	for (i = 0; i < S5KGM1SP_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = s5kgm1sp_supply_name[i];

	return devm_regulator_bulk_get(&client->dev, S5KGM1SP_NUM_SUPPLIES,
				       sensor->supplies);
}

static int s5kgm1sp_probe(struct i2c_client *client)
{
	struct s5kgm1sp *sensor;
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	int ret;
	u32 val = 0;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(sensor->regmap),
				     "failed to initialize CCI\n");

	ret = s5kgm1sp_get_regulators(sensor, client);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to get regulators\n");

	sensor->reset_gpio =
		devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(sensor->reset_gpio),
				     "failed to get reset GPIO\n");

	sensor->clk = devm_clk_get_optional(&client->dev, NULL);
	if (IS_ERR(sensor->clk))
		return dev_err_probe(&client->dev, PTR_ERR(sensor->clk),
				     "error getting clock\n");

	if (!sensor->clk) {
		dev_dbg(&client->dev,
			"no clock provided, using clock-frequency property\n");
		device_property_read_u32(&client->dev, "clock-frequency", &val);
	} else {
		val = clk_get_rate(sensor->clk);
	}

	if (val != 24000000) {
		dev_err(&client->dev,
			"input clock %u Hz not supported (expected 24 MHz)\n",
			val);
		return -EINVAL;
	}

	endpoint =
		fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
	if (!endpoint) {
		dev_err(&client->dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(&client->dev, "parsing endpoint node failed\n");
		return ret;
	}

	ret = v4l2_link_freq_to_bitmap(&client->dev, ep.link_frequencies,
				       ep.nr_of_link_frequencies,
				       s5kgm1sp_link_freq_menu,
				       ARRAY_SIZE(s5kgm1sp_link_freq_menu),
				       &sensor->link_freq_bitmap);
	if (ret) {
		dev_err(&client->dev, "link frequency not supported\n");
		goto error_endpoint_free;
	}

	if (ep.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(&client->dev, "only 4 data lanes supported, got %u\n",
			ep.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto error_endpoint_free;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &s5kgm1sp_subdev_ops);

	ret = s5kgm1sp_power_on(&client->dev);
	if (ret)
		goto error_endpoint_free;

	ret = s5kgm1sp_identify(sensor);
	if (ret)
		goto error_power_off;

	sensor->cur_mode = &s5kgm1sp_supported_modes[0];

	ret = s5kgm1sp_init_controls(sensor);
	if (ret)
		goto error_power_off;

	sensor->sd.internal_ops = &s5kgm1sp_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto error_handler_free;

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret < 0)
		goto error_media_entity;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);
	v4l2_fwnode_endpoint_free(&ep);

	return 0;

error_media_entity:
	media_entity_cleanup(&sensor->sd.entity);

error_handler_free:
	s5kgm1sp_free_controls(sensor);

error_power_off:
	s5kgm1sp_power_off(&client->dev);

error_endpoint_free:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

static void s5kgm1sp_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kgm1sp *sensor = to_s5kgm1sp(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	s5kgm1sp_free_controls(sensor);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		s5kgm1sp_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id s5kgm1sp_dt_ids[] = {
	{ .compatible = "samsung,s5kgm1sp" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5kgm1sp_dt_ids);

static struct i2c_driver s5kgm1sp_i2c_driver = {
	.driver = {
		.name = "s5kgm1sp",
		.pm = &s5kgm1sp_pm_ops,
		.of_match_table = s5kgm1sp_dt_ids,
	},
	.probe = s5kgm1sp_probe,
	.remove = s5kgm1sp_remove,
};

module_i2c_driver(s5kgm1sp_i2c_driver);

MODULE_AUTHOR("Marc Lainez <marc.lainez@gmail.com>");
MODULE_DESCRIPTION("Samsung S5KGM1SP sensor driver");
MODULE_LICENSE("GPL");
