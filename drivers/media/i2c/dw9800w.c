// SPDX-License-Identifier: GPL-2.0
/*
 * Dongwoon DW9800W voice coil motor driver
 *
 * Based on the upstream dw9714 driver (Copyright (c) 2015-2017 Intel) and
 * the MediaTek DW9800W driver in the LineageOS Mediatek tree. Stripped to
 * the minimum needed to drive focus from V4L2_CID_FOCUS_ABSOLUTE.
 *
 * Wire protocol (register-addressed, distinct from DW9714):
 *   reg 0x02: control byte (PD / ring enable / standby)
 *   reg 0x03: 16-bit big-endian DAC position (10 bits valid, 0..1023)
 *   reg 0x06: SAC operating mode
 *   reg 0x07: SAC timing (T_SRC / T_DIV defaults)
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define DW9800W_NAME		"dw9800w"
#define DW9800W_MAX_FOCUS_POS	1023
#define DW9800W_FOCUS_STEPS	1

#define DW9800W_REG_CTRL	0x02
#define DW9800W_REG_POS		0x03
#define DW9800W_REG_MODE	0x06
#define DW9800W_REG_TIMING	0x07

#define DW9800W_CTRL_PD		0x01
#define DW9800W_CTRL_ACTIVE	0x00
#define DW9800W_CTRL_RING_EN	0x02
#define DW9800W_CTRL_STANDBY	0x20

#define DW9800W_MODE_SAC2	0x40
#define DW9800W_TIMING_DEFAULT	0x60

#define DW9800W_POWER_DELAY_US	10000

struct dw9800w_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vcc;
};

static inline struct dw9800w_device *ctrl_to_dw9800w(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9800w_device, ctrls);
}

static inline struct dw9800w_device *sd_to_dw9800w(struct v4l2_subdev *sd)
{
	return container_of(sd, struct dw9800w_device, sd);
}

static int dw9800w_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);

	if (ret < 0)
		dev_err(&client->dev,
			"failed to write reg 0x%02x = 0x%02x: %d\n",
			reg, val, ret);
	return ret;
}

static int dw9800w_set_pos(struct dw9800w_device *d, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&d->sd);
	int ret;

	val &= DW9800W_MAX_FOCUS_POS;
	ret = i2c_smbus_write_word_data(client, DW9800W_REG_POS, swab16(val));
	if (ret < 0)
		dev_err(&client->dev, "set position %u failed: %d\n", val, ret);
	return ret;
}

static int dw9800w_init_chip(struct dw9800w_device *d)
{
	struct i2c_client *client = v4l2_get_subdevdata(&d->sd);
	int ret;

	usleep_range(200, 300);
	ret = dw9800w_write_reg(client, DW9800W_REG_CTRL, DW9800W_CTRL_PD);
	if (ret)
		return ret;
	usleep_range(100, 200);
	ret = dw9800w_write_reg(client, DW9800W_REG_CTRL, DW9800W_CTRL_ACTIVE);
	if (ret)
		return ret;
	usleep_range(100, 200);
	ret = dw9800w_write_reg(client, DW9800W_REG_CTRL,
				DW9800W_CTRL_RING_EN);
	if (ret)
		return ret;
	ret = dw9800w_write_reg(client, DW9800W_REG_MODE, DW9800W_MODE_SAC2);
	if (ret)
		return ret;
	ret = dw9800w_write_reg(client, DW9800W_REG_TIMING,
				DW9800W_TIMING_DEFAULT);
	if (ret)
		return ret;
	usleep_range(100, 200);
	return 0;
}

static int dw9800w_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dw9800w_device *d = ctrl_to_dw9800w(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		return dw9800w_set_pos(d, ctrl->val);
	return -EINVAL;
}

static const struct v4l2_ctrl_ops dw9800w_ctrl_ops = {
	.s_ctrl = dw9800w_set_ctrl,
};

static const struct v4l2_subdev_ops dw9800w_subdev_ops = { };

static int dw9800w_init_controls(struct dw9800w_device *d)
{
	struct v4l2_ctrl_handler *hdl = &d->ctrls;
	const struct v4l2_ctrl_ops *ops = &dw9800w_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);
	d->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
				     0, DW9800W_MAX_FOCUS_POS,
				     DW9800W_FOCUS_STEPS, 0);
	if (hdl->error) {
		dev_err(d->sd.dev, "control init error: %d\n", hdl->error);
		return hdl->error;
	}
	d->sd.ctrl_handler = hdl;
	return 0;
}

static int dw9800w_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9800w_device *d;
	int ret;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(d->vcc))
		return dev_err_probe(dev, PTR_ERR(d->vcc),
				     "failed to get vcc regulator\n");

	ret = regulator_enable(d->vcc);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to enable vcc regulator\n");

	usleep_range(DW9800W_POWER_DELAY_US,
		     DW9800W_POWER_DELAY_US + 100);

	v4l2_i2c_subdev_init(&d->sd, client, &dw9800w_subdev_ops);
	d->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = dw9800w_init_controls(d);
	if (ret)
		goto err_disable;

	ret = media_entity_pads_init(&d->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_free_ctrl;
	d->sd.entity.function = MEDIA_ENT_F_LENS;

	ret = dw9800w_init_chip(d);
	if (ret) {
		dev_err(dev, "chip init failed: %d\n", ret);
		goto err_entity;
	}

	ret = v4l2_async_register_subdev(&d->sd);
	if (ret < 0) {
		dev_err(dev, "v4l2 async register failed: %d\n", ret);
		goto err_entity;
	}

	dev_info(dev, "DW9800W VCM ready (focus 0..%d)\n",
		 DW9800W_MAX_FOCUS_POS);
	return 0;

err_entity:
	media_entity_cleanup(&d->sd.entity);
err_free_ctrl:
	v4l2_ctrl_handler_free(&d->ctrls);
err_disable:
	regulator_disable(d->vcc);
	return ret;
}

static void dw9800w_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9800w_device *d = sd_to_dw9800w(sd);

	v4l2_async_unregister_subdev(sd);
	dw9800w_write_reg(client, DW9800W_REG_CTRL, DW9800W_CTRL_STANDBY);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&d->ctrls);
	regulator_disable(d->vcc);
}

static const struct i2c_device_id dw9800w_id_table[] = {
	{ DW9800W_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dw9800w_id_table);

static const struct of_device_id dw9800w_of_table[] = {
	{ .compatible = "dongwoon,dw9800w" },
	{ }
};
MODULE_DEVICE_TABLE(of, dw9800w_of_table);

static struct i2c_driver dw9800w_i2c_driver = {
	.driver = {
		.name = DW9800W_NAME,
		.of_match_table = dw9800w_of_table,
	},
	.probe = dw9800w_probe,
	.remove = dw9800w_remove,
	.id_table = dw9800w_id_table,
};

module_i2c_driver(dw9800w_i2c_driver);

MODULE_DESCRIPTION("Dongwoon DW9800W VCM driver");
MODULE_LICENSE("GPL v2");
