// SPDX-License-Identifier: GPL-2.0
/*
 * ON Semiconductor LC898217XC voice coil motor driver
 *
 * Wire protocol, from the Qualcomm actuator configuration the Fairphone 3
 * ships for its IMX363 rear camera module:
 *   reg 0xe0: write 0x01 to leave standby, then wait 10 ms
 *   reg 0x84: 16-bit big-endian target code, 11 bits
 *
 * The lens focuses closer as the code falls. On that module it focuses at
 * infinity at code 860 and at its closest at 176, the range Qualcomm's
 * tuning drives it over, so focus positions 0 (infinity) to 1023
 * (closest) map onto it.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define LC898217XC_NAME		"lc898217xc"
#define LC898217XC_MAX_FOCUS_POS	1023
#define LC898217XC_CODE_INFINITY	860
#define LC898217XC_CODE_MACRO		176

#define LC898217XC_REG_STANDBY	0xe0
#define LC898217XC_REG_TARGET	0x84

#define LC898217XC_STANDBY_OFF	0x01

#define LC898217XC_POWER_DELAY_US	10000
#define LC898217XC_WAKE_DELAY_US	10000

struct lc898217xc_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct regulator *vdd;
	struct i2c_client *client;
};

static inline struct lc898217xc_device *ctrl_to_lc(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct lc898217xc_device, ctrls);
}

static int lc898217xc_set_pos(struct lc898217xc_device *d, u16 val)
{
	u16 code = LC898217XC_CODE_INFINITY -
		   val * (LC898217XC_CODE_INFINITY - LC898217XC_CODE_MACRO) /
		   LC898217XC_MAX_FOCUS_POS;
	int ret;

	ret = i2c_smbus_write_word_data(d->client, LC898217XC_REG_TARGET, swab16(code));
	if (ret < 0)
		dev_err(&d->client->dev, "set position %u failed: %d\n", val, ret);

	return ret;
}

static int lc898217xc_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lc898217xc_device *d = ctrl_to_lc(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		return lc898217xc_set_pos(d, ctrl->val);
	return -EINVAL;
}

static const struct v4l2_ctrl_ops lc898217xc_ctrl_ops = {
	.s_ctrl = lc898217xc_set_ctrl,
};

static const struct v4l2_subdev_ops lc898217xc_subdev_ops = { };

static int lc898217xc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lc898217xc_device *d;
	int ret;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->client = client;
	d->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(d->vdd))
		return dev_err_probe(dev, PTR_ERR(d->vdd),
				     "failed to get vdd regulator\n");

	ret = regulator_enable(d->vdd);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vdd regulator\n");
	usleep_range(LC898217XC_POWER_DELAY_US, LC898217XC_POWER_DELAY_US + 100);

	ret = i2c_smbus_write_byte_data(client, LC898217XC_REG_STANDBY,
					LC898217XC_STANDBY_OFF);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to wake the actuator\n");
		goto err_disable;
	}
	usleep_range(LC898217XC_WAKE_DELAY_US, LC898217XC_WAKE_DELAY_US + 100);

	v4l2_i2c_subdev_init(&d->sd, client, &lc898217xc_subdev_ops);
	d->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	v4l2_ctrl_handler_init(&d->ctrls, 1);
	v4l2_ctrl_new_std(&d->ctrls, &lc898217xc_ctrl_ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, LC898217XC_MAX_FOCUS_POS, 1, 0);
	if (d->ctrls.error) {
		ret = d->ctrls.error;
		goto err_free_ctrl;
	}
	d->sd.ctrl_handler = &d->ctrls;

	ret = media_entity_pads_init(&d->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_free_ctrl;
	d->sd.entity.function = MEDIA_ENT_F_LENS;

	ret = v4l2_async_register_subdev(&d->sd);
	if (ret < 0)
		goto err_entity;

	return 0;

err_entity:
	media_entity_cleanup(&d->sd.entity);
err_free_ctrl:
	v4l2_ctrl_handler_free(&d->ctrls);
err_disable:
	regulator_disable(d->vdd);
	return ret;
}

static void lc898217xc_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898217xc_device *d = container_of(sd, struct lc898217xc_device, sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&d->ctrls);
	regulator_disable(d->vdd);
}

static const struct i2c_device_id lc898217xc_id_table[] = {
	{ LC898217XC_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lc898217xc_id_table);

static const struct of_device_id lc898217xc_of_table[] = {
	{ .compatible = "onnn,lc898217xc" },
	{ }
};
MODULE_DEVICE_TABLE(of, lc898217xc_of_table);

static struct i2c_driver lc898217xc_i2c_driver = {
	.driver = {
		.name = LC898217XC_NAME,
		.of_match_table = lc898217xc_of_table,
	},
	.probe = lc898217xc_probe,
	.remove = lc898217xc_remove,
	.id_table = lc898217xc_id_table,
};

module_i2c_driver(lc898217xc_i2c_driver);

MODULE_DESCRIPTION("ON Semiconductor LC898217XC VCM driver");
MODULE_LICENSE("GPL");
