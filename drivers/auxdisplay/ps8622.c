/*
 * Parade PS8622 eDP/LVDS bridge driver
 *
 * Copyright (C) 2012 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

struct ps8622_platform_data {
	int gpio_slp_n;
	int gpio_rst_n;
};

struct ps8622_dev {
	struct device *dev;
	struct backlight_device *bl;
	struct ps8622_platform_data *pdata;
	struct i2c_client *client;
	struct regulator *v12;
};

/* Brightness scale on the Parade chip */
#define PS8622_MAX_BRIGHTNESS 0xff

static struct ps8622_platform_data *ps8622_parse_dt_pdata(struct device *dev)
{
	struct ps8622_platform_data *pd;

	pd = devm_kzalloc(dev, sizeof(struct ps8622_platform_data), GFP_KERNEL);
	if (!pd) {
		dev_err(dev, "Can't allocate platform data\n");
		return ERR_PTR(-ENOMEM);
	}

	/* Fill platform data with device tree data */
	pd->gpio_slp_n = of_get_named_gpio(dev->of_node, "sleep-gpio", 0);
	pd->gpio_rst_n = of_get_named_gpio(dev->of_node, "reset-gpio", 0);

	return pd;

}

static int ps8622_set(struct i2c_client *client, u8 page, u8 reg, u8 val)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	u8 data[] = {reg, val};

	msg.addr = client->addr + page;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1)
		pr_warn("PS8622 I2C write (0x%02x,0x%02x,0x%02x) failed: %d\n",
			client->addr + page, reg, val, ret);
	return !(ret == 1);
}

static int ps8622_send_config(struct ps8622_dev *ps8622)
{
	struct i2c_client *cl = ps8622->client;
	int err = 0;

	/* wait 20ms after power ON */
	usleep_range(20000, 30000);

	err |= ps8622_set(cl, 0x02, 0xa1, 0x01); /* HPD low */
	/* SW setting */
	err |= ps8622_set(cl, 0x04, 0x14, 0x01); /* [1:0] SW output 1.2V voltage
						  * is lower to 96% */
	/* RCO SS setting */
	err |= ps8622_set(cl, 0x04, 0xe3, 0x20); /* [5:4] = b01 0.5%, b10 1%,
						  * b11 1.5% */
	err |= ps8622_set(cl, 0x04, 0xe2, 0x80); /* [7] RCO SS enable */
	/* RPHY Setting */
	err |= ps8622_set(cl, 0x04, 0x8a, 0x0c); /* [3:2] CDR tune wait cycle
						  * before measure for fine tune
						  * b00: 1us b01: 0.5us b10:2us
						  * b11: 4us */
	err |= ps8622_set(cl, 0x04, 0x89, 0x08); /* [3] RFD always on */
	err |= ps8622_set(cl, 0x04, 0x71, 0x2d); /* CTN lock in/out:
						  * 20000ppm/80000ppm.
						  * Lock out 2 times. */
	/* 2.7G CDR settings */
	err |= ps8622_set(cl, 0x04, 0x7d, 0x07); /* NOF=40LSB for HBR CDR
						  * setting */
	err |= ps8622_set(cl, 0x04, 0x7b, 0x00); /* [1:0] Fmin=+4bands */
	err |= ps8622_set(cl, 0x04, 0x7a, 0xfd); /* [7:5] DCO_FTRNG=+-40% */
	/* 1.62G CDR settings */
	err |= ps8622_set(cl, 0x04, 0xc0, 0x12); /* [5:2]NOF=64LSB [1:0]DCO
						  * scale is 2/5 */
	err |= ps8622_set(cl, 0x04, 0xc1, 0x92); /* Gitune=-37% */
	err |= ps8622_set(cl, 0x04, 0xc2, 0x1c); /* Fbstep=100% */
	err |= ps8622_set(cl, 0x04, 0x32, 0x80); /* [7] LOS signal disable */
	/* RPIO Setting */
	err |= ps8622_set(cl, 0x04, 0x00, 0xb0); /* [7:4] LVDS driver bias
						  * current : 75% (250mV swing)
						  * */
	err |= ps8622_set(cl, 0x04, 0x15, 0x40); /* [7:6] Right-bar GPIO output
						  * strength is 8mA */
	/* EQ Training State Machine Setting */
	err |= ps8622_set(cl, 0x04, 0x54, 0x10); /* RCO calibration start */
	/* Logic, needs more than 10 I2C command */
	err |= ps8622_set(cl, 0x01, 0x02, 0x81); /* [4:0] MAX_LANE_COUNT set to
						  * one lane */
	err |= ps8622_set(cl, 0x01, 0x21, 0x81); /* [4:0] LANE_COUNT_SET set to
						  * one lane (in case no-link
						  * traing conflict) */
	err |= ps8622_set(cl, 0x00, 0x52, 0x20);
	err |= ps8622_set(cl, 0x00, 0xf1, 0x03); /* HPD CP toggle enable */
	err |= ps8622_set(cl, 0x00, 0x62, 0x41);
	err |= ps8622_set(cl, 0x00, 0xf6, 0x01); /* Counter number, add 1ms
						  * counter delay */
	err |= ps8622_set(cl, 0x00, 0x77, 0x06); /* [6]PWM function control by
						  * DPCD0040f[7], default is PWM
						  * block always works. */
	err |= ps8622_set(cl, 0x00, 0x4c, 0x04); /* 04h Adjust VTotal tolerance
						  * to fix the 30Hz no display
						  * issue */
	err |= ps8622_set(cl, 0x01, 0xc0, 0x00); /* DPCD00400='h00, Parade OUI =
						  * 'h001cf8 */
	err |= ps8622_set(cl, 0x01, 0xc1, 0x1c); /* DPCD00401='h1c */
	err |= ps8622_set(cl, 0x01, 0xc2, 0xf8); /* DPCD00402='hf8 */
	err |= ps8622_set(cl, 0x01, 0xc3, 0x44); /* DPCD403~408 = ASCII code
						  * D2SLV5='h4432534c5635 */
	err |= ps8622_set(cl, 0x01, 0xc4, 0x32); /* DPCD404 */
	err |= ps8622_set(cl, 0x01, 0xc5, 0x53); /* DPCD405 */
	err |= ps8622_set(cl, 0x01, 0xc6, 0x4c); /* DPCD406 */
	err |= ps8622_set(cl, 0x01, 0xc7, 0x56); /* DPCD407 */
	err |= ps8622_set(cl, 0x01, 0xc8, 0x35); /* DPCD408 */
	err |= ps8622_set(cl, 0x01, 0xca, 0x01); /* DPCD40A, Initial Code major
						  * revision '01' */
	err |= ps8622_set(cl, 0x01, 0xcb, 0x05); /* DPCD40B, Initial Code minor
						  * revision '05' */
	err |= ps8622_set(cl, 0x01, 0xa5, 0xa0); /* DPCD720, internal PWM */
	err |= ps8622_set(cl, 0x01, 0xa7, ps8622->bl->props.brightness);
						 /* FFh for 100% brightness,
						  *  0h for 0% brightness */
	err |= ps8622_set(cl, 0x01, 0xcc, 0x13); /* Set LVDS output as 6bit-VESA
						  * mapping, single LVDS channel
						  * */
	err |= ps8622_set(cl, 0x02, 0xb1, 0x20); /* Enable SSC set by register
						  * */
	err |= ps8622_set(cl, 0x04, 0x10, 0x16); /* Set SSC enabled and +/-1%
						  * central spreading */
	/* Logic end */
	err |= ps8622_set(cl, 0x04, 0x59, 0x60); /* MPU Clock source: LC => RCO
						  * */
	err |= ps8622_set(cl, 0x04, 0x54, 0x14); /* LC -> RCO */
	err |= ps8622_set(cl, 0x02, 0xa1, 0x91); /* HPD high */

	return err ? -EIO : 0;
}

static int ps8622_power_up(struct ps8622_dev *ps8622)
{
	struct ps8622_platform_data *pd = ps8622->pdata;

	if (ps8622->v12)
		regulator_enable(ps8622->v12);

	if (pd->gpio_slp_n > 0)
		gpio_set_value(pd->gpio_slp_n, 1);

	if (pd->gpio_rst_n > 0) {
		gpio_set_value(pd->gpio_rst_n, 0);
		udelay(10);
		gpio_set_value(pd->gpio_rst_n, 1);
	}

	return ps8622_send_config(ps8622);
}

static int ps8622_power_down(struct ps8622_dev *ps8622)
{
	struct ps8622_platform_data *pd = ps8622->pdata;

	if (pd->gpio_rst_n > 0)
		gpio_set_value(pd->gpio_rst_n, 1);

	if (pd->gpio_slp_n > 0)
		gpio_set_value(pd->gpio_slp_n, 0);

	if (ps8622->v12)
		regulator_disable(ps8622->v12);

	return 0;
}

int ps8622_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ps8622_dev *ps8622 = i2c_get_clientdata(client);

	return ps8622_power_down(ps8622);
}

int ps8622_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ps8622_dev *ps8622 = i2c_get_clientdata(client);

	return ps8622_power_up(ps8622);
}

static int ps8622_backlight_update(struct backlight_device *bl)
{
	struct ps8622_dev *ps8622 = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		brightness = 0;

	return ps8622_set(ps8622->client, 0x01, 0xa7, brightness);
}

static int ps8622_backlight_get(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops ps8622_backlight_ops = {
	.update_status	= ps8622_backlight_update,
	.get_brightness	= ps8622_backlight_get,
};

int ps8622_probe(struct i2c_client *client, const struct i2c_device_id *device)
{
	struct device *dev = &client->dev;
	struct ps8622_platform_data *pd = dev->platform_data;
	struct ps8622_dev *ps8622;
	int ret;

	ps8622 = devm_kzalloc(dev, sizeof(struct ps8622_dev), GFP_KERNEL);
	if (!ps8622) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}
	ps8622->dev = dev;
	ps8622->client = client;
	i2c_set_clientdata(client, ps8622);

	if (dev->of_node) {
		pd = ps8622_parse_dt_pdata(dev);
		if (IS_ERR(pd)) {
			ret = PTR_ERR(pd);
			goto err;
		}
	}
	if (!pd) {
		ret = -ENODEV;
		dev_warn(dev, "No platform data found\n");
		goto err;
	}
	ps8622->pdata = pd;

	ps8622->v12 = regulator_get(dev, "vdd_bridge");
	if (IS_ERR(ps8622->v12)) {
		dev_warn(dev, "no 1.2v regulator found for PS8622\n");
		ps8622->v12 = NULL;
	}

	if (pd->gpio_slp_n > 0) {
		ret = gpio_request_one(pd->gpio_slp_n, GPIOF_OUT_INIT_HIGH,
					"PS8622_SLP_N");
		if (ret)
			goto err;
	}
	if (pd->gpio_rst_n > 0) {
		/*
		 * Request the reset pin low to avoid the bridge being
		 * initialized prematurely
		 */
		ret = gpio_request_one(pd->gpio_rst_n, GPIOF_OUT_INIT_LOW,
					"PS8622_RST_N");
		if (ret)
			goto err;
	}

	/* create a backlight interface for the bridge */
	ps8622->bl = backlight_device_register("ps8622-backlight", dev, ps8622,
					&ps8622_backlight_ops, NULL);
	if (IS_ERR(ps8622->bl)) {
		dev_err(dev, "failed to register backlight\n");
		ret = PTR_ERR(ps8622->bl);
		ps8622->bl = NULL;
		goto err;
	}
	ps8622->bl->props.max_brightness = PS8622_MAX_BRIGHTNESS;
	ps8622->bl->props.brightness = PS8622_MAX_BRIGHTNESS;

	ret = ps8622_power_up(ps8622);
	if (ret)
		goto err;

	return 0;

err:
	if (ps8622->bl)
		backlight_device_unregister(ps8622->bl);
	if (ps8622->v12)
		regulator_put(ps8622->v12);
	dev_err(dev, "device probe failed : %d\n", ret);
	return ret;
}

int ps8622_remove(struct i2c_client *client)
{
	struct ps8622_dev *ps8622 = i2c_get_clientdata(client);

	if (ps8622->bl)
		backlight_device_unregister(ps8622->bl);
	if (ps8622->v12)
		regulator_put(ps8622->v12);
	return 0;
}

static const struct i2c_device_id ps8622_ids[] = {
	{ "ps8622", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ps8622_ids);

static SIMPLE_DEV_PM_OPS(ps8622_pm_ops, ps8622_suspend, ps8622_resume);

static struct i2c_driver ps8622_driver = {
	.id_table	= ps8622_ids,
	.probe          = ps8622_probe,
	.remove         = ps8622_remove,
	.driver		= {
		.name	= "ps8622",
		.owner	= THIS_MODULE,
		.pm	= &ps8622_pm_ops,
	},
};
module_i2c_driver(ps8622_driver);
