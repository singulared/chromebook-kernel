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

#include "drmP.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"

struct ps8622_bridge {
	struct drm_bridge bridge;

	struct i2c_client *client;
	struct regulator *v12;
	struct backlight_device *bl;

	int gpio_slp_n;
	int gpio_rst_n;

	bool enabled;
};

/* Brightness scale on the Parade chip */
#define PS8622_MAX_BRIGHTNESS 0xff

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

static int ps8622_send_config(struct ps8622_bridge *bridge)
{
	struct i2c_client *cl = bridge->client;
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
	err |= ps8622_set(cl, 0x01, 0xa7, bridge->bl->props.brightness);
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

static void ps8622_power_up(struct ps8622_bridge *bridge)
{
	int ret;

	if (bridge->enabled)
		return;

	if (bridge->v12)
		regulator_enable(bridge->v12);

	if (gpio_is_valid(bridge->gpio_slp_n))
		gpio_set_value(bridge->gpio_slp_n, 1);

	if (gpio_is_valid(bridge->gpio_rst_n)) {
		gpio_set_value(bridge->gpio_rst_n, 0);
		udelay(10);
		gpio_set_value(bridge->gpio_rst_n, 1);
	}

	ret = ps8622_send_config(bridge);
	if (ret)
		DRM_ERROR("Failed to send config to bridge (%d)\n", ret);

	bridge->enabled = true;
}

static void ps8622_power_down(struct ps8622_bridge *bridge)
{
	if (!bridge->enabled)
		return;

	bridge->enabled = false;

	if (gpio_is_valid(bridge->gpio_rst_n))
		gpio_set_value(bridge->gpio_rst_n, 1);

	if (gpio_is_valid(bridge->gpio_slp_n))
		gpio_set_value(bridge->gpio_slp_n, 0);

	if (bridge->v12)
		regulator_disable(bridge->v12);
}

static int ps8622_backlight_update(struct backlight_device *bl)
{
	struct ps8622_bridge *bridge = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		brightness = 0;

	return ps8622_set(bridge->client, 0x01, 0xa7, brightness);
}

static int ps8622_backlight_get(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops ps8622_backlight_ops = {
	.update_status	= ps8622_backlight_update,
	.get_brightness	= ps8622_backlight_get,
};

static void ps8622_dpms(struct drm_bridge *dbridge, int mode)
{
	struct ps8622_bridge *bridge = (struct ps8622_bridge *)dbridge;

	if (mode == DRM_MODE_DPMS_ON)
		ps8622_power_up(bridge);
	else
		ps8622_power_down(bridge);
}

static void ps8622_prepare(struct drm_bridge *dbridge)
{
	struct ps8622_bridge *bridge = (struct ps8622_bridge *)dbridge;

	ps8622_power_up(bridge);
}

struct drm_bridge_helper_funcs ps8622_bridge_helper_funcs = {
	.dpms = ps8622_dpms,
	.prepare = ps8622_prepare,
};

void ps8622_destroy(struct drm_bridge *dbridge)
{
	struct ps8622_bridge *bridge = (struct ps8622_bridge *)dbridge;

	drm_bridge_cleanup(dbridge);
	if (bridge->bl)
		backlight_device_unregister(bridge->bl);
	if (bridge->v12)
		regulator_put(bridge->v12);
	if (gpio_is_valid(bridge->gpio_slp_n))
		gpio_free(bridge->gpio_slp_n);
	if (gpio_is_valid(bridge->gpio_rst_n))
		gpio_free(bridge->gpio_rst_n);
}

struct drm_bridge_funcs ps8622_bridge_funcs = {
	.destroy = ps8622_destroy,
};

int ps8622_init(struct drm_device *dev, struct i2c_client *client,
		struct device_node *node)
{
	int ret;
	struct ps8622_bridge *bridge;

	bridge = devm_kzalloc(dev->dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge) {
		DRM_ERROR("could not allocate memory\n");
		return -ENOMEM;
	}

	bridge->client = client;
	bridge->v12 = regulator_get(&client->dev, "vdd_bridge");
	if (IS_ERR(bridge->v12)) {
		DRM_ERROR("no 1.2v regulator found for PS8622\n");
		bridge->v12 = NULL;
	}

	bridge->gpio_slp_n = of_get_named_gpio(node, "sleep-gpio", 0);
	if (gpio_is_valid(bridge->gpio_slp_n)) {
		ret = gpio_request_one(bridge->gpio_slp_n, GPIOF_OUT_INIT_HIGH,
					"PS8622_SLP_N");
		if (ret)
			goto err_reg;
	}

	bridge->gpio_rst_n = of_get_named_gpio(node, "reset-gpio", 0);
	if (gpio_is_valid(bridge->gpio_rst_n)) {
		/*
		 * Request the reset pin low to avoid the bridge being
		 * initialized prematurely
		 */
		ret = gpio_request_one(bridge->gpio_rst_n, GPIOF_OUT_INIT_LOW,
					"PS8622_RST_N");
		if (ret)
			goto err_gpio;
	}

	bridge->bl = backlight_device_register("ps8622-backlight", dev->dev,
			bridge, &ps8622_backlight_ops, NULL);
	if (IS_ERR(bridge->bl)) {
		DRM_ERROR("failed to register backlight\n");
		ret = PTR_ERR(bridge->bl);
		bridge->bl = NULL;
		goto err;
	}
	bridge->bl->props.max_brightness = PS8622_MAX_BRIGHTNESS;
	bridge->bl->props.brightness = PS8622_MAX_BRIGHTNESS;

	ret = drm_bridge_init(dev, (struct drm_bridge *)bridge,
			&ps8622_bridge_funcs);
	if (ret) {
		DRM_ERROR("Failed to initialize bridge with drm\n");
		goto err;
	}
	drm_bridge_helper_add((struct drm_bridge *)bridge,
			&ps8622_bridge_helper_funcs);
	bridge->bridge.connector_type = DRM_MODE_CONNECTOR_eDP;
	bridge->enabled = false;

	return 0;

err:
	if (bridge->bl)
		backlight_device_unregister(bridge->bl);
	if (gpio_is_valid(bridge->gpio_rst_n))
		gpio_free(bridge->gpio_rst_n);
err_gpio:
	if (gpio_is_valid(bridge->gpio_slp_n))
		gpio_free(bridge->gpio_slp_n);
err_reg:
	if (bridge->v12)
		regulator_put(bridge->v12);
	DRM_ERROR("device probe failed : %d\n", ret);
	return ret;
}
