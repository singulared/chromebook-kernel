/*
 * NXP PTN3460 DP/LVDS bridge driver
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
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "drmP.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"

#include "bridge/ptn3460.h"

#define PTN3460_EDID_EMULATION_ADDR		0x84
#define PTN3460_EDID_ENABLE_EMULATION		0
#define PTN3460_EDID_EMULATION_SELECTION	1

struct ptn3460_bridge {
	struct drm_bridge bridge;

	struct i2c_client *client;
	int gpio_pd_n;
	int gpio_rst_n;
	u32 edid_emulation;

	bool enabled;
};

static int ptn3460_write_byte(struct ptn3460_bridge *bridge, char addr,
		char val)
{
	int ret;
	char buf[2];

	buf[0] = addr;
	buf[1] = val;

	ret = i2c_master_send(bridge->client, buf, ARRAY_SIZE(buf));
	if (ret <= 0) {
		DRM_ERROR("Failed to send i2c command, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int ptn3460_select_edid(struct ptn3460_bridge *bridge)
{
	int ret;
	char val;

	val = 1 << PTN3460_EDID_ENABLE_EMULATION |
		bridge->edid_emulation << PTN3460_EDID_EMULATION_SELECTION;

	ret = ptn3460_write_byte(bridge, PTN3460_EDID_EMULATION_ADDR, val);
	if (ret) {
		DRM_ERROR("Failed to write edid value, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static void ptn3460_power_up(struct ptn3460_bridge *bridge)
{
	int ret;

	if (bridge->enabled)
		return;

	if (gpio_is_valid(bridge->gpio_pd_n))
		gpio_set_value(bridge->gpio_pd_n, 1);

	if (gpio_is_valid(bridge->gpio_rst_n)) {
		gpio_set_value(bridge->gpio_rst_n, 0);
		udelay(10);
		gpio_set_value(bridge->gpio_rst_n, 1);
	}

	/*
	 * There's a bug in the PTN chip where it falsely asserts hotplug before
	 * it is fully functional. We're forced to wait for the maximum start up
	 * time specified in the chip's datasheet to make sure we're really up.
	 */
	msleep(90);

	ret = ptn3460_select_edid(bridge);
	if (ret)
		DRM_ERROR("Select edid failed ret=%d\n", ret);

	bridge->enabled = true;
}

static void ptn3460_power_down(struct ptn3460_bridge *bridge)
{
	if (!bridge->enabled)
		return;

	bridge->enabled = false;

	if (gpio_is_valid(bridge->gpio_rst_n))
		gpio_set_value(bridge->gpio_rst_n, 1);

	if (gpio_is_valid(bridge->gpio_pd_n))
		gpio_set_value(bridge->gpio_pd_n, 0);
}

static void ptn3460_dpms(struct drm_bridge *dbridge, int mode)
{
	struct ptn3460_bridge *bridge = (struct ptn3460_bridge *)dbridge;

	if (mode == DRM_MODE_DPMS_ON)
		ptn3460_power_up(bridge);
	else
		ptn3460_power_down(bridge);
}

static void ptn3460_prepare(struct drm_bridge *dbridge)
{
	struct ptn3460_bridge *bridge = (struct ptn3460_bridge *)dbridge;

	ptn3460_power_up(bridge);
}

struct drm_bridge_helper_funcs ptn3460_bridge_helper_funcs = {
	.dpms = ptn3460_dpms,
	.prepare = ptn3460_prepare,
};

void ptn3460_destroy(struct drm_bridge *dbridge)
{
	struct ptn3460_bridge *bridge = (struct ptn3460_bridge *)dbridge;

	drm_bridge_cleanup(dbridge);
	if (gpio_is_valid(bridge->gpio_pd_n))
		gpio_free(bridge->gpio_pd_n);
	if (gpio_is_valid(bridge->gpio_rst_n))
		gpio_free(bridge->gpio_rst_n);
	/* Nothing else to free, we've got devm allocated memory */
}

struct drm_bridge_funcs ptn3460_bridge_funcs = {
	.destroy = ptn3460_destroy,
};

int ptn3460_init(struct drm_device *dev, struct i2c_client *client,
			struct device_node *node)
{
	int ret;
	struct ptn3460_bridge *bridge;

	bridge = devm_kzalloc(dev->dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge) {
		DRM_ERROR("Failed to allocate bridge\n");
		return -ENOMEM;
	}

	bridge->client = client;
	bridge->gpio_pd_n = of_get_named_gpio(node, "powerdown-gpio", 0);
	if (gpio_is_valid(bridge->gpio_pd_n)) {
		ret = gpio_request_one(bridge->gpio_pd_n,
				GPIOF_OUT_INIT_HIGH, "PTN3460_PD_N");
		if (ret) {
			DRM_ERROR("Request powerdown-gpio failed (%d)\n", ret);
			return ret;
		}
	}

	bridge->gpio_rst_n = of_get_named_gpio(node, "reset-gpio", 0);
	if (gpio_is_valid(bridge->gpio_rst_n)) {
		/*
		 * Request the reset pin low to avoid the bridge being
		 * initialized prematurely
		 */
		ret = gpio_request_one(bridge->gpio_rst_n, GPIOF_OUT_INIT_LOW,
					"PTN3460_RST_N");
		if (ret) {
			DRM_ERROR("Request reset-gpio failed (%d)\n", ret);
			gpio_free(bridge->gpio_pd_n);
			return ret;
		}
	}

	ret = of_property_read_u32(node, "edid-emulation",
			&bridge->edid_emulation);
	if (ret) {
		DRM_ERROR("Can't read edid emulation value\n");
		goto err;
	}

	ret = drm_bridge_init(dev, (struct drm_bridge *)bridge,
			&ptn3460_bridge_funcs);
	if (ret) {
		DRM_ERROR("Failed to initialize bridge with drm\n");
		goto err;
	}
	drm_bridge_helper_add((struct drm_bridge *)bridge,
			&ptn3460_bridge_helper_funcs);
	bridge->bridge.connector_type = DRM_MODE_CONNECTOR_eDP;

	return 0;

err:
	if (gpio_is_valid(bridge->gpio_pd_n))
		gpio_free(bridge->gpio_pd_n);
	if (gpio_is_valid(bridge->gpio_rst_n))
		gpio_free(bridge->gpio_rst_n);
	return ret;
}
