/*
 * NXP PTN3460 DP/LVDS bridge driver
 *
 * Copyright (C) 2014 Google, Inc.
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
#include <linux/of_i2c.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "drmP.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"

#include "bridge/ptn3460.h"

struct ptn3460_bridge {
	struct drm_connector connector;
	struct i2c_client *client;
	struct drm_encoder *encoder;
	struct drm_bridge bridge;
	struct drm_display_mode mode;
	int gpio_pd_n;
	int gpio_rst_n;
	bool enabled;
};

static void ptn3460_pre_enable(struct drm_bridge *bridge)
{
	struct ptn3460_bridge *ptn_bridge = bridge->driver_private;

	DRM_DEBUG_KMS("\n");

	if (ptn_bridge->enabled)
		return;

	if (gpio_is_valid(ptn_bridge->gpio_pd_n))
		gpio_set_value(ptn_bridge->gpio_pd_n, 1);

	if (gpio_is_valid(ptn_bridge->gpio_rst_n)) {
		gpio_set_value(ptn_bridge->gpio_rst_n, 0);
		udelay(10);
		gpio_set_value(ptn_bridge->gpio_rst_n, 1);
	}

	/*
	 * There's a bug in the PTN chip where it falsely asserts hotplug before
	 * it is fully functional. We're forced to wait for the maximum start up
	 * time specified in the chip's datasheet to make sure we're really up.
	 */
	msleep(90);

	ptn_bridge->enabled = true;
}

static void ptn3460_enable(struct drm_bridge *bridge)
{
	DRM_DEBUG_KMS("\n");
}

static void ptn3460_disable(struct drm_bridge *bridge)
{
	struct ptn3460_bridge *ptn_bridge = bridge->driver_private;

	if (!ptn_bridge->enabled)
		return;

	DRM_DEBUG_KMS("\n");

	ptn_bridge->enabled = false;

	if (gpio_is_valid(ptn_bridge->gpio_rst_n))
		gpio_set_value(ptn_bridge->gpio_rst_n, 1);

	if (gpio_is_valid(ptn_bridge->gpio_pd_n))
		gpio_set_value(ptn_bridge->gpio_pd_n, 0);
}

static void ptn3460_post_disable(struct drm_bridge *bridge)
{
	DRM_DEBUG_KMS("\n");
}

static void ptn3460_bridge_destroy(struct drm_bridge *bridge)
{
	struct ptn3460_bridge *ptn_bridge = bridge->driver_private;

	drm_bridge_cleanup(bridge);
	gpio_free(ptn_bridge->gpio_pd_n);
	gpio_free(ptn_bridge->gpio_rst_n);
	put_device(&ptn_bridge->client->dev);
	/* Nothing else to free, we've got devm allocated memory */
}

static const struct drm_bridge_funcs ptn3460_bridge_funcs = {
	.pre_enable = ptn3460_pre_enable,
	.enable = ptn3460_enable,
	.disable = ptn3460_disable,
	.post_disable = ptn3460_post_disable,
	.destroy = ptn3460_bridge_destroy,
};

static int ptn3460_get_modes(struct drm_connector *connector)
{
	struct ptn3460_bridge *ptn_bridge;
	struct drm_display_mode *new_mode;

	ptn_bridge = container_of(connector, struct ptn3460_bridge, connector);

	new_mode = drm_mode_duplicate(connector->dev, &ptn_bridge->mode);
	if (!new_mode) {
		DRM_ERROR("Failed to duplicate ptn_bridge mode!\n");
		return 0;
	}

	drm_mode_probed_add(connector, new_mode);
	return 1;
}

static int ptn3460_mode_valid(struct drm_connector *connector,
		struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_encoder *ptn3460_best_encoder(struct drm_connector *connector)
{
	struct ptn3460_bridge *ptn_bridge;

	ptn_bridge = container_of(connector, struct ptn3460_bridge, connector);

	return ptn_bridge->encoder;
}

static const struct drm_connector_helper_funcs
			ptn3460_connector_helper_funcs = {
	.get_modes = ptn3460_get_modes,
	.mode_valid = ptn3460_mode_valid,
	.best_encoder = ptn3460_best_encoder,
};

static enum drm_connector_status ptn3460_detect(struct drm_connector *connector,
		bool force)
{
	return connector_status_connected;
}

static void ptn3460_connector_destroy(struct drm_connector *connector)
{
	struct ptn3460_bridge *ptn_bridge;

	ptn_bridge = container_of(connector, struct ptn3460_bridge, connector);

	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs ptn3460_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = ptn3460_detect,
	.destroy = ptn3460_connector_destroy,
};

int ptn3460_init(struct drm_encoder *encoder)
{
	int ret;
	struct device_node *node;
	struct i2c_client *client;
	struct drm_device *dev = encoder->dev;
	struct drm_bridge *bridge;
	struct ptn3460_bridge *ptn_bridge;

	node = of_find_compatible_node(NULL, NULL, "nxp,ptn3460");
	if (!node)
		return -ENODEV;

	client = of_find_i2c_device_by_node(node);
	of_node_put(node);
	if (!client)
		return -ENODEV;

	ptn_bridge = devm_kzalloc(dev->dev, sizeof(*ptn_bridge), GFP_KERNEL);
	if (!ptn_bridge) {
		DRM_ERROR("Failed to allocate ptn bridge\n");
		ret = -ENOMEM;
		goto err_client;
	}

	ptn_bridge->client = client;
	ptn_bridge->encoder = encoder;
	bridge = &ptn_bridge->bridge;
	ptn_bridge->gpio_pd_n = of_get_named_gpio(node, "powerdown-gpio", 0);
	if (gpio_is_valid(ptn_bridge->gpio_pd_n)) {
		ret = devm_gpio_request_one(&client->dev, ptn_bridge->gpio_pd_n,
				GPIOF_OUT_INIT_HIGH, "PTN3460_PD_N");
		if (ret) {
			DRM_ERROR("Request powerdown-gpio failed (%d)\n", ret);
			goto err_client;
		}
	}

	ptn_bridge->gpio_rst_n = of_get_named_gpio(node, "reset-gpio", 0);
	if (gpio_is_valid(ptn_bridge->gpio_rst_n)) {
		/*
		 * Request the reset pin low to avoid the bridge being
		 * initialized prematurely
		 */
		ret = devm_gpio_request_one(&client->dev,
				ptn_bridge->gpio_rst_n, GPIOF_OUT_INIT_LOW,
				"PTN3460_RST_N");
		if (ret) {
			DRM_ERROR("Request reset-gpio failed (%d)\n", ret);
			gpio_free(ptn_bridge->gpio_pd_n);
			goto err_client;
		}
	}

	ret = of_get_drm_display_mode(node, &ptn_bridge->mode, 0);
	if (ret) {
		DRM_ERROR("Failed to get display mode, ret=%d\n", ret);
		goto err_gpio;
	}

	ret = drm_bridge_init(dev, bridge, &ptn3460_bridge_funcs);
	if (ret) {
		DRM_ERROR("Failed to initialize bridge with drm\n");
		goto err_gpio;
	}

	bridge->driver_private = ptn_bridge;
	encoder->bridge = bridge;

	ret = drm_connector_init(dev, &ptn_bridge->connector,
			&ptn3460_connector_funcs, DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		goto err_bridge;
	}
	drm_connector_helper_add(&ptn_bridge->connector,
			&ptn3460_connector_helper_funcs);
	drm_sysfs_connector_add(&ptn_bridge->connector);
	drm_mode_connector_attach_encoder(&ptn_bridge->connector, encoder);

	return 0;

err_bridge:
	drm_bridge_cleanup(bridge);
err_gpio:
	gpio_free(ptn_bridge->gpio_pd_n);
	gpio_free(ptn_bridge->gpio_rst_n);
err_client:
	put_device(&client->dev);
	return ret;
}
