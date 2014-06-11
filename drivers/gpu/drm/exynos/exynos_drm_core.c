/* exynos_drm_core.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Author:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <drm/drmP.h>
#include "exynos_drm_drv.h"
#include "exynos_drm_encoder.h"
#include "exynos_drm_fbdev.h"

static LIST_HEAD(exynos_drm_subdrv_list);
static LIST_HEAD(exynos_drm_display_list);

static int exynos_drm_create_enc_conn(struct drm_device *dev,
					struct exynos_drm_display *display,
					uint32_t possible_crtcs)
{
	struct drm_encoder *encoder;
	int ret;

	DRM_DEBUG_KMS("[DISPLAY:%s]\n",
			exynos_drm_output_type_name(display->type));

	/* create and initialize a encoder for this sub driver. */
	encoder = exynos_drm_encoder_create(dev, display, possible_crtcs);
	if (!encoder) {
		DRM_ERROR("failed to create encoder\n");
		return -EFAULT;
	}
	DRM_DEBUG_KMS("Created [ENCODER:%d:%s] for [DISPLAY:%s]\n",
			DRM_BASE_ID(encoder), drm_get_encoder_name(encoder),
			exynos_drm_output_type_name(display->type));

	display->encoder = encoder;

	ret = display->ops->create_connector(display->ctx, encoder);
	if (ret) {
		DRM_ERROR("failed to create connector ret=%d\n", ret);
		goto err_destroy_encoder;
	}

	return 0;

err_destroy_encoder:
	encoder->funcs->destroy(encoder);
	return ret;
}

static int exynos_drm_subdrv_probe(struct drm_device *dev,
					struct exynos_drm_subdrv *subdrv)
{
	DRM_DEBUG_DRIVER("[SUBDRV:%s]\n", dev_name(subdrv->dev));

	if (subdrv->probe) {
		int ret;

		subdrv->drm_dev = dev;

		/*
		 * this probe callback would be called by sub driver
		 * after setting of all resources to this sub driver,
		 * such as clock, irq and register map are done or by load()
		 * of exynos drm driver.
		 *
		 * P.S. note that this driver is considered for modularization.
		 */
		ret = subdrv->probe(dev, subdrv->dev);
		if (ret)
			return ret;
	}

	return 0;
}

static void exynos_drm_subdrv_remove(struct drm_device *dev,
				      struct exynos_drm_subdrv *subdrv)
{
	DRM_DEBUG_DRIVER("[SUBDRV:%s]\n", dev_name(subdrv->dev));

	if (subdrv->remove)
		subdrv->remove(dev, subdrv->dev);
}

int exynos_drm_initialize_displays(struct drm_device *dev)
{
	struct exynos_drm_display *display, *n;
	int ret, initialized = 0;

	DRM_DEBUG_DRIVER("\n");

	list_for_each_entry(display, &exynos_drm_display_list, list) {
		uint32_t possible_crtcs;

		if (display->ops->initialize) {
			ret = display->ops->initialize(display->ctx, dev,
				&possible_crtcs);
			if (ret) {
				DRM_ERROR("Display init [%d] failed with %d\n",
						display->type, ret);
				goto err;
			}
		}

		initialized++;

		ret = exynos_drm_create_enc_conn(dev, display, possible_crtcs);
		if (ret) {
			DRM_ERROR("Encoder create [%d] failed with %d\n",
					display->type, ret);
			goto err;
		}
	}
	return 0;

err:
	list_for_each_entry_safe(display, n, &exynos_drm_display_list, list) {
		if (initialized-- > 0)
			exynos_drm_display_unregister(display);
		else
			list_del(&display->list);
	}
	return ret;
}

void exynos_drm_remove_displays(struct drm_device *dev)
{
	struct exynos_drm_display *display, *n;

	DRM_DEBUG_DRIVER("\n");

	list_for_each_entry_safe(display, n, &exynos_drm_display_list, list)
		exynos_drm_display_unregister(display);
}

int exynos_drm_device_register(struct drm_device *dev)
{
	struct exynos_drm_subdrv *subdrv, *n;
	int err;

	if (!dev)
		return -EINVAL;

	DRM_DEBUG_DRIVER("\n");

	list_for_each_entry_safe(subdrv, n, &exynos_drm_subdrv_list, list) {
		err = exynos_drm_subdrv_probe(dev, subdrv);
		if (err) {
			DRM_ERROR("[SUBDRV:%s] probe failed, %d\n",
					dev_name(subdrv->dev), err);

			list_del(&subdrv->list);
			continue;
		}
	}

	return 0;
}

int exynos_drm_device_unregister(struct drm_device *dev)
{
	struct exynos_drm_subdrv *subdrv;

	if (!dev) {
		WARN(1, "Unexpected drm device unregister!\n");
		return -EINVAL;
	}

	DRM_DEBUG_DRIVER("\n");

	list_for_each_entry(subdrv, &exynos_drm_subdrv_list, list) {
		exynos_drm_subdrv_remove(dev, subdrv);
	}

	return 0;
}

int exynos_drm_display_register(struct exynos_drm_display *display)
{
	DRM_DEBUG_KMS("[DISPLAY:%s]\n",
			exynos_drm_output_type_name(display->type));

	BUG_ON(!display->ops);
	list_add_tail(&display->list, &exynos_drm_display_list);
	return 0;
}

int exynos_drm_display_unregister(struct exynos_drm_display *display)
{
	DRM_DEBUG_KMS("[DISPLAY:%s]\n",
			exynos_drm_output_type_name(display->type));

	if (display->ops->remove)
		display->ops->remove(display->ctx);

	list_del(&display->list);
	return 0;
}

int exynos_drm_subdrv_register(struct exynos_drm_subdrv *subdrv)
{
	if (!subdrv)
		return -EINVAL;

	DRM_DEBUG_DRIVER("[SUBDRV:%s]\n", dev_name(subdrv->dev));

	list_add_tail(&subdrv->list, &exynos_drm_subdrv_list);

	return 0;
}

int exynos_drm_subdrv_unregister(struct exynos_drm_subdrv *subdrv)
{
	if (!subdrv)
		return -EINVAL;

	DRM_DEBUG_DRIVER("[SUBDRV:%s]\n", dev_name(subdrv->dev));

	list_del(&subdrv->list);

	return 0;
}

int exynos_drm_subdrv_open(struct drm_device *dev, struct drm_file *file)
{
	struct exynos_drm_subdrv *subdrv;
	int ret;

	DRM_DEBUG_DRIVER("\n");

	list_for_each_entry(subdrv, &exynos_drm_subdrv_list, list) {
		if (subdrv->open) {
			ret = subdrv->open(dev, subdrv->dev, file);
			if (ret)
				goto err;
		}
	}

	return 0;

err:
	list_for_each_entry_reverse(subdrv, &subdrv->list, list) {
		if (subdrv->close)
			subdrv->close(dev, subdrv->dev, file);
	}
	return ret;
}

void exynos_drm_subdrv_close(struct drm_device *dev, struct drm_file *file)
{
	struct exynos_drm_subdrv *subdrv;

	DRM_DEBUG_DRIVER("\n");

	list_for_each_entry(subdrv, &exynos_drm_subdrv_list, list) {
		if (subdrv->close)
			subdrv->close(dev, subdrv->dev, file);
	}
}
