/*
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include "drmP.h"

#include "exynos_drm.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_encoder.h"

#define to_exynos_plane(x)	container_of(x, struct exynos_plane, base)

struct exynos_plane {
	struct drm_plane		base;
	struct exynos_drm_overlay	overlay;
	bool				enabled;
};

static const uint32_t formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV12M,
	DRM_FORMAT_NV12MT,
};

static int
exynos_update_plane(struct drm_plane *plane, struct drm_crtc *crtc,
		     struct drm_framebuffer *fb, int crtc_x, int crtc_y,
		     unsigned int crtc_w, unsigned int crtc_h,
		     uint32_t src_x, uint32_t src_y,
		     uint32_t src_w, uint32_t src_h)
{
	struct exynos_plane *exynos_plane =
		container_of(plane, struct exynos_plane, base);
	struct exynos_drm_overlay *overlay = &exynos_plane->overlay;
	struct exynos_drm_crtc_pos pos;
	unsigned int x = src_x >> 16;
	unsigned int y = src_y >> 16;
	unsigned int w = src_w >> 16;
	unsigned int h = src_h >> 16;

	DRM_DEBUG_KMS("[PLANE:%d] [CRTC:%d] %dx%d+%d+%d [FB:%d] %ux%u+%u+%u\n",
			DRM_BASE_ID(plane), DRM_BASE_ID(crtc), crtc_w, crtc_h,
			crtc_x, crtc_y, DRM_BASE_ID(fb), w, h, x, y);

	memset(&pos, 0, sizeof(struct exynos_drm_crtc_pos));
	pos.crtc_x = crtc_x;
	pos.crtc_y = crtc_y;
	pos.crtc_w = crtc_w;
	pos.crtc_h = crtc_h;

	pos.fb_x = x;
	pos.fb_y = y;
	pos.fb_w = w;
	pos.fb_h = h;

	/* TODO: scale feature */
	exynos_drm_overlay_update(overlay, fb, &crtc->mode, &pos);
	exynos_drm_crtc_apply(crtc, overlay);

	exynos_plane->enabled = true;

	return 0;
}

static int exynos_disable_plane(struct drm_plane *plane)
{
	struct exynos_plane *exynos_plane =
		container_of(plane, struct exynos_plane, base);
	struct exynos_drm_overlay *overlay = &exynos_plane->overlay;

	DRM_DEBUG_KMS("[PLANE:%d]\n", DRM_BASE_ID(plane));

	if (!exynos_plane->enabled)
		return 0;

	exynos_drm_overlay_disable(plane->crtc, overlay->zpos);
	exynos_plane->enabled = false;

	return 0;
}

static void exynos_plane_destroy(struct drm_plane *plane)
{
	struct exynos_plane *exynos_plane =
		container_of(plane, struct exynos_plane, base);

	DRM_DEBUG_KMS("[PLANE:%d]\n", DRM_BASE_ID(plane));

	exynos_disable_plane(plane);
	drm_plane_cleanup(plane);
	kfree(exynos_plane);
}

static int exynos_plane_set_property(struct drm_plane *plane,
				     struct drm_property *property,
				     uint64_t val)
{
	struct drm_device *dev = plane->dev;
	struct exynos_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_private *dev_priv = dev->dev_private;

	DRM_DEBUG_KMS("[%d] %s\n", __LINE__, __func__);

	if (property == dev_priv->plane_zpos_property) {
		exynos_plane->overlay.zpos = val;
		return 0;
	}

	return -EINVAL;
}

static struct drm_plane_funcs exynos_plane_funcs = {
	.update_plane	= exynos_update_plane,
	.disable_plane	= exynos_disable_plane,
	.destroy	= exynos_plane_destroy,
	.set_property	= exynos_plane_set_property,
};

static void exynos_plane_attach_zpos_property(struct drm_plane *plane)
{
	struct drm_device *dev = plane->dev;
	struct exynos_drm_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	DRM_DEBUG_KMS("[%d] %s\n", __LINE__, __func__);

	prop = dev_priv->plane_zpos_property;
	if (!prop) {
		prop = drm_property_create_range(dev, 0, "zpos", 0,
						 MAX_PLANE - 1);
		if (!prop)
			return;

		dev_priv->plane_zpos_property = prop;
	}

	drm_object_attach_property(&plane->base, prop, 0);
}

struct drm_plane *exynos_plane_init(struct drm_device *dev,
				    unsigned int possible_crtcs, bool priv)
{
	struct exynos_plane *exynos_plane;
	int err;

	DRM_DEBUG_KMS("[DEV:%s]\n", dev->devname);

	exynos_plane = kzalloc(sizeof(struct exynos_plane), GFP_KERNEL);
	if (!exynos_plane) {
		DRM_ERROR("failed to allocate plane\n");
		return NULL;
	}

	err = drm_plane_init(dev, &exynos_plane->base, possible_crtcs,
			      &exynos_plane_funcs, formats, ARRAY_SIZE(formats),
			      priv);
	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		kfree(exynos_plane);
		return NULL;
	}

	if (priv)
		exynos_plane->overlay.zpos = DEFAULT_ZPOS;
	else
		exynos_plane_attach_zpos_property(&exynos_plane->base);

	return &exynos_plane->base;
}

struct exynos_drm_overlay *get_exynos_drm_overlay(struct drm_plane *plane)
{
	struct exynos_plane *exynos_plane = to_exynos_plane(plane);

	return &exynos_plane->overlay;
}
