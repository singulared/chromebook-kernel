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

static struct drm_plane_funcs exynos_plane_funcs = {
	.update_plane	= exynos_update_plane,
	.disable_plane	= exynos_disable_plane,
	.destroy	= exynos_plane_destroy,
};

int exynos_plane_init(struct drm_device *dev, unsigned int nr)
{
	struct exynos_plane *exynos_plane;
	uint32_t possible_crtcs;

	DRM_DEBUG_KMS("[DEV:%s] nr: %u\n", dev->devname, nr);

	exynos_plane = kzalloc(sizeof(struct exynos_plane), GFP_KERNEL);
	if (!exynos_plane)
		return -ENOMEM;

	/* all CRTCs are available */
	possible_crtcs = (1 << EXYNOS_DRM_DISPLAY_NUM_DISPLAYS) - 1;

	exynos_plane->overlay.zpos = DEFAULT_ZPOS;

	return drm_plane_init(dev, &exynos_plane->base, possible_crtcs,
			      &exynos_plane_funcs, formats, ARRAY_SIZE(formats),
			      false);
}

int exynos_plane_set_zpos_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	struct drm_exynos_plane_set_zpos *zpos_req = data;
	struct drm_mode_object *obj;
	struct drm_plane *plane;
	struct exynos_plane *exynos_plane;
	int ret = 0;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	DRM_DEBUG_KMS("[DEV:%s] [PLANE:%d] zpos: %d\n", dev->devname,
			zpos_req->plane_id, zpos_req->zpos);

	if (zpos_req->zpos < 0 || zpos_req->zpos >= MAX_PLANE) {
		if (zpos_req->zpos != DEFAULT_ZPOS) {
			DRM_ERROR("zpos not within limits\n");
			return -EINVAL;
		}
	}

	mutex_lock(&dev->mode_config.mutex);

	obj = drm_mode_object_find(dev, zpos_req->plane_id,
			DRM_MODE_OBJECT_PLANE);
	if (!obj) {
		DRM_DEBUG_KMS("Unknown plane ID %d\n",
			      zpos_req->plane_id);
		ret = -EINVAL;
		goto out;
	}

	plane = obj_to_plane(obj);
	exynos_plane = container_of(plane, struct exynos_plane, base);

	exynos_plane->overlay.zpos = zpos_req->zpos;

out:
	mutex_unlock(&dev->mode_config.mutex);
	return ret;
}
