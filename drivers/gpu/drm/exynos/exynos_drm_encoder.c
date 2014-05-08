/* exynos_drm_encoder.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
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
#include <drm/drm_crtc_helper.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_encoder.h"

#define to_exynos_encoder(x)	container_of(x, struct exynos_drm_encoder,\
				drm_encoder)

/*
 * exynos specific encoder structure.
 *
 * @drm_encoder: encoder object.
 * @display: the display structure that maps to this encoder
 */
struct exynos_drm_encoder {
	struct drm_encoder		drm_encoder;
	struct exynos_drm_display	*display;
};


static void exynos_drm_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct exynos_drm_encoder *exynos_encoder = to_exynos_encoder(encoder);
	struct exynos_drm_display *display = exynos_encoder->display;

	DRM_DEBUG_KMS("[ENCODER:%d:%s] [DPMS:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder), drm_get_dpms_name(mode));

	mutex_lock(&dev->struct_mutex);

	if (display->ops->dpms)
		display->ops->dpms(display->ctx, mode);

	mutex_unlock(&dev->struct_mutex);
}

static int exynos_drm_encoder_mode_valid(struct drm_encoder *encoder,
					 const struct drm_display_mode *mode)
{
	struct exynos_drm_encoder *exynos_encoder = to_exynos_encoder(encoder);
	struct exynos_drm_display *display = exynos_encoder->display;
	int ret = MODE_OK;

	DRM_DEBUG_KMS("[ENCODER:%d:%s] [MODE:%s]\n",
			DRM_BASE_ID(encoder), drm_get_encoder_name(encoder),
			mode->name);

	if (display->ops->check_mode)
		if (display->ops->check_mode(display->ctx, mode))
			ret = MODE_BAD;

	return ret;
}

static bool
exynos_drm_encoder_mode_fixup(struct drm_encoder *encoder,
			       const struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct exynos_drm_encoder *exynos_encoder = to_exynos_encoder(encoder);
	struct exynos_drm_display *display = exynos_encoder->display;
	struct drm_connector *connector;

	DRM_DEBUG_KMS("[ENCODER:%d:%s] [MODE:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder), mode->name);

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (connector->encoder != encoder)
			continue;

		if (display->ops->mode_fixup)
			return display->ops->mode_fixup(display->ctx, connector,
					mode, adjusted_mode);
	}

	return true;
}

static void exynos_drm_encoder_mode_set(struct drm_encoder *encoder,
					 struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted_mode)
{
	struct exynos_drm_encoder *exynos_encoder = to_exynos_encoder(encoder);
	struct exynos_drm_display *display = exynos_encoder->display;

	DRM_DEBUG_KMS("[ENCODER:%d:%s] [MODE:%s] [ADJ_MODE:%s]\n",
			DRM_BASE_ID(encoder), drm_get_encoder_name(encoder),
			mode->name, adjusted_mode->name);

	if (display->ops->mode_set)
		display->ops->mode_set(display->ctx, adjusted_mode);
}

static void exynos_drm_encoder_prepare(struct drm_encoder *encoder)
{
	DRM_DEBUG_KMS("[ENCODER:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder));

	/* drm framework doesn't check NULL. */
}

static void exynos_drm_encoder_commit(struct drm_encoder *encoder)
{
	struct exynos_drm_encoder *exynos_encoder = to_exynos_encoder(encoder);
	struct exynos_drm_display *display = exynos_encoder->display;

	DRM_DEBUG_KMS("[ENCODER:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder));

	if (display->ops->dpms)
		display->ops->dpms(display->ctx, DRM_MODE_DPMS_ON);

	if (display->ops->commit)
		display->ops->commit(display->ctx);
}

static void exynos_drm_encoder_disable(struct drm_encoder *encoder)
{
	struct drm_plane *plane;
	struct drm_device *dev = encoder->dev;

	DRM_DEBUG_KMS("[ENCODER:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder));

	exynos_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);

	/* all planes connected to this encoder should be also disabled. */
	drm_for_each_legacy_plane(plane, &dev->mode_config.plane_list) {
		if (plane->crtc == encoder->crtc)
			plane->funcs->disable_plane(plane);
	}
}

static const struct drm_encoder_helper_funcs exynos_encoder_helper_funcs = {
	.dpms		= exynos_drm_encoder_dpms,
	.mode_valid	= exynos_drm_encoder_mode_valid,
	.mode_fixup	= exynos_drm_encoder_mode_fixup,
	.mode_set	= exynos_drm_encoder_mode_set,
	.prepare	= exynos_drm_encoder_prepare,
	.commit		= exynos_drm_encoder_commit,
	.disable	= exynos_drm_encoder_disable,
};

static int exynos_drm_encoder_set_property(struct drm_encoder *encoder,
		struct drm_property *property, uint64_t val)
{
	struct exynos_drm_encoder *exynos_encoder = to_exynos_encoder(encoder);
	struct exynos_drm_display *display = exynos_encoder->display;
	int ret;

	if (display->ops->set_property) {
		ret = display->ops->set_property(display->ctx, property, val);
		if (ret)
			return ret;
	}

	return drm_object_property_set_value(&encoder->base, property, val);
}

static void exynos_drm_encoder_destroy(struct drm_encoder *encoder)
{
	struct exynos_drm_encoder *exynos_encoder = to_exynos_encoder(encoder);

	DRM_DEBUG_KMS("[ENCODER:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder));

	drm_encoder_cleanup(encoder);
	kfree(exynos_encoder);
}

static const struct drm_encoder_funcs exynos_encoder_funcs = {
	.set_property = exynos_drm_encoder_set_property,
	.destroy = exynos_drm_encoder_destroy,
};

static unsigned int exynos_drm_encoder_clones(struct drm_encoder *encoder)
{
	struct drm_encoder *clone;
	struct drm_device *dev = encoder->dev;
	struct exynos_drm_encoder *exynos_encoder = to_exynos_encoder(encoder);
	struct exynos_drm_display *display = exynos_encoder->display;
	unsigned int clone_mask = 0;
	int cnt = 0;

	DRM_DEBUG_KMS("[ENCODER:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder));

	list_for_each_entry(clone, &dev->mode_config.encoder_list, head) {
		switch (display->type) {
		case EXYNOS_DISPLAY_TYPE_LCD:
		case EXYNOS_DISPLAY_TYPE_HDMI:
		case EXYNOS_DISPLAY_TYPE_VIDI:
			clone_mask |= (1 << (cnt++));
			break;
		default:
			continue;
		}
	}

	return clone_mask;
}

void exynos_drm_encoder_setup(struct drm_device *dev)
{
	struct drm_encoder *encoder;

	DRM_DEBUG_KMS("\n");

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head)
		encoder->possible_clones = exynos_drm_encoder_clones(encoder);
}

struct drm_encoder *
exynos_drm_encoder_create(struct drm_device *dev,
			   struct exynos_drm_display *display,
			   unsigned long possible_crtcs)
{
	struct drm_encoder *encoder;
	struct exynos_drm_encoder *exynos_encoder;

	DRM_DEBUG_KMS("[DISPLAY:%s] possible_crtcs:0x%lx\n",
			exynos_drm_output_type_name(display->type),
			possible_crtcs);

	if (!possible_crtcs)
		return NULL;

	exynos_encoder = kzalloc(sizeof(*exynos_encoder), GFP_KERNEL);
	if (!exynos_encoder) {
		DRM_ERROR("failed to allocate encoder\n");
		return NULL;
	}

	exynos_encoder->display = display;
	encoder = &exynos_encoder->drm_encoder;
	encoder->possible_crtcs = possible_crtcs;

	drm_encoder_init(dev, encoder, &exynos_encoder_funcs,
			DRM_MODE_ENCODER_TMDS);

	drm_encoder_helper_add(encoder, &exynos_encoder_helper_funcs);

	drm_object_attach_property(&encoder->base,
			dev->mode_config.content_protection_property,
			DRM_MODE_CONTENT_PROTECTION_OFF);

	DRM_DEBUG_KMS("Created [ENCODER:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder));

	return encoder;
}

struct exynos_drm_display *exynos_drm_get_display(struct drm_encoder *encoder)
{
	return to_exynos_encoder(encoder)->display;
}
