/* exynos_drm_encoder.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * VA LINUX SYSTEMS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "drmP.h"
#include "drm_crtc_helper.h"

#include "exynos_drm_drv.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_encoder.h"
#include "exynos_drm_display.h"

#define to_exynos_encoder(x)	container_of(x, struct exynos_drm_encoder,\
				drm_encoder)

/*
 * exynos specific encoder structure.
 *
 * @drm_encoder: encoder object.
 * @display: specific encoder has its own display to control a hardware
 *	appropriately and we can access a hardware drawing on this display.
 */
struct exynos_drm_encoder {
	struct drm_encoder drm_encoder;
	struct exynos_drm_display *display;
};

void exynos_drm_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct exynos_drm_display *display = exynos_drm_get_display(encoder);

	DRM_DEBUG_KMS("[ENCODER:%d:%s] [DPMS:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder), drm_get_dpms_name(mode));

	if (!drm_helper_encoder_in_use(encoder))
		return;

	if (display->panel_ops->dpms)
		display->panel_ops->dpms(display->panel_ctx, mode);
}

static bool
exynos_drm_encoder_mode_fixup(struct drm_encoder *encoder,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_connector *connector;
	struct exynos_drm_display *display = exynos_drm_get_display(encoder);

	DRM_DEBUG_KMS("[ENCODER:%d:%s] [MODE:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder), DRM_BASE_ID(mode),
			mode->name);

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (connector->encoder != encoder)
			continue;

		if (display->panel_ops->mode_fixup)
			display->panel_ops->mode_fixup(display->panel_ctx,
					connector, mode, adjusted_mode);
	}

	return true;
}

static void exynos_drm_encoder_mode_set(struct drm_encoder *encoder,
					 struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted_mode)
{
	struct exynos_drm_display *display = exynos_drm_get_display(encoder);

	DRM_DEBUG_KMS("[ENCODER:%d:%s] [MODE:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder), DRM_BASE_ID(mode),
			mode->name);

	if (!drm_helper_encoder_in_use(encoder))
		return;

	if (display->panel_ops->mode_set)
		display->panel_ops->mode_set(display->panel_ctx, adjusted_mode);
}

static void exynos_drm_encoder_prepare(struct drm_encoder *encoder)
{
	DRM_DEBUG_KMS("[ENCODER:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder));

	/* drm framework doesn't check NULL. */
}

static void exynos_drm_encoder_commit(struct drm_encoder *encoder)
{
	struct exynos_drm_display *display = exynos_drm_get_display(encoder);

	DRM_DEBUG_KMS("[ENCODER:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder));

	exynos_drm_encoder_dpms(encoder, DRM_MODE_DPMS_ON);

	if (display->panel_ops->commit)
		display->panel_ops->commit(display->panel_ctx);
}

static struct drm_crtc *
exynos_drm_encoder_get_crtc(struct drm_encoder *encoder)
{
	return encoder->crtc;
}

static struct drm_encoder_helper_funcs exynos_encoder_helper_funcs = {
	.dpms		= exynos_drm_encoder_dpms,
	.mode_fixup	= exynos_drm_encoder_mode_fixup,
	.mode_set	= exynos_drm_encoder_mode_set,
	.prepare	= exynos_drm_encoder_prepare,
	.commit		= exynos_drm_encoder_commit,
	.get_crtc	= exynos_drm_encoder_get_crtc,
};

static void exynos_drm_encoder_destroy(struct drm_encoder *encoder)
{
	struct exynos_drm_encoder *exynos_encoder = to_exynos_encoder(encoder);

	DRM_DEBUG_KMS("[ENCODER:%d:%s]\n", DRM_BASE_ID(encoder),
			drm_get_encoder_name(encoder));

	exynos_encoder->display->pipe = -1;

	drm_encoder_cleanup(encoder);
	kfree(exynos_encoder);
}

static struct drm_encoder_funcs exynos_encoder_funcs = {
	.destroy = exynos_drm_encoder_destroy,
};

static unsigned int exynos_drm_encoder_clones(struct drm_encoder *encoder)
{
	struct drm_encoder *clone;
	struct drm_device *dev = encoder->dev;
	struct exynos_drm_display *display = exynos_drm_get_display(encoder);
	unsigned int clone_mask = 0;
	int cnt = 0;

	list_for_each_entry(clone, &dev->mode_config.encoder_list, head) {
		switch (display->display_type) {
		case EXYNOS_DRM_DISPLAY_TYPE_FIMD:
		case EXYNOS_DRM_DISPLAY_TYPE_MIXER:
		case EXYNOS_DRM_DISPLAY_TYPE_VIDI:
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

	DRM_DEBUG_KMS("[DEV:%s]\n", dev->devname);

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head)
		encoder->possible_clones = exynos_drm_encoder_clones(encoder);
}

struct drm_encoder *
exynos_drm_encoder_create(struct drm_device *dev,
			   struct exynos_drm_display *display,
			   unsigned int possible_crtcs)
{
	struct drm_encoder *encoder;
	struct exynos_drm_encoder *exynos_encoder;

	DRM_DEBUG_KMS("[DEV:%s] display_type: %s possible_crtcs: 0x%x\n",
			dev->devname, exynos_display_type_name(display),
			possible_crtcs);

	if (!display || !possible_crtcs)
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

	DRM_DEBUG_KMS("[ENCODER:%d:%s] has been created\n",
			DRM_BASE_ID(encoder), drm_get_encoder_name(encoder));

	return encoder;
}

struct exynos_drm_display *exynos_drm_get_display(struct drm_encoder *encoder)
{
	return to_exynos_encoder(encoder)->display;
}
