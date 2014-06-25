/* exynos_drm_vidi.c
 *
 * Copyright (C) 2012 Samsung Electronics Co.Ltd
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <drm/drmP.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <drm/exynos_drm.h>

#include <drm/drm_edid.h>
#include <drm/drm_crtc_helper.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_vidi.h"
#include "exynos_drm_gem.h"

/* vidi has totally three virtual windows. */
#define WINDOWS_NR		3

static const uint32_t plane_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV12MT,
};

#define exynos_plane_to_win_idx(ctx, x) (unsigned int)(x - ctx->planes)
#define to_vidi_ctx(x) container_of(x, struct vidi_context, crtc)
#define get_vidi_context(dev)	platform_get_drvdata(to_platform_device(dev))
#define ctx_from_encoder(e)	container_of(e, struct vidi_context, \
					encoder)
#define ctx_from_connector(c)	container_of(c, struct vidi_context, \
					connector)

struct vidi_win_data {
	unsigned int		offset_x;
	unsigned int		offset_y;
	unsigned int		ovl_width;
	unsigned int		ovl_height;
	unsigned int		fb_width;
	unsigned int		fb_height;
	unsigned int		bpp;
	dma_addr_t		dma_addr;
	unsigned int		buf_offsize;
	unsigned int		line_size;	/* bytes */
	bool			enabled;
};

struct vidi_context {
	struct drm_device		*drm_dev;
	struct drm_crtc			crtc;
	struct drm_encoder		encoder;
	struct drm_connector		connector;
	struct exynos_drm_plane		planes[WINDOWS_NR];
	struct edid			*raw_edid;
	unsigned int			clkdiv;
	unsigned int			default_win;
	unsigned long			irq_flags;
	unsigned int			connected;
	bool				vblank_on;
	bool				suspended;
	bool				direct_vblank;
	int				pipe;
	struct work_struct		work;
	struct mutex			lock;
};

static const char fake_edid_info[] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x4c, 0x2d, 0x05, 0x05,
	0x00, 0x00, 0x00, 0x00, 0x30, 0x12, 0x01, 0x03, 0x80, 0x10, 0x09, 0x78,
	0x0a, 0xee, 0x91, 0xa3, 0x54, 0x4c, 0x99, 0x26, 0x0f, 0x50, 0x54, 0xbd,
	0xee, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x66, 0x21, 0x50, 0xb0, 0x51, 0x00,
	0x1b, 0x30, 0x40, 0x70, 0x36, 0x00, 0xa0, 0x5a, 0x00, 0x00, 0x00, 0x1e,
	0x01, 0x1d, 0x00, 0x72, 0x51, 0xd0, 0x1e, 0x20, 0x6e, 0x28, 0x55, 0x00,
	0xa0, 0x5a, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x18,
	0x4b, 0x1a, 0x44, 0x17, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
	0x00, 0x00, 0x00, 0xfc, 0x00, 0x53, 0x41, 0x4d, 0x53, 0x55, 0x4e, 0x47,
	0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xbc, 0x02, 0x03, 0x1e, 0xf1,
	0x46, 0x84, 0x05, 0x03, 0x10, 0x20, 0x22, 0x23, 0x09, 0x07, 0x07, 0x83,
	0x01, 0x00, 0x00, 0xe2, 0x00, 0x0f, 0x67, 0x03, 0x0c, 0x00, 0x10, 0x00,
	0xb8, 0x2d, 0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20, 0x58, 0x2c,
	0x25, 0x00, 0xa0, 0x5a, 0x00, 0x00, 0x00, 0x9e, 0x8c, 0x0a, 0xd0, 0x8a,
	0x20, 0xe0, 0x2d, 0x10, 0x10, 0x3e, 0x96, 0x00, 0xa0, 0x5a, 0x00, 0x00,
	0x00, 0x18, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
	0x45, 0x00, 0xa0, 0x5a, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x06
};

static void vidi_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct vidi_context *ctx = to_vidi_ctx(crtc);

	DRM_DEBUG_KMS("[CRTC:%d] [DPMS:%s]\n", DRM_BASE_ID(crtc),
			drm_get_dpms_name(mode));

	mutex_lock(&ctx->lock);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		/* TODO. */
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		/* TODO. */
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}

	mutex_unlock(&ctx->lock);
}

static void vidi_crtc_prepare(struct drm_crtc *crtc)
{
	struct vidi_context *ctx = to_vidi_ctx(crtc);

	drm_vblank_pre_modeset(crtc->dev, ctx->pipe);
}

static void vidi_crtc_commit(struct drm_crtc *crtc)
{
	struct vidi_context *ctx = to_vidi_ctx(crtc);

	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(crtc));

	drm_vblank_post_modeset(crtc->dev, ctx->pipe);
}

static bool vidi_crtc_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	return true;
}

static int vidi_crtc_mode_set(struct drm_crtc *crtc,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode, int x, int y,
		struct drm_framebuffer *old_fb)
{
	return 0;
}

int vidi_enable_vblank(struct drm_crtc *crtc)
{
	struct vidi_context *ctx = to_vidi_ctx(crtc);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return -EPERM;

	if (!test_and_set_bit(0, &ctx->irq_flags))
		ctx->vblank_on = true;

	ctx->direct_vblank = true;

	/*
	 * in case of page flip request, vidi_finish_pageflip function
	 * will not be called because direct_vblank is true and then
	 * that function will be called by update_plane callback
	 */
	schedule_work(&ctx->work);

	return 0;
}

void vidi_disable_vblank(struct drm_crtc *crtc)
{
	struct vidi_context *ctx = to_vidi_ctx(crtc);

	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(crtc));

	if (ctx->suspended)
		return;

	if (test_and_clear_bit(0, &ctx->irq_flags))
		ctx->vblank_on = false;
}

static int vidi_plane_commit(struct drm_plane *plane,
		struct drm_framebuffer *fb)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct vidi_context *ctx = exynos_plane->ctx;
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);
	struct exynos_drm_gem_buf *buffer;
	dma_addr_t dma_addr;

	DRM_DEBUG_KMS("[WIN:%d]\n", exynos_plane_to_win_idx(ctx, exynos_plane));

	if (ctx->suspended)
		return 0;

	if (WARN_ON(exynos_drm_fb_get_buf_cnt(exynos_fb) > 1))
		return -EINVAL;

	buffer = exynos_drm_fb_buffer(exynos_fb, 0);
	dma_addr = buffer->dma_addr + exynos_plane->src_x
			* (fb->bits_per_pixel >> 3) + exynos_plane->src_y
			* fb->pitches[0];
	DRM_DEBUG_KMS("dma_addr = 0x%x\n", dma_addr);

	return 0;
}

static int vidi_plane_update(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb, int crtc_x,
		int crtc_y, unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y, uint32_t src_w, uint32_t src_h)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);

	/* Copy the parameters into the plane so we can restore it later */
	exynos_plane->crtc_x = crtc_x;
	exynos_plane->crtc_y = crtc_y;
	exynos_plane->crtc_w = crtc_w;
	exynos_plane->crtc_h = crtc_h;
	exynos_plane->src_x = src_x >> 16;
	exynos_plane->src_y = src_y >> 16;
	exynos_plane->src_w = src_w >> 16;
	exynos_plane->src_h = src_h >> 16;

	exynos_sanitize_plane_coords(plane, crtc);

	return vidi_plane_commit(plane, fb);
}

static int vidi_plane_disable(struct drm_plane *plane)
{
	return 0;
}

static void vidi_plane_destroy(struct drm_plane *plane)
{
	drm_plane_cleanup(plane);
}

const struct drm_plane_funcs vidi_plane_funcs = {
	.update_plane = vidi_plane_update,
	.disable_plane = vidi_plane_disable,
	.destroy = vidi_plane_destroy,
};

static void vidi_crtc_load_lut(struct drm_crtc *crtc)
{
}

static const struct drm_crtc_helper_funcs vidi_crtc_helper_funcs = {
	.dpms = vidi_crtc_dpms,
	.prepare = vidi_crtc_prepare,
	.commit = vidi_crtc_commit,
	.mode_fixup = vidi_crtc_mode_fixup,
	.mode_set = vidi_crtc_mode_set,
	.load_lut = vidi_crtc_load_lut,
};

static int vidi_crtc_page_flip(struct drm_crtc *crtc,
		struct drm_framebuffer *fb,
		struct drm_pending_vblank_event *event,	uint32_t flip_flags)
{
	struct vidi_context *ctx = to_vidi_ctx(crtc);
	struct exynos_drm_plane *exynos_plane = &ctx->planes[ctx->default_win];
	struct drm_plane *plane = &exynos_plane->base;
	unsigned int crtc_w, crtc_h;
	int ret;

	if (ctx->suspended)
		return -ENODEV;

	crtc_w = fb->width - crtc->x;
	crtc_h = fb->height - crtc->y;

	ret = plane->funcs->update_plane(plane, crtc, fb, 0, 0, crtc_w, crtc_h,
		crtc->x << 16, crtc->y << 16, crtc_w << 16, crtc_h << 16);
	if (ret)
		goto fail;

	if (event) {
		exynos_plane->pending_event = event;
		exynos_drm_crtc_send_event(plane, crtc);
	}

	plane->fb = fb;
	plane->crtc = crtc;

	return 0;

fail:
	DRM_ERROR("Page flip failed, ret=%d\n", ret);
	return ret;
}

static void vidi_crtc_destroy(struct drm_crtc *crtc)
{
}

static const struct drm_crtc_funcs vidi_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.page_flip	= vidi_crtc_page_flip,
	.destroy	= vidi_crtc_destroy,
};

int vidi_get_crtc_id(struct drm_device *dev)
{
	struct drm_crtc *crtc;
	struct vidi_context *ctx;

	/*
	 * This is a hack which we should be able to remove once we keep tabs on
	 * each of the crtc's
	 */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (crtc->funcs == &vidi_crtc_funcs) {
			ctx = to_vidi_ctx(crtc);
			return ctx->pipe;
		}
	}

	return -ENODEV;
}

static void vidi_fake_vblank_handler(struct work_struct *work)
{
	struct vidi_context *ctx = container_of(work, struct vidi_context,
					work);

	if (ctx->pipe < 0)
		return;

	/* refresh rate is about 50Hz. */
	usleep_range(16000, 20000);

	mutex_lock(&ctx->lock);

	if (ctx->direct_vblank) {
		drm_handle_vblank(ctx->drm_dev, ctx->pipe);
		ctx->direct_vblank = false;
		mutex_unlock(&ctx->lock);
		return;
	}

	mutex_unlock(&ctx->lock);
}

static void vidi_apply(void *in_ctx)
{
	struct vidi_context *ctx = in_ctx;
	int i, ret;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	for (i = 0; i < WINDOWS_NR; i++) {
		struct exynos_drm_plane *exynos_plane = &ctx->planes[i];
		struct drm_plane *plane = &exynos_plane->base;

		if (!plane->fb)
			continue;

		ret = vidi_plane_commit(plane, plane->fb);
		if (ret)
			DRM_ERROR("Failed to update plane %d ret=%d\n", i, ret);

	}
	vidi_crtc_commit(&ctx->crtc);
}

static int vidi_power_on(struct vidi_context *ctx, bool enable)
{
	DRM_DEBUG_KMS("[CRTC:%d] enable: %u\n", DRM_BASE_ID(&ctx->crtc),
			enable);

	if (enable != false && enable != true)
		return -EINVAL;

	if (enable) {
		ctx->suspended = false;

		/* if vblank was enabled status, enable it again. */
		if (test_and_clear_bit(0, &ctx->irq_flags))
			vidi_enable_vblank(&ctx->crtc);

		vidi_apply(ctx);
	} else {
		ctx->suspended = true;
	}

	return 0;
}

static int vidi_show_connection(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int rc;
	struct vidi_context *ctx = get_vidi_context(dev);

	mutex_lock(&ctx->lock);

	rc = sprintf(buf, "%d\n", ctx->connected);

	mutex_unlock(&ctx->lock);

	return rc;
}

static int vidi_store_connection(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct vidi_context *ctx = get_vidi_context(dev);
	int ret;

	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(&ctx->crtc));

	ret = kstrtoint(buf, 0, &ctx->connected);
	if (ret)
		return ret;

	if (ctx->connected > 1)
		return -EINVAL;

	/* use fake edid data for test. */
	if (!ctx->raw_edid)
		ctx->raw_edid = (struct edid *)fake_edid_info;

	/* if raw_edid isn't same as fake data then it can't be tested. */
	if (ctx->raw_edid != (struct edid *)fake_edid_info) {
		DRM_DEBUG_KMS("edid data is not fake data.\n");
		return -EINVAL;
	}

	DRM_DEBUG_KMS("requested connection.\n");

	drm_helper_hpd_irq_event(ctx->drm_dev);

	return len;
}

static DEVICE_ATTR(connection, 0644, vidi_show_connection,
			vidi_store_connection);

int vidi_connection_ioctl(struct drm_device *drm_dev, void *data,
				struct drm_file *file_priv)
{
	struct vidi_context *ctx = NULL;
	struct drm_connector *conn;
	struct drm_exynos_vidi_connection *vidi = data;
	int edid_len;

	DRM_DEBUG_KMS("\n");

	if (!vidi) {
		DRM_DEBUG_KMS("user data for vidi is null.\n");
		return -EINVAL;
	}

	if (vidi->connection > 1) {
		DRM_DEBUG_KMS("connection should be 0 or 1.\n");
		return -EINVAL;
	}

	list_for_each_entry(conn, &drm_dev->mode_config.connector_list, head) {
		if (conn->connector_type != DRM_MODE_CONNECTOR_VIRTUAL)
			continue;

		ctx = ctx_from_connector(conn);
		break;
	}
	if (!ctx) {
		DRM_DEBUG_KMS("not found virtual device type encoder.\n");
		return -EINVAL;
	}

	if (ctx->connected == vidi->connection) {
		DRM_DEBUG_KMS("same connection request.\n");
		return -EINVAL;
	}

	if (vidi->connection) {
		struct edid *raw_edid  = (struct edid *)(uint32_t)vidi->edid;
		if (!drm_edid_is_valid(raw_edid)) {
			DRM_DEBUG_KMS("edid data is invalid.\n");
			return -EINVAL;
		}
		edid_len = (1 + raw_edid->extensions) * EDID_LENGTH;
		ctx->raw_edid = kzalloc(edid_len, GFP_KERNEL);
		if (!ctx->raw_edid) {
			DRM_DEBUG_KMS("failed to allocate raw_edid.\n");
			return -ENOMEM;
		}
		memcpy(ctx->raw_edid, raw_edid, edid_len);
	} else {
		/*
		 * with connection = 0, free raw_edid
		 * only if raw edid data isn't same as fake data.
		 */
		if (ctx->raw_edid && ctx->raw_edid !=
				(struct edid *)fake_edid_info) {
			kfree(ctx->raw_edid);
			ctx->raw_edid = NULL;
		}
	}

	ctx->connected = vidi->connection;
	drm_helper_hpd_irq_event(ctx->drm_dev);

	return 0;
}

static enum drm_connector_status vidi_detect(struct drm_connector *connector,
			bool force)
{
	struct vidi_context *ctx = ctx_from_connector(connector);

	/*
	 * connection request would come from user side
	 * to do hotplug through specific ioctl.
	 */
	return ctx->connected ? connector_status_connected :
			connector_status_disconnected;
}

static void vidi_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs vidi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = vidi_detect,
	.destroy = vidi_connector_destroy,
};

static int vidi_get_modes(struct drm_connector *connector)
{
	struct vidi_context *ctx = ctx_from_connector(connector);
	struct edid *edid;
	int edid_len;
	int count;

	/*
	 * the edid data comes from user side and it would be set
	 * to ctx->raw_edid through specific ioctl.
	 */
	if (!ctx->raw_edid) {
		DRM_DEBUG_KMS("raw_edid is null.\n");
		return 0;
	}

	edid_len = (1 + ctx->raw_edid->extensions) * EDID_LENGTH;
	edid = kmemdup(ctx->raw_edid, edid_len, GFP_KERNEL);
	if (!edid) {
		DRM_DEBUG_KMS("failed to allocate edid\n");
		return 0;
	}

	count = drm_add_edid_modes(connector, edid);
	if (!count)
		DRM_ERROR("Add edid modes failed %d\n", count);
	else
		drm_mode_connector_update_edid_property(connector, edid);

	kfree(edid);
	return count;
}

static int vidi_mode_valid(struct drm_connector *connector,
			struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_encoder *vidi_best_encoder(struct drm_connector *connector)
{
	struct vidi_context *ctx = ctx_from_connector(connector);

	return &ctx->encoder;
}

static struct drm_connector_helper_funcs vidi_connector_helper_funcs = {
	.get_modes = vidi_get_modes,
	.mode_valid = vidi_mode_valid,
	.best_encoder = vidi_best_encoder,
};

static int vidi_create_connector(struct drm_encoder *encoder)
{
	struct vidi_context *ctx = ctx_from_encoder(encoder);
	struct drm_connector *connector = &ctx->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(ctx->drm_dev, connector,
			&vidi_connector_funcs, DRM_MODE_CONNECTOR_VIRTUAL);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector, &vidi_connector_helper_funcs);
	ret = drm_sysfs_connector_add(connector);
	if (ret)
		goto err_connector;

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret) {
		DRM_ERROR("failed to attach a connector to an encoder\n");
		goto err_sysfs;
	}

	return 0;

err_sysfs:
	drm_sysfs_connector_remove(connector);
err_connector:
	drm_connector_cleanup(connector);
	return ret;
}

static const struct drm_encoder_funcs vidi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int vidi_subdrv_probe(struct drm_device *drm_dev, struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vidi_context *ctx = platform_get_drvdata(pdev);
	struct drm_crtc *crtc = &ctx->crtc;
	struct drm_encoder *encoder = &ctx->encoder;
	struct exynos_drm_private *priv = drm_dev->dev_private;
	int i, ret;

	ctx->drm_dev = drm_dev;
	ctx->pipe = drm_dev->mode_config.num_crtc;

	for (i = 0; i < WINDOWS_NR; i++) {
		struct exynos_drm_plane *exynos_plane = &ctx->planes[i];
		struct drm_plane *plane = &exynos_plane->base;

		ret = drm_universal_plane_init(drm_dev, plane,
			1 << ctx->pipe, &vidi_plane_funcs, plane_formats,
			ARRAY_SIZE(plane_formats),
			i == ctx->default_win ? DRM_PLANE_TYPE_PRIMARY
				: DRM_PLANE_TYPE_OVERLAY);
		if (ret) {
			DRM_ERROR("Init plane %d failed (ret=%d)\n", i, ret);
			goto err_plane;
		}

		exynos_plane->ctx = ctx;
	}

	ret = drm_crtc_init_with_planes(drm_dev, crtc,
		&ctx->planes[ctx->default_win].base, NULL, &vidi_crtc_funcs);
	if (ret) {
		DRM_ERROR("Init crtc failed (ret=%d)\n", ret);
		goto err_crtc;
	}

	drm_crtc_helper_add(crtc, &vidi_crtc_helper_funcs);

	ret = drm_encoder_init(drm_dev, encoder, &vidi_encoder_funcs,
			DRM_MODE_ENCODER_NONE);
	if (ret) {
		DRM_ERROR("Failed to initialize the encoder ret=%d\n", ret);
		goto err_encoder;
	}

	ret = vidi_create_connector(encoder);
	if (ret) {
		DRM_ERROR("Create hdmi connector fail, ret=%d\n", ret);
		goto err_connector;
	}

	priv->vidi_crtc = crtc;
	priv->vidi_encoder = encoder;

	return 0;

err_connector:
	drm_encoder_cleanup(encoder);
err_encoder:
	drm_crtc_cleanup(crtc);
err_crtc:
err_plane:
	for (; i >= 0; i--)
		drm_plane_cleanup(&ctx->planes[i].base);

	return ret;
}

static void vidi_subdrv_remove(struct drm_device *drm_dev, struct device *dev)
{
}

static struct exynos_drm_subdrv vidi_subdrv = {
	.probe = vidi_subdrv_probe,
	.remove = vidi_subdrv_remove,
};

static int vidi_probe(struct platform_device *pdev)
{
	struct vidi_context *ctx;
	int ret;

	DRM_DEBUG_KMS("[PDEV:%s]\n", pdev->name);

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->default_win = 0;

	INIT_WORK(&ctx->work, vidi_fake_vblank_handler);

	mutex_init(&ctx->lock);

	platform_set_drvdata(pdev, ctx);

	vidi_subdrv.dev = &pdev->dev;
	exynos_drm_subdrv_register(&vidi_subdrv);

	ret = device_create_file(&pdev->dev, &dev_attr_connection);
	if (ret < 0)
		DRM_INFO("failed to create connection sysfs.\n");

	return 0;
}

static int vidi_remove(struct platform_device *pdev)
{
	struct vidi_context *ctx = platform_get_drvdata(pdev);

	DRM_DEBUG_KMS("[PDEV:%s]\n", pdev->name);

	exynos_drm_subdrv_unregister(&vidi_subdrv);

	if (ctx->raw_edid != (struct edid *)fake_edid_info) {
		kfree(ctx->raw_edid);
		ctx->raw_edid = NULL;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int vidi_suspend(struct device *dev)
{
	struct vidi_context *ctx = get_vidi_context(dev);

	return vidi_power_on(ctx, false);
}

static int vidi_resume(struct device *dev)
{
	struct vidi_context *ctx = get_vidi_context(dev);

	return vidi_power_on(ctx, true);
}
#endif

static const struct dev_pm_ops vidi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(vidi_suspend, vidi_resume)
};

struct platform_driver vidi_driver = {
	.probe		= vidi_probe,
	.remove		= vidi_remove,
	.driver		= {
		.name	= "exynos-drm-vidi",
		.owner	= THIS_MODULE,
		.pm	= &vidi_pm_ops,
	},
};
