/* exynos_drm_crtc.c
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

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
#include <linux/dma-buf.h>
#include <linux/kds.h>
#include <linux/kfifo.h>
#endif

#include "exynos_drm_drv.h"
#include "exynos_drm_encoder.h"
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
#include "exynos_drm_fb.h"
#include "exynos_drm_gem.h"
#endif
#include "exynos_trace.h"
#include "exynos_drm_plane.h"

#define to_exynos_crtc(x)	container_of(x, struct exynos_drm_crtc,\
				drm_crtc)

enum exynos_crtc_mode {
	CRTC_MODE_NORMAL,	/* normal mode */
	CRTC_MODE_BLANK,	/* The private plane of crtc is blank */
};

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
struct exynos_drm_flip_desc {
        struct drm_framebuffer  *fb;
        struct kds_resource_set *kds;
};
#endif

#define KDS_WAITALL_MAX_TRIES 15

/*
 * Exynos specific crtc structure.
 *
 * @drm_crtc: crtc object.
 * @drm_plane: pointer of private plane object for this crtc
 * @event: vblank event that is currently queued for flip
 * @pipe: a crtc index created at load() with a new crtc object creation
 *	and the crtc object would be set to private->crtc array
 *	to get a crtc object corresponding to this pipe from private->crtc
 *	array when irq interrupt occured. the reason of using this pipe is that
 *	drm framework doesn't support multiple irq yet.
 *	we can refer to the crtc to current hardware interrupt occured through
 *	this pipe value.
 * @dpms: store the crtc dpms value
 * @mode: store the crtc mode value
 */
struct exynos_drm_crtc {
	struct drm_crtc			drm_crtc;
	struct drm_plane		*plane;
	struct drm_pending_vblank_event *event;
	unsigned int			pipe;
	unsigned int			dpms;
	enum exynos_crtc_mode		mode;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	atomic_t			flip_pending;
	DECLARE_KFIFO(flip_fifo, struct exynos_drm_flip_desc, 2);
	struct exynos_drm_flip_desc	scanout_desc;
	wait_queue_head_t		vsync_wq;
#endif
};

static void exynos_drm_crtc_flip_complete(struct drm_device *dev,
                                          struct drm_pending_vblank_event *e)
{
	struct timeval now;
	unsigned long flags;

	do_gettimeofday(&now);
	e->event.sequence = 0;
	e->event.tv_sec = now.tv_sec;
	e->event.tv_usec = now.tv_usec;
	spin_lock_irqsave(&dev->event_lock, flags);
	list_add_tail(&e->base.link, &e->base.file_priv->event_list);
	spin_unlock_irqrestore(&dev->event_lock, flags);
	wake_up_interruptible(&e->base.file_priv->event_wait);
	trace_exynos_fake_flip_complete(e->pipe);
}

static void exynos_drm_crtc_update(struct drm_crtc *crtc,
				   struct drm_framebuffer *fb)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct drm_plane *plane = exynos_crtc->plane;
	unsigned int crtc_w;
	unsigned int crtc_h;

	crtc_w = fb->width - crtc->x;
	crtc_h = fb->height - crtc->y;

	exynos_plane_mode_set(plane, crtc, fb, 0, 0, crtc_w, crtc_h,
			      crtc->x, crtc->y, crtc_w, crtc_h);

	exynos_plane_commit(exynos_crtc->plane);
	exynos_plane_dpms(exynos_crtc->plane, DRM_MODE_DPMS_ON);
}

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
static void exynos_drm_crtc_wait_and_release_kds(
		struct exynos_drm_flip_desc *desc)
{
	struct exynos_drm_fb *exynos_fb;
	struct exynos_drm_gem_obj *gem_ob;
	struct dma_buf *buf;
	unsigned long shared = 0UL;
	struct kds_resource *resource_list;
	struct kds_resource_set *resource_set;
	int i;

	if (!desc->fb) {
		BUG_ON(desc->kds);
		return;
	}

	if (desc->kds)
		kds_resource_set_release(&desc->kds);

	exynos_fb = to_exynos_fb(desc->fb);
	gem_ob = (struct exynos_drm_gem_obj *)exynos_fb->exynos_gem_obj[0];
	buf = gem_ob->base.export_dma_buf;
	if (!buf)
		return;

	if (unlikely(!exynos_fb->dma_buf)) {
		get_dma_buf(buf);
		exynos_fb->dma_buf = buf;
	}
	BUG_ON(exynos_fb->dma_buf !=  buf);

	/* Synchronously wait for the frame to render */
	resource_list = get_dma_buf_kds_resource(buf);

	for (i = 0; i < KDS_WAITALL_MAX_TRIES; i++) {
		resource_set = kds_waitall(1, &shared, &resource_list,
				msecs_to_jiffies(1000));
		if (PTR_ERR(resource_set) != -ERESTARTSYS)
			break;
	}
	if (IS_ERR(resource_set)) {
		DRM_ERROR("kds_waitall failed with ret=%ld\n",
				PTR_ERR(resource_set));
		return;
	} else if (!resource_set) {
		DRM_ERROR("kds_waitall timed out\n");
		return;
	}

	/* Clean up the remaining resource so we're not holding onto anything */
	kds_resource_set_release(&resource_set);
}

static void exynos_drm_crtc_release_flips(struct drm_crtc *crtc)
{
	struct drm_device *drm_dev = crtc->dev;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_flip_desc *cur_desc = &exynos_crtc->scanout_desc;
	struct exynos_drm_flip_desc next_desc;
	unsigned int ret;

	if (cur_desc->kds)
		kds_resource_set_release(&cur_desc->kds);

	/* If there aren't any pending frames, be merry and exit */
	if (!kfifo_len(&exynos_crtc->flip_fifo))
		return;

	/* Loop through the fifo and drop all frames except the last */
	while (kfifo_len(&exynos_crtc->flip_fifo) > 1) {
		ret = kfifo_get(&exynos_crtc->flip_fifo, &next_desc);
		BUG_ON(!ret);

		if (next_desc.fb)
			exynos_drm_fb_put(to_exynos_fb(next_desc.fb));
		if (next_desc.kds)
			kds_resource_set_release(&next_desc.kds);
	}

	/* Now we'll promote the pending frame to front */
	ret = kfifo_get(&exynos_crtc->flip_fifo, &next_desc);
	BUG_ON(!ret);

	atomic_set(&exynos_crtc->flip_pending, 0);

	exynos_drm_crtc_wait_and_release_kds(&next_desc);

	to_exynos_fb(next_desc.fb)->rendered = true;
	exynos_drm_crtc_update(crtc, next_desc.fb);
	to_exynos_fb(next_desc.fb)->prepared = true;

	if (exynos_crtc->event) {
		exynos_drm_crtc_flip_complete(drm_dev, exynos_crtc->event);
		exynos_crtc->event = NULL;
	}

	*cur_desc = next_desc;
}
#endif

static void exynos_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	DRM_DEBUG_KMS("crtc[%d] mode[%d]\n", crtc->base.id, mode);

	if (exynos_crtc->dpms == mode) {
		DRM_DEBUG_KMS("desired dpms mode is same as previous one.\n");
		return;
	}

	exynos_drm_fn_encoder(crtc, &mode, exynos_drm_encoder_crtc_dpms);
	exynos_crtc->dpms = mode;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	if (mode != DRM_MODE_DPMS_ON)
		exynos_drm_crtc_release_flips(crtc);
#endif
}

static void exynos_drm_crtc_prepare(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* drm framework doesn't check NULL. */
}

static int exynos_drm_crtc_page_flip(struct drm_crtc *crtc,
				     struct drm_framebuffer *fb,
				     struct drm_pending_vblank_event *event);


static void exynos_drm_crtc_commit(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	exynos_drm_crtc_page_flip(crtc, crtc->fb, NULL);
}

static bool
exynos_drm_crtc_mode_fixup(struct drm_crtc *crtc,
			    const struct drm_display_mode *mode,
			    struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* drm framework doesn't check NULL */
	return true;
}

static int
exynos_drm_crtc_mode_set(struct drm_crtc *crtc, struct drm_display_mode *mode,
			  struct drm_display_mode *adjusted_mode, int x, int y,
			  struct drm_framebuffer *old_fb)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	int pipe = exynos_crtc->pipe;
	int ret;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	/*
	 * copy the mode data adjusted by mode_fixup() into crtc->mode
	 * so that hardware can be seet to proper mode.
	 */
	memcpy(&crtc->mode, adjusted_mode, sizeof(*adjusted_mode));

	/* We should never timeout here. */
	ret = wait_event_timeout(exynos_crtc->vsync_wq,
				 kfifo_is_empty(&exynos_crtc->flip_fifo),
				 DRM_HZ/20);
	if (!ret)
		DRM_ERROR("Timed out waiting for flips to complete\n");

	exynos_drm_fn_encoder(crtc, &pipe, exynos_drm_encoder_crtc_pipe);

	return 0;
}

static int exynos_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					  struct drm_framebuffer *old_fb)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	int ret;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* We should never timeout here. */
	ret = wait_event_timeout(exynos_crtc->vsync_wq,
				 kfifo_is_empty(&exynos_crtc->flip_fifo),
				 DRM_HZ/20);
	if (!ret)
		DRM_ERROR("Timed out waiting for flips to complete\n");

	exynos_drm_crtc_page_flip(crtc, crtc->fb, NULL);

	return 0;
}

static void exynos_drm_crtc_load_lut(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);
	/* drm framework doesn't check NULL */
}

static void exynos_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	exynos_plane_dpms(exynos_crtc->plane, DRM_MODE_DPMS_OFF);
}

static struct drm_crtc_helper_funcs exynos_crtc_helper_funcs = {
	.dpms		= exynos_drm_crtc_dpms,
	.prepare	= exynos_drm_crtc_prepare,
	.commit		= exynos_drm_crtc_commit,
	.mode_fixup	= exynos_drm_crtc_mode_fixup,
	.mode_set	= exynos_drm_crtc_mode_set,
	.mode_set_base	= exynos_drm_crtc_mode_set_base,
	.load_lut	= exynos_drm_crtc_load_lut,
	.disable	= exynos_drm_crtc_disable,
};

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
void exynos_drm_kds_callback(void *callback_parameter,
			     void *callback_extra_parameter)
{
	struct drm_framebuffer *fb = callback_parameter;
	struct drm_crtc *crtc = callback_extra_parameter;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);

	exynos_fb->rendered = true;

	if (!atomic_cmpxchg(&exynos_crtc->flip_pending, 0, 1)) {
		exynos_drm_crtc_update(crtc, fb);
		exynos_fb->prepared = true;
	}
}
#endif

static int exynos_drm_crtc_page_flip(struct drm_crtc *crtc,
				      struct drm_framebuffer *fb,
				      struct drm_pending_vblank_event *event)
{
	struct drm_device *dev = crtc->dev;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct exynos_drm_gem_obj *exynos_gem_obj;
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);
	struct exynos_drm_flip_desc flip_desc;
	bool send_event = false;
#endif
	int ret = -EINVAL;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* We don't support flipping to non-RGBA buffers. */
	if (exynos_drm_fb_get_buf_cnt(exynos_fb) != 1) {
		DRM_ERROR("called page_flip with non-RGBA buffer\n");
		return -EINVAL;
	}

	exynos_drm_crtc_dpms(crtc, DRM_MODE_DPMS_ON);

	/*
	 * the pipe from user always is 0 so we can set pipe number
	 * of current owner to event.
	 */
	if (event)
		event->pipe = exynos_crtc->pipe;

	ret = drm_vblank_get(dev, exynos_crtc->pipe);
	if (ret) {
		DRM_DEBUG("failed to acquire vblank counter\n");
		return ret;
	}

	exynos_drm_fb_get(exynos_fb);

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	flip_desc.fb = fb;
	exynos_fb->rendered = false;
	exynos_fb->prepared = false;
	if (kfifo_is_full(&exynos_crtc->flip_fifo)) {
		DRM_ERROR("flip queue: crtc already busy\n");
		ret = -EBUSY;
		goto fail_queue_full;
	}

	/* Send flip event if no flips pending. */
	BUG_ON(exynos_crtc->event);
	if (event && kfifo_is_empty(&exynos_crtc->flip_fifo))
		send_event = true;

	exynos_gem_obj = exynos_drm_fb_obj(exynos_fb, 0);
	if (exynos_gem_obj->base.export_dma_buf) {
		struct dma_buf *buf = exynos_gem_obj->base.export_dma_buf;
		struct kds_resource *res_list = get_dma_buf_kds_resource(buf);
		struct exynos_drm_private *dev_priv = dev->dev_private;
		unsigned long shared = 0UL;

		exynos_drm_fb_attach_dma_buf(exynos_fb, buf);

		/* Waiting for the KDS resource*/
		ret = kds_async_waitall(&flip_desc.kds, KDS_FLAG_LOCKED_WAIT,
					&dev_priv->kds_cb, fb, crtc, 1,
					&shared, &res_list);
		if (ret) {
			DRM_ERROR("kds_async_waitall failed: %d\n", ret);
			goto fail_kds;
		}
	} else {
		/*
		 * For normal page-flip (i.e. non-modeset) we should
		 * never be flipping a non-kds buffer.
		 */
		if (event)
			DRM_ERROR("flipping a non-kds buffer\n");
		flip_desc.kds = NULL;
		exynos_drm_kds_callback(fb, crtc);
	}

	if (send_event)
		exynos_drm_crtc_flip_complete(dev, event);
	else
		exynos_crtc->event = event;

	kfifo_put(&exynos_crtc->flip_fifo, &flip_desc);
	crtc->fb = fb;

	trace_exynos_flip_request(exynos_crtc->pipe);

	return 0;

fail_queue_full:
fail_kds:
	exynos_drm_fb_put(exynos_fb);
	drm_vblank_put(dev, exynos_crtc->pipe);
	return ret;
#else
	crtc->fb = fb;
	exynos_crtc->event = event;
	exynos_drm_crtc_update(crtc, fb);
	return 0;
#endif
}

static void exynos_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_private *private = crtc->dev->dev_private;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	private->crtc[exynos_crtc->pipe] = NULL;

	drm_crtc_cleanup(crtc);
	kfree(exynos_crtc);
}

static int exynos_drm_crtc_set_property(struct drm_crtc *crtc,
					struct drm_property *property,
					uint64_t val)
{
	struct drm_device *dev = crtc->dev;
	struct exynos_drm_private *dev_priv = dev->dev_private;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	DRM_DEBUG_KMS("%s\n", __func__);

	if (property == dev_priv->crtc_mode_property) {
		enum exynos_crtc_mode mode = val;

		if (mode == exynos_crtc->mode)
			return 0;

		exynos_crtc->mode = mode;

		switch (mode) {
		case CRTC_MODE_NORMAL:
			exynos_drm_crtc_commit(crtc);
			break;
		case CRTC_MODE_BLANK:
			exynos_plane_dpms(exynos_crtc->plane,
					  DRM_MODE_DPMS_OFF);
			break;
		default:
			break;
		}

		return 0;
	}

	return -EINVAL;
}

static struct drm_crtc_funcs exynos_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.page_flip	= exynos_drm_crtc_page_flip,
	.destroy	= exynos_drm_crtc_destroy,
	.set_property	= exynos_drm_crtc_set_property,
};

static const struct drm_prop_enum_list mode_names[] = {
	{ CRTC_MODE_NORMAL, "normal" },
	{ CRTC_MODE_BLANK, "blank" },
};

static void exynos_drm_crtc_attach_mode_property(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct exynos_drm_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	DRM_DEBUG_KMS("%s\n", __func__);

	prop = dev_priv->crtc_mode_property;
	if (!prop) {
		prop = drm_property_create_enum(dev, 0, "mode", mode_names,
						ARRAY_SIZE(mode_names));
		if (!prop)
			return;

		dev_priv->crtc_mode_property = prop;
	}

	drm_object_attach_property(&crtc->base, prop, 0);
}

int exynos_drm_crtc_create(struct drm_device *dev, unsigned int nr)
{
	struct exynos_drm_crtc *exynos_crtc;
	struct exynos_drm_private *private = dev->dev_private;
	struct drm_crtc *crtc;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	exynos_crtc = kzalloc(sizeof(*exynos_crtc), GFP_KERNEL);
	if (!exynos_crtc) {
		DRM_ERROR("failed to allocate exynos crtc\n");
		return -ENOMEM;
	}

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	INIT_KFIFO(exynos_crtc->flip_fifo);
	init_waitqueue_head(&exynos_crtc->vsync_wq);
#endif
	exynos_crtc->pipe = nr;
	exynos_crtc->dpms = DRM_MODE_DPMS_OFF;
	exynos_crtc->plane = exynos_plane_init(dev, 1 << nr, true);
	if (!exynos_crtc->plane) {
		kfree(exynos_crtc);
		return -ENOMEM;
	}

	crtc = &exynos_crtc->drm_crtc;
	exynos_crtc->plane->crtc = crtc;

	private->crtc[nr] = crtc;

	drm_crtc_init(dev, crtc, &exynos_crtc_funcs);
	drm_crtc_helper_add(crtc, &exynos_crtc_helper_funcs);

	exynos_drm_crtc_attach_mode_property(crtc);

	return 0;
}

int exynos_drm_crtc_enable_vblank(struct drm_device *dev, int crtc)
{
	struct exynos_drm_private *private = dev->dev_private;
	struct exynos_drm_crtc *exynos_crtc =
		to_exynos_crtc(private->crtc[crtc]);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (exynos_crtc->dpms != DRM_MODE_DPMS_ON)
		return -EPERM;

	exynos_drm_fn_encoder(private->crtc[crtc], &crtc,
			exynos_drm_enable_vblank);

	return 0;
}

void exynos_drm_crtc_disable_vblank(struct drm_device *dev, int crtc)
{
	struct exynos_drm_private *private = dev->dev_private;
	struct exynos_drm_crtc *exynos_crtc =
		to_exynos_crtc(private->crtc[crtc]);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (exynos_crtc->dpms != DRM_MODE_DPMS_ON)
		return;

	exynos_drm_fn_encoder(private->crtc[crtc], &crtc,
			exynos_drm_disable_vblank);
}

void exynos_drm_crtc_finish_pageflip(struct drm_device *dev, int crtc_idx)
{
	struct exynos_drm_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->crtc[crtc_idx];
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct exynos_drm_flip_desc *cur_descp = &exynos_crtc->scanout_desc;
	struct exynos_drm_flip_desc next_desc;
#endif

	DRM_DEBUG_KMS("%s\n", __FILE__);

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	if (!kfifo_peek(&exynos_crtc->flip_fifo, &next_desc))
		return;
	if (!to_exynos_fb(next_desc.fb)->prepared)
		return;
	if (!atomic_cmpxchg(&exynos_crtc->flip_pending, 1, 0))
		return;

	trace_exynos_flip_complete(crtc_idx);

	if (cur_descp->kds)
		kds_resource_set_release(&cur_descp->kds);
	if (cur_descp->fb)
		exynos_drm_fb_put(to_exynos_fb(cur_descp->fb));
	*cur_descp = next_desc;
	kfifo_skip(&exynos_crtc->flip_fifo);

	/*
	 * exynos_drm_kds_callback can't update the shadow registers if there
	 * is a pending flip ahead. So we do the update here.
	 */
	if (kfifo_peek(&exynos_crtc->flip_fifo, &next_desc)) {
		if (unlikely(to_exynos_fb(next_desc.fb)->rendered) &&
		    !atomic_cmpxchg(&exynos_crtc->flip_pending, 0, 1)) {
			exynos_drm_crtc_update(crtc, next_desc.fb);
			to_exynos_fb(next_desc.fb)->prepared = true;
		}
	}

	wake_up(&exynos_crtc->vsync_wq);
#endif

	if (exynos_crtc->event) {
		exynos_drm_crtc_flip_complete(dev, exynos_crtc->event);
		exynos_crtc->event = NULL;
	}

	drm_vblank_put(dev, exynos_crtc->pipe);
}
