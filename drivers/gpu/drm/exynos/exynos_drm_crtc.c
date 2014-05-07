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
 * @manager: the manager associated with this crtc
 * @event: vblank event that is currently queued for flip
 * @pipe: a crtc index created at load() with a new crtc object creation
 *	and the crtc object would be set to private->crtc array
 *	to get a crtc object corresponding to this pipe from private->crtc
 *	array when irq interrupt occured. the reason of using this pipe is that
 *	drm framework doesn't support multiple irq yet.
 *	we can refer to the crtc to current hardware interrupt occured through
 *	this pipe value.
 * @mode: store the crtc mode value
 */
struct exynos_drm_crtc {
	struct drm_crtc			drm_crtc;
	struct exynos_drm_manager	*manager;
	struct drm_pending_vblank_event *event;
	unsigned int			pipe;
	enum exynos_crtc_mode		mode;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	atomic_t			flip_pending;
	DECLARE_KFIFO(flip_fifo, struct exynos_drm_flip_desc, 2);
	struct exynos_drm_flip_desc	scanout_desc;
	wait_queue_head_t		vsync_wq;
#endif
};

static void exynos_drm_crtc_flip_complete(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	unsigned long flags;
	struct timeval now;

	do_gettimeofday(&now);

	spin_lock_irqsave(&dev->event_lock, flags);
	if (exynos_crtc->event) {
		struct drm_pending_vblank_event *e = exynos_crtc->event;
		e->event.sequence = 0;
		e->event.tv_sec = now.tv_sec;
		e->event.tv_usec = now.tv_usec;
		list_add_tail(&e->base.link, &e->base.file_priv->event_list);
		exynos_crtc->event = NULL;
		spin_unlock_irqrestore(&dev->event_lock, flags);
		wake_up_interruptible(&e->base.file_priv->event_wait);
	} else {
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

}

static void exynos_drm_crtc_update(struct drm_crtc *crtc,
				   struct drm_framebuffer *fb)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_manager *manager = exynos_crtc->manager;
	int ret;
	unsigned int crtc_w;
	unsigned int crtc_h;

	crtc_w = fb->width - crtc->x;
	crtc_h = fb->height - crtc->y;

	if (manager->ops->update) {
		ret = manager->ops->update(manager->ctx, crtc, fb);
		if (ret)
			DRM_ERROR("Failed to update crtc, ret=%d\n", ret);
	}
}

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
static void exynos_drm_crtc_try_do_flip(struct drm_crtc *crtc)
{
	struct exynos_drm_flip_desc next_desc;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	/*
	 * Barrier in case we've recently enqueued the head of the framebuffer
	 * in one thread, and it's had rendered=true set in another. Without
	 * the barrier, one thread could get "fifo empty, rendered is true",
	 * and the other could get "fifo non-empty, rendered is false".
	 *
	 * Also ensures we observe prepared and flip_pending strictly after any
	 * other thread has written to prepared (which would've followed from a
	 * successful cmpxchg of flip_pending)
	 */
	smp_mb();

	/*
	 * Only try the flip if there was a rendered framebuffer at the head of
	 * the fifo, and we're not already doing a flip.
	 *
	 * Note that we can only flip the head of flip_fifo since we must
	 * always do flips in the order they are enqueued.
	 *
	 * Note also that we must check if the fb has already been prepared to
	 * ensure we don't race with exynos_drm_crtc_finish_pageflip() between
	 * when it clears flip_pending and removes the head fb from the
	 * flip_fifo.  Otherwise we may try flipping an already-flipped fb just
	 * before it is removed from flip_fifo.  Since the fb would no longer
	 * be the head of the flip_fifo, flip_pending would never get cleared,
	 * blocking all future flips.
	 */
	if (kfifo_peek(&exynos_crtc->flip_fifo, &next_desc)) {
		struct exynos_drm_fb *exynos_fb = to_exynos_fb(next_desc.fb);
		if (exynos_fb->rendered &&
		    !(exynos_fb->prepared & (1ul << exynos_crtc->pipe)) &&
		    !atomic_cmpxchg(&exynos_crtc->flip_pending, 0, 1)) {
			exynos_drm_crtc_update(crtc, next_desc.fb);
			trace_exynos_page_flip_state(exynos_crtc->pipe,
					DRM_BASE_ID(next_desc.fb), "prepared");
			exynos_fb->prepared |= 1ul << exynos_crtc->pipe;
		}
	}

}

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
	buf = gem_ob->base.dma_buf;
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

	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(crtc));

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
			drm_framebuffer_unreference(next_desc.fb);
		if (next_desc.kds)
			kds_resource_set_release(&next_desc.kds);
		drm_vblank_put(drm_dev, exynos_crtc->pipe);
	}

	/* Now we'll promote the pending frame to front */
	ret = kfifo_get(&exynos_crtc->flip_fifo, &next_desc);
	BUG_ON(!ret);

	atomic_set(&exynos_crtc->flip_pending, 0);
	drm_vblank_put(drm_dev, exynos_crtc->pipe);

	exynos_drm_crtc_wait_and_release_kds(&next_desc);

	to_exynos_fb(next_desc.fb)->rendered = true;
	exynos_drm_crtc_update(crtc, next_desc.fb);
	to_exynos_fb(next_desc.fb)->prepared |= 1ul << exynos_crtc->pipe;

	exynos_drm_crtc_flip_complete(crtc);

	*cur_desc = next_desc;
}
#endif

static void exynos_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_manager *manager = exynos_crtc->manager;

	DRM_DEBUG_KMS("[CRTC:%d] [DPMS:%s]\n", DRM_BASE_ID(crtc),
			drm_get_dpms_name(mode));

	if (manager->ops->dpms)
		manager->ops->dpms(manager->ctx, mode);

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	if (mode != DRM_MODE_DPMS_ON)
		exynos_drm_crtc_release_flips(crtc);
#endif
}

static void exynos_drm_crtc_prepare(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(crtc));

	/* drm framework doesn't check NULL. */
}

static int exynos_drm_crtc_page_flip(struct drm_crtc *crtc,
				     struct drm_framebuffer *fb,
				     struct drm_pending_vblank_event *event,
				     uint32_t page_flip_flags);


static void exynos_drm_crtc_commit(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_manager *manager = exynos_crtc->manager;

	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(crtc));

	exynos_drm_crtc_page_flip(crtc, crtc->primary->fb, NULL, 0);

	if (manager->ops->commit)
		manager->ops->commit(manager->ctx);

	drm_vblank_post_modeset(crtc->dev, exynos_crtc->pipe);
}

static bool
exynos_drm_crtc_mode_fixup(struct drm_crtc *crtc,
			    const struct drm_display_mode *mode,
			    struct drm_display_mode *adjusted_mode)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_manager *manager = exynos_crtc->manager;

	DRM_DEBUG_KMS("[CRTC:%d] [MODE:%s]\n", DRM_BASE_ID(crtc), mode->name);

	if (manager->ops->mode_fixup)
		return manager->ops->mode_fixup(manager->ctx, mode,
				adjusted_mode);

	return true;
}

static int
exynos_drm_crtc_mode_set(struct drm_crtc *crtc, struct drm_display_mode *mode,
			  struct drm_display_mode *adjusted_mode, int x, int y,
			  struct drm_framebuffer *old_fb)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_manager *manager = exynos_crtc->manager;
	int ret;

	DRM_DEBUG_KMS("[CRTC:%d] [MODE:%s] [ADJ_MODE:%s] @ (%d, %d) [OLD_FB:%d]\n",
			DRM_BASE_ID(crtc), mode->name, adjusted_mode->name,
			x, y, DRM_BASE_ID(old_fb));

	drm_vblank_pre_modeset(crtc->dev, exynos_crtc->pipe);

	/* We should never timeout here. */
	ret = wait_event_timeout(exynos_crtc->vsync_wq,
				 kfifo_is_empty(&exynos_crtc->flip_fifo),
				 DRM_HZ/20);
	if (!ret)
		DRM_ERROR("Timed out waiting for flips to complete\n");

	/*
	 * manager mode_set() computes new register values for adjusted mode.
	 * These new registers values are saved in the manager ctx, and
	 * committed to hardware when we call manager->commit().
	 */
	if (manager->ops->mode_set)
		manager->ops->mode_set(manager->ctx, adjusted_mode);

	return 0;
}

static int exynos_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					  struct drm_framebuffer *old_fb)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	int ret;

	DRM_DEBUG_KMS("[CRTC:%d] @ (%d, %d) [OLD_FB:%d]\n",
			DRM_BASE_ID(crtc), x, y, DRM_BASE_ID(old_fb));

	drm_framebuffer_reference(crtc->primary->fb);

	/* We should never timeout here. */
	ret = wait_event_timeout(exynos_crtc->vsync_wq,
				 kfifo_is_empty(&exynos_crtc->flip_fifo),
				 DRM_HZ/20);
	if (!ret)
		DRM_ERROR("Timed out waiting for flips to complete\n");

	exynos_drm_crtc_page_flip(crtc, crtc->primary->fb, NULL, 0);

	drm_framebuffer_unreference(crtc->primary->fb);

	return 0;
}

static void exynos_drm_crtc_load_lut(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(crtc));
	/* drm framework doesn't check NULL */
}

static void exynos_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct drm_plane *plane;
	int ret;

	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(crtc));

	drm_for_each_legacy_plane(plane, &crtc->dev->mode_config.plane_list) {
		if (plane->crtc != crtc)
			continue;

		ret = plane->funcs->disable_plane(plane);
		if (ret)
			DRM_ERROR("Failed to disable plane %d\n", ret);
	}
}

static const struct drm_crtc_helper_funcs exynos_crtc_helper_funcs = {
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
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);

	trace_exynos_page_flip_state(to_exynos_crtc(crtc)->pipe,
			DRM_BASE_ID(fb), "rendered");
	exynos_fb->rendered = true;

	/*
	 * KDS callbacks can come out-of-order, particularly during job failure
	 * or abnormal termination.
	 *
	 * Hence, only try to do the head of the queue flip. Note: If the head
	 * is a different (but also rendered) framebuffer, then it is still
	 * safe to do this at this time, since either:
	 * a) the flip to the head fb will already be pending, or
	 * b) finish_pageflip is about to flip to the head fb anyway.
	 */
	exynos_drm_crtc_try_do_flip(crtc);
}
#endif

static int exynos_drm_crtc_page_flip(struct drm_crtc *crtc,
				     struct drm_framebuffer *fb,
				     struct drm_pending_vblank_event *event,
				     uint32_t page_flip_flags)
{
	struct drm_device *dev = crtc->dev;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct exynos_drm_gem_obj *exynos_gem_obj;
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);
	struct exynos_drm_flip_desc flip_desc;
	bool send_event = false;
	unsigned long flags;
#endif
	int ret = -EINVAL;

	DRM_DEBUG_KMS("[CRTC:%d] [FB:%d]\n", DRM_BASE_ID(crtc),
			DRM_BASE_ID(fb));

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

	drm_framebuffer_reference(fb);

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	flip_desc.fb = fb;
	exynos_fb->rendered = false;
	exynos_fb->prepared &= ~(1ul << exynos_crtc->pipe);
	if (kfifo_is_full(&exynos_crtc->flip_fifo)) {
		DRM_ERROR("flip queue: crtc already busy\n");
		ret = -EBUSY;
		goto fail_queue_full;
	}

	/* Send flip event if no flips pending. */
	if (event && kfifo_is_empty(&exynos_crtc->flip_fifo))
		send_event = true;

	exynos_gem_obj = exynos_drm_fb_obj(exynos_fb, 0);
	if (exynos_gem_obj->base.dma_buf) {
		struct dma_buf *buf = exynos_gem_obj->base.dma_buf;
		struct kds_resource *res_list = get_dma_buf_kds_resource(buf);
		struct exynos_drm_private *dev_priv = dev->dev_private;
		unsigned long shared = 0UL;

		exynos_drm_fb_attach_dma_buf(exynos_fb, buf);

		/* Waiting for the KDS resource*/
		trace_exynos_page_flip_state(exynos_crtc->pipe, DRM_BASE_ID(fb),
				"wait_kds");
		ret = kds_async_waitall(&flip_desc.kds,
					&dev_priv->kds_cb, fb, crtc, 1,
					&shared, &res_list);
		if (ret) {
			DRM_ERROR("kds_async_waitall failed: %d\n", ret);
			goto fail_kds;
		}
	} else {
		flip_desc.kds = NULL;
		exynos_drm_kds_callback(fb, crtc);
	}

	/* abuse event_lock to protect the pointer */
	spin_lock_irqsave(&dev->event_lock, flags);
	BUG_ON(exynos_crtc->event);
	exynos_crtc->event = event;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	if (send_event)
		exynos_drm_crtc_flip_complete(crtc);

	kfifo_put(&exynos_crtc->flip_fifo, &flip_desc);
	crtc->primary->fb = fb;

	/*
	 * The KDS callback could've been triggered before we put on the fifo,
	 * in which case we might need to take care of the flip ourselves
	 * (since the cb can only take care of flips that were put on the
	 * queue)
	 */
	exynos_drm_crtc_try_do_flip(crtc);

	return 0;

fail_queue_full:
fail_kds:
	drm_framebuffer_unreference(fb);
	drm_vblank_put(dev, exynos_crtc->pipe);
	return ret;
#else
	crtc->primary->fb = fb;
	exynos_crtc->event = event;
	exynos_drm_crtc_update(crtc, fb);
	return 0;
#endif
}

static void exynos_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_private *private = crtc->dev->dev_private;

	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(crtc));

	private->crtc[exynos_crtc->pipe] = NULL;

	drm_crtc_cleanup(crtc);
	kfree(exynos_crtc);
}

static int exynos_drm_crtc_set_property(struct drm_crtc *crtc,
					void *state,
					struct drm_property *property,
					uint64_t val,
					void *blob_data)
{
	struct drm_device *dev = crtc->dev;
	struct exynos_drm_private *dev_priv = dev->dev_private;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	DRM_DEBUG_KMS("[CRTC:%d] [PROPERTY:%s] = %llu\n", DRM_BASE_ID(crtc),
			property->name, val);

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
			exynos_drm_crtc_disable(crtc);
			break;
		default:
			break;
		}

		return 0;
	}

	return -EINVAL;
}

static const struct drm_crtc_funcs exynos_crtc_funcs = {
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

	prop = dev_priv->crtc_mode_property;
	if (!prop) {
		prop = drm_property_create_enum(dev, 0, "mode", mode_names,
						ARRAY_SIZE(mode_names));
		if (!prop)
			return;

		dev_priv->crtc_mode_property = prop;
	}

	drm_object_attach_property(&crtc->base, prop, 0);

	DRM_DEBUG_KMS("[CRTC:%d]: attached [PROPERTY:%s] == %llu\n",
			DRM_BASE_ID(crtc), prop->name, prop->values[0]);
}

int exynos_drm_crtc_create(struct exynos_drm_manager *manager)
{
	struct exynos_drm_crtc *exynos_crtc;
	struct exynos_drm_private *private = manager->drm_dev->dev_private;
	struct drm_crtc *crtc;

	DRM_DEBUG_KMS("[MANAGER:%s]\n",
			exynos_drm_output_type_name(manager->type));

	exynos_crtc = kzalloc(sizeof(*exynos_crtc), GFP_KERNEL);
	if (!exynos_crtc) {
		DRM_ERROR("failed to allocate exynos crtc\n");
		return -ENOMEM;
	}

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	INIT_KFIFO(exynos_crtc->flip_fifo);
	init_waitqueue_head(&exynos_crtc->vsync_wq);
#endif
	exynos_crtc->manager = manager;
	exynos_crtc->pipe = manager->pipe;

	crtc = &exynos_crtc->drm_crtc;

	private->crtc[manager->pipe] = crtc;

	drm_crtc_init(manager->drm_dev, crtc, &exynos_crtc_funcs);
	drm_crtc_helper_add(crtc, &exynos_crtc_helper_funcs);

	exynos_drm_crtc_attach_mode_property(crtc);

	DRM_DEBUG_KMS("Created [CRTC:%d] for [MANAGER:%s] on pipe:%d\n",
			DRM_BASE_ID(crtc),
			exynos_drm_output_type_name(manager->type),
			manager->pipe);

	manager->crtc = crtc;
	return 0;
}

int exynos_drm_crtc_enable_vblank(struct drm_device *dev, int pipe)
{
	struct exynos_drm_private *private = dev->dev_private;
	struct drm_crtc *crtc = private->crtc[pipe];
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_manager *manager = exynos_crtc->manager;
	int ret = 0;

	DRM_DEBUG_KMS("[CRTC:%d] pipe: %d\n", DRM_BASE_ID(crtc), pipe);

	if (manager->ops->enable_vblank)
		ret = manager->ops->enable_vblank(manager->ctx);

	DRM_DEBUG_KMS("enable_vblank(pipe: %d) = %d\n", pipe, ret);

	return ret;
}

void exynos_drm_crtc_disable_vblank(struct drm_device *dev, int pipe)
{
	struct exynos_drm_private *private = dev->dev_private;
	struct drm_crtc *crtc = private->crtc[pipe];
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_manager *manager = exynos_crtc->manager;

	DRM_DEBUG_KMS("[CRTC:%d] pipe: %d\n", DRM_BASE_ID(crtc), pipe);

	if (manager->ops->disable_vblank)
		manager->ops->disable_vblank(manager->ctx);
}

void exynos_drm_crtc_finish_pageflip(struct drm_device *dev, int pipe)
{
	struct exynos_drm_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->crtc[pipe];
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct exynos_drm_flip_desc *cur_descp = &exynos_crtc->scanout_desc;
	struct exynos_drm_flip_desc next_desc;
#endif

	DRM_DEBUG_KMS("[CRTC:%d] pipe: %d\n", DRM_BASE_ID(crtc), pipe);

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	if (!kfifo_peek(&exynos_crtc->flip_fifo, &next_desc))
		return;
	if (!(to_exynos_fb(next_desc.fb)->prepared & (1ul << exynos_crtc->pipe)))
		return;
	if (!atomic_cmpxchg(&exynos_crtc->flip_pending, 1, 0))
		return;

	trace_exynos_page_flip_state(pipe, DRM_BASE_ID(next_desc.fb),
			"flipped");

	if (cur_descp->kds)
		kds_resource_set_release(&cur_descp->kds);
	if (cur_descp->fb)
		drm_framebuffer_unreference(cur_descp->fb);
	*cur_descp = next_desc;
	kfifo_skip(&exynos_crtc->flip_fifo);

	/*
	 * exynos_drm_kds_callback can't update the shadow registers if there
	 * is a pending flip ahead. So we do the update here.
	 */
	exynos_drm_crtc_try_do_flip(crtc);

	wake_up(&exynos_crtc->vsync_wq);
#endif

	exynos_drm_crtc_flip_complete(crtc);

	drm_vblank_put(dev, exynos_crtc->pipe);
}
