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

#include <drm/drmP.h>
#include <drm/drm_atomic.h>

#if defined(CONFIG_DMA_SHARED_BUFFER_USES_KDS) || defined(CONFIG_DRM_DMA_SYNC)
#include <linux/dma-buf.h>
#endif
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
#include <linux/kds.h>
#endif

#include <drm/exynos_drm.h>
#include "exynos_drm_drv.h"
#if defined(CONFIG_DMA_SHARED_BUFFER_USES_KDS) || defined(CONFIG_DRM_DMA_SYNC)
#include "exynos_drm_fb.h"
#include "exynos_drm_gem.h"
#endif
#include "exynos_trace.h"

/*
 * This function is to get X or Y size shown via screen. This needs length and
 * start position of CRTC.
 *
 *      <--- length --->
 * CRTC ----------------
 *      ^ start        ^ end
 *
 * There are six cases from a to f.
 *
 *             <----- SCREEN ----->
 *             0                 last
 *   ----------|------------------|----------
 * CRTCs
 * a -------
 *        b -------
 *        c --------------------------
 *                 d --------
 *                           e -------
 *                                  f -------
 */
static int exynos_plane_get_size(int start, unsigned length, unsigned last)
{
	int end = start + length;
	int size = 0;

	if (start <= 0) {
		if (end > 0)
			size = min_t(unsigned, end, last);
	} else if (start <= last) {
		size = min_t(unsigned, last - start, length);
	}

	return size;
}

void exynos_sanitize_plane_coords(struct drm_plane *plane,
		struct drm_crtc *crtc)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);

	exynos_plane->crtc_w = exynos_plane_get_size(exynos_plane->crtc_x,
				exynos_plane->crtc_w, crtc->mode.hdisplay);
	exynos_plane->crtc_h = exynos_plane_get_size(exynos_plane->crtc_y,
				exynos_plane->crtc_h, crtc->mode.vdisplay);

	if (exynos_plane->crtc_x < 0) {
		if (exynos_plane->crtc_w > 0)
			exynos_plane->src_x -= exynos_plane->crtc_x;
		exynos_plane->crtc_x = 0;
	}

	if (exynos_plane->crtc_y < 0) {
		if (exynos_plane->crtc_h)
			exynos_plane->src_y -= exynos_plane->crtc_y;
		exynos_plane->crtc_y = 0;
	}

	exynos_plane->src_w = min(exynos_plane->src_w, exynos_plane->crtc_w);
	exynos_plane->src_h = min(exynos_plane->src_h, exynos_plane->crtc_h);
}

void exynos_plane_copy_state(struct exynos_drm_plane *src,
		struct exynos_drm_plane *dst)
{
	dst->ctx = src->ctx;
	dst->crtc_x = src->crtc_x;
	dst->crtc_y = src->crtc_y;
	dst->crtc_w = src->crtc_w;
	dst->crtc_h = src->crtc_h;
	dst->src_x = src->src_x;
	dst->src_y = src->src_y;
	dst->src_w = src->src_w;
	dst->src_h = src->src_h;
}

struct sync_callback_cookie {
	struct drm_plane *plane;
	struct drm_crtc *crtc;
	struct drm_framebuffer *fb;
#ifdef CONFIG_DRM_DMA_SYNC
	struct drm_reservation_cb rcb;
#endif
};

void exynos_drm_crtc_send_event(struct drm_plane *plane, struct drm_crtc *crtc)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct drm_pending_vblank_event *event = exynos_plane->pending_event;
	unsigned long flags;
	struct timeval now;

	if (!exynos_plane->pending_event)
		return;

	do_gettimeofday(&now);

	spin_lock_irqsave(&plane->dev->event_lock, flags);

	event->pipe = exynos_drm_pipe_from_crtc(crtc);
	event->event.sequence = 0;
	event->event.tv_sec = now.tv_sec;
	event->event.tv_usec = now.tv_usec;
	list_add_tail(&event->base.link, &event->base.file_priv->event_list);
	wake_up_interruptible(&event->base.file_priv->event_wait);

	spin_unlock_irqrestore(&plane->dev->event_lock, flags);

	exynos_plane->pending_event = NULL;
}

void exynos_plane_helper_finish_update(struct drm_plane *plane,
		struct drm_crtc *crtc, bool update_fb)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct drm_framebuffer *old_fb;

	if (crtc)
		drm_vblank_put(crtc->dev, exynos_drm_pipe_from_crtc(crtc));

	old_fb = exynos_plane->fb;

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	if (exynos_plane->kds) {
		kds_resource_set_release(&exynos_plane->kds);
		exynos_plane->kds = NULL;
	}

	if (exynos_plane->kds_cb.user_cb)
		kds_callback_term(&exynos_plane->kds_cb);
#endif
#ifdef CONFIG_DRM_DMA_SYNC
	drm_fence_signal_and_put(&exynos_plane->fence);
#endif
	if (update_fb)
		exynos_plane->fb = exynos_plane->pending_fb;

	exynos_plane->pending_fb = NULL;

	if (crtc)
		exynos_drm_crtc_send_event(plane, crtc);

	if (old_fb)
		drm_framebuffer_unreference(old_fb);

	complete(&exynos_plane->completion);
}

static void exynos_plane_helper_commit_cb(void *cookie, void *unused)
{
	struct sync_callback_cookie *sync_cookie = cookie;
	struct drm_plane *plane = sync_cookie->plane;
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct drm_framebuffer *fb = sync_cookie->fb;
	struct drm_crtc *crtc = sync_cookie->crtc;
	int ret;

	kfree(sync_cookie);

	drm_vblank_get(crtc->dev, exynos_drm_pipe_from_crtc(crtc));

	ret = exynos_plane->helper_funcs->commit_plane(plane, crtc, fb);
	if (ret)
		goto err;

	/*
	 * If the fb is already on the screen, finish the commit early. If the
	 * fb is not on the screen, mark it as pending and wait for the crtc
	 * driver to put it on the screen and call finish_update
	 */
	if (exynos_plane->fb == fb)
		exynos_plane_helper_finish_update(&exynos_plane->base, crtc,
				false);
	else
		exynos_plane->pending_fb = fb;

	return;
err:
	/*
	 * If we have an error on commit, that's bad news for a few reasons:
	 *	- there's no way to communicate that back up to userspace
	 *	- we can't restore the previous fb
	 *	- our sw state will not match hw state
	 * We'll try to clean things as best we can, and maybe limp along, but
	 * there's a bug somewhere if you're in here.
	 */
	WARN(1, "Plane commit failed");

	exynos_plane_helper_finish_update(&exynos_plane->base, crtc, true);
}

#ifdef CONFIG_DRM_DMA_SYNC
static void exynos_plane_reservation_cb(struct drm_reservation_cb *rcb,
					void *cookie)
{
	exynos_plane_helper_commit_cb(cookie, NULL);
}
#endif

int exynos_plane_helper_freeze_plane(struct drm_plane *plane)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	int ret;

	wait_for_completion(&exynos_plane->completion);

	ret = exynos_plane->helper_funcs->disable_plane(plane);

	complete(&exynos_plane->completion);

	return ret;
}

void exynos_plane_helper_thaw_plane(struct drm_plane *plane,
	struct drm_crtc *crtc)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct drm_framebuffer *fb;

	wait_for_completion(&exynos_plane->completion);

	fb = exynos_plane->fb;

	/* If the plane has an fb, commit it and then set it as pending so we
	 * don't signal the comletion until it's actually up on the screen.
	 * Otherwise, it should just stay disabled and we'll signal completion
	 * immediately.
	 */
	if (fb) {
		drm_vblank_get(crtc->dev, exynos_drm_pipe_from_crtc(crtc));

		exynos_plane->helper_funcs->commit_plane(plane, crtc, fb);

		/* Take a reference here since we'll drop it in finish_update */
		drm_framebuffer_reference(fb);

		exynos_plane->pending_fb = fb;
	} else {
		complete(&exynos_plane->completion);
	}
}

void exynos_plane_helper_init(struct drm_plane *plane,
			const struct exynos_plane_helper_funcs *funcs)

{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);

	exynos_plane->helper_funcs = funcs;

#ifdef CONFIG_DRM_DMA_SYNC
	exynos_plane->fence_context = fence_context_alloc(1);
	atomic_set(&exynos_plane->fence_seqno, 0);
#endif

	init_completion(&exynos_plane->completion);

	/* Start this completed so the first wait_for_completion is a noop */
	complete(&exynos_plane->completion);
}

#ifdef CONFIG_DRM_DMA_SYNC
static int exynos_drm_plane_update_sync(struct reservation_object *resv,
					struct sync_callback_cookie *cookie)
{
	struct drm_plane *plane = cookie->plane;
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct fence *fence;
	int ret = 0;

	ww_mutex_lock(&resv->lock, NULL);
	ret = reservation_object_reserve_shared(resv);
	if (ret < 0) {
		DRM_ERROR("Reserving space for shared fence failed: %d.\n",
			ret);
		ww_mutex_unlock(&resv->lock);
		return ret;
	}
	fence = drm_sw_fence_new(exynos_plane->fence_context,
				 atomic_add_return(1,
					&exynos_plane->fence_seqno));
	if (IS_ERR(fence)) {
		ret = PTR_ERR(fence);
		DRM_ERROR("Failed to create fence: %d.\n", ret);
		ww_mutex_unlock(&resv->lock);
		return ret;
	}
	exynos_plane->fence = fence;
	drm_reservation_cb_init(&cookie->rcb,
				exynos_plane_reservation_cb,
				cookie);
	ret = drm_reservation_cb_add(&cookie->rcb,
				     resv, false);
	if (ret < 0) {
		DRM_ERROR("Adding reservation to callback failed: %d.\n", ret);
		fence_put(exynos_plane->fence);
		exynos_plane->fence = NULL;
		ww_mutex_unlock(&resv->lock);
		return ret;
	}
	drm_reservation_cb_done(&cookie->rcb);
	reservation_object_add_shared_fence(resv,
					exynos_plane->fence);
	ww_mutex_unlock(&resv->lock);
	return ret;
}
#endif

static int do_update_plane(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb, int crtc_x,
		int crtc_y, unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y, uint32_t src_w, uint32_t src_h)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
#if defined(CONFIG_DMA_SHARED_BUFFER_USES_KDS) || defined(CONFIG_DRM_DMA_SYNC)
	int ret;
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);
	struct exynos_drm_gem_obj *exynos_gem_obj;
	struct dma_buf *buf;
	struct sync_callback_cookie *cookie;
#endif
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct kds_resource *res_list;
	unsigned long shared = 0UL;
#endif

	/* Copy the plane parameters so we can restore it later */
	exynos_plane->crtc_x = crtc_x;
	exynos_plane->crtc_y = crtc_y;
	exynos_plane->crtc_w = crtc_w;
	exynos_plane->crtc_h = crtc_h;
	exynos_plane->src_x = src_x >> 16;
	exynos_plane->src_y = src_y >> 16;
	exynos_plane->src_w = src_w >> 16;
	exynos_plane->src_h = src_h >> 16;

	exynos_sanitize_plane_coords(plane, crtc);

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie) {
		DRM_ERROR("Failed to allocate kds cookie\n");
		return -ENOMEM;
	}
	cookie->plane = plane;
	cookie->crtc = crtc;

	/* This reference is released once the fb is removed from the screen */
	drm_framebuffer_reference(fb);

	cookie->fb = fb;

#if defined(CONFIG_DMA_SHARED_BUFFER_USES_KDS)
	BUG_ON(exynos_plane->kds);

	exynos_gem_obj = exynos_drm_fb_obj(exynos_fb, 0);
	if (!exynos_gem_obj->base.dma_buf) {
		exynos_plane_helper_commit_cb(cookie, NULL);
		return 0;
	}

	ret = kds_callback_init(&exynos_plane->kds_cb, 1,
			exynos_plane_helper_commit_cb);
	if (ret) {
		DRM_ERROR("Failed to initialize kds callback ret=%d\n", ret);
		goto err;
	}

	buf = exynos_gem_obj->base.dma_buf;
	res_list = get_dma_buf_kds_resource(buf);

	exynos_drm_fb_attach_dma_buf(exynos_fb, buf);

	/* Waiting for the KDS resource*/
	trace_exynos_page_flip_state(exynos_drm_pipe_from_crtc(crtc),
			DRM_BASE_ID(fb), "wait_kds");

	ret = kds_async_waitall(&exynos_plane->kds, &exynos_plane->kds_cb,
			cookie, NULL, 1, &shared, &res_list);
	if (ret) {
		DRM_ERROR("Failed kds waitall ret=%d\n", ret);
		goto err;
	}

	return 0;

err:
	drm_framebuffer_unreference(fb);
	kfree(cookie);

	return ret;
#elif defined(CONFIG_DRM_DMA_SYNC)
	BUG_ON(exynos_plane->fence);

	exynos_gem_obj = exynos_drm_fb_obj(exynos_fb, 0);
	if (!exynos_gem_obj->base.dma_buf) {
		exynos_plane_helper_commit_cb(cookie, NULL);
		return 0;
	}

	buf = exynos_gem_obj->base.dma_buf;
	exynos_drm_fb_attach_dma_buf(exynos_fb, buf);

	trace_exynos_page_flip_state(exynos_drm_pipe_from_crtc(crtc),
			DRM_BASE_ID(fb), "wait_resv");

	ret = exynos_drm_plane_update_sync(buf->resv, cookie);
	if (ret) {
		DRM_ERROR("Failed dma sync ret=%d\n", ret);
		goto err;
	}

	return 0;

err:
	drm_framebuffer_unreference(fb);
	kfree(cookie);

	return ret;

#else
	/*
	 * If we don't have dma buffer synchronization,
	 * just put the fb on the plane.
	 */
	exynos_plane_helper_commit_cb(cookie, NULL);
	return 0;
#endif
}

int exynos_plane_helper_update_plane_with_event(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		struct drm_pending_vblank_event *event, int crtc_x,
		int crtc_y, unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y, uint32_t src_w, uint32_t src_h)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_plane old_plane;
	int ret;

	wait_for_completion(&exynos_plane->completion);

	exynos_plane_copy_state(exynos_plane, &old_plane);

	if (event) {
		WARN_ON(exynos_plane->pending_event);
		exynos_plane->pending_event = event;
	}

	ret = do_update_plane(plane, crtc, fb, crtc_x, crtc_y, crtc_w, crtc_h,
		src_x, src_y, src_w, src_h);
	if (ret)
		goto err;

	return 0;

err:
	WARN_ON(exynos_plane->pending_event != event);
	exynos_plane_copy_state(&old_plane, exynos_plane);
	if (event)
		exynos_plane->pending_event = NULL;
	complete(&exynos_plane->completion);

	return ret;
}

int exynos_plane_helper_update_plane(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb, int crtc_x,
		int crtc_y, unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y, uint32_t src_w, uint32_t src_h)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);
	struct exynos_drm_plane old_plane;
	int ret;

	wait_for_completion(&exynos_plane->completion);

	exynos_plane_copy_state(exynos_plane, &old_plane);

	ret = do_update_plane(plane, crtc, fb, crtc_x, crtc_y, crtc_w, crtc_h,
		src_x, src_y, src_w, src_h);
	if (ret)
		goto err;

	return 0;

err:
	exynos_plane_copy_state(&old_plane, exynos_plane);
	complete(&exynos_plane->completion);

	return ret;
}

int exynos_plane_helper_disable_plane(struct drm_plane *plane)
{
	struct exynos_drm_plane *exynos_plane = to_exynos_plane(plane);

	wait_for_completion(&exynos_plane->completion);

	/* We shouldn't have anything pending at this point */
	BUG_ON(exynos_plane->pending_fb);

	if (!exynos_plane->fb)
		goto out;

	exynos_plane->helper_funcs->disable_plane(plane);

out:
	/* Finish any updates that were unfinished and clean up references */
	exynos_plane_helper_finish_update(plane, NULL, true);

	return 0;
}
