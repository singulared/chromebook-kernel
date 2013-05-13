/* exynos_drm_crtc.c
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

#include <linux/kfifo.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_encoder.h"
#include "exynos_drm_display.h"
#include "exynos_drm_gem.h"
#include "exynos_trace.h"
#include "exynos_drm_plane.h"

#define KDS_WAITALL_MAX_TRIES 15

struct exynos_drm_flip_desc {
	struct drm_framebuffer	*fb;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct kds_resource_set	*kds;
#endif
};

/*
 * Exynos specific crtc structure.
 *
 * @drm_crtc: crtc object.
 * @current_fb: current fb that is being scanned out
 * @event: vblank event that is currently queued for flip
 * @plane: pointer of private plane object for this crtc
 * @pipe: a crtc index created at load() with a new crtc object creation
 *	and the crtc object would be set to private->crtc array
 *	to get a crtc object corresponding to this pipe from private->crtc
 *	array when irq interrupt occured. the reason of using this pipe is that
 *	drm framework doesn't support multiple irq yet.
 *	we can refer to the crtc to current hardware interrupt occured through
 *	this pipe value.
 * @flip_pending: there is a flip pending that we need to process next vblank
 */
struct exynos_drm_crtc {
	struct drm_crtc			drm_crtc;
	struct drm_pending_vblank_event *event;
	DECLARE_KFIFO(flip_fifo, struct exynos_drm_flip_desc, 2);
	struct exynos_drm_flip_desc	scanout_desc;
	struct exynos_drm_display	*display;
	struct drm_plane		*plane;
	unsigned int			pipe;
	atomic_t			flip_pending;
};

#define to_exynos_crtc(x)	container_of(x, struct exynos_drm_crtc,\
				drm_crtc)

void exynos_drm_crtc_apply(struct drm_crtc *crtc,
		struct exynos_drm_overlay *overlay)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_display *display = exynos_crtc->display;

	if (display->controller_ops && display->controller_ops->mode_set)
		display->controller_ops->mode_set(display->controller_ctx,
				overlay);

	display->pipe = exynos_crtc->pipe;

	if (display->controller_ops && display->controller_ops->win_commit)
		display->controller_ops->win_commit(display->controller_ctx,
				overlay->zpos);
}

void exynos_drm_overlay_update(struct exynos_drm_overlay *overlay,
			       struct drm_framebuffer *fb,
			       struct drm_display_mode *mode,
			       struct exynos_drm_crtc_pos *pos)
{
	struct exynos_drm_gem_buf *buffer;
	unsigned int actual_w;
	unsigned int actual_h;
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);
	int nr = exynos_drm_format_num_buffers(fb->pixel_format);
	int i;

	for (i = 0; i < nr; i++) {
		buffer = exynos_drm_fb_buffer(exynos_fb, i);

		overlay->dma_addr[i] = buffer->dma_addr;

		DRM_DEBUG_KMS("buffer: %d, dma_addr = 0x%lx\n",
				i, (unsigned long)overlay->dma_addr[i]);
	}

	actual_w = min((mode->hdisplay - pos->crtc_x), pos->crtc_w);
	actual_h = min((mode->vdisplay - pos->crtc_y), pos->crtc_h);

	/* set drm framebuffer data. */
	overlay->fb_x = pos->fb_x;
	overlay->fb_y = pos->fb_y;
	overlay->fb_width = min(pos->fb_w, actual_w);
	overlay->fb_height = min(pos->fb_h, actual_h);
	overlay->fb_pitch = fb->pitches[0];
	overlay->bpp = fb->bits_per_pixel;
	overlay->pixel_format = fb->pixel_format;

	/* set overlay range to be displayed. */
	overlay->crtc_x = pos->crtc_x;
	overlay->crtc_y = pos->crtc_y;
	overlay->crtc_width = actual_w;
	overlay->crtc_height = actual_h;
	overlay->crtc_htotal = mode->crtc_htotal;
	overlay->crtc_hsync_len = mode->hsync_end - mode->hsync_start;
	overlay->crtc_vtotal = mode->crtc_vtotal;
	overlay->crtc_vsync_len = mode->vsync_end - mode->vsync_start;

	/* set drm mode data. */
	overlay->mode_width = mode->hdisplay;
	overlay->mode_height = mode->vdisplay;
	overlay->refresh = mode->vrefresh;
	overlay->scan_flag = mode->flags;

	DRM_DEBUG_KMS("overlay : offset_x/y(%d,%d), width/height(%d,%d)",
			overlay->crtc_x, overlay->crtc_y,
			overlay->crtc_width, overlay->crtc_height);
}

void exynos_drm_overlay_disable(struct drm_crtc *crtc, int zpos)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_display *display = exynos_crtc->display;

	if (display->controller_ops && display->controller_ops->win_disable)
		display->controller_ops->win_disable(display->controller_ctx,
				zpos);
}

static void exynos_drm_crtc_update(struct drm_crtc *crtc,
				   struct drm_framebuffer *fb)
{
	struct exynos_drm_crtc *exynos_crtc;
	struct exynos_drm_overlay *overlay;
	struct exynos_drm_crtc_pos pos;
	struct drm_display_mode *mode = &crtc->mode;

	exynos_crtc = to_exynos_crtc(crtc);
	overlay = get_exynos_drm_overlay(exynos_crtc->plane);

	memset(&pos, 0, sizeof(struct exynos_drm_crtc_pos));

	/* it means the offset of framebuffer to be displayed. */
	pos.fb_x = crtc->x;
	pos.fb_y = crtc->y;
	pos.fb_w = fb->width;
	pos.fb_h = fb->height;

	/* OSD position to be displayed. */
	pos.crtc_x = 0;
	pos.crtc_y = 0;
	pos.crtc_w = fb->width - crtc->x;
	pos.crtc_h = fb->height - crtc->y;

	exynos_drm_overlay_update(overlay, fb, mode, &pos);
}

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
	exynos_drm_crtc_apply(crtc, get_exynos_drm_overlay(exynos_crtc->plane));
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
	struct exynos_drm_display *display = exynos_crtc->display;

	DRM_DEBUG_KMS("[CRTC:%d] [DPMS:%s]\n", DRM_BASE_ID(crtc),
			drm_get_dpms_name(mode));

	if (display->controller_ops && display->controller_ops->dpms)
		display->controller_ops->dpms(display->controller_ctx, mode);

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
				     struct drm_pending_vblank_event *event);

static void exynos_drm_crtc_commit(struct drm_crtc *crtc)
{
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_display *display = exynos_crtc->display;
	int ret;

	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(crtc));

	/*
	 * when set_crtc is requested from user or at booting time,
	 * crtc->commit would be called without dpms call so crtc->dpms
	 * should be called with DRM_MODE_DPMS_ON for the controller power
	 * to be on.
	 */
	exynos_drm_crtc_dpms(crtc, DRM_MODE_DPMS_ON);

	ret = exynos_drm_crtc_page_flip(crtc, crtc->fb, NULL);
	if (ret)
		DRM_ERROR("page_flip failed\n");

	if (display->controller_ops && display->controller_ops->commit)
		display->controller_ops->commit(display->controller_ctx);
}

static bool
exynos_drm_crtc_mode_fixup(struct drm_crtc *crtc,
			    struct drm_display_mode *mode,
			    struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_KMS("[CRTC:%d] [MODE:%d:%s]\n", DRM_BASE_ID(crtc),
			DRM_BASE_ID(mode), mode->name);

	/* drm framework doesn't check NULL */
	return true;
}

static int
exynos_drm_crtc_mode_set(struct drm_crtc *crtc, struct drm_display_mode *mode,
			  struct drm_display_mode *adjusted_mode, int x, int y,
			  struct drm_framebuffer *old_fb)
{
	struct exynos_drm_private *dev_priv = crtc->dev->dev_private;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_display *display = exynos_crtc->display;
	struct drm_framebuffer *fb = crtc->fb;
	int ret;

	DRM_DEBUG_KMS("[CRTC:%d] [MODE:%d:%s] @ (%d, %d) [OLD_FB:%d]\n",
			DRM_BASE_ID(crtc), DRM_BASE_ID(mode), mode->name, x, y,
			DRM_BASE_ID(old_fb));

	if (!fb)
		return -EINVAL;

	/*
	 * copy the mode data adjusted by mode_fixup() into crtc->mode
	 * so that hardware can be seet to proper mode.
	 */
	memcpy(&crtc->mode, adjusted_mode, sizeof(*adjusted_mode));

	/* We should never timeout here. */
	ret = wait_event_timeout(dev_priv->wait_vsync_queue,
				 kfifo_is_empty(&exynos_crtc->flip_fifo),
				 DRM_HZ/20);
	if (!ret)
		DRM_ERROR("Timed out waiting for flips to complete\n");

	exynos_drm_crtc_update(crtc, fb);

	if (display->controller_ops && display->controller_ops->mode_set)
		display->controller_ops->mode_set(display->controller_ctx,
				get_exynos_drm_overlay(exynos_crtc->plane));

	return 0;
}

static int exynos_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					  struct drm_framebuffer *old_fb)
{
	struct drm_device *dev = crtc->dev;
	struct exynos_drm_private *dev_priv = crtc->dev->dev_private;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct drm_framebuffer *fb = crtc->fb;
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);
	int ret;

	DRM_DEBUG_KMS("[CRTC:%d] @ (%d, %d) [OLD_FB:%d]\n",
			DRM_BASE_ID(crtc), x, y, DRM_BASE_ID(old_fb));

	if (!fb)
		return -EINVAL;

	/* Make sure the framebuffer doesn't disappear while we wait */
	exynos_drm_fb_get(exynos_fb);

	/*
	 * Force DPMS ON, which allows us to enables vblank irqs which
	 * we need to drain the flip fifo below while we wait.
	 */
	exynos_drm_crtc_dpms(crtc, DRM_MODE_DPMS_ON);

	ret = drm_vblank_get(dev, exynos_crtc->pipe);
	if (ret)
		DRM_ERROR("Unable to get vblank\n");

	/* We should never timeout here. */
	ret = wait_event_timeout(dev_priv->wait_vsync_queue,
				 kfifo_is_empty(&exynos_crtc->flip_fifo),
				 DRM_HZ/20);
	if (!ret)
		DRM_ERROR("Timed out waiting for flips to complete\n");

	drm_vblank_put(dev, exynos_crtc->pipe);

	ret = exynos_drm_crtc_page_flip(crtc, fb, NULL);

	exynos_drm_fb_put(exynos_fb);

	if (ret)
		DRM_ERROR("page_flip failed\n");

	return ret;
}

static void exynos_drm_crtc_load_lut(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("[CRTC:%d]\n", DRM_BASE_ID(crtc));
	/* drm framework doesn't check NULL */
}

static struct drm_crtc_helper_funcs exynos_crtc_helper_funcs = {
	.dpms		= exynos_drm_crtc_dpms,
	.prepare	= exynos_drm_crtc_prepare,
	.commit		= exynos_drm_crtc_commit,
	.mode_fixup	= exynos_drm_crtc_mode_fixup,
	.mode_set	= exynos_drm_crtc_mode_set,
	.mode_set_base	= exynos_drm_crtc_mode_set_base,
	.load_lut	= exynos_drm_crtc_load_lut,
};

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
void exynos_drm_kds_callback(void *callback_parameter, void *callback_extra_parameter)
{
	struct drm_framebuffer	*fb = callback_parameter;
	struct drm_crtc *crtc =  callback_extra_parameter;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);

	/*
	 * Under rare circumstances (modeset followed by flip) it is possible
	 * to have two fbs ready in time for the same vblank. We don't want to
	 * drop the latter fb because it is most recent. We can't drop the
	 * former either because we've already applied it and the mixer does
	 * not allow applying more than one frame on a vblank. So instead of
	 * dropping a frame, we store the latter frame in the extra slot and
	 * apply it on the next vblank.
	 */

	to_exynos_fb(fb)->rendered = true;

	if (!atomic_cmpxchg(&exynos_crtc->flip_pending, 0, 1)) {
		struct exynos_drm_overlay *overlay;

		overlay = get_exynos_drm_overlay(exynos_crtc->plane);
		exynos_drm_crtc_update(crtc, fb);
		exynos_drm_crtc_apply(crtc, overlay);
		to_exynos_fb(fb)->prepared = true;
	}
}
#endif

static int exynos_drm_crtc_page_flip(struct drm_crtc *crtc,
				     struct drm_framebuffer *fb,
				     struct drm_pending_vblank_event *event)
{
	struct drm_device *dev = crtc->dev;
	struct exynos_drm_private *dev_priv = dev->dev_private;
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_flip_desc flip_desc;
	bool send_event = false;
	int ret;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);
	struct exynos_drm_gem_obj *gem_ob = (struct exynos_drm_gem_obj *)exynos_fb->exynos_gem_obj[0];
#endif
	DRM_DEBUG_KMS("[CRTC:%d] [FB:%d]\n", DRM_BASE_ID(crtc),
			DRM_BASE_ID(fb));

	/*
	 * the pipe from user always is 0 so we can set pipe number
	 * of current owner to event.
	 */
	if (event)
		event->pipe = exynos_crtc->pipe;

	ret = drm_vblank_get(dev, exynos_crtc->pipe);
	if (ret) {
		DRM_ERROR("Unable to get vblank\n");
		return -EINVAL;
	}

	if (kfifo_is_full(&exynos_crtc->flip_fifo)) {
		DRM_DEBUG_DRIVER("flip queue: crtc already busy\n");
		ret = -EBUSY;
		goto fail_queue_full;
	}

	/* Send flip event if no flips pending. */
	BUG_ON(exynos_crtc->event);
	if (kfifo_is_empty(&exynos_crtc->flip_fifo))
		send_event = true;

	exynos_drm_fb_get(exynos_fb);

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	flip_desc.fb = fb;
	exynos_fb->rendered = false;
	exynos_fb->prepared = false;
	if (gem_ob->base.export_dma_buf) {
		struct dma_buf *buf = gem_ob->base.export_dma_buf;
		unsigned long shared = 0UL;
		struct kds_resource *res_list = get_dma_buf_kds_resource(buf);

		/*
		 * If we don't already have a reference to the dma_buf,
		 * grab one now. We'll release it in exynos_drm_fb_destory().
		 */
		if (!exynos_fb->dma_buf) {
			get_dma_buf(buf);
			exynos_fb->dma_buf = buf;
		}
		BUG_ON(exynos_fb->dma_buf !=  buf);

		/* Waiting for the KDS resource*/
		ret = kds_async_waitall(&flip_desc.kds,
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
#endif

	if (event) {
		if (send_event)
			exynos_drm_crtc_flip_complete(dev, event);
		else
			exynos_crtc->event = event;
	}
	kfifo_put(&exynos_crtc->flip_fifo, &flip_desc);
	crtc->fb = fb;

	trace_exynos_flip_request(exynos_crtc->pipe);

	return 0;

fail_kds:
	exynos_drm_fb_put(exynos_fb);
fail_queue_full:
	drm_vblank_put(dev, exynos_crtc->pipe);
	return ret;
}

void exynos_drm_crtc_finish_pageflip(struct drm_device *drm_dev, int crtc_idx)
{
	struct exynos_drm_private *dev_priv = drm_dev->dev_private;
	struct drm_crtc *crtc = dev_priv->crtc[crtc_idx];
	struct exynos_drm_crtc *exynos_crtc = to_exynos_crtc(crtc);
	struct exynos_drm_flip_desc *cur_descp = &exynos_crtc->scanout_desc;
	struct exynos_drm_flip_desc next_desc;

	/* set wait vsync wake up queue. */
	DRM_WAKEUP(&dev_priv->wait_vsync_queue);

	if (!atomic_read(&exynos_crtc->flip_pending))
		return;
	if (!kfifo_peek(&exynos_crtc->flip_fifo, &next_desc))
		return;
	if (!to_exynos_fb(next_desc.fb)->prepared)
		return;
	if (!atomic_cmpxchg(&exynos_crtc->flip_pending, 1, 0))
		return;

	trace_exynos_flip_complete(crtc_idx);

	if (cur_descp->fb)
		exynos_drm_fb_put(to_exynos_fb(cur_descp->fb));
	if (cur_descp->kds)
		kds_resource_set_release(&cur_descp->kds);
	*cur_descp = next_desc;
	kfifo_skip(&exynos_crtc->flip_fifo);

	if (exynos_crtc->event) {
		exynos_drm_crtc_flip_complete(drm_dev, exynos_crtc->event);
		exynos_crtc->event = NULL;
	}

	/*
	 * exynos_drm_kds_callback can't update the shadow registers if there
	 * is a pending flip ahead. So we do the update here.
	 */
	if (kfifo_peek(&exynos_crtc->flip_fifo, &next_desc)) {
		if (unlikely(to_exynos_fb(next_desc.fb)->rendered) &&
		    !atomic_cmpxchg(&exynos_crtc->flip_pending, 0, 1)) {
			struct exynos_drm_overlay *overlay;

			overlay = get_exynos_drm_overlay(exynos_crtc->plane);
			exynos_drm_crtc_update(crtc, next_desc.fb);
			exynos_drm_crtc_apply(crtc, overlay);
			to_exynos_fb(next_desc.fb)->prepared = true;
		}
	}

	drm_vblank_put(drm_dev, crtc_idx);
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

static struct drm_crtc_funcs exynos_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.page_flip	= exynos_drm_crtc_page_flip,
	.destroy	= exynos_drm_crtc_destroy,
};

int exynos_drm_crtc_create(struct drm_device *dev, unsigned int nr,
		struct exynos_drm_display *display)
{
	struct exynos_drm_crtc *exynos_crtc;
	struct exynos_drm_private *private = dev->dev_private;
	struct drm_crtc *crtc;

	DRM_DEBUG_KMS("[DEV:%s] pipe: %d\n", dev->devname, nr);

	exynos_crtc = kzalloc(sizeof(*exynos_crtc), GFP_KERNEL);
	if (!exynos_crtc) {
		DRM_ERROR("failed to allocate exynos crtc\n");
		return -ENOMEM;
	}

	INIT_KFIFO(exynos_crtc->flip_fifo);
	exynos_crtc->pipe = nr;
	exynos_crtc->display = display;

	exynos_crtc->plane = exynos_plane_init(dev, 1 << nr, true);
	if (!exynos_crtc->plane) {
		kfree(exynos_crtc);
		return -ENOMEM;
	}

	crtc = &exynos_crtc->drm_crtc;

	private->crtc[nr] = crtc;

	drm_crtc_init(dev, crtc, &exynos_crtc_funcs);
	drm_crtc_helper_add(crtc, &exynos_crtc_helper_funcs);

	DRM_DEBUG_KMS("Created [CRTC:%d]\n", DRM_BASE_ID(crtc));

	return 0;
}

int exynos_drm_crtc_enable_vblank(struct drm_device *dev, int crtc)
{
	struct exynos_drm_private *private = dev->dev_private;
	struct exynos_drm_crtc *exynos_crtc =
		to_exynos_crtc(private->crtc[crtc]);
	struct exynos_drm_display *display = exynos_crtc->display;
	int ret = 0;

	DRM_DEBUG_KMS("[DEV:%s] crtc: %d\n", dev->devname, crtc);

	if (display->pipe == -1)
		display->pipe = crtc;

	if (display->controller_ops && display->controller_ops->enable_vblank)
		ret = display->controller_ops->enable_vblank(
				display->controller_ctx, crtc);
	return ret;
}

void exynos_drm_crtc_disable_vblank(struct drm_device *dev, int crtc)
{
	struct exynos_drm_private *private = dev->dev_private;
	struct exynos_drm_crtc *exynos_crtc =
		to_exynos_crtc(private->crtc[crtc]);
	struct exynos_drm_display *display = exynos_crtc->display;

	DRM_DEBUG_KMS("[DEV:%s] pipe: %d\n", dev->devname, crtc);

	/*
	 * TODO(seanpaul): This seems like a hack. I don't think it's actually
	 * needed for 2 reasons:
	 *   (1) disable_vblank implies vblank has been enabled. If
	 *       enable_vblank hasn't already been called, that's a bug.
	 *   (2) Even if (1) isn't true, this function should just disable an
	 *       interrupt, and shouldn't affect pipe.
	 */
	if (display->pipe == -1)
		display->pipe = crtc;

	if (display->controller_ops && display->controller_ops->disable_vblank)
		display->controller_ops->disable_vblank(
				display->controller_ctx);
}
