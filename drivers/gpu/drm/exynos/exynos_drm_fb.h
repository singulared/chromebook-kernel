/*
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

#ifndef _EXYNOS_DRM_FB_H_
#define _EXYNOS_DRM_FB_H

#include <linux/kref.h>
#include <drm/drm_sync_helper.h>

#define to_exynos_fb(x)	container_of(x, struct exynos_drm_fb, fb)

/*
 * exynos specific framebuffer structure.
 *
 * @release_work: for the worker destroying the fb after it's done being used
 * @fb: drm framebuffer object.
 * @buf_cnt: a buffer count to drm framebuffer.
 * @exynos_gem_obj: array of exynos specific gem object containing a gem object.
 * @dma_buf: shared dma buffer object
 * @rendered: true when framebuffer has been rendered to and ready to display
 * @prepared: bitmask of CRTCs for which registers were programmed
 * @resv_cb: reservation callback to receive notification when fb has been
 *           rendered
 */
struct exynos_drm_fb {
	struct kref			refcount;
	struct drm_framebuffer		fb;
	struct work_struct		release_work;
	unsigned int			buf_cnt;
	struct exynos_drm_gem_obj	*exynos_gem_obj[MAX_FB_BUFFER];
	struct dma_buf			*dma_buf;
	bool				rendered;
	unsigned long			prepared;
#ifdef CONFIG_DRM_DMA_SYNC
	struct drm_reservation_cb	resv_cb[2];
#endif
};

struct exynos_drm_fb *exynos_drm_fb_init(struct drm_device *dev,
					 struct drm_mode_fb_cmd2 *mode_cmd,
					 struct drm_gem_object *obj);

/* get memory information of a drm framebuffer */
struct exynos_drm_gem_buf *exynos_drm_fb_buffer(struct exynos_drm_fb *exynos_fb,
						int index);
struct exynos_drm_gem_obj *exynos_drm_fb_obj(struct exynos_drm_fb *exynos_fb,
					     int index);

void exynos_drm_mode_config_init(struct drm_device *dev);

/* set a buffer count to drm framebuffer. */
void exynos_drm_fb_set_buf_cnt(struct exynos_drm_fb *exynos_fb,
			       unsigned int cnt);

/* get a buffer count to drm framebuffer. */
unsigned int exynos_drm_fb_get_buf_cnt(struct exynos_drm_fb *exynos_fb);

/* store a reference to the dma_buf for this fb */
void exynos_drm_fb_attach_dma_buf(struct exynos_drm_fb *exynos_fb,
				  struct dma_buf *buf);

#endif
