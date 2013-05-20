/* exynos_drm_fb.c
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
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <uapi/drm/exynos_drm.h>

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
#include <linux/dma-buf.h>
#endif

#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_gem.h"
#include "exynos_drm_iommu.h"

static int check_fb_gem_memory_type(struct drm_device *drm_dev,
				struct exynos_drm_gem_obj *exynos_gem_obj)
{
	unsigned int flags;

	/*
	 * if exynos drm driver supports iommu then framebuffer can use
	 * all the buffer types.
	 */
	if (is_drm_iommu_supported(drm_dev))
		return 0;

	flags = exynos_gem_obj->flags;

	/*
	 * without iommu support, not support physically non-continuous memory
	 * for framebuffer.
	 */
	if (IS_NONCONTIG_BUFFER(flags)) {
		DRM_ERROR("cannot use this gem memory type for fb.\n");
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
void exynos_drm_fb_attach_dma_buf(struct exynos_drm_fb *exynos_fb,
				  struct dma_buf *buf)
{
	/*
	 * If we don't already have a reference to the dma_buf,
	 * grab one now. We'll release it in exynos_drm_fb_destroy().
	 */
	if (!exynos_fb->dma_buf) {
		get_dma_buf(buf);
		exynos_fb->dma_buf = buf;
	}
	BUG_ON(exynos_fb->dma_buf != buf);
}
#endif

void exynos_drm_fb_release(struct kref *kref)
{
	struct exynos_drm_fb *exynos_fb;

	exynos_fb = container_of(kref, struct exynos_drm_fb, refcount);
	schedule_work(&exynos_fb->release_work);
}

static void exynos_drm_fb_release_work_fn(struct work_struct *work)
{
	struct exynos_drm_fb *exynos_fb;
	unsigned int i;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	exynos_fb = container_of(work, struct exynos_drm_fb, release_work);

	drm_framebuffer_cleanup(&exynos_fb->fb);

	for (i = 0; i < ARRAY_SIZE(exynos_fb->exynos_gem_obj); i++) {
		struct drm_gem_object *obj;

		if (exynos_fb->exynos_gem_obj[i] == NULL)
			continue;

		obj = &exynos_fb->exynos_gem_obj[i]->base;
		drm_gem_object_unreference_unlocked(obj);
	}

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	if (exynos_fb->dma_buf)
		dma_buf_put(exynos_fb->dma_buf);
#endif

	kfree(exynos_fb);
}

static void exynos_drm_fb_destroy(struct drm_framebuffer *fb)
{
	exynos_drm_fb_put(to_exynos_fb(fb));
}

static int exynos_drm_fb_create_handle(struct drm_framebuffer *fb,
					struct drm_file *file_priv,
					unsigned int *handle)
{
	struct exynos_drm_fb *exynos_fb = to_exynos_fb(fb);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	return drm_gem_handle_create(file_priv,
			&exynos_fb->exynos_gem_obj[0]->base, handle);
}

static int exynos_drm_fb_dirty(struct drm_framebuffer *fb,
				struct drm_file *file_priv, unsigned flags,
				unsigned color, struct drm_clip_rect *clips,
				unsigned num_clips)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* TODO */

	return 0;
}

static struct drm_framebuffer_funcs exynos_drm_fb_funcs = {
	.destroy	= exynos_drm_fb_destroy,
	.create_handle	= exynos_drm_fb_create_handle,
	.dirty		= exynos_drm_fb_dirty,
};

void exynos_drm_fb_set_buf_cnt(struct exynos_drm_fb *exynos_fb,
			       unsigned int cnt)
{
	exynos_fb->buf_cnt = cnt;
}

unsigned int exynos_drm_fb_get_buf_cnt(struct exynos_drm_fb *exynos_fb)
{
	return exynos_fb->buf_cnt;
}

struct exynos_drm_fb *exynos_drm_fb_init(struct drm_device *dev,
					 struct drm_mode_fb_cmd2 *mode_cmd,
					 struct drm_gem_object *obj)
{
	struct exynos_drm_fb *exynos_fb;
	struct exynos_drm_gem_obj *exynos_gem_obj;
	int ret;

	exynos_gem_obj = to_exynos_gem_obj(obj);

	ret = check_fb_gem_memory_type(dev, exynos_gem_obj);
	if (ret < 0) {
		DRM_ERROR("cannot use this gem memory type for fb.\n");
		return ERR_PTR(-EINVAL);
	}

	exynos_fb = kzalloc(sizeof(*exynos_fb), GFP_KERNEL);
	if (!exynos_fb) {
		DRM_ERROR("failed to allocate exynos drm framebuffer\n");
		return ERR_PTR(-ENOMEM);
	}

	drm_helper_mode_fill_fb_struct(&exynos_fb->fb, mode_cmd);
	exynos_fb->exynos_gem_obj[0] = exynos_gem_obj;

	kref_init(&exynos_fb->refcount);
	INIT_WORK(&exynos_fb->release_work, exynos_drm_fb_release_work_fn);

	ret = drm_framebuffer_init(dev, &exynos_fb->fb, &exynos_drm_fb_funcs);
	if (ret) {
		DRM_ERROR("failed to initialize framebuffer\n");
		goto err_free_fb;
	}

	return exynos_fb;

err_free_fb:
	kfree(exynos_fb);
	return ERR_PTR(ret);
}

static u32 exynos_drm_format_num_buffers(struct drm_mode_fb_cmd2 *mode_cmd)
{
	unsigned int cnt = 0;

	if (mode_cmd->pixel_format != DRM_FORMAT_NV12)
		return drm_format_num_planes(mode_cmd->pixel_format);

	while (cnt != MAX_FB_BUFFER) {
		if (!mode_cmd->handles[cnt])
			break;
		cnt++;
	}

	/*
	 * check if NV12 or NV12M.
	 *
	 * NV12
	 * handles[0] = base1, offsets[0] = 0
	 * handles[1] = base1, offsets[1] = Y_size
	 *
	 * NV12M
	 * handles[0] = base1, offsets[0] = 0
	 * handles[1] = base2, offsets[1] = 0
	 */
	if (cnt == 2) {
		/*
		 * in case of NV12 format, offsets[1] is not 0 and
		 * handles[0] is same as handles[1].
		 */
		if (mode_cmd->offsets[1] &&
			mode_cmd->handles[0] == mode_cmd->handles[1])
			cnt = 1;
	}

	return cnt;
}

static struct drm_framebuffer *
exynos_user_fb_create(struct drm_device *dev, struct drm_file *file_priv,
		      struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_gem_object *obj;
	struct exynos_drm_fb *exynos_fb;
	int i, ret;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	obj = drm_gem_object_lookup(dev, file_priv, mode_cmd->handles[0]);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object\n");
		return ERR_PTR(-ENOENT);
	}

	exynos_fb = kzalloc(sizeof(*exynos_fb), GFP_KERNEL);
	if (!exynos_fb) {
		DRM_ERROR("failed to allocate exynos drm framebuffer\n");
		return ERR_PTR(-ENOMEM);
	}

	drm_helper_mode_fill_fb_struct(&exynos_fb->fb, mode_cmd);
	exynos_fb->exynos_gem_obj[0] = to_exynos_gem_obj(obj);
	exynos_fb->buf_cnt = exynos_drm_format_num_buffers(mode_cmd);

	DRM_DEBUG_KMS("buf_cnt = %d\n", exynos_fb->buf_cnt);

	for (i = 1; i < exynos_fb->buf_cnt; i++) {
		struct exynos_drm_gem_obj *exynos_gem_obj;
		int ret;

		obj = drm_gem_object_lookup(dev, file_priv,
				mode_cmd->handles[i]);
		if (!obj) {
			DRM_ERROR("failed to lookup gem object\n");
			kfree(exynos_fb);
			return ERR_PTR(-ENOENT);
		}

		exynos_gem_obj = to_exynos_gem_obj(obj);

		ret = check_fb_gem_memory_type(dev, exynos_gem_obj);
		if (ret < 0) {
			DRM_ERROR("cannot use this gem memory type for fb.\n");
			kfree(exynos_fb);
			return ERR_PTR(ret);
		}

		exynos_fb->exynos_gem_obj[i] = to_exynos_gem_obj(obj);
	}

	kref_init(&exynos_fb->refcount);
	INIT_WORK(&exynos_fb->release_work, exynos_drm_fb_release_work_fn);

	ret = drm_framebuffer_init(dev, &exynos_fb->fb, &exynos_drm_fb_funcs);
	if (ret) {
		for (i = 0; i < exynos_fb->buf_cnt; i++) {
			struct exynos_drm_gem_obj *gem_obj;

			gem_obj = exynos_fb->exynos_gem_obj[i];
			drm_gem_object_unreference_unlocked(&gem_obj->base);
		}

		kfree(exynos_fb);
		return ERR_PTR(ret);
	}

	return &exynos_fb->fb;
}

struct exynos_drm_gem_buf *
exynos_drm_fb_buffer(struct exynos_drm_fb *exynos_fb, int index)
{
	struct exynos_drm_gem_buf *buffer;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	BUG_ON(index >= MAX_FB_BUFFER);

	buffer = exynos_fb->exynos_gem_obj[index]->buffer;

	DRM_DEBUG_KMS("dma_addr = 0x%lx\n", (unsigned long)buffer->dma_addr);

	return buffer;
}

struct exynos_drm_gem_obj *exynos_drm_fb_obj(struct exynos_drm_fb *exynos_fb,
					     int index)
{
	DRM_DEBUG_KMS("%s\n", __func__);

	BUG_ON(index >= MAX_FB_BUFFER);

	return exynos_fb->exynos_gem_obj[index];
}

static void exynos_drm_output_poll_changed(struct drm_device *dev)
{
	struct exynos_drm_private *private = dev->dev_private;
	struct drm_fb_helper *fb_helper = private->fb_helper;

	if (fb_helper)
		drm_fb_helper_hotplug_event(fb_helper);
}

static const struct drm_mode_config_funcs exynos_drm_mode_config_funcs = {
	.fb_create = exynos_user_fb_create,
	.output_poll_changed = exynos_drm_output_poll_changed,
};

void exynos_drm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	/*
	 * set max width and height as default value(4096x4096).
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &exynos_drm_mode_config_funcs;
}
