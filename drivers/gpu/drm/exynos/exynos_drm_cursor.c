/*
 * exynos_drm_cursor.c
 *
 * Copyright (c) 2014 The Chromium OS Authors
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

#include <drm/exynos_drm.h>

#include "exynos_drm_cursor.h"
#include "exynos_drm_debugfs.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_gem.h"

/* Explanation of workaround at play here:
 * * FIMD has trouble with the cursor overlay when the visible portion has
 *   width less than the minimum FIMD DMA burst, i.e. 32 bytes (8 pixels
 *   ARGB).  This is a problem when the cursor is at the left or right edge
 *   of the screen.  Work around this padding the cursor on the left and
 *   right sides.
 * We create a 96x64 pixel cursor overlay, with 16 pixels' (64 bytes')
 * worth of padding on each side, so we can export a 64x64 size to X as we'd
 * like to have HW accelerated cursors at least to this size.  We stay with
 * multiple-of-32-pixel sizes internally for efficiency since the longest FIMD
 * DMA burst length is 128 bytes (32 pixels ARGB).
 */
#define CURSORW_KERNEL	96
#define CURSORH_KERNEL	64
#define CURSORPAD	16

#define CURSORW_USER	64
#define CURSORH_USER	64

int exynos_drm_cursor_init(struct drm_device *dev,
		struct exynos_drm_cursor *cursor)
{
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct exynos_drm_gem_obj *exynos_gem_obj;
	struct exynos_drm_fb *exynos_fb;
	int ret;

	cursor->width = CURSORW_KERNEL;
	cursor->height = CURSORH_KERNEL;
	cursor->format = DRM_FORMAT_ARGB8888;
	cursor->cpp = drm_format_plane_cpp(cursor->format, 0);
	cursor->pitch = cursor->width * cursor->cpp;
	cursor->padding = CURSORPAD * cursor->cpp;

	exynos_gem_obj = exynos_drm_gem_create(dev,
			EXYNOS_BO_NONCONTIG | EXYNOS_BO_WC,
			cursor->pitch * cursor->height);
	if (!exynos_gem_obj) {
		DRM_ERROR("failed to allocate exynos gem object\n");
		return -ENOMEM;
	}
	cursor->exynos_gem_obj = exynos_gem_obj;

	mode_cmd.width = cursor->width;
	mode_cmd.height = cursor->height;
	mode_cmd.pitches[0] = cursor->pitch;
	mode_cmd.pixel_format = cursor->format;
	exynos_fb = exynos_drm_fb_init(dev, &mode_cmd, &exynos_gem_obj->base);
	if (!exynos_fb) {
		DRM_ERROR("failed to allocate exynos fb\n");
		ret = -ENOMEM;
		goto err_free_gem_obj;
	}

	exynos_drm_fb_set_buf_cnt(exynos_fb, 1);
	cursor->exynos_fb = exynos_fb;
	return 0;

err_free_gem_obj:
	exynos_drm_gem_destroy(exynos_gem_obj);
	return ret;
}

void exynos_drm_cursor_fini(struct exynos_drm_cursor *cursor)
{
	if (cursor->exynos_gem_obj)
		drm_gem_object_unreference(&cursor->exynos_gem_obj->base);
	kfree(cursor->exynos_fb);
}

static int exynos_crtc_copy_cursor(struct exynos_drm_cursor *cursor,
		struct drm_gem_object *obj)
{
	struct exynos_drm_gem_obj *exynos_gem_obj;
	struct exynos_drm_gem_buf *src_buf;
	struct exynos_drm_gem_buf *dst_buf;
	uint8_t *src_kvaddr, *dst_kvaddr;
	int src_nr_pages, dst_nr_pages;
	int src_pitch, src_offset, dst_offset, i, ret;

	/* map src buffer to kvaddr */
	exynos_gem_obj = to_exynos_gem_obj(obj);
	src_buf = exynos_gem_obj->buffer;
	if (!src_buf || !src_buf->pages) {
		DRM_ERROR("invalid gem object buffer.\n");
		return -EINVAL;
	}
	src_nr_pages = DIV_ROUND_UP(src_buf->size, PAGE_SIZE);
	src_kvaddr = vmap(src_buf->pages, src_nr_pages, VM_MAP,
			pgprot_writecombine(PAGE_KERNEL));
	if (!src_kvaddr) {
		DRM_ERROR("failed to map src cursor buffer.\n");
		return -ENOMEM;
	}

	/* map dst buffer to kvaddr */
	dst_buf = cursor->exynos_gem_obj->buffer;
	dst_nr_pages = DIV_ROUND_UP(dst_buf->size, PAGE_SIZE);
	dst_kvaddr = vmap(dst_buf->pages, dst_nr_pages, VM_MAP,
			pgprot_writecombine(PAGE_KERNEL));
	if (!dst_kvaddr) {
		DRM_ERROR("failed to map dst cursor buffer.\n");
		ret = -ENOMEM;
		goto unmap_src_kvaddr;
	}

	/* copy data from src to dst */
	src_pitch = CURSORW_USER * 4;
	src_offset = 0;
	dst_offset = cursor->padding;
	for (i = 0; i < cursor->height; ++i) {
		memcpy(dst_kvaddr + dst_offset, src_kvaddr + src_offset,
				src_pitch);
		src_offset += src_pitch;
		dst_offset += cursor->pitch;
	}
	ret = 0;

	vunmap(dst_kvaddr);
unmap_src_kvaddr:
	vunmap(src_kvaddr);
	return ret;
}

static int exynos_crtc_update_cursor(struct drm_crtc *crtc,
		struct exynos_drm_cursor *cursor, struct drm_plane *plane)
{
	if (!cursor->enabled)
		return plane->funcs->disable_plane(plane);

	return plane->funcs->update_plane(plane, crtc, &cursor->exynos_fb->fb,
			cursor->x - CURSORPAD, cursor->y,
			cursor->width, cursor->height, 0, 0,
			cursor->width << 16, cursor->height << 16);
}

int exynos_crtc_cursor_set(struct drm_crtc *crtc, struct drm_file *file,
		uint32_t handle, uint32_t width, uint32_t height,
		struct exynos_drm_cursor *cursor, struct drm_plane *plane)
{
	if (handle) {
		struct drm_gem_object *obj;
		int err;
		/* Currently we only support 64x64 cursors */
		if (width != CURSORW_USER || height != CURSORH_USER) {
			DRM_ERROR("we currently only support %dx%d cursors\n",
					CURSORW_USER, CURSORH_USER);
			return -EINVAL;
		}
		obj = drm_gem_object_lookup(crtc->dev, file, handle);
		if (!obj) {
			DRM_ERROR("failed to lookup gem object.\n");
			return -EINVAL;
		}
		err = exynos_crtc_copy_cursor(cursor, obj);
		drm_gem_object_unreference(obj);
		if (err != 0) {
			DRM_ERROR("failed to copy cursor.\n");
			return err;
		}
		cursor->enabled = true;
	} else {
		cursor->enabled = false;
	}

	return exynos_crtc_update_cursor(crtc, cursor, plane);
}

int exynos_crtc_cursor_move(struct drm_crtc *crtc, int x, int y,
		struct exynos_drm_cursor *cursor, struct drm_plane *plane)
{
	cursor->x = x;
	cursor->y = y;
	return exynos_crtc_update_cursor(crtc, cursor, plane);
}
