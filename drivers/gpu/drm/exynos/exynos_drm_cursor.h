/*
 * exynos_drm_cursor.h
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

#ifndef _EXYNOS_DRM_CURSOR_H_
#define _EXYNOS_DRM_CURSOR_H_

#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>

/*
 * Exynos drm cursor private structure.
 */
struct exynos_drm_cursor {
	struct exynos_drm_gem_obj *exynos_gem_obj;
	struct exynos_drm_fb *exynos_fb;

	/* cursor dimensions and format */
	uint32_t width;
	uint32_t height;
	uint32_t format;
	uint32_t cpp;
	uint32_t pitch;
	uint32_t padding;

	/* current cursor state */
	bool enabled;
	int x;
	int y;
};

extern int exynos_drm_cursor_init(struct drm_device *dev,
		struct exynos_drm_cursor *cursor);
extern void exynos_drm_cursor_fini(struct exynos_drm_cursor *cursor);
extern int exynos_crtc_cursor_set(struct drm_crtc *crtc, struct drm_file *file,
		uint32_t handle, uint32_t width, uint32_t height,
		struct exynos_drm_cursor *cursor, struct drm_plane *plane);
extern int exynos_crtc_cursor_move(struct drm_crtc *crtc, int x, int y,
		struct exynos_drm_cursor *cursor, struct drm_plane *plane);

#endif
