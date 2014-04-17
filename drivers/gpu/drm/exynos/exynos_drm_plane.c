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

#include <drm/exynos_drm.h>
#include "exynos_drm_drv.h"

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
