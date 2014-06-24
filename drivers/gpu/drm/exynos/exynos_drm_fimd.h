/*
 * Copyright (C) 2014 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _EXYNOS_FIMD_H_
#define _EXYNOS_FIMD_H_

struct drm_device;
struct drm_crtc;

int fimd_get_crtc_id(struct drm_device *dev);

int fimd_enable_vblank(struct drm_crtc *crtc);
void fimd_disable_vblank(struct drm_crtc *crtc);

#endif
