/*
 * Copyright (C) 2013 Google, Inc.
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

#ifndef _EXYNOS_MIXER_H_
#define _EXYNOS_MIXER_H_

enum hdmi_version {
	HDMI_VER_EXYNOS4210,
	HDMI_VER_EXYNOS4212,
	HDMI_VER_EXYNOS5420,
};

enum mixer_version_id {
	MXR_VER_0_0_0_16 = 1 << 0,
	MXR_VER_16_0_33_0 = 1 << 1,
	MXR_VER_128_0_0_184 = 1 << 2,
};

static inline enum mixer_version_id
			mixer_get_version(enum hdmi_version version) {
	enum mixer_version_id mxr_version;
	switch (version) {
	case HDMI_VER_EXYNOS4210:
		mxr_version = MXR_VER_0_0_0_16;
		break;
	case HDMI_VER_EXYNOS4212:
		mxr_version = MXR_VER_16_0_33_0;
		break;
	case HDMI_VER_EXYNOS5420:
		mxr_version = MXR_VER_128_0_0_184;
		break;
	}
	return mxr_version;
}

/* This function returns 0 if the given timing is valid for the mixer */
int mixer_check_mode(struct drm_display_mode *mode,
					enum mixer_version_id version);

/* This function returns the horizontal offset for a mode */
unsigned mixer_get_horizontal_offset(unsigned width, unsigned height);
#endif
