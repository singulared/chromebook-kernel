/* exynos_drm_drv.h
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

#ifndef _EXYNOS_DRM_DRV_H_
#define _EXYNOS_DRM_DRV_H_

#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
#include <linux/kds.h>
#endif

#define DRM_BASE_ID(obj)	((obj) ? (obj)->base.id : -1)

#define MAX_CRTC	3
#define MAX_PLANE	5
#define MAX_FB_BUFFER	4
#define DEFAULT_ZPOS	-1

/*
 * _wait_for - magic (register) wait macro
 *
 * Does the right thing for modeset paths when run under kdgb or similar atomic
 * contexts. Note that it's important that we check the condition again after
 * having timed out, since the timeout could be due to preemption or similar and
 * we've never had a chance to check the condition before the timeout.
 */
#define _wait_for(COND, TO_MS, INTVL_MS) ({ \
	unsigned long timeout__ = jiffies + msecs_to_jiffies(TO_MS) + 1;\
	int ret__ = 0;							\
	while (!(COND)) {						\
		if (time_after(jiffies, timeout__)) {			\
			if (!(COND))					\
				ret__ = -ETIMEDOUT;			\
			break;						\
		}							\
		if (drm_can_sleep())  {					\
			usleep_range(INTVL_MS * 1000,			\
				(INTVL_MS + 5) * 1000);			\
		} else {						\
			cpu_relax();					\
		}							\
	}								\
	ret__;								\
})

#define wait_for(COND, TO_MS, INTVL_MS) _wait_for(COND, TO_MS, INTVL_MS)

/* private flag to mark adjusted display modes */
#define EXYNOS_MODE_ADJUSTED 0x1

struct drm_device;
struct drm_connector;

/* this enumerates display type. */
enum exynos_drm_output_type {
	EXYNOS_DISPLAY_TYPE_NONE,
	/* RGB or CPU Interface. */
	EXYNOS_DISPLAY_TYPE_LCD,
	/* HDMI Interface. */
	EXYNOS_DISPLAY_TYPE_HDMI,
	/* Virtual Display Interface. */
	EXYNOS_DISPLAY_TYPE_VIDI,
};

const char *exynos_drm_output_type_name(enum exynos_drm_output_type type);

int exynos_drm_pipe_from_crtc(struct drm_crtc *crtc);

struct exynos_plane_helper_funcs {
	int (*commit_plane)(struct drm_plane *plane, struct drm_crtc *crtc,
				struct drm_framebuffer *fb);
	int (*disable_plane)(struct drm_plane *plane);
};

struct exynos_drm_plane {
	struct drm_plane base;

	void *ctx;

	int crtc_x;
	int crtc_y;
	unsigned int crtc_w;
	unsigned int crtc_h;
	uint32_t src_x;
	uint32_t src_y;
	uint32_t src_w;
	uint32_t src_h;

	struct completion completion;
	struct drm_framebuffer *fb;
	struct drm_framebuffer *pending_fb;
	struct drm_pending_vblank_event *pending_event;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct kds_resource_set *kds;
	struct kds_callback kds_cb;
#endif

	const struct exynos_plane_helper_funcs *helper_funcs;
};
#define to_exynos_plane(x) container_of(x, struct exynos_drm_plane, base)

void exynos_plane_copy_state(struct exynos_drm_plane *src,
		struct exynos_drm_plane *dst);
void exynos_sanitize_plane_coords(struct drm_plane *plane,
		struct drm_crtc *crtc);

void exynos_plane_helper_init(struct drm_plane *plane,
			const struct exynos_plane_helper_funcs *funcs);

int exynos_plane_helper_update_plane_with_event(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		struct drm_pending_vblank_event *event, int crtc_x,
		int crtc_y, unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y, uint32_t src_w, uint32_t src_h);

int exynos_plane_helper_update_plane(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb, int crtc_x,
		int crtc_y, unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y, uint32_t src_w, uint32_t src_h);

void exynos_plane_helper_finish_update(struct drm_plane *plane,
		struct drm_crtc *crtc, bool update_fb);

int exynos_plane_helper_disable_plane(struct drm_plane *plane);

void exynos_drm_crtc_send_event(struct drm_plane *plane, struct drm_crtc *crtc);

int exynos_plane_helper_freeze_plane(struct drm_plane *plane);

void exynos_plane_helper_thaw_plane(struct drm_plane *plane,
		struct drm_crtc *crtc);

/*
 * Exynos DRM Display Structure.
 *	- this structure is common to analog tv, digital tv and lcd panel.
 *
 * @initialize: initializes the display with drm_dev
 * @remove: cleans up the display for removal
 * @mode_fixup: fix mode data comparing to hw specific display mode.
 * @mode_set: convert drm_display_mode to hw specific display mode and
 *	      would be called by encoder->mode_set().
 * @check_mode: check if mode is valid or not.
 * @dpms: display device on or off.
 * @commit: apply changes to hw
 * @set_property: sets a drm propery on the display
 */
struct exynos_drm_display_ops {
	int (*initialize)(void *ctx, struct drm_device *drm_dev,
			uint32_t *possible_crtcs);
	int (*create_connector)(void *ctx,
			struct drm_encoder *encoder);
	void (*remove)(void *ctx);
	bool (*mode_fixup)(void *ctx, struct drm_connector *connector,
			const struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode);
	void (*mode_set)(void *ctx, struct drm_display_mode *mode);
	int (*check_mode)(void *ctx, const struct drm_display_mode *mode);
	void (*dpms)(void *ctx, int mode);
	void (*commit)(void *ctx);
	int (*set_property)(void *ctx, struct drm_property *property,
			uint64_t val);
};

/*
 * Exynos drm display structure, maps 1:1 with an encoder/connector
 *
 * @list: the list entry for this display
 * @type: one of EXYNOS_DISPLAY_TYPE_LCD and HDMI.
 * @encoder: encoder object this display maps to
 * @ops: pointer to callbacks for exynos drm specific functionality
 * @ctx: A pointer to the display's implementation specific context
 */
struct exynos_drm_display {
	struct list_head list;
	enum exynos_drm_output_type type;
	struct drm_encoder *encoder;
	const struct exynos_drm_display_ops *ops;
	void *ctx;
};

struct exynos_drm_g2d_private {
	struct device		*dev;
	struct list_head	inuse_cmdlist;
	struct list_head	event_list;
	struct list_head	userptr_list;
};

struct exynos_drm_ipp_private {
	struct device	*dev;
	struct list_head	event_list;
};

/*
 * Exynos drm_file private structure.
 *
 * @g2d_priv: g2d private structs
 * @ipp_priv: img processor private structs
 * @gem_cpu_acquire_list: list of GEM objects we hold acquires on
 */
struct drm_exynos_file_private {
	struct exynos_drm_g2d_private	*g2d_priv;
	struct exynos_drm_ipp_private	*ipp_priv;
	struct list_head		gem_cpu_acquire_list;
};

/*
 * Exynos drm private structure.
 */
struct exynos_drm_private {
	struct drm_fb_helper *fb_helper;

	struct drm_crtc *fimd_crtc;
	struct drm_crtc *mixer_crtc;
	struct drm_crtc *vidi_crtc;

	struct drm_encoder *dp_encoder;
	struct drm_encoder *hdmi_encoder;


#ifdef CONFIG_DRM_EXYNOS_DEBUG
	struct {
		atomic_t object_memory;
		atomic_t object_count;
	} mm;
#endif
};

/*
 * Exynos drm sub driver structure.
 *
 * @list: sub driver has its own list object to register to exynos drm driver.
 * @dev: pointer to device object for subdrv device driver.
 * @drm_dev: pointer to drm_device and this pointer would be set
 *	when sub driver calls exynos_drm_subdrv_register().
 * @probe: this callback would be called by exynos drm driver after
 *	subdrv is registered to it.
 * @remove: this callback is used to release resources created
 *	by probe callback.
 * @open: this would be called with drm device file open.
 * @close: this would be called with drm device file close.
 */
struct exynos_drm_subdrv {
	struct list_head list;
	struct device *dev;
	struct drm_device *drm_dev;

	int (*probe)(struct drm_device *drm_dev, struct device *dev);
	void (*remove)(struct drm_device *drm_dev, struct device *dev);
	int (*open)(struct drm_device *drm_dev, struct device *dev,
			struct drm_file *file);
	void (*close)(struct drm_device *drm_dev, struct device *dev,
			struct drm_file *file);
};

/*
 * this function calls a probe callback registered to sub driver list and
 * create its own encoder and connector and then set drm_device object
 * to global one.
 */
int exynos_drm_device_register(struct drm_device *dev);
/*
 * this function calls a remove callback registered to sub driver list and
 * destroy its own encoder and connetor.
 */
int exynos_drm_device_unregister(struct drm_device *dev);

int exynos_drm_initialize_displays(struct drm_device *dev);
void exynos_drm_remove_displays(struct drm_device *dev);

int exynos_drm_display_register(struct exynos_drm_display *display);
int exynos_drm_display_unregister(struct exynos_drm_display *display);

/*
 * this function would be called by sub drivers such as display controller
 * or hdmi driver to register this sub driver object to exynos drm driver
 * and when a sub driver is registered to exynos drm driver a probe callback
 * of the sub driver is called and creates its own encoder and connector.
 */
int exynos_drm_subdrv_register(struct exynos_drm_subdrv *drm_subdrv);

/* this function removes subdrv list from exynos drm driver */
int exynos_drm_subdrv_unregister(struct exynos_drm_subdrv *drm_subdrv);

int exynos_drm_subdrv_open(struct drm_device *dev, struct drm_file *file);
void exynos_drm_subdrv_close(struct drm_device *dev, struct drm_file *file);

/*
 * this function registers exynos drm hdmi platform device. It ensures only one
 * instance of the device is created.
 */
extern int exynos_platform_device_hdmi_register(void);

/*
 * this function unregisters exynos drm hdmi platform device if it exists.
 */
void exynos_platform_device_hdmi_unregister(void);

extern struct platform_driver fimd_driver;
extern struct platform_driver hdmi_driver;
extern struct platform_driver dp_driver;
extern struct platform_driver mixer_driver;
extern struct platform_driver exynos_drm_common_hdmi_driver;
extern struct platform_driver vidi_driver;
extern struct platform_driver g2d_driver;
extern struct platform_driver fimc_driver;
extern struct platform_driver rotator_driver;
extern struct platform_driver gsc_driver;
extern struct platform_driver ipp_driver;
#endif
