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
struct exynos_drm_overlay;
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
};
#define to_exynos_plane(x) container_of(x, struct exynos_drm_plane, base)

void exynos_plane_copy_state(struct exynos_drm_plane *src,
		struct exynos_drm_plane *dst);
void exynos_sanitize_plane_coords(struct drm_plane *plane,
		struct drm_crtc *crtc);

/*
 * Exynos drm common overlay structure.
 *
 * @fb_x: offset x on a framebuffer to be displayed.
 *	- the unit is screen coordinates.
 * @fb_y: offset y on a framebuffer to be displayed.
 *	- the unit is screen coordinates.
 * @fb_width: width of a framebuffer.
 * @fb_height: height of a framebuffer.
 * @src_width: width of a partial image to be displayed from framebuffer.
 * @src_height: height of a partial image to be displayed from framebuffer.
 * @crtc_x: offset x on hardware screen.
 * @crtc_y: offset y on hardware screen.
 * @crtc_width: window width to be displayed (hardware screen).
 * @crtc_height: window height to be displayed (hardware screen).
 * @mode_width: width of screen mode.
 * @mode_height: height of screen mode.
 * @refresh: refresh rate.
 * @scan_flag: interlace or progressive way.
 *	(it could be DRM_MODE_FLAG_*)
 * @bpp: pixel size.(in bit)
 * @pitch: pitch of a framebuffer.
 * @pixel_format: fourcc pixel format of this overlay
 * @dma_addr: array of bus(accessed by dma) address to the memory region
 *	      allocated for a overlay.
 * @zpos: order of overlay layer(z position).
 * @default_win: a window to be enabled.
 * @color_key: color key on or off.
 * @index_color: if using color key feature then this value would be used
 *			as index color.
 * @local_path: in case of lcd type, local path mode on or off.
 * @transparency: transparency on or off.
 * @activated: activated or not.
 *
 * this structure is common to exynos SoC and its contents would be copied
 * to hardware specific overlay info.
 */
struct exynos_drm_overlay {
	unsigned int fb_x;
	unsigned int fb_y;
	unsigned int fb_width;
	unsigned int fb_height;
	unsigned int src_width;
	unsigned int src_height;
	unsigned int crtc_x;
	unsigned int crtc_y;
	unsigned int crtc_width;
	unsigned int crtc_height;
	unsigned int mode_width;
	unsigned int mode_height;
	unsigned int refresh;
	unsigned int scan_flag;
	unsigned int bpp;
	unsigned int pitch;
	uint32_t pixel_format;
	dma_addr_t dma_addr[MAX_FB_BUFFER];
	int zpos;

	bool default_win;
	bool color_key;
	unsigned int index_color;
	bool local_path;
	bool transparency;
	bool activated;
};

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
	int (*initialize)(void *ctx, struct drm_device *drm_dev);
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
 * @list: the list entry for this manager
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

/*
 * Exynos drm manager ops
 *
 * @initialize: initializes the manager
 * @remove: cleans up the manager for removal
 * @dpms: control device power.
 * @adjust_mode: allows manager to adjust mode in check mode path
 * @mode_fixup: fix mode data before applying it
 * @mode_set: copy drm display mode to hw specific display mode.
 * @update: updates the primary plane
 * @commit: appl current hw specific display mode to hw.
 * @enable_vblank: specific driver callback for enabling vblank interrupt.
 * @disable_vblank: specific driver callback for disabling vblank interrupt.
 */
struct exynos_drm_manager_ops {
	int (*initialize)(void *ctx, struct drm_crtc *crtc, int pipe);
	void (*remove)(void *ctx);
	void (*dpms)(void *ctx, int mode);
	void (*adjust_mode)(void *ctx, struct drm_connector *connector,
				struct drm_display_mode *mode_to_adjust);
	bool (*mode_fixup)(void *ctx, const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode);
	void (*mode_set)(void *ctx, const struct drm_display_mode *mode);
	int (*update)(void *ctx, struct drm_crtc *crtc,
				struct drm_framebuffer *fb);
	void (*commit)(void *ctx);
	int (*enable_vblank)(void *ctx);
	void (*disable_vblank)(void *ctx);
};

/*
 * Exynos drm common manager structure, maps 1:1 with a crtc
 *
 * @list: the list entry for this manager
 * @type: one of EXYNOS_DISPLAY_TYPE_LCD and HDMI.
 * @drm_dev: pointer to the drm device
 * @crtc: pointer to the drm crtc
 * @pipe: the pipe number for this crtc/manager
 * @ops: pointer to callbacks for exynos drm specific functionality
 * @ctx: A pointer to the manager's implementation specific context
 */
struct exynos_drm_manager {
	struct list_head list;
	enum exynos_drm_output_type type;
	struct drm_device *drm_dev;
	struct drm_crtc *crtc;
	int pipe;
	const struct exynos_drm_manager_ops *ops;
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

	/*
	 * created crtc object would be contained at this array and
	 * this array is used to be aware of which crtc did it request vblank.
	 */
	struct drm_crtc *crtc[MAX_CRTC];
	struct drm_property *plane_zpos_property;
	struct drm_property *crtc_mode_property;

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct kds_callback kds_cb;
#endif
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
 * @manager: subdrv has its own manager to control a hardware appropriately
 *	and we can access a hardware drawing on this manager.
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

int exynos_drm_initialize_managers(struct drm_device *dev);
void exynos_drm_remove_managers(struct drm_device *dev);
int exynos_drm_initialize_displays(struct drm_device *dev);
void exynos_drm_remove_displays(struct drm_device *dev);

int exynos_drm_manager_register(struct exynos_drm_manager *manager);
int exynos_drm_manager_unregister(struct exynos_drm_manager *manager);
int exynos_drm_display_register(struct exynos_drm_display *display);
int exynos_drm_display_unregister(struct exynos_drm_display *display);

/* This function returns a manager of the same type as the given display */
struct exynos_drm_manager *exynos_drm_manager_from_display(
		struct exynos_drm_display *display);

/*
 * this function would be called by sub drivers such as display controller
 * or hdmi driver to register this sub driver object to exynos drm driver
 * and when a sub driver is registered to exynos drm driver a probe callback
 * of the sub driver is called and creates its own encoder and connector.
 */
int exynos_drm_subdrv_register(struct exynos_drm_subdrv *drm_subdrv);

/* this function removes subdrv list from exynos drm driver */
int exynos_drm_subdrv_unregister(struct exynos_drm_subdrv *drm_subdrv);

void exynos_fimd_dp_attach(struct device *dev);

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
