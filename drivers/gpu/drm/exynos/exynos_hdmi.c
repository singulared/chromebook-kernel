/*
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors:
 * Seung-Woo Kim <sw0312.kim@samsung.com>
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * Based on drivers/media/video/s5p-tv/hdmi_drv.c
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <drm/drmP.h>
#include <drm/drm_edid.h>
#include <drm/drm_crtc_helper.h>

#include "regs-hdmi.h"

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_i2c.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>

#include <drm/exynos_drm.h>

#include "exynos_drm_drv.h"
#include "exynos_mixer.h"

#include <linux/gpio.h>
#include <media/s5p_hdmi.h>

#define HOTPLUG_DEBOUNCE_MS	1100
#define MAX_WIDTH		1920
#define MAX_HEIGHT		1080
#define get_hdmi_context(dev)	platform_get_drvdata(to_platform_device(dev))

/* AVI header and aspect ratio */
#define HDMI_AVI_VERSION		0x02
#define HDMI_AVI_LENGTH		0x0D
#define AVI_PIC_ASPECT_RATIO_16_9	(2 << 4)
#define AVI_SAME_AS_PIC_ASPECT_RATIO	8

/* AUI header info */
#define HDMI_AUI_VERSION	0x01
#define HDMI_AUI_LENGTH	0x0A

#define HDCP_MAX_TRIES                  10
#define LOAD_KEY_TIMEOUT_MSECS          2000
#define AUTH_ACK_TIMEOUT_MSECS          5000
#define RI_VERIFY_TIMEOUT_MSECS         1000
#define KSV_READY_TIMEOUT_MSECS         5000
#define KSV_READ_KEY_TIMEOUT_MSECS      500
#define SHA_COMPARE_TIMEOUT_MSECS       250

#define HDCP_I2C_ADDR_BKSV              0x00
#define HDCP_I2C_ADDR_RI                0x08
#define HDCP_I2C_ADDR_AKSV              0x10
#define HDCP_I2C_ADDR_AN                0x18
#define HDCP_I2C_ADDR_V_PRIME           0x20
#define HDCP_I2C_ADDR_BCAPS             0x40
#define HDCP_I2C_ADDR_BSTATUS           0x41
#define HDCP_I2C_ADDR_KSV_FIFO          0x43

#define HDCP_I2C_LEN_BKSV               5
#define HDCP_I2C_LEN_RI                 2
#define HDCP_I2C_LEN_AKSV               5
#define HDCP_I2C_LEN_AN                 8
#define HDCP_I2C_LEN_V_PRIME            20
#define HDCP_I2C_LEN_BCAPS              1
#define HDCP_I2C_LEN_BSTATUS            2

#define HDCP_I2C_MASK_BCAPS_REPEATER            (1 << 6)
#define HDCP_I2C_MASK_BCAPS_READY               (1 << 5)
#define HDCP_I2C_MASK_BSTATUS_0_MAX_DEVS        (1 << 7)
#define HDCP_I2C_MASK_BSTATUS_1_MAX_CASCADE     (1 << 3)

enum exynos_hdcp_state {
	HDCP_STATE_OFF,
	HDCP_STATE_WAIT_ACTIVE_RX,
	HDCP_STATE_EXCHANGE_KSV,
	HDCP_STATE_COMPUTATIONS,
	HDCP_STATE_AUTHENTICATED,
	HDCP_STATE_READ_KSV_LIST,
};

/* HDMI infoframe to configure HDMI out packet header, AUI and AVI */
enum HDMI_PACKET_TYPE {
	/* refer to Table 5-8 Packet Type in HDMI specification v1.4a */
	/* InfoFrame packet type */
	HDMI_PACKET_TYPE_INFOFRAME = 0x80,
	/* Vendor-Specific InfoFrame */
	HDMI_PACKET_TYPE_VSI = HDMI_PACKET_TYPE_INFOFRAME + 1,
	/* Auxiliary Video information InfoFrame */
	HDMI_PACKET_TYPE_AVI = HDMI_PACKET_TYPE_INFOFRAME + 2,
	/* Audio information InfoFrame */
	HDMI_PACKET_TYPE_AUI = HDMI_PACKET_TYPE_INFOFRAME + 4
};

struct hdmi_resources {
	struct clk			*hdmi;
	struct clk			*sclk_hdmi;
	struct clk			*sclk_pixel;
	struct clk			*sclk_hdmiphy;
	struct clk			*mout_hdmi;
	struct regulator_bulk_data	*regul_bulk;
	int				regul_count;
};

struct hdmi_tg_regs {
	u8 cmd[1];
	u8 h_fsz[2];
	u8 hact_st[2];
	u8 hact_sz[2];
	u8 v_fsz[2];
	u8 vsync[2];
	u8 vsync2[2];
	u8 vact_st[2];
	u8 vact_sz[2];
	u8 field_chg[2];
	u8 vact_st2[2];
	u8 vact_st3[2];
	u8 vact_st4[2];
	u8 vsync_top_hdmi[2];
	u8 vsync_bot_hdmi[2];
	u8 field_top_hdmi[2];
	u8 field_bot_hdmi[2];
	u8 tg_3d[1];
};

struct hdmi_4210_core_regs {
	u8 h_blank[2];
	u8 v_blank[3];
	u8 h_v_line[3];
	u8 vsync_pol[1];
	u8 int_pro_mode[1];
	u8 v_blank_f[3];
	u8 h_sync_gen[3];
	u8 v_sync_gen1[3];
	u8 v_sync_gen2[3];
	u8 v_sync_gen3[3];
};

struct hdmi_4212_core_regs {
	u8 h_blank[2];
	u8 v2_blank[2];
	u8 v1_blank[2];
	u8 v_line[2];
	u8 h_line[2];
	u8 hsync_pol[1];
	u8 vsync_pol[1];
	u8 int_pro_mode[1];
	u8 v_blank_f0[2];
	u8 v_blank_f1[2];
	u8 h_sync_start[2];
	u8 h_sync_end[2];
	u8 v_sync_line_bef_2[2];
	u8 v_sync_line_bef_1[2];
	u8 v_sync_line_aft_2[2];
	u8 v_sync_line_aft_1[2];
	u8 v_sync_line_aft_pxl_2[2];
	u8 v_sync_line_aft_pxl_1[2];
	u8 v_blank_f2[2]; /* for 3D mode */
	u8 v_blank_f3[2]; /* for 3D mode */
	u8 v_blank_f4[2]; /* for 3D mode */
	u8 v_blank_f5[2]; /* for 3D mode */
	u8 v_sync_line_aft_3[2];
	u8 v_sync_line_aft_4[2];
	u8 v_sync_line_aft_5[2];
	u8 v_sync_line_aft_6[2];
	u8 v_sync_line_aft_pxl_3[2];
	u8 v_sync_line_aft_pxl_4[2];
	u8 v_sync_line_aft_pxl_5[2];
	u8 v_sync_line_aft_pxl_6[2];
	u8 vact_space_1[2];
	u8 vact_space_2[2];
	u8 vact_space_3[2];
	u8 vact_space_4[2];
	u8 vact_space_5[2];
	u8 vact_space_6[2];
};

struct hdmi_4210_conf {
	struct hdmi_4210_core_regs core;
	struct hdmi_tg_regs tg;
};

struct hdmi_4212_conf {
	struct hdmi_4212_core_regs core;
	struct hdmi_tg_regs tg;
};

struct hdmi_conf_regs {
	int pixel_clock;
	int cea_video_id;
	union {
		struct hdmi_4210_conf v4210_conf;
		struct hdmi_4212_conf v4212_conf;
	} conf;
};

struct hdmi_context {
	struct device			*dev;
	struct drm_device		*drm_dev;
	bool				hpd;
	bool				powered;
	bool				dvi_mode;
	spinlock_t                      writemask_lock;

	bool                            hdcp_desired;
	struct work_struct              hdcp_work;
	enum exynos_hdcp_state          hdcp_state;
	bool                            hdcp_repeater;
	int                             hdcp_tries;

	void __iomem			*regs;
	void __iomem			*phy_pow_ctrl_reg;
	void __iomem			*regs_hdmiphy;
	int				irq;
	int				hdcp_irq;
	struct delayed_work		hotplug_work;

	struct i2c_client		*ddc_port;
	struct i2c_client		*hdmiphy_port;
	struct i2c_client               *hdcp_port;

	/* current hdmiphy conf regs */
	struct drm_display_mode		current_mode;
	struct hdmi_conf_regs		mode_conf;

	struct hdmi_resources		res;

	int				hpd_gpio;

	enum hdmi_version		version;
};

struct hdmiphy_config {
	int pixel_clock;
	u8 conf[32];
};

/* list of phy config settings */
static const struct hdmiphy_config hdmiphy_4210_configs[] = {
	{
		.pixel_clock = 27000000,
		.conf = {
			0x01, 0x05, 0x00, 0xD8, 0x10, 0x1C, 0x30, 0x40,
			0x6B, 0x10, 0x02, 0x51, 0xDF, 0xF2, 0x54, 0x87,
			0x84, 0x00, 0x30, 0x38, 0x00, 0x08, 0x10, 0xE0,
			0x22, 0x40, 0xE3, 0x26, 0x00, 0x00, 0x00, 0x00,
		},
	},
	{
		.pixel_clock = 27027000,
		.conf = {
			0x01, 0x05, 0x00, 0xD4, 0x10, 0x9C, 0x09, 0x64,
			0x6B, 0x10, 0x02, 0x51, 0xDF, 0xF2, 0x54, 0x87,
			0x84, 0x00, 0x30, 0x38, 0x00, 0x08, 0x10, 0xE0,
			0x22, 0x40, 0xE3, 0x26, 0x00, 0x00, 0x00, 0x00,
		},
	},
	{
		.pixel_clock = 74176000,
		.conf = {
			0x01, 0x05, 0x00, 0xD8, 0x10, 0x9C, 0xef, 0x5B,
			0x6D, 0x10, 0x01, 0x51, 0xef, 0xF3, 0x54, 0xb9,
			0x84, 0x00, 0x30, 0x38, 0x00, 0x08, 0x10, 0xE0,
			0x22, 0x40, 0xa5, 0x26, 0x01, 0x00, 0x00, 0x00,
		},
	},
	{
		.pixel_clock = 74250000,
		.conf = {
			0x01, 0x05, 0x00, 0xd8, 0x10, 0x9c, 0xf8, 0x40,
			0x6a, 0x10, 0x01, 0x51, 0xff, 0xf1, 0x54, 0xba,
			0x84, 0x00, 0x10, 0x38, 0x00, 0x08, 0x10, 0xe0,
			0x22, 0x40, 0xa4, 0x26, 0x01, 0x00, 0x00, 0x00,
		},
	},
	{
		.pixel_clock = 148500000,
		.conf = {
			0x01, 0x05, 0x00, 0xD8, 0x10, 0x9C, 0xf8, 0x40,
			0x6A, 0x18, 0x00, 0x51, 0xff, 0xF1, 0x54, 0xba,
			0x84, 0x00, 0x10, 0x38, 0x00, 0x08, 0x10, 0xE0,
			0x22, 0x40, 0xa4, 0x26, 0x02, 0x00, 0x00, 0x00,
		},
	},
};

static const struct hdmiphy_config hdmiphy_4212_configs[] = {
	{
		.pixel_clock = 25200000,
		.conf = {
			0x01, 0x51, 0x2A, 0x75, 0x40, 0x01, 0x00, 0x08,
			0x82, 0x80, 0xfc, 0xd8, 0x45, 0xa0, 0xac, 0x80,
			0x08, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xf4, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 27000000,
		.conf = {
			0x01, 0xd1, 0x22, 0x51, 0x40, 0x08, 0xfc, 0x20,
			0x98, 0xa0, 0xcb, 0xd8, 0x45, 0xa0, 0xac, 0x80,
			0x06, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xe4, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 27027000,
		.conf = {
			0x01, 0xd1, 0x2d, 0x72, 0x40, 0x64, 0x12, 0x08,
			0x43, 0xa0, 0x0e, 0xd9, 0x45, 0xa0, 0xac, 0x80,
			0x08, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xe3, 0x24, 0x00, 0x00, 0x00, 0x01, 0x00,
		},
	},
	{
		.pixel_clock = 36000000,
		.conf = {
			0x01, 0x51, 0x2d, 0x55, 0x40, 0x01, 0x00, 0x08,
			0x82, 0x80, 0x0e, 0xd9, 0x45, 0xa0, 0xac, 0x80,
			0x08, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xab, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 40000000,
		.conf = {
			0x01, 0x51, 0x32, 0x55, 0x40, 0x01, 0x00, 0x08,
			0x82, 0x80, 0x2c, 0xd9, 0x45, 0xa0, 0xac, 0x80,
			0x08, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0x9a, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 65000000,
		.conf = {
			0x01, 0xd1, 0x36, 0x34, 0x40, 0x1e, 0x0a, 0x08,
			0x82, 0xa0, 0x45, 0xd9, 0x45, 0xa0, 0xac, 0x80,
			0x08, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xbd, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 71000000,
		.conf = {
			0x01, 0x91, 0x1E, 0x15, 0x40, 0x3C, 0xCE, 0x08,
			0x04, 0x20, 0xB2, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x06, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xAD, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 73250000,
		.conf = {
			0x01, 0xD1, 0x1F, 0x15, 0x40, 0x18, 0xE9, 0x08,
			0x02, 0xA0, 0xB7, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x06, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xA8, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 74176000,
		.conf = {
			0x01, 0xd1, 0x3e, 0x35, 0x40, 0x5b, 0xde, 0x08,
			0x82, 0xa0, 0x73, 0xd9, 0x45, 0xa0, 0xac, 0x80,
			0x56, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xa6, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 74250000,
		.conf = {
			0x01, 0xd1, 0x1f, 0x10, 0x40, 0x40, 0xf8, 0x08,
			0x81, 0xa0, 0xba, 0xd8, 0x45, 0xa0, 0xac, 0x80,
			0x3c, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xa5, 0x24, 0x01, 0x00, 0x00, 0x01, 0x00,
		},
	},
	{
		.pixel_clock = 83500000,
		.conf = {
			0x01, 0xd1, 0x23, 0x11, 0x40, 0x0c, 0xfb, 0x08,
			0x85, 0xa0, 0xd1, 0xd8, 0x45, 0xa0, 0xac, 0x80,
			0x08, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0x93, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 88750000,
		.conf = {
			0x01, 0x91, 0x25, 0x17, 0x40, 0x30, 0xFE, 0x08,
			0x06, 0x20, 0xDE, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x06, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0x8A, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 106500000,
		.conf = {
			0x01, 0xd1, 0x2c, 0x12, 0x40, 0x0c, 0x09, 0x08,
			0x84, 0xa0, 0x0a, 0xd9, 0x45, 0xa0, 0xac, 0x80,
			0x08, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0x73, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 108000000,
		.conf = {
			0x01, 0x51, 0x2d, 0x15, 0x40, 0x01, 0x00, 0x08,
			0x82, 0x80, 0x0e, 0xd9, 0x45, 0xa0, 0xac, 0x80,
			0x08, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xc7, 0x25, 0x03, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 115500000,
		.conf = {
			0x01, 0xD1, 0x30, 0x1A, 0x40, 0x40, 0x10, 0x04,
			0x04, 0xA0, 0x21, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x06, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0xAA, 0x25, 0x03, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 119000000,
		.conf = {
			0x01, 0x91, 0x32, 0x14, 0x40, 0x60, 0xD8, 0x08,
			0x06, 0x20, 0x2A, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x06, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0x9D, 0x25, 0x03, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 146250000,
		.conf = {
			0x01, 0xd1, 0x3d, 0x15, 0x40, 0x18, 0xfd, 0x08,
			0x83, 0xa0, 0x6e, 0xd9, 0x45, 0xa0, 0xac, 0x80,
			0x08, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0x50, 0x25, 0x03, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 148500000,
		.conf = {
			0x01, 0xd1, 0x1f, 0x00, 0x40, 0x40, 0xf8, 0x08,
			0x81, 0xa0, 0xba, 0xd8, 0x45, 0xa0, 0xac, 0x80,
			0x3c, 0x80, 0x11, 0x04, 0x02, 0x22, 0x44, 0x86,
			0x54, 0x4b, 0x25, 0x03, 0x00, 0x00, 0x01, 0x00,
		},
	},
};

static const struct hdmiphy_config hdmiphy_5420_configs[] = {
	{
		.pixel_clock = 25200000,
		.conf = {
			0x01, 0x52, 0x3F, 0x55, 0x40, 0x01, 0x00, 0xC8,
			0x82, 0xC8, 0xBD, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x06, 0x80, 0x01, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0xF4, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 27000000,
		.conf = {
			0x01, 0xD1, 0x22, 0x51, 0x40, 0x08, 0xFC, 0xE0,
			0x98, 0xE8, 0xCB, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x06, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0xE4, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 27027000,
		.conf = {
			0x01, 0xD1, 0x2D, 0x72, 0x40, 0x64, 0x12, 0xC8,
			0x43, 0xE8, 0x0E, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x26, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0xE3, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 36000000,
		.conf = {
			0x01, 0x51, 0x2D, 0x55, 0x40, 0x40, 0x00, 0xC8,
			0x02, 0xC8, 0x0E, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0xAB, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,

		},
	},
	{
		.pixel_clock = 40000000,
		.conf = {
			0x01, 0xD1, 0x21, 0x31, 0x40, 0x3C, 0x28, 0xC8,
			0x87, 0xE8, 0xC8, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0x9A, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 65000000,
		.conf = {
			0x01, 0xD1, 0x36, 0x34, 0x40, 0x0C, 0x04, 0xC8,
			0x82, 0xE8, 0x45, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0xBD, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 71000000,
		.conf = {
			0x01, 0xD1, 0x3B, 0x35, 0x40, 0x0C, 0x04, 0xC8,
			0x85, 0xE8, 0x63, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0x57, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 73250000,
		.conf = {
			0x01, 0xD1, 0x1F, 0x10, 0x40, 0x78, 0x8D, 0xC8,
			0x81, 0xE8, 0xB7, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x56, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0xA8, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 74176000,
		.conf = {
			0x01, 0xD1, 0x1F, 0x10, 0x40, 0x5B, 0xEF, 0xC8,
			0x81, 0xE8, 0xB9, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x56, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0xA6, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 74250000,
		.conf = {
			0x01, 0xD1, 0x1F, 0x10, 0x40, 0x40, 0xF8, 0x08,
			0x81, 0xE8, 0xBA, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x26, 0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x66,
			0x54, 0xA5, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 83500000,
		.conf = {
			0x01, 0xD1, 0x23, 0x11, 0x40, 0x0C, 0xFB, 0xC8,
			0x85, 0xE8, 0xD1, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0x4A, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 88750000,
		.conf = {
			0x01, 0xD1, 0x25, 0x11, 0x40, 0x18, 0xFF, 0xC8,
			0x83, 0xE8, 0xDE, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0x45, 0x24, 0x00, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 106500000,
		.conf = {
			0x01, 0xD1, 0x2C, 0x12, 0x40, 0x0C, 0x09, 0xC8,
			0x84, 0xE8, 0x0A, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0x73, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 108000000,
		.conf = {
			0x01, 0x51, 0x2D, 0x15, 0x40, 0x01, 0x00, 0xC8,
			0x82, 0xC8, 0x0E, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0xC7, 0x25, 0x03, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 115500000,
		.conf = {
			0x01, 0xD1, 0x30, 0x14, 0x40, 0x0C, 0x03, 0xC8,
			0x88, 0xE8, 0x21, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0x6A, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 119000000,
		.conf = {
			0x01, 0xD1, 0x32, 0x14, 0x40, 0x18, 0xEC, 0xC8,
			0x86, 0xE8, 0x2A, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0x67, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 146250000,
		.conf = {
			0x01, 0xD1, 0x3D, 0x15, 0x40, 0x18, 0xFD, 0xC8,
			0x83, 0xE8, 0x6E, 0xD9, 0x45, 0xA0, 0xAC, 0x80,
			0x08, 0x80, 0x09, 0x84, 0x05, 0x02, 0x24, 0x66,
			0x54, 0x54, 0x24, 0x01, 0x00, 0x00, 0x01, 0x80,
		},
	},
	{
		.pixel_clock = 148500000,
		.conf = {
			0x01, 0xD1, 0x1F, 0x00, 0x40, 0x40, 0xF8, 0x08,
			0x81, 0xE8, 0xBA, 0xD8, 0x45, 0xA0, 0xAC, 0x80,
			0x26, 0x80, 0x09, 0x84, 0x05, 0x22, 0x24, 0x66,
			0x54, 0x4B, 0x25, 0x03, 0x00, 0x80, 0x01, 0x80,
		},
	},
};

struct hdmi_infoframe {
	enum HDMI_PACKET_TYPE type;
	u8 ver;
	u8 len;
};

static inline u32 support_hdmi_audio_through_alsa(struct hdmi_context *hdata)
{
	return (hdata->version == HDMI_VER_EXYNOS4212 ||
		hdata->version == HDMI_VER_EXYNOS5420);
}

static inline u32 hdmi_reg_read(struct hdmi_context *hdata, u32 reg_id)
{
	return readl(hdata->regs + reg_id);
}

static inline void hdmi_reg_writeb(struct hdmi_context *hdata,
				 u32 reg_id, u8 value)
{
	writeb(value, hdata->regs + reg_id);
}

static inline void hdmi_reg_writemask(struct hdmi_context *hdata,
				 u32 reg_id, u32 value, u32 mask)
{
	u32 old = readl(hdata->regs + reg_id);
	value = (value & mask) | (old & ~mask);
	writel(value, hdata->regs + reg_id);
}

static void hdmi_reg_writemask_atomic(struct hdmi_context *hdata,
				u32 reg_id, u32 value, u32 mask)
{
	unsigned long flags;

	spin_lock_irqsave(&hdata->writemask_lock, flags);
	hdmi_reg_writemask(hdata, reg_id, value, mask);
	spin_unlock_irqrestore(&hdata->writemask_lock, flags);
}

static inline void hdmi_phy_pow_ctrl_reg_writemask(struct hdmi_context *hdata,
				 u32 value, u32 mask)
{
	u32 old = readl(hdata->phy_pow_ctrl_reg);
	value = (value & mask) | (old & ~mask);
	writel(value, hdata->phy_pow_ctrl_reg);
}

static int hdmiphy_reg_writeb(struct hdmi_context *hdata,
			u32 reg_offset, u8 value)
{
	if (hdata->hdmiphy_port) {
		u8 buffer[2];
		int ret;

		buffer[0] = reg_offset;
		buffer[1] = value;

		ret = i2c_master_send(hdata->hdmiphy_port, buffer, 2);
		if (ret == 2)
			return 0;
		return ret;
	} else {
		writeb(value, hdata->regs_hdmiphy + (reg_offset<<2));
		return 0;
	}
}

static int hdmiphy_reg_write_buf(struct hdmi_context *hdata,
			u32 reg_offset, const u8 *buf, u32 len)
{
	if ((reg_offset + len) > 32)
		return -EINVAL;

	if (hdata->hdmiphy_port) {
		int ret;

		ret = i2c_master_send(hdata->hdmiphy_port, buf, len);
		if (ret == len)
			return 0;
		return ret;
	} else {
		int i;
		for (i = 0; i < len; i++)
			writeb(buf[i], hdata->regs_hdmiphy +
				((reg_offset + i)<<2));
		return 0;
	}
}

static void hdmi_4210_regs_dump(struct hdmi_context *hdata, char *prefix)
{
#define DUMPREG(reg_id) \
	DRM_DEBUG_KMS("%s:" #reg_id " = %08x\n", prefix, \
	readl(hdata->regs + reg_id))
	DRM_DEBUG_KMS("%s: ---- CONTROL REGISTERS ----\n", prefix);
	DUMPREG(HDMI_INTC_FLAG);
	DUMPREG(HDMI_INTC_CON);
	DUMPREG(HDMI_HPD_STATUS);
	DUMPREG(HDMI_4210_PHY_RSTOUT);
	DUMPREG(HDMI_4210_PHY_VPLL);
	DUMPREG(HDMI_4210_PHY_CMU);
	DUMPREG(HDMI_4210_CORE_RSTOUT);

	DRM_DEBUG_KMS("%s: ---- CORE REGISTERS ----\n", prefix);
	DUMPREG(HDMI_CON_0);
	DUMPREG(HDMI_CON_1);
	DUMPREG(HDMI_CON_2);
	DUMPREG(HDMI_SYS_STATUS);
	DUMPREG(HDMI_4210_PHY_STATUS);
	DUMPREG(HDMI_STATUS_EN);
	DUMPREG(HDMI_HPD);
	DUMPREG(HDMI_MODE_SEL);
	DUMPREG(HDMI_4210_HPD_GEN);
	DUMPREG(HDMI_4210_DC_CONTROL);
	DUMPREG(HDMI_4210_VIDEO_PATTERN_GEN);

	DRM_DEBUG_KMS("%s: ---- CORE SYNC REGISTERS ----\n", prefix);
	DUMPREG(HDMI_H_BLANK_0);
	DUMPREG(HDMI_H_BLANK_1);
	DUMPREG(HDMI_4210_V_BLANK_0);
	DUMPREG(HDMI_4210_V_BLANK_1);
	DUMPREG(HDMI_4210_V_BLANK_2);
	DUMPREG(HDMI_4210_H_V_LINE_0);
	DUMPREG(HDMI_4210_H_V_LINE_1);
	DUMPREG(HDMI_4210_H_V_LINE_2);
	DUMPREG(HDMI_VSYNC_POL);
	DUMPREG(HDMI_INT_PRO_MODE);
	DUMPREG(HDMI_4210_V_BLANK_F_0);
	DUMPREG(HDMI_4210_V_BLANK_F_1);
	DUMPREG(HDMI_4210_V_BLANK_F_2);
	DUMPREG(HDMI_4210_H_SYNC_GEN_0);
	DUMPREG(HDMI_4210_H_SYNC_GEN_1);
	DUMPREG(HDMI_4210_H_SYNC_GEN_2);
	DUMPREG(HDMI_4210_V_SYNC_GEN_1_0);
	DUMPREG(HDMI_4210_V_SYNC_GEN_1_1);
	DUMPREG(HDMI_4210_V_SYNC_GEN_1_2);
	DUMPREG(HDMI_4210_V_SYNC_GEN_2_0);
	DUMPREG(HDMI_4210_V_SYNC_GEN_2_1);
	DUMPREG(HDMI_4210_V_SYNC_GEN_2_2);
	DUMPREG(HDMI_4210_V_SYNC_GEN_3_0);
	DUMPREG(HDMI_4210_V_SYNC_GEN_3_1);
	DUMPREG(HDMI_4210_V_SYNC_GEN_3_2);

	DRM_DEBUG_KMS("%s: ---- TG REGISTERS ----\n", prefix);
	DUMPREG(HDMI_TG_CMD);
	DUMPREG(HDMI_TG_H_FSZ_L);
	DUMPREG(HDMI_TG_H_FSZ_H);
	DUMPREG(HDMI_TG_HACT_ST_L);
	DUMPREG(HDMI_TG_HACT_ST_H);
	DUMPREG(HDMI_TG_HACT_SZ_L);
	DUMPREG(HDMI_TG_HACT_SZ_H);
	DUMPREG(HDMI_TG_V_FSZ_L);
	DUMPREG(HDMI_TG_V_FSZ_H);
	DUMPREG(HDMI_TG_VSYNC_L);
	DUMPREG(HDMI_TG_VSYNC_H);
	DUMPREG(HDMI_TG_VSYNC2_L);
	DUMPREG(HDMI_TG_VSYNC2_H);
	DUMPREG(HDMI_TG_VACT_ST_L);
	DUMPREG(HDMI_TG_VACT_ST_H);
	DUMPREG(HDMI_TG_VACT_SZ_L);
	DUMPREG(HDMI_TG_VACT_SZ_H);
	DUMPREG(HDMI_TG_FIELD_CHG_L);
	DUMPREG(HDMI_TG_FIELD_CHG_H);
	DUMPREG(HDMI_TG_VACT_ST2_L);
	DUMPREG(HDMI_TG_VACT_ST2_H);
	DUMPREG(HDMI_TG_VSYNC_TOP_HDMI_L);
	DUMPREG(HDMI_TG_VSYNC_TOP_HDMI_H);
	DUMPREG(HDMI_TG_VSYNC_BOT_HDMI_L);
	DUMPREG(HDMI_TG_VSYNC_BOT_HDMI_H);
	DUMPREG(HDMI_TG_FIELD_TOP_HDMI_L);
	DUMPREG(HDMI_TG_FIELD_TOP_HDMI_H);
	DUMPREG(HDMI_TG_FIELD_BOT_HDMI_L);
	DUMPREG(HDMI_TG_FIELD_BOT_HDMI_H);
#undef DUMPREG
}

static void hdmi_4212_regs_dump(struct hdmi_context *hdata, char *prefix)
{
	int i;

#define DUMPREG(reg_id) \
	DRM_DEBUG_KMS("%s:" #reg_id " = %08x\n", prefix, \
	readl(hdata->regs + reg_id))

	DRM_DEBUG_KMS("%s: ---- CONTROL REGISTERS ----\n", prefix);
	DUMPREG(HDMI_INTC_CON);
	DUMPREG(HDMI_INTC_FLAG);
	DUMPREG(HDMI_HPD_STATUS);
	DUMPREG(HDMI_INTC_CON_1);
	DUMPREG(HDMI_INTC_FLAG_1);
	DUMPREG(HDMI_PHY_STATUS_0);
	DUMPREG(HDMI_PHY_STATUS_PLL);
	DUMPREG(HDMI_PHY_CON_0);
	DUMPREG(HDMI_PHY_RSTOUT);
	DUMPREG(HDMI_PHY_VPLL);
	DUMPREG(HDMI_PHY_CMU);
	DUMPREG(HDMI_CORE_RSTOUT);

	DRM_DEBUG_KMS("%s: ---- CORE REGISTERS ----\n", prefix);
	DUMPREG(HDMI_CON_0);
	DUMPREG(HDMI_CON_1);
	DUMPREG(HDMI_CON_2);
	DUMPREG(HDMI_SYS_STATUS);
	DUMPREG(HDMI_PHY_STATUS_0);
	DUMPREG(HDMI_STATUS_EN);
	DUMPREG(HDMI_HPD);
	DUMPREG(HDMI_MODE_SEL);
	DUMPREG(HDMI_ENC_EN);
	DUMPREG(HDMI_DC_CONTROL);
	DUMPREG(HDMI_VIDEO_PATTERN_GEN);

	DRM_DEBUG_KMS("%s: ---- CORE SYNC REGISTERS ----\n", prefix);
	DUMPREG(HDMI_H_BLANK_0);
	DUMPREG(HDMI_H_BLANK_1);
	DUMPREG(HDMI_V2_BLANK_0);
	DUMPREG(HDMI_V2_BLANK_1);
	DUMPREG(HDMI_V1_BLANK_0);
	DUMPREG(HDMI_V1_BLANK_1);
	DUMPREG(HDMI_V_LINE_0);
	DUMPREG(HDMI_V_LINE_1);
	DUMPREG(HDMI_H_LINE_0);
	DUMPREG(HDMI_H_LINE_1);
	DUMPREG(HDMI_HSYNC_POL);

	DUMPREG(HDMI_VSYNC_POL);
	DUMPREG(HDMI_INT_PRO_MODE);
	DUMPREG(HDMI_V_BLANK_F0_0);
	DUMPREG(HDMI_V_BLANK_F0_1);
	DUMPREG(HDMI_V_BLANK_F1_0);
	DUMPREG(HDMI_V_BLANK_F1_1);

	DUMPREG(HDMI_H_SYNC_START_0);
	DUMPREG(HDMI_H_SYNC_START_1);
	DUMPREG(HDMI_H_SYNC_END_0);
	DUMPREG(HDMI_H_SYNC_END_1);

	DUMPREG(HDMI_V_SYNC_LINE_BEF_2_0);
	DUMPREG(HDMI_V_SYNC_LINE_BEF_2_1);
	DUMPREG(HDMI_V_SYNC_LINE_BEF_1_0);
	DUMPREG(HDMI_V_SYNC_LINE_BEF_1_1);

	DUMPREG(HDMI_V_SYNC_LINE_AFT_2_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_2_1);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_1_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_1_1);

	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_2_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_2_1);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_1_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_1_1);

	DUMPREG(HDMI_V_BLANK_F2_0);
	DUMPREG(HDMI_V_BLANK_F2_1);
	DUMPREG(HDMI_V_BLANK_F3_0);
	DUMPREG(HDMI_V_BLANK_F3_1);
	DUMPREG(HDMI_V_BLANK_F4_0);
	DUMPREG(HDMI_V_BLANK_F4_1);
	DUMPREG(HDMI_V_BLANK_F5_0);
	DUMPREG(HDMI_V_BLANK_F5_1);

	DUMPREG(HDMI_V_SYNC_LINE_AFT_3_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_3_1);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_4_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_4_1);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_5_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_5_1);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_6_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_6_1);

	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_3_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_3_1);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_4_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_4_1);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_5_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_5_1);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_6_0);
	DUMPREG(HDMI_V_SYNC_LINE_AFT_PXL_6_1);

	DUMPREG(HDMI_VACT_SPACE_1_0);
	DUMPREG(HDMI_VACT_SPACE_1_1);
	DUMPREG(HDMI_VACT_SPACE_2_0);
	DUMPREG(HDMI_VACT_SPACE_2_1);
	DUMPREG(HDMI_VACT_SPACE_3_0);
	DUMPREG(HDMI_VACT_SPACE_3_1);
	DUMPREG(HDMI_VACT_SPACE_4_0);
	DUMPREG(HDMI_VACT_SPACE_4_1);
	DUMPREG(HDMI_VACT_SPACE_5_0);
	DUMPREG(HDMI_VACT_SPACE_5_1);
	DUMPREG(HDMI_VACT_SPACE_6_0);
	DUMPREG(HDMI_VACT_SPACE_6_1);

	DRM_DEBUG_KMS("%s: ---- TG REGISTERS ----\n", prefix);
	DUMPREG(HDMI_TG_CMD);
	DUMPREG(HDMI_TG_H_FSZ_L);
	DUMPREG(HDMI_TG_H_FSZ_H);
	DUMPREG(HDMI_TG_HACT_ST_L);
	DUMPREG(HDMI_TG_HACT_ST_H);
	DUMPREG(HDMI_TG_HACT_SZ_L);
	DUMPREG(HDMI_TG_HACT_SZ_H);
	DUMPREG(HDMI_TG_V_FSZ_L);
	DUMPREG(HDMI_TG_V_FSZ_H);
	DUMPREG(HDMI_TG_VSYNC_L);
	DUMPREG(HDMI_TG_VSYNC_H);
	DUMPREG(HDMI_TG_VSYNC2_L);
	DUMPREG(HDMI_TG_VSYNC2_H);
	DUMPREG(HDMI_TG_VACT_ST_L);
	DUMPREG(HDMI_TG_VACT_ST_H);
	DUMPREG(HDMI_TG_VACT_SZ_L);
	DUMPREG(HDMI_TG_VACT_SZ_H);
	DUMPREG(HDMI_TG_FIELD_CHG_L);
	DUMPREG(HDMI_TG_FIELD_CHG_H);
	DUMPREG(HDMI_TG_VACT_ST2_L);
	DUMPREG(HDMI_TG_VACT_ST2_H);
	DUMPREG(HDMI_TG_VACT_ST3_L);
	DUMPREG(HDMI_TG_VACT_ST3_H);
	DUMPREG(HDMI_TG_VACT_ST4_L);
	DUMPREG(HDMI_TG_VACT_ST4_H);
	DUMPREG(HDMI_TG_VSYNC_TOP_HDMI_L);
	DUMPREG(HDMI_TG_VSYNC_TOP_HDMI_H);
	DUMPREG(HDMI_TG_VSYNC_BOT_HDMI_L);
	DUMPREG(HDMI_TG_VSYNC_BOT_HDMI_H);
	DUMPREG(HDMI_TG_FIELD_TOP_HDMI_L);
	DUMPREG(HDMI_TG_FIELD_TOP_HDMI_H);
	DUMPREG(HDMI_TG_FIELD_BOT_HDMI_L);
	DUMPREG(HDMI_TG_FIELD_BOT_HDMI_H);
	DUMPREG(HDMI_TG_3D);

	DRM_DEBUG_KMS("%s: ---- PACKET REGISTERS ----\n", prefix);
	DUMPREG(HDMI_AVI_CON);
	DUMPREG(HDMI_AVI_HEADER0);
	DUMPREG(HDMI_AVI_HEADER1);
	DUMPREG(HDMI_AVI_HEADER2);
	DUMPREG(HDMI_AVI_CHECK_SUM);
	DUMPREG(HDMI_VSI_CON);
	DUMPREG(HDMI_VSI_HEADER0);
	DUMPREG(HDMI_VSI_HEADER1);
	DUMPREG(HDMI_VSI_HEADER2);
	for (i = 0; i < 7; ++i)
		DUMPREG(HDMI_VSI_DATA(i));

#undef DUMPREG
}

static void hdmi_regs_dump(struct hdmi_context *hdata, char *prefix)
{
	if (hdata->version == HDMI_VER_EXYNOS4210)
		hdmi_4210_regs_dump(hdata, prefix);
	else
		hdmi_4212_regs_dump(hdata, prefix);
}

static u8 hdmi_chksum(struct hdmi_context *hdata,
			u32 start, u8 len, u32 hdr_sum)
{
	int i;

	/* hdr_sum : header0 + header1 + header2
	* start : start address of packet byte1
	* len : packet bytes - 1 */
	for (i = 0; i < len; ++i)
		hdr_sum += 0xff & hdmi_reg_read(hdata, start + i * 4);

	/* return 2's complement of 8 bit hdr_sum */
	return (u8)(~(hdr_sum & 0xff) + 1);
}

static void hdmi_reg_infoframe(struct hdmi_context *hdata,
			struct hdmi_infoframe *infoframe)
{
	u32 hdr_sum;
	u8 chksum;
	u32 aspect_ratio;
	u32 mod;
	u32 vic;

	/* TODO: stringify HDMI_PACKET_TYPE */
	DRM_DEBUG_KMS("type: %d ver: %d len: %d\n", infoframe->type,
			infoframe->ver, infoframe->len);

	mod = hdmi_reg_read(hdata, HDMI_MODE_SEL);
	if (hdata->dvi_mode) {
		hdmi_reg_writeb(hdata, HDMI_VSI_CON,
				HDMI_VSI_CON_DO_NOT_TRANSMIT);
		hdmi_reg_writeb(hdata, HDMI_AVI_CON,
				HDMI_AVI_CON_DO_NOT_TRANSMIT);
		hdmi_reg_writeb(hdata, HDMI_AUI_CON, HDMI_AUI_CON_NO_TRAN);
		return;
	}

	switch (infoframe->type) {
	case HDMI_PACKET_TYPE_AVI:
		hdmi_reg_writeb(hdata, HDMI_AVI_CON, HDMI_AVI_CON_EVERY_VSYNC);
		hdmi_reg_writeb(hdata, HDMI_AVI_HEADER0, infoframe->type);
		hdmi_reg_writeb(hdata, HDMI_AVI_HEADER1, infoframe->ver);
		hdmi_reg_writeb(hdata, HDMI_AVI_HEADER2, infoframe->len);
		hdr_sum = infoframe->type + infoframe->ver + infoframe->len;

		/* Output format zero hardcoded ,RGB YBCR selection */
		hdmi_reg_writeb(hdata, HDMI_AVI_BYTE(1), 0 << 5 |
			AVI_ACTIVE_FORMAT_VALID |
			AVI_UNDERSCANNED_DISPLAY_VALID);

		aspect_ratio = AVI_PIC_ASPECT_RATIO_16_9;

		hdmi_reg_writeb(hdata, HDMI_AVI_BYTE(2), aspect_ratio |
				AVI_SAME_AS_PIC_ASPECT_RATIO);

		vic = hdata->mode_conf.cea_video_id;
		hdmi_reg_writeb(hdata, HDMI_AVI_BYTE(4), vic);

		chksum = hdmi_chksum(hdata, HDMI_AVI_BYTE(1),
					infoframe->len, hdr_sum);
		DRM_DEBUG_KMS("AVI checksum = 0x%x\n", chksum);
		hdmi_reg_writeb(hdata, HDMI_AVI_CHECK_SUM, chksum);
		break;
	case HDMI_PACKET_TYPE_AUI:
		hdmi_reg_writeb(hdata, HDMI_AUI_CON, 0x02);
		hdmi_reg_writeb(hdata, HDMI_AUI_HEADER0, infoframe->type);
		hdmi_reg_writeb(hdata, HDMI_AUI_HEADER1, infoframe->ver);
		hdmi_reg_writeb(hdata, HDMI_AUI_HEADER2, infoframe->len);
		hdr_sum = infoframe->type + infoframe->ver + infoframe->len;
		chksum = hdmi_chksum(hdata, HDMI_AUI_BYTE(1),
					infoframe->len, hdr_sum);
		DRM_DEBUG_KMS("AUI checksum = 0x%x\n", chksum);
		hdmi_reg_writeb(hdata, HDMI_AUI_CHECK_SUM, chksum);
		break;
	default:
		break;
	}
}

static int hdmi_initialize(void *ctx, struct drm_device *drm_dev)
{
	struct hdmi_context *hdata = ctx;

	hdata->drm_dev = drm_dev;

	return 0;
}

static bool hdmi_is_connected(void *ctx)
{
	struct hdmi_context *hdata = ctx;

	hdata->hpd = gpio_get_value(hdata->hpd_gpio);

	return hdata->hpd;
}

static struct edid *hdmi_get_edid(void *ctx, struct drm_connector *connector)
{
	struct edid *raw_edid;
	struct hdmi_context *hdata = ctx;

	if (!hdata->ddc_port)
		return ERR_PTR(-ENODEV);

	DRM_DEBUG_KMS("[CONNECTOR:%d:%s]\n", DRM_BASE_ID(connector),
			drm_get_connector_name(connector));

	raw_edid = drm_get_edid(connector, hdata->ddc_port->adapter);
	if (!raw_edid)
		return ERR_PTR(-ENODEV);

	hdata->dvi_mode = !drm_detect_hdmi_monitor(raw_edid);
	DRM_DEBUG_KMS("%s : width[%d] x height[%d]\n",
		(hdata->dvi_mode ? "dvi monitor" : "hdmi monitor"),
		raw_edid->width_cm, raw_edid->height_cm);

	return raw_edid;
}

static int hdmi_find_phy_conf(struct hdmi_context *hdata, u32 pixel_clock)
{
	const struct hdmiphy_config *confs;
	int count, i;

	DRM_DEBUG_KMS("pixel_clock: %u\n", pixel_clock);

	if (hdata->version == HDMI_VER_EXYNOS4210) {
		confs = hdmiphy_4210_configs;
		count = ARRAY_SIZE(hdmiphy_4210_configs);
	} else if (hdata->version == HDMI_VER_EXYNOS4212) {
		confs = hdmiphy_4212_configs;
		count = ARRAY_SIZE(hdmiphy_4212_configs);
	} else if (hdata->version == HDMI_VER_EXYNOS5420) {
		confs = hdmiphy_5420_configs;
		count = ARRAY_SIZE(hdmiphy_5420_configs);
	} else
		return -EINVAL;

	for (i = 0; i < count; i++)
		if (confs[i].pixel_clock == pixel_clock)
			return i;

	DRM_DEBUG_KMS("Could not find phy config for pixel_clock: %u\n",
			pixel_clock);
	return -EINVAL;
}

static int hdmi_check_mode(void *ctx, struct drm_display_mode *mode)
{
	struct hdmi_context *hdata = ctx;
	int ret;

	/* We're only dependent on mixer in 4212 & 5420 hdmi */
	if (hdata->version != HDMI_VER_EXYNOS4210) {
		ret = mixer_check_mode(mode, mixer_get_version(hdata->version));
		if (ret)
			return ret;
	}

	ret = hdmi_find_phy_conf(hdata, mode->clock * 1000);
	if (ret < 0)
		return ret;

	DRM_DEBUG_KMS("[MODE:%s] %ux%u vrefresh: %d, interlace:%d clock: %d kHz is OK\n",
			mode->name, mode->hdisplay, mode->vdisplay,
			mode->vrefresh,
			!!(mode->flags & DRM_MODE_FLAG_INTERLACE),
			mode->clock);

	return 0;
}

static void hdmi_mode_fixup(void *in_ctx, struct drm_connector *connector,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct drm_display_mode *m;
	int mode_ok;

	DRM_DEBUG_KMS("[CONNECTOR:%d:%s] [MODE:%s]\n", DRM_BASE_ID(connector),
			drm_get_connector_name(connector), mode->name);

	/*
	 * Match the incoming mode to a mode in the connector list and copy it
	 * over. This is important since this might be an adjusted mode from the
	 * mixer and those have differing crtc_* values.
	 */
	list_for_each_entry(m, &connector->modes, head) {
		if (mode->hdisplay == m->hdisplay &&
		    mode->vdisplay == m->vdisplay &&
		    mode->clock == m->clock) {
			drm_mode_copy(adjusted_mode, m);
			return;
		}
	}

	/*
	 * otherwise, find the most suitable mode among modes and change it
	 * to adjusted_mode.
	 */
	list_for_each_entry(m, &connector->modes, head) {
		mode_ok = hdmi_check_mode(in_ctx, m);

		if (mode_ok == 0) {
			DRM_INFO("HDMI: desired [MODE:%s] does not exist. Using [%dx%d @ %d Hz]\n",
					mode->name, m->hdisplay, m->vdisplay,
					m->vrefresh);

			drm_mode_copy(adjusted_mode, m);
			break;
		}
	}
}

static int hdcp_exchange_ksvs(struct hdmi_context *hdata)
{
	u8 bksv[HDCP_I2C_LEN_BKSV], bcaps[HDCP_I2C_LEN_BCAPS];
	int ret, i, ones = 0;

	if (hdata->hdcp_state != HDCP_STATE_WAIT_ACTIVE_RX)
		return -EINVAL;

	ret = i2c_smbus_read_i2c_block_data(hdata->hdcp_port,
			HDCP_I2C_ADDR_BCAPS, HDCP_I2C_LEN_BCAPS, bcaps);
	if (ret != HDCP_I2C_LEN_BCAPS) {
		DRM_ERROR("Could not read bcaps (%d)\n", ret);
		return -EIO;
	}
	hdmi_reg_writeb(hdata, HDMI_HDCP_BCAPS, bcaps[0]);
	hdata->hdcp_repeater = bcaps[0] & HDCP_I2C_MASK_BCAPS_REPEATER;

	ret = i2c_smbus_read_i2c_block_data(hdata->hdcp_port,
			HDCP_I2C_ADDR_BKSV, HDCP_I2C_LEN_BKSV, bksv);
	if (ret != HDCP_I2C_LEN_BKSV) {
		DRM_ERROR("Could not read bksv (%d)\n", ret);
		return -EIO;
	}

	/* Validate bksv by ensuring there are 20 1's */
	for (i = 0; i < HDCP_I2C_LEN_BKSV; i++) {
		u32 tmp = bksv[i];

		ones += hweight_long(tmp);
	}
	if (ones != 20) {
		DRM_ERROR("Invalid bksv, ones=%d\n", ones);
		return -ENODEV;
	}

	for (i = 0; i < HDCP_I2C_LEN_BKSV; i++)
		hdmi_reg_writeb(hdata, HDMI_HDCP_BKSV(i), bksv[i]);

	hdata->hdcp_state = HDCP_STATE_EXCHANGE_KSV;
	return 0;
}

static int hdcp_an_write(struct hdmi_context *hdata)
{
	int ret, i;
	u8 an[HDCP_I2C_LEN_AN], aksv[HDCP_I2C_LEN_AKSV];
	u32 value;

	if (hdata->hdcp_state != HDCP_STATE_EXCHANGE_KSV)
		return -EINVAL;

	for (i = 0; i < HDCP_I2C_LEN_AN; i++) {
		value = hdmi_reg_read(hdata, HDMI_HDCP_AN(i));
		an[i] = value & 0xff;
	}

	ret = i2c_smbus_write_i2c_block_data(hdata->hdcp_port, HDCP_I2C_ADDR_AN,
			HDCP_I2C_LEN_AN, an);
	if (ret) {
		DRM_ERROR("Failed to send An value to receiver (%d)\n", ret);
		return ret;
	}

	for (i = 0; i < HDCP_I2C_LEN_AKSV; i++) {
		value = hdmi_reg_read(hdata, HDMI_HDCP_AKSV(i));
		aksv[i] = value & 0xff;
	}

	ret = i2c_smbus_write_i2c_block_data(hdata->hdcp_port,
			HDCP_I2C_ADDR_AKSV, HDCP_I2C_LEN_AKSV, aksv);
	if (ret) {
		DRM_ERROR("Failed to send Aksv value to receiver (%d)\n", ret);
		return ret;
	}

	hdata->hdcp_state = HDCP_STATE_COMPUTATIONS;
	return 0;
}

static int hdcp_enable_encryption(struct hdmi_context *hdata)
{
	int ret;

	if (hdata->hdcp_state == HDCP_STATE_AUTHENTICATED)
		return 0;

	ret = wait_for(hdmi_reg_read(hdata, HDMI_SYS_STATUS) &
			HDMI_SYS_STATUS_MASK_AUTH_ACK, AUTH_ACK_TIMEOUT_MSECS,
			1);
	if (ret) {
		DRM_ERROR("Wait for auth ack failed with %d\n", ret);
		return ret;
	}

	hdmi_reg_writemask(hdata, HDMI_ENC_EN, ~0, HDMI_ENC_EN_MASK_ENABLE);
	hdata->hdcp_state = HDCP_STATE_AUTHENTICATED;
	DRM_INFO("HDCP has been successfully enabled\n");
	return 0;
}

static int hdcp_compare_ri(struct hdmi_context *hdata)
{
	int ret, i;
	u8 ari[HDCP_I2C_LEN_RI], bri[HDCP_I2C_LEN_RI];
	u32 value;

	for (i = 0; i < HDCP_I2C_LEN_RI; i++) {
		value = hdmi_reg_read(hdata, HDMI_HDCP_RI(i));
		ari[i] = value & 0xff;
	}

	ret = i2c_smbus_read_i2c_block_data(hdata->hdcp_port,
			HDCP_I2C_ADDR_RI, HDCP_I2C_LEN_RI, bri);
	if (ret != HDCP_I2C_LEN_RI) {
		DRM_ERROR("Failed to read Ri from i2c (%d)\n", ret);
		return ret;
	}

	if (!memcmp(ari, bri, HDCP_I2C_LEN_RI))
		return 0;

	return -EAGAIN;
}

static int hdcp_update_ri(struct hdmi_context *hdata)
{
	int ret, compare_ret;

	if (hdata->hdcp_state != HDCP_STATE_COMPUTATIONS &&
	    hdata->hdcp_state != HDCP_STATE_AUTHENTICATED)
		return -EINVAL;

	ret = wait_for((compare_ret = hdcp_compare_ri(hdata)) != -EAGAIN,
			RI_VERIFY_TIMEOUT_MSECS, 5);
	if (ret || compare_ret) {
		DRM_ERROR("Transmitter and receiver Ri differ\n");
		hdmi_reg_writemask(hdata, HDMI_HDCP_CHECK_RESULT, ~0,
				HDMI_HDCP_CHECK_RESULT_MASK_DIFFER);
		return -EINVAL;
	}

	hdmi_reg_writemask(hdata, HDMI_HDCP_CHECK_RESULT, ~0,
			HDMI_HDCP_CHECK_RESULT_MASK_EQUAL);

	if (!hdata->hdcp_repeater &&
	    hdata->hdcp_state != HDCP_STATE_AUTHENTICATED)
		return hdcp_enable_encryption(hdata);
	else if (hdata->hdcp_state == HDCP_STATE_COMPUTATIONS)
		hdata->hdcp_state = HDCP_STATE_READ_KSV_LIST;

	return 0;
}

static int hdcp_add_repeater_ksv(struct hdmi_context *hdata, u8 *ksv, bool last)
{
	int ret, i;
	u8 val = HDMI_HDCP_KSV_LIST_CON_MASK_DONE;

	for (i = 0; i < HDCP_I2C_LEN_BKSV; i++)
		hdmi_reg_writeb(hdata, HDMI_HDCP_KSV_LIST(i), ksv[i]);

	if (last)
		val |= HDMI_HDCP_KSV_LIST_CON_MASK_END;

	hdmi_reg_writeb(hdata, HDMI_HDCP_KSV_LIST_CON, val);

	/* Don't wait for the last key */
	if (last)
		return 0;

	/* Wait for the key to be read from KSV registers before continuing */
	ret = wait_for(hdmi_reg_read(hdata, HDMI_HDCP_KSV_LIST_CON) &
			HDMI_HDCP_KSV_LIST_CON_MASK_READ,
			KSV_READ_KEY_TIMEOUT_MSECS, 5);
	if (ret) {
		DRM_ERROR("Failed waiting for KSV key read with %d\n", ret);
		return ret;
	}

	return 0;
}

static int hdcp_check_ksv_ready(struct hdmi_context *hdata, u8 *bcaps)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(hdata->hdcp_port,
			HDCP_I2C_ADDR_BCAPS, HDCP_I2C_LEN_BCAPS, bcaps);
	if (ret != HDCP_I2C_LEN_BCAPS) {
		DRM_ERROR("Failed to read BCAPS (%d)\n", ret);
		return ret;
	}

	if (bcaps[0] & HDCP_I2C_MASK_BCAPS_READY)
		return 0;

	return -EAGAIN;
}

static int hdcp_repeater_auth(struct hdmi_context *hdata)
{
	int ret, ksv_ret, ksv_len, i;
	u8 bcaps[HDCP_I2C_LEN_BCAPS], bstatus[HDCP_I2C_LEN_BSTATUS],
		vprime[HDCP_I2C_LEN_V_PRIME], val, *ksv_list;

	if (hdata->hdcp_state != HDCP_STATE_READ_KSV_LIST)
		return -EINVAL;

	/* Wait for the repeater to compile a list of KSVs from downstream */
	ret = wait_for((ksv_ret = hdcp_check_ksv_ready(hdata, bcaps)) !=
			-EAGAIN, KSV_READY_TIMEOUT_MSECS, 5);
	if (ret || ksv_ret) {
		DRM_ERROR("KSV list from repeater not ready %d\n", ret);
		return ret;
	}
	hdmi_reg_writeb(hdata, HDMI_HDCP_BCAPS, bcaps[0]);

	/* Make sure the repeater hasn't encountered an error */
	ret = i2c_smbus_read_i2c_block_data(hdata->hdcp_port,
			HDCP_I2C_ADDR_BSTATUS, HDCP_I2C_LEN_BSTATUS, bstatus);
	if (ret != HDCP_I2C_LEN_BSTATUS) {
		DRM_ERROR("Failed to read status from repeater (%d)\n", ret);
		return ret;
	}
	if (bstatus[0] & HDCP_I2C_MASK_BSTATUS_0_MAX_DEVS) {
		DRM_ERROR("Connected repeater has exceeded the max devices!\n");
		return -ENODEV;
	}
	if (bstatus[1] & HDCP_I2C_MASK_BSTATUS_1_MAX_CASCADE) {
		DRM_ERROR("Connected repeater has exceeded the max depth!\n");
		return -ENODEV;
	}
	hdmi_reg_writeb(hdata, HDMI_HDCP_BSTATUS_0, bstatus[0]);
	hdmi_reg_writeb(hdata, HDMI_HDCP_BSTATUS_1, bstatus[1]);

	/* No receivers downstream of repeater */
	if (!bstatus[0]) {
		hdmi_reg_writemask(hdata, HDMI_HDCP_KSV_LIST_CON, ~0,
				HDMI_HDCP_KSV_LIST_CON_MASK_EMPTY);
		return 0;
	}

	ksv_len = bstatus[0] * HDCP_I2C_LEN_BKSV;
	ksv_list = kzalloc(ksv_len, GFP_KERNEL);
	if (!ksv_list) {
		DRM_ERROR("Could not allocate ksv_list\n");
		return -ENOMEM;
	}

	/* Read the repeater's KSV list */
	ret = i2c_smbus_read_i2c_block_data(hdata->hdcp_port,
			HDCP_I2C_ADDR_KSV_FIFO, ksv_len, ksv_list);
	if (ret != ksv_len) {
		DRM_ERROR("Failed to read ksv list from repeater (%d)\n", ret);
		goto out;
	}
	for (i = 0; i < bstatus[0]; i++) {
		u8 *ksv = ksv_list + i * HDCP_I2C_LEN_BKSV;

		ret = hdcp_add_repeater_ksv(hdata, ksv, i == (bstatus[0] - 1));
		if (ret) {
			DRM_ERROR("Add repeater key %d failed (%d)\n", i, ret);
			goto out;
		}
	}

	/* Read the repeater's SHA-1 hash of KSV/BStatus/M0 */
	ret = i2c_smbus_read_i2c_block_data(hdata->hdcp_port,
			HDCP_I2C_ADDR_V_PRIME, HDCP_I2C_LEN_V_PRIME, vprime);
	if (ret != HDCP_I2C_LEN_V_PRIME) {
		DRM_ERROR("Failed to read V' from i2c (%d)\n", ret);
		goto out;
	}

	/* Write the SHA-1 hash back to the AP */
	hdmi_reg_writeb(hdata, HDMI_HDCP_SHA_RESULT, 0);
	for (i = 0; i < HDCP_I2C_LEN_V_PRIME; i++)
		hdmi_reg_writeb(hdata, HDMI_HDCP_SHA1(i), vprime[i]);

	ret = wait_for((val = hdmi_reg_read(hdata, HDMI_HDCP_SHA_RESULT)) &
			HDMI_HDCP_SHA_RESULT_MASK_READY,
			SHA_COMPARE_TIMEOUT_MSECS, 5);
	if (ret) {
		DRM_ERROR("Waiting for SHA-1 result failed with %d\n", ret);
		goto sha_result;
	}

	if (val & HDMI_HDCP_SHA_RESULT_MASK_VALID) {
		ret = hdcp_enable_encryption(hdata);
	} else {
		DRM_ERROR("SHA-1 result is not valid\n");
		ret = -ENODEV;
	}

sha_result:
	hdmi_reg_writeb(hdata, HDMI_HDCP_SHA_RESULT, 0);
out:
	kfree(ksv_list);
	return ret;
}

static int hdcp_update_drm_property(struct hdmi_context *hdata, int value)
{
	struct drm_mode_config *mode_config = &hdata->drm_dev->mode_config;
	struct drm_connector *connector;

	WARN_ON(!mutex_is_locked(&mode_config->mutex));

	list_for_each_entry(connector, &mode_config->connector_list, head) {
		if (connector->connector_type == DRM_MODE_CONNECTOR_HDMIA)
			return drm_object_property_set_value(&connector->base,
				mode_config->content_protection_property,
				value);
	}
	return -ENODEV;
}

static void hdcp_disable(struct hdmi_context *hdata)
{
	hdmi_reg_writemask_atomic(hdata, HDMI_INTC_CON, 0, HDMI_INTC_EN_HDCP);
	hdmi_reg_writemask(hdata, HDMI_HDCP_CTRL1, 0,
			HDMI_HDCP_CTRL1_MASK_CP_DESIRED);
	hdmi_reg_writemask(hdata, HDMI_STATUS_EN, 0,
			HDMI_STATUS_EN_MASK_ACTIVE_RX |
			HDMI_STATUS_EN_MASK_WATCHDOG |
			HDMI_STATUS_EN_MASK_AN_WRITE |
			HDMI_STATUS_EN_MASK_UPDATE_RI);
	hdmi_reg_writemask(hdata, HDMI_SYS_STATUS, ~0,
			HDMI_SYS_STATUS_MASK_ACTIVE_RX |
			HDMI_SYS_STATUS_MASK_WATCHDOG |
			HDMI_SYS_STATUS_MASK_AN_WRITE |
			HDMI_SYS_STATUS_MASK_UPDATE_RI);
	hdmi_reg_writemask(hdata, HDMI_ENC_EN, 0, HDMI_ENC_EN_MASK_ENABLE);
	hdmi_reg_writemask(hdata, HDMI_HDCP_CHECK_RESULT, 0,
			HDMI_HDCP_CHECK_RESULT_MASK_CLEAR);

	/*
	 * Avert your eyes! We simulate a hotplug here, because that's the only
	 * way to reset the HDCP core so we can restart it later.
	 */
	hdmi_reg_writemask(hdata, HDMI_HPD, 0, HDMI_HPD_HPD_SEL);
	hdmi_reg_writemask(hdata, HDMI_HPD, ~0, HDMI_HPD_HPD_SEL);

	hdata->hdcp_state = HDCP_STATE_OFF;

	/*
	 * It's possible we're stopping because of hotplug, so make sure we
	 * maintain the hdcp desiredness
	 */
	if (hdata->hdcp_desired)
		hdcp_update_drm_property(hdata,
				DRM_MODE_CONTENT_PROTECTION_DESIRED);
}

static void hdcp_enable(struct hdmi_context *hdata)
{
	hdata->hdcp_state = HDCP_STATE_WAIT_ACTIVE_RX;

	hdmi_reg_writemask(hdata, HDMI_STATUS_EN, ~0,
			HDMI_STATUS_EN_MASK_ACTIVE_RX |
			HDMI_STATUS_EN_MASK_WATCHDOG |
			HDMI_STATUS_EN_MASK_AN_WRITE |
			HDMI_STATUS_EN_MASK_UPDATE_RI);

	hdmi_reg_writemask(hdata, HDMI_HDCP_CTRL1, ~0,
			HDMI_HDCP_CTRL1_MASK_CP_DESIRED);

	/* Enable hdcp interrupts and activate content protection */
	hdmi_reg_writemask_atomic(hdata, HDMI_INTC_CON, ~0,
			HDMI_INTC_EN_GLOBAL | HDMI_INTC_EN_HDCP);


	hdcp_update_drm_property(hdata, DRM_MODE_CONTENT_PROTECTION_DESIRED);
}

static void hdcp_work_func(struct work_struct *work)
{
	struct hdmi_context *hdata;
	int ret;
	u32 irq;

	hdata = container_of(work, struct hdmi_context, hdcp_work);

	irq = hdmi_reg_read(hdata, HDMI_SYS_STATUS);
	hdmi_reg_writemask(hdata, HDMI_SYS_STATUS, ~0, irq);

	if (irq & HDMI_SYS_STATUS_MASK_ACTIVE_RX) {
		ret = hdcp_exchange_ksvs(hdata);
		hdmi_reg_writeb(hdata, HDMI_HDCP_I2C_INT, 0);
		if (ret)
			goto reset;
	}

	if (irq & HDMI_SYS_STATUS_MASK_AN_WRITE) {
		ret = hdcp_an_write(hdata);
		hdmi_reg_writeb(hdata, HDMI_HDCP_AN_INT, 0);
		if (ret)
			goto reset;
	}

	if (irq & HDMI_SYS_STATUS_MASK_UPDATE_RI) {
		ret = hdcp_update_ri(hdata);
		hdmi_reg_writeb(hdata, HDMI_HDCP_RI_INT, 0);
		if (ret)
			goto reset;
	}

	if (irq & HDMI_SYS_STATUS_MASK_WATCHDOG) {
		ret = hdcp_repeater_auth(hdata);
		hdmi_reg_writeb(hdata, HDMI_HDCP_WDT_INT, 0);
		if (ret)
			goto reset;
	}

	if (hdata->hdcp_state == HDCP_STATE_AUTHENTICATED) {
		mutex_lock(&hdata->drm_dev->mode_config.mutex);
		hdcp_update_drm_property(hdata,
				DRM_MODE_CONTENT_PROTECTION_ENABLED);
		mutex_unlock(&hdata->drm_dev->mode_config.mutex);

		hdata->hdcp_tries = 0;
	}

	/*
	 * We unmask here to prevent any races with the interrupt handler. That
	 * is, we don't want a situation where 2 HDCP interrupts occur without
	 * the worker running in between. It's also worth noting that we only
	 * need to unmask the HDCP interrupt if we were successful. If things
	 * fail, we restart HDCP which will enable things for us.
	 */
	hdmi_reg_writemask_atomic(hdata, HDMI_INTC_CON, ~0, HDMI_INTC_EN_HDCP);

	return;

reset:
	mutex_lock(&hdata->drm_dev->mode_config.mutex);
	hdcp_disable(hdata);
	mutex_unlock(&hdata->drm_dev->mode_config.mutex);

	if (hdata->hdcp_tries > HDCP_MAX_TRIES) {
		DRM_ERROR("Maximum HDCP tries exceeded, giving up\n");
		return;
	}

	msleep(20);

	mutex_lock(&hdata->drm_dev->mode_config.mutex);
	hdcp_enable(hdata);
	mutex_unlock(&hdata->drm_dev->mode_config.mutex);

	hdata->hdcp_tries++;
}

static int hdcp_load_efuse_key(struct hdmi_context *hdata)
{
	int ret;
	u32 status;

	hdmi_reg_writemask(hdata, HDCP_E_FUSE_CTRL, ~0,
			HDCP_E_FUSE_CTRL_MASK_READ_KEY);

	ret = wait_for(!((status = hdmi_reg_read(hdata, HDCP_E_FUSE_STATUS)) &
			HDCP_E_FUSE_STATUS_MASK_BUSY), LOAD_KEY_TIMEOUT_MSECS,
			1);
	if (ret) {
		DRM_ERROR("Load hdcp key from efuse failed with %d\n", ret);
		return ret;
	}

	if (status & HDCP_E_FUSE_STATUS_MASK_FAIL) {
		DRM_ERROR("Failed to load hdcp key from efuse\n");
		return -ENODEV;
	}

	WARN_ON(!(status & HDCP_E_FUSE_STATUS_MASK_DONE));
	return 0;

}

static void hdcp_stop(struct hdmi_context *hdata)
{
	if (hdata->hdcp_state == HDCP_STATE_OFF)
		return;

	hdmi_reg_writemask_atomic(hdata, HDMI_INTC_CON, 0, HDMI_INTC_EN_HDCP);
	cancel_work_sync(&hdata->hdcp_work);

	hdcp_disable(hdata);
}

static int hdcp_start(struct hdmi_context *hdata)
{
	int ret = 0;

	if (!hdata->hdcp_desired || hdata->hdcp_state != HDCP_STATE_OFF)
		return 0;

	ret = hdcp_load_efuse_key(hdata);
	if (ret) {
		DRM_ERROR("Could not load hdcp key from efuse! %d\n", ret);
		return ret;
	}

	hdata->hdcp_tries = 0;

	hdcp_enable(hdata);

	return 0;
}

static void hdcp_initialize(struct hdmi_context *hdata)
{
	hdata->hdcp_desired = false;
	hdata->hdcp_state = HDCP_STATE_OFF;
	INIT_WORK(&hdata->hdcp_work, hdcp_work_func);
}

static void hdmi_set_acr(u32 freq, u8 *acr)
{
	u32 n, cts;

	switch (freq) {
	case 32000:
		n = 4096;
		cts = 27000;
		break;
	case 44100:
		n = 6272;
		cts = 30000;
		break;
	case 88200:
		n = 12544;
		cts = 30000;
		break;
	case 176400:
		n = 25088;
		cts = 30000;
		break;
	case 48000:
		n = 6144;
		cts = 27000;
		break;
	case 96000:
		n = 12288;
		cts = 27000;
		break;
	case 192000:
		n = 24576;
		cts = 27000;
		break;
	default:
		n = 0;
		cts = 0;
		break;
	}

	acr[1] = cts >> 16;
	acr[2] = cts >> 8 & 0xff;
	acr[3] = cts & 0xff;

	acr[4] = n >> 16;
	acr[5] = n >> 8 & 0xff;
	acr[6] = n & 0xff;
}

static void hdmi_reg_acr(struct hdmi_context *hdata, u8 *acr)
{
	hdmi_reg_writeb(hdata, HDMI_ACR_N0, acr[6]);
	hdmi_reg_writeb(hdata, HDMI_ACR_N1, acr[5]);
	hdmi_reg_writeb(hdata, HDMI_ACR_N2, acr[4]);
	hdmi_reg_writeb(hdata, HDMI_ACR_MCTS0, acr[3]);
	hdmi_reg_writeb(hdata, HDMI_ACR_MCTS1, acr[2]);
	hdmi_reg_writeb(hdata, HDMI_ACR_MCTS2, acr[1]);
	hdmi_reg_writeb(hdata, HDMI_ACR_CTS0, acr[3]);
	hdmi_reg_writeb(hdata, HDMI_ACR_CTS1, acr[2]);
	hdmi_reg_writeb(hdata, HDMI_ACR_CTS2, acr[1]);

	if (hdata->version == HDMI_VER_EXYNOS4210)
		hdmi_reg_writeb(hdata, HDMI_4210_ACR_CON, 4);
	else
		hdmi_reg_writeb(hdata, HDMI_ACR_CON, 4);
}

static void hdmi_audio_init(struct hdmi_context *hdata)
{
	u32 sample_rate, bits_per_sample, frame_size_code;
	u32 data_num, bit_ch, sample_frq;
	u32 val;
	u8 acr[7];

	sample_rate = 44100;
	bits_per_sample = 16;
	frame_size_code = 0;

	switch (bits_per_sample) {
	case 20:
		data_num = 2;
		bit_ch  = 1;
		break;
	case 24:
		data_num = 3;
		bit_ch  = 1;
		break;
	default:
		data_num = 1;
		bit_ch  = 0;
		break;
	}

	hdmi_set_acr(sample_rate, acr);
	hdmi_reg_acr(hdata, acr);

	hdmi_reg_writeb(hdata, HDMI_I2S_MUX_CON, HDMI_I2S_IN_DISABLE
				| HDMI_I2S_AUD_I2S | HDMI_I2S_CUV_I2S_ENABLE
				| HDMI_I2S_MUX_ENABLE);

	hdmi_reg_writeb(hdata, HDMI_I2S_MUX_CH, HDMI_I2S_CH0_EN
			| HDMI_I2S_CH1_EN | HDMI_I2S_CH2_EN);

	hdmi_reg_writeb(hdata, HDMI_I2S_MUX_CUV, HDMI_I2S_CUV_RL_EN);

	sample_frq = (sample_rate == 44100) ? 0 :
			(sample_rate == 48000) ? 2 :
			(sample_rate == 32000) ? 3 :
			(sample_rate == 96000) ? 0xa : 0x0;

	hdmi_reg_writeb(hdata, HDMI_I2S_CLK_CON, HDMI_I2S_CLK_DIS);
	hdmi_reg_writeb(hdata, HDMI_I2S_CLK_CON, HDMI_I2S_CLK_EN);

	val = hdmi_reg_read(hdata, HDMI_I2S_DSD_CON) | 0x01;
	hdmi_reg_writeb(hdata, HDMI_I2S_DSD_CON, val);

	/* Configuration I2S input ports. Configure I2S_PIN_SEL_0~4 */
	hdmi_reg_writeb(hdata, HDMI_I2S_PIN_SEL_0, HDMI_I2S_SEL_SCLK(5)
			| HDMI_I2S_SEL_LRCK(6));
	hdmi_reg_writeb(hdata, HDMI_I2S_PIN_SEL_1, HDMI_I2S_SEL_SDATA1(1)
			| HDMI_I2S_SEL_SDATA2(4));
	hdmi_reg_writeb(hdata, HDMI_I2S_PIN_SEL_2, HDMI_I2S_SEL_SDATA3(1)
			| HDMI_I2S_SEL_SDATA2(2));
	hdmi_reg_writeb(hdata, HDMI_I2S_PIN_SEL_3, HDMI_I2S_SEL_DSD(0));

	/* I2S_CON_1 & 2 */
	hdmi_reg_writeb(hdata, HDMI_I2S_CON_1, HDMI_I2S_SCLK_FALLING_EDGE
			| HDMI_I2S_L_CH_LOW_POL);
	hdmi_reg_writeb(hdata, HDMI_I2S_CON_2, HDMI_I2S_MSB_FIRST_MODE
			| HDMI_I2S_SET_BIT_CH(bit_ch)
			| HDMI_I2S_SET_SDATA_BIT(data_num)
			| HDMI_I2S_BASIC_FORMAT);

	/* Configure register related to CUV information */
	hdmi_reg_writeb(hdata, HDMI_I2S_CH_ST_0, HDMI_I2S_CH_STATUS_MODE_0
			| HDMI_I2S_2AUD_CH_WITHOUT_PREEMPH
			| HDMI_I2S_COPYRIGHT
			| HDMI_I2S_LINEAR_PCM
			| HDMI_I2S_CONSUMER_FORMAT);
	hdmi_reg_writeb(hdata, HDMI_I2S_CH_ST_1, HDMI_I2S_CD_PLAYER);
	hdmi_reg_writeb(hdata, HDMI_I2S_CH_ST_2, HDMI_I2S_SET_SOURCE_NUM(0));
	hdmi_reg_writeb(hdata, HDMI_I2S_CH_ST_3, HDMI_I2S_CLK_ACCUR_LEVEL_2
			| HDMI_I2S_SET_SMP_FREQ(sample_frq));
	hdmi_reg_writeb(hdata, HDMI_I2S_CH_ST_4,
			HDMI_I2S_ORG_SMP_FREQ_44_1
			| HDMI_I2S_WORD_LEN_MAX24_24BITS
			| HDMI_I2S_WORD_LEN_MAX_24BITS);

	hdmi_reg_writeb(hdata, HDMI_I2S_CH_ST_CON, HDMI_I2S_CH_STATUS_RELOAD);
}

static void hdmi_audio_control(struct hdmi_context *hdata, bool onoff)
{
	if (hdata->dvi_mode)
		return;

	hdmi_reg_writeb(hdata, HDMI_AUI_CON, onoff ? 2 : 0);
	hdmi_reg_writemask(hdata, HDMI_CON_0, onoff ?
			HDMI_ASP_EN : HDMI_ASP_DIS, HDMI_ASP_MASK);
}

static void hdmi_start(struct hdmi_context *hdata, bool start)
{
	u32 val = start ? ~0 : 0;

	hdmi_reg_writemask(hdata, HDMI_CON_0, val, HDMI_EN);

	if (hdata->current_mode.flags & DRM_MODE_FLAG_INTERLACE)
		hdmi_reg_writemask(hdata, HDMI_TG_CMD, val, HDMI_TG_EN |
				HDMI_FIELD_EN);
	else
		hdmi_reg_writemask(hdata, HDMI_TG_CMD, val, HDMI_TG_EN);
}

static void hdmi_conf_init(struct hdmi_context *hdata)
{
	struct hdmi_infoframe infoframe;

	/* disable HPD interrupts from HDMI IP block, use GPIO instead */
	hdmi_reg_writemask_atomic(hdata, HDMI_INTC_CON, 0,
		HDMI_INTC_EN_HPD_PLUG | HDMI_INTC_EN_HPD_UNPLUG);

	/* Update the block's internal HPD status (necessary for HDCP) */
	hdmi_reg_writemask_atomic(hdata, HDMI_HPD, ~0, HDMI_HPD_HPD_SEL |
		HDMI_HPD_SW_HPD);

	/* choose HDMI mode */
	hdmi_reg_writemask(hdata, HDMI_MODE_SEL,
		HDMI_MODE_HDMI_EN, HDMI_MODE_MASK);
	/* disable bluescreen */
	hdmi_reg_writemask(hdata, HDMI_CON_0, 0, HDMI_BLUE_SCR_EN);

	if (hdata->dvi_mode) {
		/* choose DVI mode */
		hdmi_reg_writemask(hdata, HDMI_MODE_SEL,
				HDMI_MODE_DVI_EN, HDMI_MODE_MASK);
		hdmi_reg_writeb(hdata, HDMI_CON_2,
				HDMI_VID_PREAMBLE_DIS | HDMI_GUARD_BAND_DIS);
	}

	if (hdata->version == HDMI_VER_EXYNOS4210) {
		/* choose bluescreen (fecal) color */
		hdmi_reg_writeb(hdata, HDMI_4210_BLUE_SCREEN_0, 0x12);
		hdmi_reg_writeb(hdata, HDMI_4210_BLUE_SCREEN_1, 0x34);
		hdmi_reg_writeb(hdata, HDMI_4210_BLUE_SCREEN_2, 0x56);

		/* enable AVI packet every vsync, fixes purple line problem */
		hdmi_reg_writeb(hdata, HDMI_4210_AVI_CON, 0x02);
		/* force RGB, look to CEA-861-D, table 7 for more detail */
		hdmi_reg_writeb(hdata, HDMI_4210_AVI_BYTE(0), 0 << 5);
		hdmi_reg_writemask(hdata, HDMI_CON_1, 0x10 << 5, 0x11 << 5);

		hdmi_reg_writeb(hdata, HDMI_4210_SPD_CON, 0x02);
		hdmi_reg_writeb(hdata, HDMI_4210_AUI_CON, 0x02);
		hdmi_reg_writeb(hdata, HDMI_4210_ACR_CON, 0x04);
	} else {
		infoframe.type = HDMI_PACKET_TYPE_AVI;
		infoframe.ver = HDMI_AVI_VERSION;
		infoframe.len = HDMI_AVI_LENGTH;
		hdmi_reg_infoframe(hdata, &infoframe);

		infoframe.type = HDMI_PACKET_TYPE_AUI;
		infoframe.ver = HDMI_AUI_VERSION;
		infoframe.len = HDMI_AUI_LENGTH;
		hdmi_reg_infoframe(hdata, &infoframe);

		/* enable AVI packet every vsync, fixes purple line problem */
		hdmi_reg_writemask(hdata, HDMI_CON_1, 2, 3 << 5);
	}
}

static void hdmi_4210_mode_apply(struct hdmi_context *hdata)
{
	const struct hdmi_tg_regs *tg = &hdata->mode_conf.conf.v4210_conf.tg;
	const struct hdmi_4210_core_regs *core =
		&hdata->mode_conf.conf.v4210_conf.core;
	int tries;

	/* setting core registers */
	hdmi_reg_writeb(hdata, HDMI_H_BLANK_0, core->h_blank[0]);
	hdmi_reg_writeb(hdata, HDMI_H_BLANK_1, core->h_blank[1]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_BLANK_0, core->v_blank[0]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_BLANK_1, core->v_blank[1]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_BLANK_2, core->v_blank[2]);
	hdmi_reg_writeb(hdata, HDMI_4210_H_V_LINE_0, core->h_v_line[0]);
	hdmi_reg_writeb(hdata, HDMI_4210_H_V_LINE_1, core->h_v_line[1]);
	hdmi_reg_writeb(hdata, HDMI_4210_H_V_LINE_2, core->h_v_line[2]);
	hdmi_reg_writeb(hdata, HDMI_VSYNC_POL, core->vsync_pol[0]);
	hdmi_reg_writeb(hdata, HDMI_INT_PRO_MODE, core->int_pro_mode[0]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_BLANK_F_0, core->v_blank_f[0]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_BLANK_F_1, core->v_blank_f[1]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_BLANK_F_2, core->v_blank_f[2]);
	hdmi_reg_writeb(hdata, HDMI_4210_H_SYNC_GEN_0, core->h_sync_gen[0]);
	hdmi_reg_writeb(hdata, HDMI_4210_H_SYNC_GEN_1, core->h_sync_gen[1]);
	hdmi_reg_writeb(hdata, HDMI_4210_H_SYNC_GEN_2, core->h_sync_gen[2]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_SYNC_GEN_1_0, core->v_sync_gen1[0]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_SYNC_GEN_1_1, core->v_sync_gen1[1]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_SYNC_GEN_1_2, core->v_sync_gen1[2]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_SYNC_GEN_2_0, core->v_sync_gen2[0]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_SYNC_GEN_2_1, core->v_sync_gen2[1]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_SYNC_GEN_2_2, core->v_sync_gen2[2]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_SYNC_GEN_3_0, core->v_sync_gen3[0]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_SYNC_GEN_3_1, core->v_sync_gen3[1]);
	hdmi_reg_writeb(hdata, HDMI_4210_V_SYNC_GEN_3_2, core->v_sync_gen3[2]);
	/* Timing generator registers */
	hdmi_reg_writeb(hdata, HDMI_TG_H_FSZ_L, tg->h_fsz[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_H_FSZ_H, tg->h_fsz[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_HACT_ST_L, tg->hact_st[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_HACT_ST_H, tg->hact_st[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_HACT_SZ_L, tg->hact_sz[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_HACT_SZ_H, tg->hact_sz[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_V_FSZ_L, tg->v_fsz[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_V_FSZ_H, tg->v_fsz[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_L, tg->vsync[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_H, tg->vsync[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC2_L, tg->vsync2[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC2_H, tg->vsync2[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST_L, tg->vact_st[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST_H, tg->vact_st[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_SZ_L, tg->vact_sz[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_SZ_H, tg->vact_sz[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_CHG_L, tg->field_chg[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_CHG_H, tg->field_chg[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST2_L, tg->vact_st2[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST2_H, tg->vact_st2[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_TOP_HDMI_L, tg->vsync_top_hdmi[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_TOP_HDMI_H, tg->vsync_top_hdmi[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_BOT_HDMI_L, tg->vsync_bot_hdmi[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_BOT_HDMI_H, tg->vsync_bot_hdmi[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_TOP_HDMI_L, tg->field_top_hdmi[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_TOP_HDMI_H, tg->field_top_hdmi[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_BOT_HDMI_L, tg->field_bot_hdmi[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_BOT_HDMI_H, tg->field_bot_hdmi[1]);

	/* waiting for HDMIPHY's PLL to get to steady state */
	for (tries = 100; tries; --tries) {
		u32 val = hdmi_reg_read(hdata, HDMI_4210_PHY_STATUS);
		if (val & HDMI_PHY_STATUS_READY)
			break;
		usleep_range(1000, 2000);
	}
	/* steady state not achieved */
	if (tries == 0) {
		DRM_ERROR("hdmiphy's pll could not reach steady state.\n");
		hdmi_regs_dump(hdata, "timing apply");
	}

	clk_disable_unprepare(hdata->res.sclk_hdmi);
	clk_set_parent(hdata->res.mout_hdmi, hdata->res.sclk_hdmiphy);
	clk_prepare_enable(hdata->res.sclk_hdmi);

	hdmi_start(hdata, true);
}

static void hdmi_4212_mode_apply(struct hdmi_context *hdata)
{
	const struct hdmi_tg_regs *tg = &hdata->mode_conf.conf.v4212_conf.tg;
	const struct hdmi_4212_core_regs *core =
		&hdata->mode_conf.conf.v4212_conf.core;
	int tries;

	/* setting core registers */
	hdmi_reg_writeb(hdata, HDMI_H_BLANK_0, core->h_blank[0]);
	hdmi_reg_writeb(hdata, HDMI_H_BLANK_1, core->h_blank[1]);
	hdmi_reg_writeb(hdata, HDMI_V2_BLANK_0, core->v2_blank[0]);
	hdmi_reg_writeb(hdata, HDMI_V2_BLANK_1, core->v2_blank[1]);
	hdmi_reg_writeb(hdata, HDMI_V1_BLANK_0, core->v1_blank[0]);
	hdmi_reg_writeb(hdata, HDMI_V1_BLANK_1, core->v1_blank[1]);
	hdmi_reg_writeb(hdata, HDMI_V_LINE_0, core->v_line[0]);
	hdmi_reg_writeb(hdata, HDMI_V_LINE_1, core->v_line[1]);
	hdmi_reg_writeb(hdata, HDMI_H_LINE_0, core->h_line[0]);
	hdmi_reg_writeb(hdata, HDMI_H_LINE_1, core->h_line[1]);
	hdmi_reg_writeb(hdata, HDMI_HSYNC_POL, core->hsync_pol[0]);
	hdmi_reg_writeb(hdata, HDMI_VSYNC_POL, core->vsync_pol[0]);
	hdmi_reg_writeb(hdata, HDMI_INT_PRO_MODE, core->int_pro_mode[0]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F0_0, core->v_blank_f0[0]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F0_1, core->v_blank_f0[1]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F1_0, core->v_blank_f1[0]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F1_1, core->v_blank_f1[1]);
	hdmi_reg_writeb(hdata, HDMI_H_SYNC_START_0, core->h_sync_start[0]);
	hdmi_reg_writeb(hdata, HDMI_H_SYNC_START_1, core->h_sync_start[1]);
	hdmi_reg_writeb(hdata, HDMI_H_SYNC_END_0, core->h_sync_end[0]);
	hdmi_reg_writeb(hdata, HDMI_H_SYNC_END_1, core->h_sync_end[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_BEF_2_0,
			core->v_sync_line_bef_2[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_BEF_2_1,
			core->v_sync_line_bef_2[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_BEF_1_0,
			core->v_sync_line_bef_1[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_BEF_1_1,
			core->v_sync_line_bef_1[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_2_0,
			core->v_sync_line_aft_2[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_2_1,
			core->v_sync_line_aft_2[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_1_0,
			core->v_sync_line_aft_1[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_1_1,
			core->v_sync_line_aft_1[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_2_0,
			core->v_sync_line_aft_pxl_2[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_2_1,
			core->v_sync_line_aft_pxl_2[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_1_0,
			core->v_sync_line_aft_pxl_1[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_1_1,
			core->v_sync_line_aft_pxl_1[1]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F2_0, core->v_blank_f2[0]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F2_1, core->v_blank_f2[1]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F3_0, core->v_blank_f3[0]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F3_1, core->v_blank_f3[1]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F4_0, core->v_blank_f4[0]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F4_1, core->v_blank_f4[1]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F5_0, core->v_blank_f5[0]);
	hdmi_reg_writeb(hdata, HDMI_V_BLANK_F5_1, core->v_blank_f5[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_3_0,
			core->v_sync_line_aft_3[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_3_1,
			core->v_sync_line_aft_3[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_4_0,
			core->v_sync_line_aft_4[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_4_1,
			core->v_sync_line_aft_4[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_5_0,
			core->v_sync_line_aft_5[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_5_1,
			core->v_sync_line_aft_5[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_6_0,
			core->v_sync_line_aft_6[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_6_1,
			core->v_sync_line_aft_6[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_3_0,
			core->v_sync_line_aft_pxl_3[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_3_1,
			core->v_sync_line_aft_pxl_3[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_4_0,
			core->v_sync_line_aft_pxl_4[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_4_1,
			core->v_sync_line_aft_pxl_4[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_5_0,
			core->v_sync_line_aft_pxl_5[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_5_1,
			core->v_sync_line_aft_pxl_5[1]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_6_0,
			core->v_sync_line_aft_pxl_6[0]);
	hdmi_reg_writeb(hdata, HDMI_V_SYNC_LINE_AFT_PXL_6_1,
			core->v_sync_line_aft_pxl_6[1]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_1_0, core->vact_space_1[0]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_1_1, core->vact_space_1[1]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_2_0, core->vact_space_2[0]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_2_1, core->vact_space_2[1]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_3_0, core->vact_space_3[0]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_3_1, core->vact_space_3[1]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_4_0, core->vact_space_4[0]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_4_1, core->vact_space_4[1]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_5_0, core->vact_space_5[0]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_5_1, core->vact_space_5[1]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_6_0, core->vact_space_6[0]);
	hdmi_reg_writeb(hdata, HDMI_VACT_SPACE_6_1, core->vact_space_6[1]);

	/* Timing generator registers */
	hdmi_reg_writeb(hdata, HDMI_TG_H_FSZ_L, tg->h_fsz[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_H_FSZ_H, tg->h_fsz[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_HACT_ST_L, tg->hact_st[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_HACT_ST_H, tg->hact_st[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_HACT_SZ_L, tg->hact_sz[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_HACT_SZ_H, tg->hact_sz[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_V_FSZ_L, tg->v_fsz[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_V_FSZ_H, tg->v_fsz[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_L, tg->vsync[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_H, tg->vsync[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC2_L, tg->vsync2[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC2_H, tg->vsync2[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST_L, tg->vact_st[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST_H, tg->vact_st[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_SZ_L, tg->vact_sz[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_SZ_H, tg->vact_sz[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_CHG_L, tg->field_chg[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_CHG_H, tg->field_chg[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST2_L, tg->vact_st2[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST2_H, tg->vact_st2[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST3_L, tg->vact_st3[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST3_H, tg->vact_st3[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST4_L, tg->vact_st4[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VACT_ST4_H, tg->vact_st4[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_TOP_HDMI_L, tg->vsync_top_hdmi[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_TOP_HDMI_H, tg->vsync_top_hdmi[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_BOT_HDMI_L, tg->vsync_bot_hdmi[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_VSYNC_BOT_HDMI_H, tg->vsync_bot_hdmi[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_TOP_HDMI_L, tg->field_top_hdmi[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_TOP_HDMI_H, tg->field_top_hdmi[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_BOT_HDMI_L, tg->field_bot_hdmi[0]);
	hdmi_reg_writeb(hdata, HDMI_TG_FIELD_BOT_HDMI_H, tg->field_bot_hdmi[1]);
	hdmi_reg_writeb(hdata, HDMI_TG_3D, tg->tg_3d[0]);

	/* waiting for HDMIPHY's PLL to get to steady state */
	for (tries = 100; tries; --tries) {
		u32 val = hdmi_reg_read(hdata, HDMI_PHY_STATUS_0);
		if (val & HDMI_PHY_STATUS_READY)
			break;
		usleep_range(1000, 2000);
	}
	/* steady state not achieved */
	if (tries == 0) {
		DRM_ERROR("hdmiphy's pll could not reach steady state.\n");
		hdmi_regs_dump(hdata, "timing apply");
	}

	clk_disable_unprepare(hdata->res.sclk_hdmi);
	clk_set_parent(hdata->res.mout_hdmi, hdata->res.sclk_hdmiphy);
	clk_prepare_enable(hdata->res.sclk_hdmi);

	hdmi_start(hdata, true);
}

static void hdmi_mode_apply(struct hdmi_context *hdata)
{
	if (hdata->version == HDMI_VER_EXYNOS4210)
		hdmi_4210_mode_apply(hdata);
	else
		hdmi_4212_mode_apply(hdata);
}

static void hdmiphy_conf_reset(struct hdmi_context *hdata)
{
	u32 reg;

	clk_disable_unprepare(hdata->res.sclk_hdmi);
	clk_set_parent(hdata->res.mout_hdmi, hdata->res.sclk_pixel);
	clk_prepare_enable(hdata->res.sclk_hdmi);

	hdmiphy_reg_writeb(hdata, HDMIPHY_MODE_SET_DONE,
						HDMI_PHY_ENABLE_MODE_SET);

	if (hdata->version == HDMI_VER_EXYNOS4210)
		reg = HDMI_4210_PHY_RSTOUT;
	else
		reg = HDMI_PHY_RSTOUT;

	/* reset hdmiphy */
	hdmi_reg_writemask(hdata, reg, ~0, HDMI_PHY_SW_RSTOUT);
	usleep_range(10000, 12000);
	hdmi_reg_writemask(hdata, reg,  0, HDMI_PHY_SW_RSTOUT);
	usleep_range(10000, 12000);
}

static void hdmiphy_poweron(struct hdmi_context *hdata)
{
	if (hdata->version == HDMI_VER_EXYNOS4210)
		return;

	DRM_DEBUG_KMS("\n");

	/* For PHY Mode Setting */
	hdmiphy_reg_writeb(hdata, HDMIPHY_MODE_SET_DONE,
						HDMI_PHY_ENABLE_MODE_SET);
	/* Phy Power On */
	hdmiphy_reg_writeb(hdata, HDMIPHY_POWER,
						HDMI_PHY_POWER_ON);
	/* For PHY Mode Setting */
	hdmiphy_reg_writeb(hdata, HDMIPHY_MODE_SET_DONE,
						HDMI_PHY_DISABLE_MODE_SET);
	/* PHY SW Reset */
	hdmiphy_conf_reset(hdata);
}

static void hdmiphy_poweroff(struct hdmi_context *hdata)
{
	if (hdata->version == HDMI_VER_EXYNOS4210)
		return;

	DRM_DEBUG_KMS("\n");

	/* PHY SW Reset */
	hdmiphy_conf_reset(hdata);
	/* For PHY Mode Setting */
	hdmiphy_reg_writeb(hdata, HDMIPHY_MODE_SET_DONE,
						HDMI_PHY_ENABLE_MODE_SET);
	/* PHY Power Off */
	hdmiphy_reg_writeb(hdata, HDMIPHY_POWER,
						HDMI_PHY_POWER_OFF);
	/* For PHY Mode Setting */
	hdmiphy_reg_writeb(hdata, HDMIPHY_MODE_SET_DONE,
						HDMI_PHY_DISABLE_MODE_SET);
}

static void hdmiphy_conf_apply(struct hdmi_context *hdata)
{
	const u8 *hdmiphy_data;
	int ret;
	int i;

	DRM_DEBUG_KMS("\n");

	/* pixel clock */
	i = hdmi_find_phy_conf(hdata, hdata->mode_conf.pixel_clock);
	if (i < 0) {
		DRM_ERROR("failed to find hdmiphy conf\n");
		return;
	}

	if (hdata->version == HDMI_VER_EXYNOS4210) {
		hdmiphy_data = hdmiphy_4210_configs[i].conf;
	} else if (hdata->version == HDMI_VER_EXYNOS4212) {
		hdmiphy_data = hdmiphy_4212_configs[i].conf;
	} else if (hdata->version == HDMI_VER_EXYNOS5420) {
		hdmiphy_data = hdmiphy_5420_configs[i].conf;
	} else {
		DRM_ERROR("invalid hdmi version.\n");
		return;
	}

	ret = hdmiphy_reg_write_buf(hdata, 0, hdmiphy_data, 32);
	if (ret) {
		DRM_ERROR("failed to configure HDMIPHY\n");
		return;
	}

	usleep_range(10000, 12000);

	ret = hdmiphy_reg_writeb(hdata, HDMIPHY_MODE_SET_DONE,
						HDMI_PHY_DISABLE_MODE_SET);
	if (ret < 0) {
		DRM_ERROR("failed to enable hdmiphy\n");
		return;
	}
}

static void hdmi_conf_apply(struct hdmi_context *hdata)
{
	DRM_DEBUG_KMS("\n");

	hdmiphy_conf_reset(hdata);
	hdmiphy_conf_apply(hdata);

	hdmi_start(hdata, false);
	hdmi_conf_init(hdata);
	if (!support_hdmi_audio_through_alsa(hdata))
		hdmi_audio_init(hdata);

	/* setting core registers */
	hdmi_mode_apply(hdata);
	if (!support_hdmi_audio_through_alsa(hdata))
		hdmi_audio_control(hdata, true);

	hdcp_stop(hdata);
	hdcp_start(hdata);

	hdmi_regs_dump(hdata, "start");
}

static void hdmi_set_reg(u8 *reg_pair, int num_bytes, u32 value)
{
	int i;
	BUG_ON(num_bytes > 4);
	for (i = 0; i < num_bytes; i++)
		reg_pair[i] = (value >> (8 * i)) & 0xff;
}

static void hdmi_4210_mode_set(struct hdmi_context *hdata,
			struct drm_display_mode *m)
{
	struct hdmi_4210_core_regs *core =
		&hdata->mode_conf.conf.v4210_conf.core;
	struct hdmi_tg_regs *tg = &hdata->mode_conf.conf.v4210_conf.tg;
	unsigned int val;

	DRM_DEBUG_KMS("[MODE:%s]\n", m->name);

	hdata->mode_conf.cea_video_id =
		drm_match_cea_mode((struct drm_display_mode *)m);
	hdata->mode_conf.pixel_clock = m->clock * 1000;

	hdmi_set_reg(core->h_blank, 2, m->htotal - m->hdisplay);
	hdmi_set_reg(core->h_v_line, 3, (m->htotal << 12) | m->vtotal);

	val = (m->flags & DRM_MODE_FLAG_NVSYNC) ? 1 : 0;
	hdmi_set_reg(core->vsync_pol, 1, val);

	val = (m->flags & DRM_MODE_FLAG_INTERLACE) ? 1 : 0;
	hdmi_set_reg(core->int_pro_mode, 1, val);

	val = (m->hsync_start - m->hdisplay - 2);
	val |= ((m->hsync_end - m->hdisplay - 2) << 10);
	val |= ((m->flags & DRM_MODE_FLAG_NHSYNC)  ? 1 : 0)<<20;
	hdmi_set_reg(core->h_sync_gen, 3, val);

	/*
	 * Quirk requirement for exynos HDMI IP design,
	 * 2 pixels less than the actual calculation for hsync_start
	 * and end.
	 */

	/* Following values & calculations differ for different type of modes */
	if (m->flags & DRM_MODE_FLAG_INTERLACE) {
		/* Interlaced Mode */
		val = ((m->vsync_end - m->vdisplay) / 2);
		val |= ((m->vsync_start - m->vdisplay) / 2) << 12;
		hdmi_set_reg(core->v_sync_gen1, 3, val);

		val = m->vtotal / 2;
		val |= ((m->vtotal - m->vdisplay) / 2) << 11;
		hdmi_set_reg(core->v_blank, 3, val);

		val = (m->vtotal +
			((m->vsync_end - m->vsync_start) * 4) + 5) / 2;
		val |= m->vtotal << 11;
		hdmi_set_reg(core->v_blank_f, 3, val);

		val = ((m->vtotal / 2) + 7);
		val |= ((m->vtotal / 2) + 2) << 12;
		hdmi_set_reg(core->v_sync_gen2, 3, val);

		val = ((m->htotal / 2) + (m->hsync_start - m->hdisplay));
		val |= ((m->htotal / 2) +
			(m->hsync_start - m->hdisplay)) << 12;
		hdmi_set_reg(core->v_sync_gen3, 3, val);

		hdmi_set_reg(tg->vact_st, 2, (m->vtotal - m->vdisplay) / 2);
		hdmi_set_reg(tg->vact_sz, 2, m->vdisplay / 2);

		hdmi_set_reg(tg->vact_st2, 2, 0x249);/* Reset value + 1*/
	} else {
		/* Progressive Mode */

		val = m->vtotal;
		val |= (m->vtotal - m->vdisplay) << 11;
		hdmi_set_reg(core->v_blank, 3, val);

		hdmi_set_reg(core->v_blank_f, 3, 0);

		val = (m->vsync_end - m->vdisplay);
		val |= ((m->vsync_start - m->vdisplay) << 12);
		hdmi_set_reg(core->v_sync_gen1, 3, val);

		hdmi_set_reg(core->v_sync_gen2, 3, 0x1001);/* Reset value  */
		hdmi_set_reg(core->v_sync_gen3, 3, 0x1001);/* Reset value  */
		hdmi_set_reg(tg->vact_st, 2, m->vtotal - m->vdisplay);
		hdmi_set_reg(tg->vact_sz, 2, m->vdisplay);
		hdmi_set_reg(tg->vact_st2, 2, 0x248); /* Reset value */
	}

	/* Timing generator registers */
	hdmi_set_reg(tg->cmd, 1, 0x0);
	hdmi_set_reg(tg->h_fsz, 2, m->htotal);
	hdmi_set_reg(tg->hact_st, 2, m->htotal - m->hdisplay);
	hdmi_set_reg(tg->hact_sz, 2, m->hdisplay);
	hdmi_set_reg(tg->v_fsz, 2, m->vtotal);
	hdmi_set_reg(tg->vsync, 2, 0x1);
	hdmi_set_reg(tg->vsync2, 2, 0x233); /* Reset value */
	hdmi_set_reg(tg->field_chg, 2, 0x233); /* Reset value */
	hdmi_set_reg(tg->vsync_top_hdmi, 2, 0x1); /* Reset value */
	hdmi_set_reg(tg->vsync_bot_hdmi, 2, 0x233); /* Reset value */
	hdmi_set_reg(tg->field_top_hdmi, 2, 0x1); /* Reset value */
	hdmi_set_reg(tg->field_bot_hdmi, 2, 0x233); /* Reset value */
	hdmi_set_reg(tg->tg_3d, 1, 0x0); /* Not used */
}

static void hdmi_4212_mode_set(struct hdmi_context *hdata,
			struct drm_display_mode *m)
{
	struct hdmi_tg_regs *tg = &hdata->mode_conf.conf.v4212_conf.tg;
	struct hdmi_4212_core_regs *core =
		&hdata->mode_conf.conf.v4212_conf.core;
	u32 hact_offset = 0;

	DRM_DEBUG_KMS("[MODE:%s]\n", m->name);

	hdata->mode_conf.cea_video_id =
		drm_match_cea_mode((struct drm_display_mode *)m);
	hdata->mode_conf.pixel_clock = m->clock * 1000;

	hdmi_set_reg(core->h_blank, 2, m->crtc_htotal - m->crtc_hdisplay);
	hdmi_set_reg(core->h_line, 2, m->crtc_htotal);
	hdmi_set_reg(core->hsync_pol, 1,
			(m->flags & DRM_MODE_FLAG_NHSYNC)  ? 1 : 0);
	hdmi_set_reg(core->vsync_pol, 1,
			(m->flags & DRM_MODE_FLAG_NVSYNC) ? 1 : 0);
	hdmi_set_reg(core->int_pro_mode, 1,
			(m->flags & DRM_MODE_FLAG_INTERLACE) ? 1 : 0);
	hdmi_set_reg(core->v_sync_line_bef_2, 2, m->crtc_vsync_end -
			m->crtc_vdisplay);
	hdmi_set_reg(core->v_sync_line_bef_1, 2, m->crtc_vsync_start -
			m->crtc_vdisplay);
	hdmi_set_reg(core->v2_blank, 2, m->crtc_vtotal);
	hdmi_set_reg(core->v1_blank, 2, m->crtc_vtotal - m->crtc_vdisplay);

	/*
	 * Quirk requirement for exynos 5 HDMI IP design,
	 * 2 pixels less than the actual calculation for hsync_start
	 * and end.
	 */

	/* Following values & calculations differ for different type of modes */
	if (m->flags & DRM_MODE_FLAG_INTERLACE) {
		/* Interlaced Mode */
		hdmi_set_reg(core->v_line, 2, m->crtc_vtotal * 2 + 1);
		hdmi_set_reg(core->v_blank_f0, 2, m->crtc_vtotal * 2 -
			m->crtc_vdisplay + 1);
		hdmi_set_reg(core->v_blank_f1, 2, m->crtc_vtotal * 2 + 1);
		hdmi_set_reg(core->v_sync_line_aft_2, 2, m->crtc_vtotal + 7);
		hdmi_set_reg(core->v_sync_line_aft_1, 2, m->crtc_vtotal + 2);
		hdmi_set_reg(core->v_sync_line_aft_pxl_2, 2,
			(m->crtc_htotal / 2) +
			(m->crtc_hsync_start - m->crtc_hdisplay));
		hdmi_set_reg(core->v_sync_line_aft_pxl_1, 2,
			(m->crtc_htotal / 2) +
			(m->crtc_hsync_start - m->crtc_hdisplay));
		hdmi_set_reg(tg->vact_st2, 2, m->crtc_vtotal * 2 -
			m->crtc_vdisplay + 1);
		hdmi_set_reg(tg->vsync2, 2, m->crtc_vtotal +
			(m->crtc_vsync_end - m->crtc_vsync_start) + 1);
		hdmi_set_reg(tg->vsync_bot_hdmi, 2, m->crtc_vtotal +
			(m->crtc_vsync_end - m->crtc_vsync_start) + 1);
		hdmi_set_reg(tg->field_bot_hdmi, 2, m->crtc_vtotal +
			(m->crtc_vsync_end - m->crtc_vsync_start) + 1);
		hdmi_set_reg(tg->v_fsz, 2, m->crtc_vtotal * 2 + 1);
		hdmi_set_reg(tg->vact_st3, 2, 0x0);
		hdmi_set_reg(tg->vact_st4, 2, 0x0);
	} else {
		/* Progressive Mode */
		hdmi_set_reg(core->v_line, 2, m->crtc_vtotal);
		hdmi_set_reg(core->v_blank_f0, 2, 0xffff);
		hdmi_set_reg(core->v_blank_f1, 2, 0xffff);
		hdmi_set_reg(core->v_sync_line_aft_2, 2, 0xffff);
		hdmi_set_reg(core->v_sync_line_aft_1, 2, 0xffff);
		hdmi_set_reg(core->v_sync_line_aft_pxl_2, 2, 0xffff);
		hdmi_set_reg(core->v_sync_line_aft_pxl_1, 2, 0xffff);
		hdmi_set_reg(tg->v_fsz, 2, m->crtc_vtotal);
		hdmi_set_reg(tg->vact_st2, 2, 0x248); /* Reset value */
		hdmi_set_reg(tg->vact_st3, 2, 0x47b); /* Reset value */
		hdmi_set_reg(tg->vact_st4, 2, 0x6ae); /* Reset value */
		hdmi_set_reg(tg->vsync2, 2, 0x233); /* Reset value */
		hdmi_set_reg(tg->vsync_bot_hdmi, 2, 0x233); /* Reset value */
		hdmi_set_reg(tg->field_bot_hdmi, 2, 0x233); /* Reset value */
	}

	/* Following values & calculations are same irrespective of mode type */
	hdmi_set_reg(core->h_sync_start, 2,
		m->crtc_hsync_start - m->crtc_hdisplay - 2);
	hdmi_set_reg(core->h_sync_end, 2,
		m->crtc_hsync_end - m->crtc_hdisplay - 2);
	hdmi_set_reg(core->vact_space_1, 2, 0xffff);
	hdmi_set_reg(tg->vact_st, 2, m->crtc_vtotal - m->crtc_vdisplay);
	hdmi_set_reg(tg->vact_sz, 2, m->crtc_vdisplay);
	hdmi_set_reg(core->vact_space_2, 2, 0xffff);
	hdmi_set_reg(core->vact_space_3, 2, 0xffff);
	hdmi_set_reg(core->vact_space_4, 2, 0xffff);
	hdmi_set_reg(core->vact_space_5, 2, 0xffff);
	hdmi_set_reg(core->vact_space_6, 2, 0xffff);
	hdmi_set_reg(core->v_blank_f2, 2, 0xffff);
	hdmi_set_reg(core->v_blank_f3, 2, 0xffff);
	hdmi_set_reg(core->v_blank_f4, 2, 0xffff);
	hdmi_set_reg(core->v_blank_f5, 2, 0xffff);
	hdmi_set_reg(core->v_sync_line_aft_3, 2, 0xffff);
	hdmi_set_reg(core->v_sync_line_aft_4, 2, 0xffff);
	hdmi_set_reg(core->v_sync_line_aft_5, 2, 0xffff);
	hdmi_set_reg(core->v_sync_line_aft_6, 2, 0xffff);
	hdmi_set_reg(core->v_sync_line_aft_pxl_3, 2, 0xffff);
	hdmi_set_reg(core->v_sync_line_aft_pxl_4, 2, 0xffff);
	hdmi_set_reg(core->v_sync_line_aft_pxl_5, 2, 0xffff);
	hdmi_set_reg(core->v_sync_line_aft_pxl_6, 2, 0xffff);

	/*
	 * The mixer supports some additional resolutions by shifting stuff
	 * horizontally in 3d mode (<-- simplified explanation). We need to
	 * mirror that here, so ask it for the offset.
	 */
	if (hdata->version == HDMI_VER_EXYNOS4212)
		hact_offset = mixer_get_horizontal_offset(m->crtc_hdisplay,
				m->crtc_vdisplay);

	/* Timing generator registers */
	hdmi_set_reg(tg->cmd, 1, 0x0);
	hdmi_set_reg(tg->h_fsz, 2, m->crtc_htotal);
	hdmi_set_reg(tg->hact_st, 2, m->crtc_htotal - m->crtc_hdisplay -
			hact_offset);
	hdmi_set_reg(tg->hact_sz, 2, m->crtc_hdisplay + hact_offset);
	hdmi_set_reg(tg->vsync, 2, 0x1);
	hdmi_set_reg(tg->field_chg, 2, 0x233); /* Reset value */
	hdmi_set_reg(tg->vsync_top_hdmi, 2, 0x1); /* Reset value */
	hdmi_set_reg(tg->field_top_hdmi, 2, 0x1); /* Reset value */
	hdmi_set_reg(tg->tg_3d, 1, 0x0);
}

static void hdmi_mode_set(void *ctx, struct drm_display_mode *mode)
{
	struct hdmi_context *hdata = ctx;

	DRM_DEBUG_KMS("[MODE:%s]\n", mode->name);

	/* preserve mode information for later use. */
	drm_mode_copy(&hdata->current_mode, mode);

	if (hdata->version == HDMI_VER_EXYNOS4210)
		hdmi_4210_mode_set(hdata, mode);
	else
		hdmi_4212_mode_set(hdata, mode);
}

static void hdmi_get_max_resol(void *ctx, unsigned int *width,
					unsigned int *height)
{
	*width = MAX_WIDTH;
	*height = MAX_HEIGHT;

	DRM_DEBUG_KMS("%ux%u\n", *width, *height);
}

static void hdmi_commit(void *ctx)
{
	struct hdmi_context *hdata = ctx;

	if (!hdata->powered)
		return;

	DRM_DEBUG_KMS("\n");

	hdmi_conf_apply(hdata);
}

static int hdmi_set_property(void *ctx, struct drm_property *property,
			uint64_t val)
{
	struct hdmi_context *hdata = ctx;
	struct drm_mode_config *mode_config = &hdata->drm_dev->mode_config;
	int ret = 0;

	WARN_ON(!mutex_is_locked(&mode_config->mutex));

	if (property != mode_config->content_protection_property)
		return 0;

	DRM_DEBUG_KMS("[PROPERTY:%s] = %llu\n", property->name, val);

	hdata->hdcp_desired = val;

	if (!hdata->powered)
		return 0;

	if (hdata->hdcp_desired)
		ret = hdcp_start(hdata);
	else
		hdcp_stop(hdata);

	return ret;
}

static void hdmi_poweron(struct hdmi_context *hdata)
{
	struct hdmi_resources *res = &hdata->res;

	if (hdata->powered)
		return;

	DRM_DEBUG_KMS("\n");

	regulator_bulk_enable(res->regul_count, res->regul_bulk);
	/* HDMI PHY Enable */
	hdmi_phy_pow_ctrl_reg_writemask(hdata, PMU_HDMI_PHY_ENABLE,
		PMU_HDMI_PHY_CONTROL_MASK);

	/* HDMI PHY Enable and Power-On */
	hdmiphy_poweron(hdata);

	hdata->powered = true;
	/*
	 * In hdmi poweroff sequence we have cleared
	 * all the hdmi configuration.
	 * For the next poweron we need to restore the
	 * lost configuration.
	 */
	hdmi_commit(hdata);
}

static void hdmi_poweroff(struct hdmi_context *hdata)
{
	struct hdmi_resources *res = &hdata->res;

	if (!hdata->powered)
		return;

	DRM_DEBUG_KMS("\n");

	hdcp_stop(hdata);

	/* HDMI System Disable */
	hdmi_reg_writemask(hdata, HDMI_CON_0, 0, HDMI_EN);

	hdmiphy_poweroff(hdata);

	cancel_delayed_work(&hdata->hotplug_work);

	/* HDMI PHY Disable */
	hdmi_phy_pow_ctrl_reg_writemask(hdata, PMU_HDMI_PHY_DISABLE,
		PMU_HDMI_PHY_CONTROL_MASK);

	regulator_bulk_disable(res->regul_count, res->regul_bulk);
	hdata->powered = false;
}

static void hdmi_dpms(void *ctx, int mode)
{
	struct hdmi_context *hdata = ctx;

	DRM_DEBUG_KMS("[DPMS:%s]\n", drm_get_dpms_name(mode));

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		hdmi_poweron(hdata);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		hdmi_poweroff(hdata);
		break;
	default:
		DRM_DEBUG_KMS("unknown dpms mode: %d\n", mode);
		break;
	}
}

static struct exynos_drm_display_ops hdmi_display_ops = {
	.initialize	= hdmi_initialize,
	.is_connected	= hdmi_is_connected,
	.get_max_resol	= hdmi_get_max_resol,
	.get_edid	= hdmi_get_edid,
	.check_mode	= hdmi_check_mode,
	.mode_fixup	= hdmi_mode_fixup,
	.mode_set	= hdmi_mode_set,
	.dpms		= hdmi_dpms,
	.commit		= hdmi_commit,
	.set_property	= hdmi_set_property,
};

static struct exynos_drm_display hdmi_display = {
	.type = EXYNOS_DISPLAY_TYPE_HDMI,
	.ops = &hdmi_display_ops,
};

static void hdmi_hotplug_work_func(struct work_struct *work)
{
	struct hdmi_context *hdata;

	hdata = container_of(work, struct hdmi_context, hotplug_work.work);

	hdata->hpd = gpio_get_value(hdata->hpd_gpio);

	DRM_DEBUG_KMS("hpd: %d\n", hdata->hpd);

	/* Update the block's internal HPD status (necessary for HDCP) */
	if (!hdata->hpd)
		hdmi_reg_writemask_atomic(hdata, HDMI_HPD, 0, HDMI_HPD_HPD_SEL);

	if (hdata->drm_dev)
		drm_helper_hpd_irq_event(hdata->drm_dev);
}

static irqreturn_t hdmi_irq_thread(int irq, void *arg)
{
	struct hdmi_context *hdata = arg;

	mod_delayed_work(system_wq, &hdata->hotplug_work,
				msecs_to_jiffies(HOTPLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static irqreturn_t hdcp_irq_thread(int irq, void *arg)
{
	struct hdmi_context *hdata = arg;
	u32 intc_flag;

	intc_flag = hdmi_reg_read(hdata, HDMI_INTC_FLAG);
	if (!(intc_flag & HDMI_INTC_FLAG_HDCP)) {
		DRM_INFO("Received unknown interrupt 0x%x\n", intc_flag);
		hdmi_reg_writemask(hdata, HDMI_INTC_FLAG, ~0, intc_flag);
		return IRQ_HANDLED;
	}

	/*
	 * Unlike the hotplug interrupts, the HDCP interrupt is
	 * not acked through the FLAG register. This means that
	 * we'll keep getting these suckers unless we mask it
	 * off manually.
	 */
	hdmi_reg_writemask_atomic(hdata, HDMI_INTC_CON, 0, HDMI_INTC_EN_HDCP);

	schedule_work(&hdata->hdcp_work);

	return IRQ_HANDLED;
}

static int hdmi_resources_init(struct hdmi_context *hdata)
{
	struct device *dev = hdata->dev;
	struct hdmi_resources *res = &hdata->res;
	static char *supply[] = {
		"hdmi-en",
		"vdd",
		"vdd_osc",
		"vdd_pll",
	};
	int i, ret;

	DRM_DEBUG_KMS("HDMI resource init\n");

	/* get clocks, power */
	res->hdmi = devm_clk_get(dev, "hdmi");
	if (IS_ERR_OR_NULL(res->hdmi)) {
		DRM_ERROR("failed to get clock 'hdmi'\n");
		goto fail;
	}
	res->sclk_hdmi = devm_clk_get(dev, "sclk_hdmi");
	if (IS_ERR_OR_NULL(res->sclk_hdmi)) {
		DRM_ERROR("failed to get clock 'sclk_hdmi'\n");
		goto fail;
	}
	res->sclk_pixel = devm_clk_get(dev, "sclk_pixel");
	if (IS_ERR_OR_NULL(res->sclk_pixel)) {
		DRM_ERROR("failed to get clock 'sclk_pixel'\n");
		goto fail;
	}
	res->sclk_hdmiphy = devm_clk_get(dev, "sclk_hdmiphy");
	if (IS_ERR_OR_NULL(res->sclk_hdmiphy)) {
		DRM_ERROR("failed to get clock 'sclk_hdmiphy'\n");
		goto fail;
	}
	res->mout_hdmi = devm_clk_get(dev, "mout_hdmi");
	if (IS_ERR(res->mout_hdmi)) {
		DRM_ERROR("failed to get clock 'mout_hdmi'\n");
		goto fail;
	}

	clk_set_parent(res->mout_hdmi, res->sclk_pixel);

	res->regul_bulk = devm_kzalloc(dev, ARRAY_SIZE(supply) *
		sizeof(res->regul_bulk[0]), GFP_KERNEL);
	if (!res->regul_bulk) {
		DRM_ERROR("failed to get memory for regulators\n");
		goto fail;
	}
	for (i = 0; i < ARRAY_SIZE(supply); ++i) {
		res->regul_bulk[i].supply = supply[i];
		res->regul_bulk[i].consumer = NULL;
	}
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(supply), res->regul_bulk);
	if (ret) {
		DRM_ERROR("failed to get regulators\n");
		goto fail;
	}
	res->regul_count = ARRAY_SIZE(supply);

	/*
	 * These two clocks are not moved into hdmi_poweron/off since system
	 * fails to suspend if VPLL clock of 70.5 MHz is used and these
	 * clocks are disabled before suspend. So enable them here.
	 */
	clk_prepare_enable(res->sclk_hdmi);
	clk_prepare_enable(res->hdmi);

	return 0;
fail:
	DRM_ERROR("HDMI resource init - failed\n");
	return -ENODEV;
}

static struct s5p_hdmi_platform_data *drm_hdmi_dt_parse_pdata
					(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct s5p_hdmi_platform_data *pd;
	u32 value;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		DRM_ERROR("memory allocation for pdata failed\n");
		goto err_data;
	}

	if (!of_find_property(np, "hpd-gpio", &value)) {
		DRM_ERROR("no hpd gpio property found\n");
		goto err_data;
	}

	pd->hpd_gpio = of_get_named_gpio(np, "hpd-gpio", 0);

	return pd;

err_data:
	return NULL;
}

static int drm_hdmi_dt_parse_phy_pow_control(struct hdmi_context *hdata)
{
	struct device_node *phy_pow_ctrl_node;
	u32 buf[2];
	int ret = 0;

	phy_pow_ctrl_node = of_find_node_by_name(NULL, "phy-power-control");
	if (!phy_pow_ctrl_node)
		return 0;

	/* reg property holds two informations: addr of pmu register, size */
	if (of_property_read_u32_array(phy_pow_ctrl_node, "reg",
			(u32 *)&buf, 2)) {
		DRM_ERROR("faild to get phy power control reg\n");
		ret = -EINVAL;
		goto fail;
	}

	hdata->phy_pow_ctrl_reg = devm_ioremap(hdata->dev, buf[0],  buf[1]);
	if (!hdata->phy_pow_ctrl_reg) {
		DRM_ERROR("failed to ioremap phy pmu reg\n");
		ret = -ENOMEM;
		goto fail;
	}

fail:
	of_node_put(phy_pow_ctrl_node);
	return ret;
}

static struct platform_device_id hdmi_driver_types[] = {
	{
		.name		= "s5pv210-hdmi",
		.driver_data    = HDMI_VER_EXYNOS4210,
	}, {
		.name		= "exynos4-hdmi",
		.driver_data    = HDMI_VER_EXYNOS4210,
	}, {
		.name		= "exynos4-hdmi14",
		.driver_data	= HDMI_VER_EXYNOS4212,
	}, {
		.name		= "exynos5-hdmi",
		.driver_data	= HDMI_VER_EXYNOS4212,
	}, {
		/* end node */
	}
};

static struct of_device_id hdmi_match_types[] = {
	{
		.compatible = "samsung,exynos4212-hdmi",
		.data	= (void	*)HDMI_VER_EXYNOS4212,
	}, {
		.compatible = "samsung,exynos5420-hdmi",
		.data	= (void	*)HDMI_VER_EXYNOS5420,
	}, {
		/* end node */
	}
};

struct platform_device *hdmi_audio_device;

int hdmi_register_audio_device(struct platform_device *pdev)
{
	struct hdmi_context *hdata = platform_get_drvdata(pdev);
	struct platform_device *audio_dev;
	int ret;

	DRM_DEBUG_KMS("[PDEV:%s]\n", pdev->name);

	audio_dev = platform_device_alloc("exynos-hdmi-audio", -1);
	if (!audio_dev) {
		DRM_ERROR("hdmi audio device allocation failed.\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = platform_device_add_resources(audio_dev, pdev->resource,
			pdev->num_resources);
	if (ret) {
		ret = -ENOMEM;
		goto err_device;
	}

	audio_dev->dev.of_node = of_get_next_child(pdev->dev.of_node, NULL);
	audio_dev->dev.platform_data = (void *)hdata->hpd_gpio;

	ret = platform_device_add(audio_dev);
	if (ret) {
		DRM_ERROR("hdmi audio device add failed.\n");
		goto err_device;
	}

	hdmi_audio_device = audio_dev;
	return 0;

err_device:
	platform_device_put(audio_dev);

err:
	return ret;
}

void hdmi_unregister_audio_device(void)
{
	DRM_DEBUG_KMS("\n");
	platform_device_unregister(hdmi_audio_device);
}


static int hdmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hdmi_context *hdata;
	struct s5p_hdmi_platform_data *pdata;
	struct resource *res;
	struct device_node *ddc_node, *phy_node, *hdcp_node;
	int ret;

	DRM_DEBUG_KMS("[PDEV:%s]\n", pdev->name);

	if (pdev->dev.of_node) {
		pdata = drm_hdmi_dt_parse_pdata(dev);
		if (IS_ERR(pdata)) {
			DRM_ERROR("failed to parse dt\n");
			return PTR_ERR(pdata);
		}
	} else {
		pdata = pdev->dev.platform_data;
	}

	if (!pdata) {
		DRM_ERROR("no platform data specified\n");
		return -EINVAL;
	}

	hdata = devm_kzalloc(&pdev->dev, sizeof(struct hdmi_context),
								GFP_KERNEL);
	if (!hdata) {
		DRM_ERROR("out of memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, hdata);

	if (dev->of_node) {
		const struct of_device_id *match;
		match = of_match_node(of_match_ptr(hdmi_match_types),
					pdev->dev.of_node);
		if (match == NULL)
			return -ENODEV;
		hdata->version = (enum hdmi_version)match->data;
	} else {
		hdata->version = (enum hdmi_version)platform_get_device_id
					(pdev)->driver_data;
	}

	hdata->hpd_gpio = pdata->hpd_gpio;
	hdata->dev = dev;

	ret = hdmi_resources_init(hdata);

	if (ret) {
		DRM_ERROR("hdmi_resources_init failed\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		DRM_ERROR("failed to find registers\n");
		return -ENOENT;
	}

	hdata->regs = devm_request_and_ioremap(&pdev->dev, res);
	if (!hdata->regs) {
		DRM_ERROR("failed to map registers\n");
		return -ENXIO;
	}

	/* map hdmiphy power control reg */
	ret = drm_hdmi_dt_parse_phy_pow_control(hdata);
	if (ret) {
		DRM_ERROR("failed to map phy power control registers\n");
		return ret;
	}

	/* DDC i2c driver */
	ddc_node = of_find_node_by_name(NULL, "hdmiddc");
	if (!ddc_node) {
		DRM_ERROR("Failed to find ddc node in device tree\n");
		return -ENODEV;
	}
	hdata->ddc_port = of_find_i2c_device_by_node(ddc_node);
	if (!hdata->ddc_port) {
		DRM_ERROR("Failed to get ddc i2c client by node\n");
		return -ENODEV;
	}

	/* hdmiphy i2c driver */
	phy_node = of_find_node_by_name(NULL, "hdmiphy");
	if (!phy_node) {
		DRM_ERROR("Failed to find hdmiphy node in device tree\n");
		ret = -ENODEV;
		goto err_ddc;
	}

	if (hdata->version == HDMI_VER_EXYNOS5420) {
		u32 buffer[2];

		if (of_property_read_u32_array(phy_node, "reg", buffer, 2)) {
			DRM_ERROR("faild to get reg for hdmi-phy\n");
			ret = -EINVAL;
			goto err_ddc;
		}

		hdata->regs_hdmiphy = ioremap(buffer[0], buffer[1]);
		if (!hdata->regs_hdmiphy) {
			DRM_ERROR("failed to ioremap hdmi-phy\n");
			ret = -ENOMEM;
			goto err_ddc;
		}
		hdata->hdmiphy_port = NULL;
	} else {
		hdata->hdmiphy_port = of_find_i2c_device_by_node(phy_node);
		if (!hdata->hdmiphy_port) {
			DRM_ERROR("Failed to get hdmi phy i2c client\n");
			ret = -ENODEV;
			goto err_ddc;
		}
	}

	spin_lock_init(&hdata->writemask_lock);

	/* HDCP i2c driver */
	hdcp_node = of_find_node_by_name(NULL, "hdmihdcp");
	if (!hdcp_node) {
		DRM_ERROR("Failed to find hdcp node in device tree\n");
		return -ENODEV;
	}
	hdata->hdcp_port = of_find_i2c_device_by_node(hdcp_node);
	if (!hdata->hdcp_port) {
		DRM_ERROR("Failed to get hdcp i2c client by node\n");
		return -ENODEV;
	}

	hdata->hdcp_irq = platform_get_irq(pdev, 0);
	if (hdata->hdcp_irq < 0) {
		DRM_ERROR("Failed to get HDCP irq\n");
		ret = hdata->hdcp_irq;
		goto err_hdmiphy;
	}

	hdcp_initialize(hdata);

	ret = request_threaded_irq(hdata->hdcp_irq, NULL, hdcp_irq_thread,
			IRQF_ONESHOT, "hdmi_hdcp", hdata);
	if (ret) {
		DRM_ERROR("Failed to request hdcp interrupt %d\n", ret);
		goto err_hdmiphy;
	}

	hdata->irq = gpio_to_irq(hdata->hpd_gpio);
	if (hdata->irq < 0) {
		DRM_ERROR("failed to get GPIO irq\n");
		ret = hdata->irq;
		goto err_hdcp;
	}

	hdata->hpd = gpio_get_value(hdata->hpd_gpio);

	ret = request_threaded_irq(hdata->irq, NULL,
			hdmi_irq_thread, IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED,
			"hdmi", hdata);
	if (ret) {
		DRM_ERROR("failed to register hdmi interrupt\n");
		goto err_hdcp;
	}

	hdmi_display.ctx = hdata;
	exynos_drm_display_register(&hdmi_display);

	INIT_DELAYED_WORK(&hdata->hotplug_work, hdmi_hotplug_work_func);

	if (support_hdmi_audio_through_alsa(hdata)) {
		ret = hdmi_register_audio_device(pdev);
		if (ret) {
			DRM_ERROR("hdmi-audio device registering failed.\n");
			goto err_hdcp;
		}
	}

	return 0;

err_hdcp:
	if (hdata->hdcp_port)
		put_device(&hdata->hdcp_port->dev);
err_hdmiphy:
	if (hdata->hdmiphy_port)
		put_device(&hdata->hdmiphy_port->dev);
err_ddc:
	put_device(&hdata->ddc_port->dev);
	return ret;
}

static int hdmi_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hdmi_context *hdata = get_hdmi_context(dev);
	struct hdmi_resources *res = &hdata->res;

	DRM_DEBUG_KMS("[PDEV:%s]\n", pdev->name);

	hdmi_unregister_audio_device();

	free_irq(hdata->irq, hdata);

	clk_disable_unprepare(res->sclk_hdmi);
	clk_disable_unprepare(res->hdmi);

	cancel_delayed_work_sync(&hdata->hotplug_work);

	put_device(&hdata->hdcp_port->dev);
	put_device(&hdata->hdmiphy_port->dev);
	put_device(&hdata->ddc_port->dev);

	return 0;
}
struct platform_driver hdmi_driver = {
	.probe		= hdmi_probe,
	.remove		= hdmi_remove,
	.id_table = hdmi_driver_types,
	.driver		= {
		.name	= "exynos-hdmi",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(hdmi_match_types),
	},
};
