/* exynos_drm_fimd.c
 *
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors:
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Inki Dae <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <drm/drmP.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>

#include <video/samsung_fimd.h>
#include <drm/exynos_drm.h>

#include "exynos_dp_core.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fbdev.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_iommu.h"

/*
 * FIMD is stand for Fully Interactive Mobile Display and
 * as a display controller, it transfers contents drawn on memory
 * to a LCD Panel through Display Interfaces such as RGB or
 * CPU Interface.
 */

/* position control register for hardware window 0, 2 ~ 4.*/
#define VIDOSD_A(win)		(VIDOSD_BASE + 0x00 + (win) * 16)
#define VIDOSD_B(win)		(VIDOSD_BASE + 0x04 + (win) * 16)
/* size control register for hardware window 0. */
#define VIDOSD_C_SIZE_W0	(VIDOSD_BASE + 0x08)
/* alpha control register for hardware window 1 ~ 4. */
#define VIDOSD_C(win)		(VIDOSD_BASE + 0x18 + (win) * 16)
/* size control register for hardware window 1 ~ 4. */
#define VIDOSD_D(win)		(VIDOSD_BASE + 0x0C + (win) * 16)

#define VIDWx_BUF_START(win, buf)	(VIDW_BUF_START(buf) + (win) * 8)
#define VIDWx_BUF_START_S(win, buf)	(VIDW_BUF_START_S(buf) + (win) * 8)
#define VIDWx_BUF_END(win, buf)		(VIDW_BUF_END(buf) + (win) * 8)
#define VIDWx_BUF_SIZE(win, buf)	(VIDW_BUF_SIZE(buf) + (win) * 4)

/* color key control register for hardware window 1 ~ 4. */
#define WKEYCON0_BASE(x)		((WKEYCON0 + 0x140) + (x * 8))
/* color key value register for hardware window 1 ~ 4. */
#define WKEYCON1_BASE(x)		((WKEYCON1 + 0x140) + (x * 8))

/* FIMD has totally five hardware windows. */
#define FIMD_WIN_NR	5

#define get_fimd_context(dev)	platform_get_drvdata(to_platform_device(dev))

struct fimd_driver_data {
	unsigned int timing_base;
};

static struct fimd_driver_data exynos4_fimd_driver_data = {
	.timing_base = 0x0,
};

static struct fimd_driver_data exynos5_fimd_driver_data = {
	.timing_base = 0x20000,
};

struct fimd_win_data {
	unsigned int		offset_x;
	unsigned int		offset_y;
	unsigned int		ovl_width;
	unsigned int		ovl_height;
	unsigned int		fb_width;
	unsigned int		fb_height;
	unsigned int		pitch;
	unsigned int		bpp;
	dma_addr_t		dma_addr;
	unsigned int		buf_offsize;
	unsigned int		line_size;	/* bytes */
	bool			enabled;
	bool			resume;
};

struct fimd_mode_data {
	unsigned		vtotal;
	unsigned		vdisplay;
	unsigned		vsync_len;
	unsigned		vbpd;
	unsigned		vfpd;
	unsigned		htotal;
	unsigned		hdisplay;
	unsigned		hsync_len;
	unsigned		hbpd;
	unsigned		hfpd;
	u32			clkdiv;
};

struct fimd_context {
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct clk			*sclk_mout_fimd;
	struct clk			*fimd_mux_clk;
	struct clk			*bus_clk;
	struct clk			*lcd_clk;
	void __iomem			*regs;
	void __iomem			*regs_mie;
	struct fimd_mode_data		mode;
	struct fimd_win_data		win_data[FIMD_WIN_NR];
	unsigned int			default_win;
	u32				vidcon0;
	u32				vidcon1;
	bool				suspended;
	int				pipe;
	wait_queue_head_t		wait_vsync_queue;
	atomic_t			wait_vsync_event;
};

static struct device *dp_dev;

void exynos_fimd_dp_attach(struct device *dev)
{
	DRM_DEBUG_KMS("%s. %s.\n", __FILE__, __func__);
	dp_dev = dev;
}

static const struct of_device_id fimd_driver_dt_match[] = {
	{ .compatible = "samsung,exynos4-fimd",
	  .data = &exynos4_fimd_driver_data },
	{ .compatible = "samsung,exynos5-fimd",
	  .data = &exynos5_fimd_driver_data },
	{},
};
MODULE_DEVICE_TABLE(of, fimd_driver_dt_match);

static inline struct fimd_driver_data *drm_fimd_get_driver_data(
	struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(fimd_driver_dt_match, &pdev->dev);

	if (of_id)
		return (struct fimd_driver_data *)of_id->data;

	return (struct fimd_driver_data *)
		platform_get_device_id(pdev)->driver_data;
}

static void fimd_win_mode_set(void *in_ctx, struct exynos_drm_overlay *overlay)
{
	struct fimd_context *ctx = in_ctx;
	struct fimd_win_data *win_data;
	int win;
	unsigned long offset;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (!overlay) {
		DRM_ERROR("overlay is NULL\n");
		return;
	}

	win = overlay->zpos;
	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win >= FIMD_WIN_NR)
		return;

	offset = overlay->fb_x * (overlay->bpp >> 3);
	offset += overlay->fb_y * overlay->pitch;

	DRM_DEBUG_KMS("offset = 0x%lx, pitch = %x\n", offset, overlay->pitch);

	win_data = &ctx->win_data[win];

	win_data->offset_x = overlay->crtc_x;
	win_data->offset_y = overlay->crtc_y;
	win_data->ovl_width = overlay->crtc_width;
	win_data->ovl_height = overlay->crtc_height;
	win_data->fb_width = overlay->fb_width;
	win_data->fb_height = overlay->fb_height;
	win_data->pitch = overlay->pitch;
	win_data->dma_addr = overlay->dma_addr[0] + offset;
	win_data->bpp = overlay->bpp;
	win_data->line_size = overlay->fb_width * (overlay->bpp >> 3);
	win_data->buf_offsize = overlay->pitch - win_data->line_size;

	DRM_DEBUG_KMS("offset_x = %d, offset_y = %d\n",
			win_data->offset_x, win_data->offset_y);
	DRM_DEBUG_KMS("ovl_width = %d, ovl_height = %d\n",
			win_data->ovl_width, win_data->ovl_height);
	DRM_DEBUG_KMS("paddr = 0x%lx\n", (unsigned long)win_data->dma_addr);
	DRM_DEBUG_KMS("fb_width = %d, crtc_width = %d\n",
			overlay->fb_width, overlay->crtc_width);
}

static void fimd_win_set_pixfmt(struct fimd_context *ctx, unsigned int win)
{
	struct fimd_win_data *win_data = &ctx->win_data[win];
	unsigned long val;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	val = WINCONx_ENWIN;

	switch (win_data->bpp) {
	case 1:
		val |= WINCON0_BPPMODE_1BPP;
		val |= WINCONx_BITSWP;
		break;
	case 2:
		val |= WINCON0_BPPMODE_2BPP;
		val |= WINCONx_BITSWP;
		break;
	case 4:
		val |= WINCON0_BPPMODE_4BPP;
		val |= WINCONx_BITSWP;
		break;
	case 8:
		val |= WINCON0_BPPMODE_8BPP_PALETTE;
		val |= WINCONx_BYTSWP;
		break;
	case 16:
		val |= WINCON0_BPPMODE_16BPP_565;
		val |= WINCONx_HAWSWP;
		break;
	case 24:
		val |= WINCON0_BPPMODE_24BPP_888;
		val |= WINCONx_WSWP;
		break;
	case 32:
		val |= WINCON1_BPPMODE_28BPP_A4888
			| WINCON1_BLD_PIX | WINCON1_ALPHA_SEL;
		val |= WINCONx_WSWP;
		break;
	default:
		DRM_DEBUG_KMS("invalid pixel size so using unpacked 24bpp.\n");

		val |= WINCON0_BPPMODE_24BPP_888;
		val |= WINCONx_WSWP;
		break;
	}

	DRM_DEBUG_KMS("bpp = %d\n", win_data->bpp);

	/* Restrict the burst length to 4WORD for cursor */
	if (win_data->fb_width <= 96)
		val |= WINCONx_BURSTLEN_4WORD;
	else
		val |= WINCONx_BURSTLEN_16WORD;

	writel(val, ctx->regs + WINCON(win));
}

static void fimd_win_set_colkey(struct fimd_context *ctx, unsigned int win)
{
	unsigned int keycon0 = 0, keycon1 = 0;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	keycon0 = ~(WxKEYCON0_KEYBL_EN | WxKEYCON0_KEYEN_F |
			WxKEYCON0_DIRCON) | WxKEYCON0_COMPKEY(0);

	keycon1 = WxKEYCON1_COLVAL(0xffffffff);

	writel(keycon0, ctx->regs + WKEYCON0_BASE(win));
	writel(keycon1, ctx->regs + WKEYCON1_BASE(win));
}


static void mie_set_6bit_dithering(struct fimd_context *ctx)
{
	struct fimd_mode_data *mode = &ctx->mode;
	int i;

	writel(MIE_HRESOL(mode->hdisplay) | MIE_VRESOL(mode->vdisplay) |
			MIE_MODE_UI, ctx->regs_mie + MIE_CTRL1);

	writel(MIE_WINHADDR0(0) | MIE_WINHADDR1(mode->hdisplay),
			ctx->regs_mie + MIE_WINHADDR);

	writel(MIE_WINVADDR0(0) | MIE_WINVADDR1(mode->vdisplay),
		ctx->regs_mie + MIE_WINVADDR);

	writel(PWMCLKCNT(mode->vtotal * mode->htotal /
		(MIE_PWMCLKVAL + 1)), ctx->regs_mie + MIE_PWMCLKCNT);

	writel(MIE_VBPD(mode->vbpd) | MIE_VFPD(mode->vfpd) |
		MIE_VSPW(mode->vsync_len), ctx->regs_mie + MIE_PWMVIDTCON1);

	writel(MIE_HBPD(mode->hbpd) | MIE_HFPD(mode->hbpd) |
		MIE_HSPW(mode->hsync_len), ctx->regs_mie + MIE_PWMVIDTCON2);

	writel(MIE_DITHCON_EN | MIE_RGB6MODE, ctx->regs_mie + MIE_AUXCON);

	/* Bypass MIE image brightness enhancement */
	for (i = 0; i <= 0x30; i += 4) {
		writel(0, ctx->regs_mie + 0x100 + i);
		writel(0, ctx->regs_mie + 0x200 + i);
	}
}

static void fimd_win_commit(void *in_ctx, int zpos)
{
	struct fimd_context *ctx = in_ctx;
	struct fimd_win_data *win_data;
	int win = zpos;
	unsigned long val, alpha, size;
	unsigned int last_x;
	unsigned int last_y;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return;

	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win >= FIMD_WIN_NR)
		return;

	win_data = &ctx->win_data[win];

	/*
	 * SHADOWCON register is used for enabling timing.
	 *
	 * for example, once only width value of a register is set,
	 * if the dma is started then fimd hardware could malfunction so
	 * with protect window setting, the register fields with prefix '_F'
	 * wouldn't be updated at vsync also but updated once unprotect window
	 * is set.
	 */

	/* protect windows */
	val = readl(ctx->regs + SHADOWCON);
	val |= SHADOWCON_WINx_PROTECT(win);
	writel(val, ctx->regs + SHADOWCON);

	/* buffer start address */
	val = (unsigned long)win_data->dma_addr;
	writel(val, ctx->regs + VIDWx_BUF_START(win, 0));

	/* buffer end address */
	size = win_data->fb_height * win_data->pitch;
	val = (unsigned long)(win_data->dma_addr + size);
	writel(val, ctx->regs + VIDWx_BUF_END(win, 0));

	DRM_DEBUG_KMS("start addr = 0x%lx, end addr = 0x%lx, size = 0x%lx\n",
			(unsigned long)win_data->dma_addr, val, size);
	DRM_DEBUG_KMS("ovl_width = %d, ovl_height = %d\n",
			win_data->ovl_width, win_data->ovl_height);

	/* buffer size */
	val = VIDW_BUF_SIZE_OFFSET(win_data->buf_offsize) |
		VIDW_BUF_SIZE_PAGEWIDTH(win_data->line_size) |
		VIDW_BUF_SIZE_OFFSET_E(win_data->buf_offsize) |
		VIDW_BUF_SIZE_PAGEWIDTH_E(win_data->line_size);
	writel(val, ctx->regs + VIDWx_BUF_SIZE(win, 0));

	/* OSD position */
	val = VIDOSDxA_TOPLEFT_X(win_data->offset_x) |
		VIDOSDxA_TOPLEFT_Y(win_data->offset_y) |
		VIDOSDxA_TOPLEFT_X_E(win_data->offset_x) |
		VIDOSDxA_TOPLEFT_Y_E(win_data->offset_y);
	writel(val, ctx->regs + VIDOSD_A(win));

	last_x = win_data->offset_x + win_data->ovl_width;
	if (last_x)
		last_x--;
	last_y = win_data->offset_y + win_data->ovl_height;
	if (last_y)
		last_y--;

	val = VIDOSDxB_BOTRIGHT_X(last_x) | VIDOSDxB_BOTRIGHT_Y(last_y) |
		VIDOSDxB_BOTRIGHT_X_E(last_x) | VIDOSDxB_BOTRIGHT_Y_E(last_y);

	writel(val, ctx->regs + VIDOSD_B(win));

	DRM_DEBUG_KMS("osd pos: tx = %d, ty = %d, bx = %d, by = %d\n",
			win_data->offset_x, win_data->offset_y, last_x, last_y);

	/* hardware window 0 doesn't support alpha channel. */
	if (win != 0) {
		/* OSD alpha */
		alpha = VIDISD14C_ALPHA1_R(0xf) |
			VIDISD14C_ALPHA1_G(0xf) |
			VIDISD14C_ALPHA1_B(0xf);

		writel(alpha, ctx->regs + VIDOSD_C(win));
	}

	/* OSD size */
	if (win != 3 && win != 4) {
		u32 offset = VIDOSD_D(win);
		if (win == 0)
			offset = VIDOSD_C_SIZE_W0;
		val = win_data->ovl_width * win_data->ovl_height;
		writel(val, ctx->regs + offset);

		DRM_DEBUG_KMS("osd size = 0x%x\n", (unsigned int)val);
	}

	fimd_win_set_pixfmt(ctx, win);

	/* hardware window 0 doesn't support color key. */
	if (win != 0)
		fimd_win_set_colkey(ctx, win);

	/* wincon */
	val = readl(ctx->regs + WINCON(win));
	val |= WINCONx_ENWIN;
	writel(val, ctx->regs + WINCON(win));

	/* only apply dithering on default window */
	if (win == ctx->default_win)
		mie_set_6bit_dithering(ctx);

	/* Enable DMA channel and unprotect windows */
	val = readl(ctx->regs + SHADOWCON);
	val |= SHADOWCON_CHx_ENABLE(win);
	val &= ~SHADOWCON_WINx_PROTECT(win);
	writel(val, ctx->regs + SHADOWCON);

	win_data->enabled = true;
}

static void fimd_win_disable(void *in_ctx, int zpos)
{
	struct fimd_context *ctx = in_ctx;
	struct fimd_win_data *win_data;
	int win = zpos;
	u32 val;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win >= FIMD_WIN_NR)
		return;

	win_data = &ctx->win_data[win];

	if (ctx->suspended) {
		/* do not resume this window*/
		win_data->resume = false;
		return;
	}

	/* protect windows */
	val = readl(ctx->regs + SHADOWCON);
	val |= SHADOWCON_WINx_PROTECT(win);
	writel(val, ctx->regs + SHADOWCON);

	/* wincon */
	val = readl(ctx->regs + WINCON(win));
	val &= ~WINCONx_ENWIN;
	writel(val, ctx->regs + WINCON(win));

	/* unprotect windows */
	val = readl(ctx->regs + SHADOWCON);
	val &= ~SHADOWCON_CHx_ENABLE(win);
	val &= ~SHADOWCON_WINx_PROTECT(win);
	writel(val, ctx->regs + SHADOWCON);

	win_data->enabled = false;
}

static int fimd_mgr_initialize(void *in_ctx, struct drm_device *drm_dev,
		int pipe)
{
	struct fimd_context *ctx = in_ctx;

	ctx->drm_dev = drm_dev;
	ctx->pipe = pipe;

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = 1, we can use the vblank feature.
	 *
	 * P.S. note that we wouldn't use drm irq handler but
	 *	just specific driver own one instead because
	 *	drm framework supports only one irq handler.
	 */
	ctx->drm_dev->irq_enabled = 1;

	if (is_drm_iommu_supported(ctx->drm_dev))
		drm_iommu_attach_device(ctx->drm_dev, ctx->dev);

	return 0;
}

static void fimd_mgr_remove(void *in_ctx)
{
	struct fimd_context *ctx = in_ctx;

	if (is_drm_iommu_supported(ctx->drm_dev))
		drm_iommu_detach_device(ctx->drm_dev, ctx->dev);
}

static void fimd_dpms(void *in_ctx, int mode)
{
	struct fimd_context *ctx = in_ctx;

	DRM_DEBUG_KMS("%s, %d\n", __FILE__, mode);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		/*
		 * enable fimd hardware only if suspended status.
		 *
		 * P.S. fimd_dpms function would be called at booting time so
		 * clk_enable could be called double time.
		 */
		if (ctx->suspended)
			pm_runtime_get_sync(ctx->dev);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		if (!ctx->suspended)
			pm_runtime_put_sync(ctx->dev);
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}
}

static u32 fimd_calc_clkdiv(struct fimd_context *ctx,
		const struct drm_display_mode *mode)
{
	unsigned long ideal_clk = mode->htotal * mode->vtotal * mode->vrefresh;
	u32 clkdiv;

	/* Find the clock divider value that gets us closest to ideal_clk */
	clkdiv = DIV_ROUND_CLOSEST(clk_get_rate(ctx->lcd_clk), ideal_clk);

	return (clkdiv < 0x100) ? clkdiv : 0xff;
}

static bool fimd_mode_fixup(void *in_ctx, const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	struct fimd_context *ctx = in_ctx;

	if (adjusted_mode->vrefresh == 0)
		adjusted_mode->vrefresh = 60;

	adjusted_mode->clock = clk_get_rate(ctx->lcd_clk) /
			fimd_calc_clkdiv(ctx, adjusted_mode);
	return true;
}

static void fimd_update(void *in_ctx, const struct drm_display_mode *in_mode)
{
	struct fimd_context *ctx = in_ctx;
	struct fimd_mode_data *mode = &ctx->mode;
	int hblank, vblank;

	vblank = in_mode->crtc_vblank_end - in_mode->crtc_vblank_start;
	mode->vtotal = in_mode->crtc_vtotal;
	mode->vdisplay = in_mode->crtc_vdisplay;
	mode->vsync_len = in_mode->crtc_vsync_end - in_mode->crtc_vsync_start;
	mode->vbpd = (vblank - mode->vsync_len) / 2;
	mode->vfpd = vblank - mode->vsync_len - mode->vbpd;

	hblank = in_mode->crtc_hblank_end - in_mode->crtc_hblank_start;
	mode->htotal = in_mode->crtc_htotal;
	mode->hdisplay = in_mode->crtc_hdisplay;
	mode->hsync_len = in_mode->crtc_hsync_end - in_mode->crtc_hsync_start;
	mode->hbpd = (hblank - mode->hsync_len) / 2;
	mode->hfpd = hblank - mode->hsync_len - mode->hbpd;

	mode->clkdiv = fimd_calc_clkdiv(ctx, in_mode);
}

static void fimd_commit(void *in_ctx)
{
	struct fimd_context *ctx = in_ctx;
	struct fimd_mode_data *mode = &ctx->mode;
	struct fimd_driver_data *driver_data;
	struct platform_device *pdev = to_platform_device(ctx->dev);
	u32 val;

	driver_data = drm_fimd_get_driver_data(pdev);
	if (ctx->suspended)
		return;

	/* nothing to do if we haven't set the mode yet */
	if (mode->htotal == 0 || mode->vtotal == 0)
		return;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* setup polarity values from machine code. */
	writel(ctx->vidcon1, ctx->regs + driver_data->timing_base + VIDCON1);

	/* setup vertical timing values. */
	val = VIDTCON0_VBPD(mode->vbpd - 1) |
		VIDTCON0_VFPD(mode->vfpd - 1) |
		VIDTCON0_VSPW(mode->vsync_len - 1);
	writel(val, ctx->regs + driver_data->timing_base + VIDTCON0);

	/* setup horizontal timing values.  */
	val = VIDTCON1_HBPD(mode->hbpd - 1) |
		VIDTCON1_HFPD(mode->hfpd - 1) |
		VIDTCON1_HSPW(mode->hsync_len - 1);
	writel(val, ctx->regs + driver_data->timing_base + VIDTCON1);

	/* setup horizontal and vertical display size. */
	val = VIDTCON2_LINEVAL(mode->vdisplay - 1) |
	       VIDTCON2_HOZVAL(mode->hdisplay - 1) |
	       VIDTCON2_LINEVAL_E(mode->vdisplay - 1) |
	       VIDTCON2_HOZVAL_E(mode->hdisplay - 1);
	writel(val, ctx->regs + driver_data->timing_base + VIDTCON2);

	/* setup clock source, clock divider, enable dma. */
	val = ctx->vidcon0;
	val &= ~(VIDCON0_CLKVAL_F_MASK | VIDCON0_CLKDIR);

	if (mode->clkdiv > 1)
		val |= VIDCON0_CLKVAL_F(mode->clkdiv - 1) | VIDCON0_CLKDIR;
	else
		val &= ~VIDCON0_CLKDIR;	/* 1:1 clock */

	/*
	 * fields of register with prefix '_F' would be updated
	 * at vsync(same as dma start)
	 */
	val |= VIDCON0_ENVID | VIDCON0_ENVID_F;
	writel(val, ctx->regs + VIDCON0);
}

static void fimd_apply(void *in_ctx)
{
	struct fimd_context *ctx = in_ctx;
	struct fimd_win_data *win_data;
	int i;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	for (i = 0; i < FIMD_WIN_NR; i++) {
		win_data = &ctx->win_data[i];
		if (win_data->enabled)
			fimd_win_commit(ctx, i);
	}

	fimd_commit(ctx);
}

static int fimd_enable_vblank(void *in_ctx)
{
	struct fimd_context *ctx = in_ctx;
	u32 val;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return -EPERM;

	val = readl(ctx->regs + VIDINTCON0);

	val |= VIDINTCON0_INT_ENABLE;
	val |= VIDINTCON0_INT_FRAME;

	val &= ~VIDINTCON0_FRAMESEL0_MASK;
	val |= VIDINTCON0_FRAMESEL0_VSYNC;
	val &= ~VIDINTCON0_FRAMESEL1_MASK;
	val |= VIDINTCON0_FRAMESEL1_NONE;

	writel(val, ctx->regs + VIDINTCON0);

	return 0;
}

static void fimd_disable_vblank(void *in_ctx)
{
	struct fimd_context *ctx = in_ctx;
	u32 val;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return;

	val = readl(ctx->regs + VIDINTCON0);

	val &= ~VIDINTCON0_INT_FRAME;
	val &= ~VIDINTCON0_INT_ENABLE;

	writel(val, ctx->regs + VIDINTCON0);
}

static void fimd_wait_for_vblank(struct fimd_context *ctx)
{
	u32 val;

	if (ctx->suspended)
		return;

	val = readl(ctx->regs + VIDINTCON0);

	drm_vblank_get(ctx->drm_dev, ctx->pipe);

	atomic_set(&ctx->wait_vsync_event, 1);

	/*
	 * wait for FIMD to signal VSYNC interrupt or return after
	 * timeout which is set to 50ms (refresh rate of 20).
	 */
	if (!wait_event_timeout(ctx->wait_vsync_queue,
				!atomic_read(&ctx->wait_vsync_event),
				DRM_HZ/20))
		DRM_DEBUG_KMS("vblank wait timed out.\n");

	drm_vblank_put(ctx->drm_dev, ctx->pipe);
}

static struct exynos_drm_manager_ops fimd_manager_ops = {
	.initialize = fimd_mgr_initialize,
	.remove = fimd_mgr_remove,
	.dpms = fimd_dpms,
	.apply = fimd_apply,
	.mode_fixup = fimd_mode_fixup,
	.update = fimd_update,
	.commit = fimd_commit,
	.enable_vblank = fimd_enable_vblank,
	.disable_vblank = fimd_disable_vblank,
	.win_mode_set = fimd_win_mode_set,
	.win_commit = fimd_win_commit,
	.win_disable = fimd_win_disable,
};

static struct exynos_drm_manager fimd_manager = {
	.type = EXYNOS_DISPLAY_TYPE_LCD,
	.ops = &fimd_manager_ops,
};

static irqreturn_t fimd_irq_handler(int irq, void *dev_id)
{
	struct fimd_context *ctx = (struct fimd_context *)dev_id;
	u32 val;

	WARN_ON(ctx->suspended);
	val = readl(ctx->regs + VIDINTCON1);

	if (val & VIDINTCON1_INT_FRAME)
		/* VSYNC interrupt */
		writel(VIDINTCON1_INT_FRAME, ctx->regs + VIDINTCON1);

	/* check the crtc is detached already from encoder */
	if (ctx->pipe < 0 || !ctx->drm_dev)
		goto out;

	drm_handle_vblank(ctx->drm_dev, ctx->pipe);
	exynos_drm_crtc_finish_pageflip(ctx->drm_dev, ctx->pipe);

	/* set wait vsync event to zero and wake up queue. */
	if (atomic_read(&ctx->wait_vsync_event)) {
		atomic_set(&ctx->wait_vsync_event, 0);
		DRM_WAKEUP(&ctx->wait_vsync_queue);
	}
out:
	return IRQ_HANDLED;
}

static void fimd_clear_win(struct fimd_context *ctx, int win)
{
	u32 val;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	writel(0, ctx->regs + WINCON(win));
	writel(0, ctx->regs + VIDOSD_A(win));
	writel(0, ctx->regs + VIDOSD_B(win));
	writel(0, ctx->regs + VIDOSD_C(win));

	if (win == 1 || win == 2)
		writel(0, ctx->regs + VIDOSD_D(win));

	val = readl(ctx->regs + SHADOWCON);
	val &= ~SHADOWCON_WINx_PROTECT(win);
	writel(val, ctx->regs + SHADOWCON);
}

static int fimd_clock(struct fimd_context *ctx, bool enable)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (enable) {
		int ret;

		ret = clk_prepare_enable(ctx->bus_clk);
		if (ret < 0)
			return ret;

		ret = clk_prepare_enable(ctx->lcd_clk);
		if  (ret < 0) {
			clk_disable_unprepare(ctx->bus_clk);
			return ret;
		}
	} else {
		clk_disable_unprepare(ctx->lcd_clk);
		clk_disable_unprepare(ctx->bus_clk);
	}

	return 0;
}

static void fimd_window_suspend(struct fimd_context *ctx)
{
	struct fimd_win_data *win_data;
	int i;

	for (i = 0; i < FIMD_WIN_NR; i++) {
		win_data = &ctx->win_data[i];
		if (win_data->enabled) {
			win_data->resume = win_data->enabled;
			fimd_win_disable(ctx, i);
		}
	}
	fimd_wait_for_vblank(ctx);
}

static void fimd_window_resume(struct fimd_context *ctx)
{
	struct fimd_win_data *win_data;
	int i;

	for (i = 0; i < FIMD_WIN_NR; i++) {
		win_data = &ctx->win_data[i];
		win_data->enabled = win_data->resume;
		win_data->resume = false;
	}
}

static int fimd_activate(struct fimd_context *ctx, bool enable)
{
	if (enable && ctx->suspended) {
		int ret;

		ret = fimd_clock(ctx, true);
		if (ret < 0)
			return ret;

		ctx->suspended = false;

		writel(MIE_CLK_ENABLE, ctx->regs + DPCLKCON);

		fimd_window_resume(ctx);

		if (dp_dev)
			exynos_dp_resume(dp_dev);
	} else if (!enable && !ctx->suspended) {
		if (dp_dev)
			exynos_dp_suspend(dp_dev);

		/*
		 * We need to make sure that all windows are disabled before we
		 * suspend that connector. Otherwise we might try to scan from
		 * a destroyed buffer later.
		 */
		fimd_window_suspend(ctx);

		fimd_clock(ctx, false);
		ctx->suspended = true;
	}

	return 0;
}

static struct exynos_drm_fimd_pdata *drm_fimd_dt_parse_pdata(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct exynos_drm_fimd_pdata *pd;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		dev_err(dev, "memory allocation for pdata failed\n");
		return ERR_PTR(-ENOMEM);
	}

	if (of_get_property(np, "samsung,fimd-vidout-rgb", NULL))
		pd->vidcon0 |= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB;
	if (of_get_property(np, "samsung,fimd-inv-hsync", NULL))
		pd->vidcon1 |= VIDCON1_INV_HSYNC;
	if (of_get_property(np, "samsung,fimd-inv-vsync", NULL))
		pd->vidcon1 |= VIDCON1_INV_VSYNC;
	if (of_get_property(np, "samsung,fimd-inv-vclk", NULL))
		pd->vidcon1 |= VIDCON1_INV_VCLK;
	if (of_get_property(np, "samsung,fimd-inv-vden", NULL))
		pd->vidcon1 |= VIDCON1_INV_VDEN;

	of_property_read_u32(np, "samsung,default-window", &pd->default_win);

	of_property_read_u32(np, "samsung,fimd-win-bpp", &pd->bpp);

	of_property_read_u32(np, "samsung,fimd-src-clk-rate",
							&pd->src_clk_rate);

	return pd;
}

static int fimd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fimd_context *ctx;
	struct exynos_drm_fimd_pdata *pdata = pdev->dev.platform_data;
	struct exynos_drm_panel_info *panel;
	struct resource *res;
	int win;
	int ret = -EINVAL;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (pdev->dev.of_node) {
		pdata = drm_fimd_dt_parse_pdata(&pdev->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}
	if (!pdata) {
		dev_err(dev, "no platform data specified\n");
		return -EINVAL;
	}

	panel = &pdata->panel;
	if (!panel) {
		dev_err(dev, "panel is null.\n");
		return -EINVAL;
	}

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->bus_clk = devm_clk_get(dev, "fimd");
	if (IS_ERR(ctx->bus_clk)) {
		dev_err(dev, "failed to get bus clock\n");
		return PTR_ERR(ctx->bus_clk);
	}

	ctx->lcd_clk = devm_clk_get(dev, "sclk_fimd");
	if (IS_ERR(ctx->lcd_clk)) {
		dev_err(dev, "failed to get lcd clock\n");
		return PTR_ERR(ctx->lcd_clk);
	}

	/* Set the parent for FIMD pixel clock */
	ctx->fimd_mux_clk = devm_clk_get(dev, "mout_fimd");
	if (IS_ERR(ctx->fimd_mux_clk)) {
		dev_err(dev, "failed to get fimd mux clk\n");
		return PTR_ERR(ctx->fimd_mux_clk);
	}

	ctx->sclk_mout_fimd = devm_clk_get(dev, "sclk_mout_fimd");
	if (IS_ERR(ctx->sclk_mout_fimd)) {
		dev_err(dev, "failed to get mout_fimd parent\n");
		return PTR_ERR(ctx->sclk_mout_fimd);
	}
	clk_set_parent(ctx->fimd_mux_clk, ctx->sclk_mout_fimd);

	/* Set the FIMD pixel clock to desired value */
	clk_set_rate(ctx->lcd_clk, pdata->src_clk_rate);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	ctx->regs = devm_request_and_ioremap(&pdev->dev, res);
	if (!ctx->regs) {
		dev_err(dev, "failed to map registers\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "irq request failed.\n");
		return -ENXIO;
	}

	ret = devm_request_irq(&pdev->dev, res->start, fimd_irq_handler,
							0, "drm_fimd", ctx);
	if (ret) {
		dev_err(dev, "irq request failed.\n");
		return ret;
	}

	ctx->regs_mie = ioremap(MIE_BASE_ADDRESS, 0x400);
	if (!ctx->regs_mie) {
		dev_err(dev, "failed to map registers\n");
		return -ENXIO;
	}
	ctx->vidcon0 = pdata->vidcon0;
	ctx->vidcon1 = pdata->vidcon1;
	ctx->default_win = pdata->default_win;
	ctx->dev = dev;
	ctx->suspended = true;
	DRM_INIT_WAITQUEUE(&ctx->wait_vsync_queue);
	atomic_set(&ctx->wait_vsync_event, 0);

	platform_set_drvdata(pdev, ctx);

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	for (win = 0; win < FIMD_WIN_NR; win++)
		fimd_clear_win(ctx, win);

	writel(MIE_CLK_ENABLE, ctx->regs + DPCLKCON);

	fimd_manager.ctx = ctx;
	exynos_drm_manager_register(&fimd_manager);

	return 0;
}

static int fimd_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fimd_context *ctx = platform_get_drvdata(pdev);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	exynos_drm_manager_unregister(&fimd_manager);

	if (ctx->suspended)
		goto out;

	pm_runtime_set_suspended(dev);
	pm_runtime_put_sync(dev);

out:
	pm_runtime_disable(dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int fimd_suspend(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);

	/*
	 * do not use pm_runtime_suspend(). if pm_runtime_suspend() is
	 * called here, an error would be returned by that interface
	 * because the usage_count of pm runtime is more than 1.
	 */
	if (!pm_runtime_suspended(dev))
		return fimd_activate(ctx, false);

	return 0;
}

static int fimd_resume(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);

	/*
	 * if entered to sleep when lcd panel was on, the usage_count
	 * of pm runtime would still be 1 so in this case, fimd driver
	 * should be on directly not drawing on pm runtime interface.
	 */
	if (!pm_runtime_suspended(dev)) {
		int ret;

		ret = fimd_activate(ctx, true);
		if (ret < 0)
			return ret;

		/*
		 * in case of dpms on(standby), fimd_apply function will
		 * be called by encoder's dpms callback to update fimd's
		 * registers but in case of sleep wakeup, it's not.
		 * so fimd_apply function should be called at here.
		 */
		fimd_apply(ctx);

		/*
		 * Restore the vblank interrupts to whichever state DRM
		 * wants them.
		 */
		if (ctx->drm_dev && ctx->drm_dev->vblank_enabled[ctx->pipe])
			fimd_enable_vblank(ctx);
	}

	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int fimd_runtime_suspend(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	return fimd_activate(ctx, false);
}

static int fimd_runtime_resume(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	return fimd_activate(ctx, true);
}
#endif

static struct platform_device_id fimd_driver_ids[] = {
	{
		.name		= "exynos4-fb",
		.driver_data	= (unsigned long)&exynos4_fimd_driver_data,
	}, {
		.name		= "exynos5-fb",
		.driver_data	= (unsigned long)&exynos5_fimd_driver_data,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, fimd_driver_ids);

static const struct dev_pm_ops fimd_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(fimd_suspend, fimd_resume)
	SET_RUNTIME_PM_OPS(fimd_runtime_suspend, fimd_runtime_resume, NULL)
};

struct platform_driver fimd_driver = {
	.probe		= fimd_probe,
	.remove		= fimd_remove,
	.id_table       = fimd_driver_ids,
	.driver		= {
		.name	= "exynos4-fb",
		.owner	= THIS_MODULE,
		.pm	= &fimd_pm_ops,
		.of_match_table = of_match_ptr(fimd_driver_dt_match),
	},
};
