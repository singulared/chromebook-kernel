/* Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com
 *
 * Base Exynos FIMD1 configuration
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/fb.h>
#include <linux/gpio.h>

#include <plat/gpio-cfg.h>
#include <video/samsung_fimd.h>

#include <mach/map.h>

void exynos5_fimd1_gpio_setup_24bpp(void)
{
	unsigned int reg;

	/*
	* Set DISP1BLK_CFG register for Display path selection
	*
	* FIMD of DISP1_BLK Bypass selection : DISP1BLK_CFG[15]
	* ---------------------
	*  0 | MIE/MDINE
	*  1 | FIMD : selected
	*/
	reg = __raw_readl(S3C_VA_SYS + 0x0214);
	reg &= ~(1 << 15);      /* To save other reset values */
	__raw_writel(reg, S3C_VA_SYS + 0x0214);
}
