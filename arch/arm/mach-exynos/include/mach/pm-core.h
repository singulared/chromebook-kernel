/* linux/arch/arm/mach-exynos4/include/mach/pm-core.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Based on arch/arm/mach-s3c2410/include/mach/pm-core.h,
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * EXYNOS4210 - PM core support for arch/arm/plat-s5p/pm.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_PM_CORE_H
#define __ASM_ARCH_PM_CORE_H __FILE__

#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/regs-pmu.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-core.h>

static inline void s3c_pm_debug_init_uart(void)
{
	/* nothing here yet */
}

#ifdef CONFIG_SAMSUNG_PM_DEBUG

static inline void s3c_pm_arch_restore_uarts(void)
{
	if (soc_is_exynos5250()) {
		WARN_ON_ONCE(!(__raw_readl(S5P_PAD_RET_UART_OPTION) &
			       EXYNOS5_PAD_RET_UART_AUTOMATIC_WAKEUP));

		/* force gpio pins GPA1(4) and GPA1(5) to UART3 mode */
		s3c_gpio_cfgpin(EXYNOS5_GPA1(4), S3C_GPIO_SPECIAL(2));
		s3c_gpio_cfgpin(EXYNOS5_GPA1(5), S3C_GPIO_SPECIAL(2));

		/* wait for things to settle */
		mdelay(1);
	}
}

#endif

static inline void s3c_pm_arch_prepare_irqs(void)
{
	__raw_writel(s3c_irqwake_eintmask, S5P_EINT_WAKEUP_MASK);
	__raw_writel(s3c_irqwake_intmask & ~(1 << 31), S5P_WAKEUP_MASK);
}

static inline void s3c_pm_arch_stop_clocks(void)
{
	/* nothing here yet */
}

static inline void s3c_pm_arch_show_resume_irqs(void)
{
	/* nothing here yet */
}

static inline void s3c_pm_arch_update_uart(void __iomem *regs,
					   struct pm_uart_save *save)
{
	/* nothing here yet */
}

static inline void s3c_pm_restored_gpios(void)
{
	/* nothing here yet */
}

static inline void samsung_pm_saved_gpios(void)
{
	/* nothing here yet */
}

#endif /* __ASM_ARCH_PM_CORE_H */
