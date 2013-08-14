/*
 * arch/arm/mach-exynos/resetreason.c
 *
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *
 * http://www.samsung.com
 *
 * This is based on Android implementation for exynos5250 by
 * Todd Poynor <toddpoynor@google.com>
 * ref: commit-ID 33e5f1acfdd1dd3cf0efbabf21c8218ed7bc6b9b
 *
 * Copyright (C) 2012 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/bug.h>
#include <linux/io.h>
#include <mach/regs-pmu.h>
#include <plat/cpu.h>

struct resetreason_flags {
	const char *str;
	u32 mask;
};

#define EXYNOS5420_RST_STAT_ISP_ARM_WDTRESET	(1<<29)
#define EXYNOS5420_RST_STAT_SWRESET		(1<<11)
#define EXYNOS5420_RST_STAT_WRSET		(1<<10)
#define EXYNOS5420_RST_STAT_SYS_WDTRESET	(1<<9)
#define EXYNOS5420_RST_STAT_PINRESET		(1<<8)
#define EXYNOS5420_RST_STAT_KFC_WRESET3		(1<<7)
#define EXYNOS5420_RST_STAT_KFC_WRESET2		(1<<6)
#define EXYNOS5420_RST_STAT_KFC_WRESET1		(1<<5)
#define EXYNOS5420_RST_STAT_KFC_WRESET0		(1<<4)
#define EXYNOS5420_RST_STAT_EAGLE_WRESET3	(1<<3)
#define EXYNOS5420_RST_STAT_EAGLE_WRESET2	(1<<2)
#define EXYNOS5420_RST_STAT_EAGLE_WRESET1	(1<<1)
#define EXYNOS5420_RST_STAT_EAGLE_WRESET0	(1<<0)

struct resetreason_flags reset_reason_exynos5420[] = {
	{ "ISP_ARM watchdog timer",  EXYNOS5420_RST_STAT_ISP_ARM_WDTRESET },
	{ "software",		     EXYNOS5420_RST_STAT_SWRESET },
	{ "warm",		     EXYNOS5420_RST_STAT_WRSET },
	{ "system watchdog timer",   EXYNOS5420_RST_STAT_SYS_WDTRESET },
	{ "XnRESET pin",	     EXYNOS5420_RST_STAT_PINRESET },
	{ "KFC warm",		     EXYNOS5420_RST_STAT_KFC_WRESET3 |
				     EXYNOS5420_RST_STAT_KFC_WRESET2 |
				     EXYNOS5420_RST_STAT_KFC_WRESET1 |
				     EXYNOS5420_RST_STAT_KFC_WRESET0 },
	{ "Eagle warm",		     EXYNOS5420_RST_STAT_EAGLE_WRESET3 |
				     EXYNOS5420_RST_STAT_EAGLE_WRESET2 |
				     EXYNOS5420_RST_STAT_EAGLE_WRESET1 |
				     EXYNOS5420_RST_STAT_EAGLE_WRESET0 },
};

#define EXYNOS5250_RST_STAT_SWRESET		(1<<29)
#define EXYNOS5250_RST_STAT_WRSET		(1<<28)
#define EXYNOS5250_RST_STAT_ISP_ARM_WDTRESET	(1<<26)
#define EXYNOS5250_RST_STAT_FSYS_ARM_WDTRESET	(1<<25)
#define EXYNOS5250_RST_STAT_SYS_WDTRESET	(1<<20)
#define EXYNOS5250_RST_STAT_PINRESET		(1<<16)
#define EXYNOS5250_RST_STAT_EAGLE_WRESET1	(1<<1)
#define EXYNOS5250_RST_STAT_EAGLE_WRESET0	(1<<0)

struct resetreason_flags reset_reason_exynos5250[] = {
	{ "software",		      EXYNOS5250_RST_STAT_SWRESET },
	{ "warm",		      EXYNOS5250_RST_STAT_WRSET },
	{ "ISP_ARM watchdog timer",   EXYNOS5250_RST_STAT_ISP_ARM_WDTRESET },
	{ "FSYS_ARM watchdog timer",  EXYNOS5250_RST_STAT_FSYS_ARM_WDTRESET },
	{ "system watchdog timer",    EXYNOS5250_RST_STAT_SYS_WDTRESET },
	{ "XnRESET pin",	      EXYNOS5250_RST_STAT_PINRESET },
	{ "Eagle warm",		      EXYNOS5250_RST_STAT_EAGLE_WRESET1 |
				      EXYNOS5250_RST_STAT_EAGLE_WRESET0 },
};

static int __init resetreason_init(void)
{
	int i, array_size;
	u32 reasons = __raw_readl(EXYNOS_RST_STAT);
	struct resetreason_flags *soc_reset_reason;

	if (soc_is_exynos5420()) {
		soc_reset_reason = &reset_reason_exynos5420[0];
		array_size = ARRAY_SIZE(reset_reason_exynos5420);
	} else if (soc_is_exynos5250()) {
		soc_reset_reason = &reset_reason_exynos5250[0];
		array_size = ARRAY_SIZE(reset_reason_exynos5250);
	} else	/* In case of Exynos4 */
		return 0;

	for (i = 0; i < array_size; i++)
		if (reasons & soc_reset_reason[i].mask)
			break;

	/*
	 * Reset reason will always match an entry in one of the arrays
	 * above, so index 'i' will not exceed the array_size value.
	 */
	pr_info("Last reset was %s reset (RST_STAT=0x%x)\n",
					soc_reset_reason[i].str, reasons);
	return 0;
}

postcore_initcall(resetreason_init);
