/* arch/arm/mach-exynos4/include/mach/regs-debug.h
 *
 * Copyright (c) 2013 Samsung Electronics
 *		http://www.samsung.com
 *
 * Exynos5 debug register definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __PLAT_REGS_DEBUG_H
#define __PLAT_REGS_DEBUG_H __FILE__

#define EXYNOS5250_DEBUG_PA_CPU(x)		(0x10890000 + (x * 0x2000))
#define EXYNOS5420_A15_DEBUG_PA_CPU(x)		(0x1088E000 + (x * 0x2000))
#define EXYNOS5420_A7_DEBUG_PA_CPU(x)		(0x1089F000 + (x * 0x2000))

#define CPU_DBGPCSR		0xA0

#endif /* _PLAT_REGS_DEBUG_H */
