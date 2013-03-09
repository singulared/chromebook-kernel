/*
 * Samsung's EXYNOS4 flattened device tree enabled machine
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 * Copyright (c) 2010-2011 Linaro Ltd.
 *		www.linaro.org
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/serial_core.h>
#include <linux/clocksource.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>

#include "common.h"

#include <plat/mfc.h>

static void __init exynos4_dt_map_io(void)
{
	exynos_init_io(NULL, 0);
}

static void __init exynos4_dt_machine_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static char const *exynos4_dt_compat[] __initdata = {
	"samsung,exynos4210",
	"samsung,exynos4212",
	"samsung,exynos4412",
	NULL
};

DT_MACHINE_START(EXYNOS4210_DT, "Samsung Exynos4 (Flattened Device Tree)")
	/* Maintainer: Thomas Abraham <thomas.abraham@linaro.org> */
	.smp		= smp_ops(exynos_smp_ops),
	.init_irq	= exynos4_init_irq,
	.map_io		= exynos4_dt_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= exynos4_dt_machine_init,
	.init_late	= exynos_init_late,
	.init_time	= exynos_init_time,
	.dt_compat	= exynos4_dt_compat,
	.restart        = exynos4_restart,
MACHINE_END
