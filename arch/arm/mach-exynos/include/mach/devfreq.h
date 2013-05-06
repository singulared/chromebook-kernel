/* linux/arch/arm/mach-exynos/include/mach/exynos-devfreq.h
 *
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

struct exynos_devfreq_platdata {
	/* The default frequency in KHz to be set in case
	 * the driver provides a spurious value or doesnt
	 * specify the initial frequency */
	unsigned int default_qos;
};
