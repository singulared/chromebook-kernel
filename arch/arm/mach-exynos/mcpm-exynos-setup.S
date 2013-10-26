/*
 * Exynos low-level MCPM setup
 *
 * Copyright (C) 2013 Google, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/linkage.h>

ENTRY(exynos_power_up_setup)

	cmp	r0, #0			@ check affinity level
	beq	1f

/*
 * Enable cluster-level coherency, in preparation for turning on the MMU.
 * The ACTLR SMP bit does not need to be set here, because cpu_resume()
 * already restores that.
 */
	b	cci_enable_port_for_self

1:	@ Implementation-specific local CPU setup operations should go here,
	@ if any.  In this case, there is nothing to do.

	bx	lr

ENDPROC(exynos_power_up_setup)

ENTRY(wfi_entry_point)
1:	wfi
	b	1b
ENDPROC(wfi_entry_point)
