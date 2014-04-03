/*
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Common S5P Sleep Code
 * Based on S3C64XX sleep code by:
 *	Ben Dooks, (c) 2008 Simtec Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/linkage.h>
#include <asm/asm-offsets.h>
#include <asm/hardware/cache-l2x0.h>

/*
 *	 The following code is located into the .data section. This is to
 *	 allow l2x0_regs_phys to be accessed with a relative load while we
 *	 can't rely on any MMU translation. We could have put l2x0_regs_phys
 *	 in the .text section as well, but some setups might insist on it to
 *	 be truly read-only. (Reference from: arch/arm/kernel/sleep.S)
 */
	.data
	.align

	/*
	 * sleep magic, to allow the bootloader to check for an valid
	 * image to resume to. Must be the first word before the
	 * s3c_cpu_resume entry.
	 */

	.word	0x2bedf00d

	/*
	 * s3c_cpu_resume
	 *
	 * resume code entry for bootloader to call
	 */

ENTRY(s3c_cpu_resume)

	/*
	 * On exynos5420 and exynos5422 the L2 cache settings may come up with
	 * a value that could cause some instability (not all firmware sets
	 * things up correctly).  If we detect 5420 / 5422 make sure to enable
	 * tag ram setup and data ram setup which will fix things.
	 */
	ldr	r0, s5p_resume_cpu_id
	lsr	r0, r0, #12
	ldr	r1, =0xe5420
	cmp	r0, r1
	beq	exynos_542x_l2_cache
	add	r1, r1, #2
	cmp	r0, r1
	bne	resume_after_542x_l2_cache

exynos_542x_l2_cache:
	/* Skip if we're not on an A15 */
	mrc	p15, 0, r0, c0, c0, 0
	ubfx	r0, r0, #4, #12
	ldr	r1, =0x00000c0f
	teq	r0, r1
	bne	resume_after_542x_l2_cache

	mrc	p15, 1, r0, c9, c0, 2
	orr	r0, r0, #0x220
	mcr	p15, 1, r0, c9, c0, 2

resume_after_542x_l2_cache:

#ifdef CONFIG_CACHE_L2X0
	adr	r0, l2x0_regs_phys
	ldr	r0, [r0]
	ldr	r1, [r0, #L2X0_R_PHY_BASE]
	ldr	r2, [r1, #L2X0_CTRL]
	tst	r2, #0x1
	bne	resume_l2on
	ldr	r2, [r0, #L2X0_R_AUX_CTRL]
	str	r2, [r1, #L2X0_AUX_CTRL]
	ldr	r2, [r0, #L2X0_R_TAG_LATENCY]
	str	r2, [r1, #L2X0_TAG_LATENCY_CTRL]
	ldr	r2, [r0, #L2X0_R_DATA_LATENCY]
	str	r2, [r1, #L2X0_DATA_LATENCY_CTRL]
	ldr	r2, [r0, #L2X0_R_PREFETCH_CTRL]
	str	r2, [r1, #L2X0_PREFETCH_CTRL]
	ldr	r2, [r0, #L2X0_R_PWR_CTRL]
	str	r2, [r1, #L2X0_POWER_CTRL]
	mov	r2, #1
	str	r2, [r1, #L2X0_CTRL]
resume_l2on:
#endif
	b	cpu_resume
ENDPROC(s3c_cpu_resume)
#ifdef CONFIG_CACHE_L2X0
	.globl l2x0_regs_phys
l2x0_regs_phys:
	.long	0
#endif

	/* ID populated by boot code to help in applying fixups */
	.globl s5p_resume_cpu_id
s5p_resume_cpu_id:
	.long	0
