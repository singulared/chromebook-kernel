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
	 * tag ram setup and data ram setup and bump up latency of both tag and
	 * data ram by 1 which will fix things.
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
	orr	r0, r0, #0xc3
	mcr	p15, 1, r0, c9, c0, 2

resume_after_542x_l2_cache:

	/*
	 * On Exynos5422, the A15 L2 auto clock gating needs to be re-enabled
	 * since u-boot/bl2 would have disabled it. For Exynos5420, the L2 auto
	 * clock gating should continue to be disabled.
	 */

	/* L2 auto clock gating should be re-enabled only for Exynos5422 */
	ldr	r0, s5p_resume_cpu_id
	lsr	r0, r0, #12
	ldr	r1, =0xe5422
	cmp	r0, r1
	bne	skip_l2_auto_clock_gating

	/* Is this A15 cluster. If not, this step is not applicable. */
	mrc	p15, 0, r0, c0, c0, 0
	ubfx	r0, r0, #4, #12
	ldr	r1, =0x00000c0f
	teq	r0, r1
	bne	skip_l2_auto_clock_gating

	/*
	 * Set CMU_SPARE1 bits 1 and 3 register as suggested by hardware manual
	 * before re-enabling L2 Auto clock gating.
	 */
	adr	r2, cmu_spare1_reg
	ldr	r2, [r2]
	ldr	r3, [r2]
	orr	r3, r3, #0xa
	str	r3, [r2]
	dsb

	/* Re-enable auto clock gating, bl2/u-boot would have disabled it */
	mrc	p15, 1, r2, c15, c0, 0	/* L2 ACTLR register, bit 27 */
	bic	r2, r2, #0x08000000
	mcr	p15, 1, r2, c15, c0, 0
	dsb
	isb
skip_l2_auto_clock_gating:

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

	.align	4
cmu_spare1_reg:
	.word	0x10012004	/* SFR address of CMU_SPARE1 register */
