/*
 * Copyright (c) 2011-2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS - CPU PMU(Power Management Unit) support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/cpumask.h>

#include <mach/regs-clock.h>
#include <mach/pmu.h>
#include <asm/cputype.h>

static struct exynos_pmu_conf *exynos_pmu_config;

static struct exynos_pmu_conf exynos4210_pmu_config[] = {
	/* { .reg = address, .val = { AFTR, LPA, SLEEP } */
	{ S5P_ARM_CORE0_LOWPWR,			{ 0x0, 0x0, 0x2 } },
	{ S5P_DIS_IRQ_CORE0,			{ 0x0, 0x0, 0x0 } },
	{ S5P_DIS_IRQ_CENTRAL0,			{ 0x0, 0x0, 0x0 } },
	{ S5P_ARM_CORE1_LOWPWR,			{ 0x0, 0x0, 0x2 } },
	{ S5P_DIS_IRQ_CORE1,			{ 0x0, 0x0, 0x0 } },
	{ S5P_DIS_IRQ_CENTRAL1,			{ 0x0, 0x0, 0x0 } },
	{ S5P_ARM_COMMON_LOWPWR,		{ 0x0, 0x0, 0x2 } },
	{ S5P_L2_0_LOWPWR,			{ 0x2, 0x2, 0x3 } },
	{ S5P_L2_1_LOWPWR,			{ 0x2, 0x2, 0x3 } },
	{ S5P_CMU_ACLKSTOP_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_SCLKSTOP_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_LOWPWR,			{ 0x1, 0x1, 0x0 } },
	{ S5P_APLL_SYSCLK_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_MPLL_SYSCLK_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_VPLL_SYSCLK_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_EPLL_SYSCLK_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_CLKSTOP_GPS_ALIVE_LOWPWR,	{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_RESET_GPSALIVE_LOWPWR,	{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_CLKSTOP_CAM_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_CLKSTOP_TV_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_CLKSTOP_MFC_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_CLKSTOP_G3D_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_CLKSTOP_LCD0_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_CLKSTOP_LCD1_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_CLKSTOP_MAUDIO_LOWPWR,	{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_CLKSTOP_GPS_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_RESET_CAM_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_RESET_TV_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_RESET_MFC_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_RESET_G3D_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_RESET_LCD0_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_RESET_LCD1_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_RESET_MAUDIO_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_RESET_GPS_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_TOP_BUS_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_TOP_RETENTION_LOWPWR,		{ 0x1, 0x0, 0x1 } },
	{ S5P_TOP_PWR_LOWPWR,			{ 0x3, 0x0, 0x3 } },
	{ S5P_LOGIC_RESET_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_ONENAND_MEM_LOWPWR,		{ 0x3, 0x0, 0x0 } },
	{ S5P_MODIMIF_MEM_LOWPWR,		{ 0x3, 0x0, 0x0 } },
	{ S5P_G2D_ACP_MEM_LOWPWR,		{ 0x3, 0x0, 0x0 } },
	{ S5P_USBOTG_MEM_LOWPWR,		{ 0x3, 0x0, 0x0 } },
	{ S5P_HSMMC_MEM_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_CSSYS_MEM_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_SECSS_MEM_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_PCIE_MEM_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_SATA_MEM_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_DRAM_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_MAUDIO_LOWPWR,	{ 0x1, 0x1, 0x0 } },
	{ S5P_PAD_RETENTION_GPIO_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_UART_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_MMCA_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_MMCB_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_EBIA_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_EBIB_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_ISOLATION_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_ALV_SEL_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_XUSBXTI_LOWPWR,			{ 0x1, 0x1, 0x0 } },
	{ S5P_XXTI_LOWPWR,			{ 0x1, 0x1, 0x0 } },
	{ S5P_EXT_REGULATOR_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_GPIO_MODE_LOWPWR,			{ 0x1, 0x0, 0x0 } },
	{ S5P_GPIO_MODE_MAUDIO_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CAM_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_TV_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_MFC_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_G3D_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_LCD0_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_LCD1_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_MAUDIO_LOWPWR,			{ 0x7, 0x7, 0x0 } },
	{ S5P_GPS_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_GPS_ALIVE_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ PMU_TABLE_END,},
};

static struct exynos_pmu_conf exynos4x12_pmu_config[] = {
	{ S5P_ARM_CORE0_LOWPWR,			{ 0x0, 0x0, 0x2 } },
	{ S5P_DIS_IRQ_CORE0,			{ 0x0, 0x0, 0x0 } },
	{ S5P_DIS_IRQ_CENTRAL0,			{ 0x0, 0x0, 0x0 } },
	{ S5P_ARM_CORE1_LOWPWR,			{ 0x0, 0x0, 0x2 } },
	{ S5P_DIS_IRQ_CORE1,			{ 0x0, 0x0, 0x0 } },
	{ S5P_DIS_IRQ_CENTRAL1,			{ 0x0, 0x0, 0x0 } },
	{ S5P_ISP_ARM_LOWPWR,			{ 0x1, 0x0, 0x0 } },
	{ S5P_DIS_IRQ_ISP_ARM_LOCAL_LOWPWR,	{ 0x0, 0x0, 0x0 } },
	{ S5P_DIS_IRQ_ISP_ARM_CENTRAL_LOWPWR,	{ 0x0, 0x0, 0x0 } },
	{ S5P_ARM_COMMON_LOWPWR,		{ 0x0, 0x0, 0x2 } },
	{ S5P_L2_0_LOWPWR,			{ 0x0, 0x0, 0x3 } },
	/* XXX_OPTION register should be set other field */
	{ S5P_ARM_L2_0_OPTION,			{ 0x10, 0x10, 0x0 } },
	{ S5P_L2_1_LOWPWR,			{ 0x0, 0x0, 0x3 } },
	{ S5P_ARM_L2_1_OPTION,			{ 0x10, 0x10, 0x0 } },
	{ S5P_CMU_ACLKSTOP_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_SCLKSTOP_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_LOWPWR,			{ 0x1, 0x1, 0x0 } },
	{ S5P_DRAM_FREQ_DOWN_LOWPWR,		{ 0x1, 0x1, 0x1 } },
	{ S5P_DDRPHY_DLLOFF_LOWPWR,		{ 0x1, 0x1, 0x1 } },
	{ S5P_LPDDR_PHY_DLL_LOCK_LOWPWR,	{ 0x1, 0x1, 0x1 } },
	{ S5P_CMU_ACLKSTOP_COREBLK_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_SCLKSTOP_COREBLK_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_COREBLK_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_APLL_SYSCLK_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_MPLL_SYSCLK_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_VPLL_SYSCLK_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_EPLL_SYSCLK_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_MPLLUSER_SYSCLK_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_CLKSTOP_GPS_ALIVE_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_GPSALIVE_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_CLKSTOP_CAM_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_CLKSTOP_TV_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_CLKSTOP_MFC_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_CLKSTOP_G3D_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_CLKSTOP_LCD0_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_CLKSTOP_ISP_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_CLKSTOP_MAUDIO_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_CLKSTOP_GPS_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_CAM_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_TV_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_MFC_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_G3D_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_LCD0_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_ISP_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_RESET_MAUDIO_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_CMU_RESET_GPS_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_TOP_BUS_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_TOP_RETENTION_LOWPWR,		{ 0x1, 0x0, 0x1 } },
	{ S5P_TOP_PWR_LOWPWR,			{ 0x3, 0x0, 0x3 } },
	{ S5P_TOP_BUS_COREBLK_LOWPWR,		{ 0x3, 0x0, 0x0 } },
	{ S5P_TOP_RETENTION_COREBLK_LOWPWR,	{ 0x1, 0x0, 0x1 } },
	{ S5P_TOP_PWR_COREBLK_LOWPWR,		{ 0x3, 0x0, 0x3 } },
	{ S5P_LOGIC_RESET_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_OSCCLK_GATE_LOWPWR,		{ 0x1, 0x0, 0x1 } },
	{ S5P_LOGIC_RESET_COREBLK_LOWPWR,	{ 0x1, 0x1, 0x0 } },
	{ S5P_OSCCLK_GATE_COREBLK_LOWPWR,	{ 0x1, 0x0, 0x1 } },
	{ S5P_ONENAND_MEM_LOWPWR,		{ 0x3, 0x0, 0x0 } },
	{ S5P_ONENAND_MEM_OPTION,		{ 0x10, 0x10, 0x0 } },
	{ S5P_HSI_MEM_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_HSI_MEM_OPTION,			{ 0x10, 0x10, 0x0 } },
	{ S5P_G2D_ACP_MEM_LOWPWR,		{ 0x3, 0x0, 0x0 } },
	{ S5P_G2D_ACP_MEM_OPTION,		{ 0x10, 0x10, 0x0 } },
	{ S5P_USBOTG_MEM_LOWPWR,		{ 0x3, 0x0, 0x0 } },
	{ S5P_USBOTG_MEM_OPTION,		{ 0x10, 0x10, 0x0 } },
	{ S5P_HSMMC_MEM_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_HSMMC_MEM_OPTION,			{ 0x10, 0x10, 0x0 } },
	{ S5P_CSSYS_MEM_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_CSSYS_MEM_OPTION,			{ 0x10, 0x10, 0x0 } },
	{ S5P_SECSS_MEM_LOWPWR,			{ 0x3, 0x0, 0x0 } },
	{ S5P_SECSS_MEM_OPTION,			{ 0x10, 0x10, 0x0 } },
	{ S5P_ROTATOR_MEM_LOWPWR,		{ 0x3, 0x0, 0x0 } },
	{ S5P_ROTATOR_MEM_OPTION,		{ 0x10, 0x10, 0x0 } },
	{ S5P_PAD_RETENTION_DRAM_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_MAUDIO_LOWPWR,	{ 0x1, 0x1, 0x0 } },
	{ S5P_PAD_RETENTION_GPIO_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_UART_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_MMCA_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_MMCB_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_EBIA_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_EBIB_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_GPIO_COREBLK_LOWPWR,{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_ISOLATION_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_ISOLATION_COREBLK_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_PAD_RETENTION_ALV_SEL_LOWPWR,	{ 0x1, 0x0, 0x0 } },
	{ S5P_XUSBXTI_LOWPWR,			{ 0x1, 0x1, 0x0 } },
	{ S5P_XXTI_LOWPWR,			{ 0x1, 0x1, 0x0 } },
	{ S5P_EXT_REGULATOR_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_GPIO_MODE_LOWPWR,			{ 0x1, 0x0, 0x0 } },
	{ S5P_GPIO_MODE_COREBLK_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_GPIO_MODE_MAUDIO_LOWPWR,		{ 0x1, 0x1, 0x0 } },
	{ S5P_TOP_ASB_RESET_LOWPWR,		{ 0x1, 0x1, 0x1 } },
	{ S5P_TOP_ASB_ISOLATION_LOWPWR,		{ 0x1, 0x0, 0x1 } },
	{ S5P_CAM_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_TV_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_MFC_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_G3D_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_LCD0_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_ISP_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_MAUDIO_LOWPWR,			{ 0x7, 0x7, 0x0 } },
	{ S5P_GPS_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_GPS_ALIVE_LOWPWR,			{ 0x7, 0x0, 0x0 } },
	{ S5P_CMU_SYSCLK_ISP_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ S5P_CMU_SYSCLK_GPS_LOWPWR,		{ 0x1, 0x0, 0x0 } },
	{ PMU_TABLE_END,},
};

static struct exynos_pmu_conf exynos4412_pmu_config[] = {
	{ S5P_ARM_CORE2_LOWPWR,			{ 0x0, 0x0, 0x2 } },
	{ S5P_DIS_IRQ_CORE2,			{ 0x0, 0x0, 0x0 } },
	{ S5P_DIS_IRQ_CENTRAL2,			{ 0x0, 0x0, 0x0 } },
	{ S5P_ARM_CORE3_LOWPWR,			{ 0x0, 0x0, 0x2 } },
	{ S5P_DIS_IRQ_CORE3,			{ 0x0, 0x0, 0x0 } },
	{ S5P_DIS_IRQ_CENTRAL3,			{ 0x0, 0x0, 0x0 } },
	{ PMU_TABLE_END,},
};

static struct exynos_pmu_conf exynos5250_pmu_config[] = {
	/* { .reg = address, .val = { AFTR, LPA, SLEEP } */
	{ EXYNOS5_ARM_CORE0_SYS_PWR_REG,		{ 0x0, 0x0, 0x2} },
	{ EXYNOS5_DIS_IRQ_ARM_CORE0_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_ARM_CORE0_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_ARM_CORE1_SYS_PWR_REG,		{ 0x0, 0x0, 0x2} },
	{ EXYNOS5_DIS_IRQ_ARM_CORE1_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_ARM_CORE1_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_FSYS_ARM_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_FSYS_ARM_CENTRAL_SYS_PWR_REG,	{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_ISP_ARM_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_ISP_ARM_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_ISP_ARM_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_ARM_COMMON_SYS_PWR_REG,		{ 0x0, 0x0, 0x2} },
	{ EXYNOS5_ARM_L2_SYS_PWR_REG,			{ 0x3, 0x3, 0x3} },
	{ EXYNOS5_CMU_ACLKSTOP_SYS_PWR_REG,		{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_CMU_SCLKSTOP_SYS_PWR_REG,		{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_CMU_RESET_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_CMU_ACLKSTOP_SYSMEM_SYS_PWR_REG,	{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_CMU_SCLKSTOP_SYSMEM_SYS_PWR_REG,	{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_CMU_RESET_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_DRAM_FREQ_DOWN_SYS_PWR_REG,		{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_DDRPHY_DLLOFF_SYS_PWR_REG,		{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_DDRPHY_DLLLOCK_SYS_PWR_REG,		{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_APLL_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_MPLL_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_VPLL_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_EPLL_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_BPLL_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CPLL_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_MPLLUSER_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_BPLLUSER_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_TOP_BUS_SYS_PWR_REG,			{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_TOP_RETENTION_SYS_PWR_REG,		{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_TOP_PWR_SYS_PWR_REG,			{ 0x3, 0x0, 0x3} },
	{ EXYNOS5_TOP_BUS_SYSMEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_TOP_RETENTION_SYSMEM_SYS_PWR_REG,	{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_TOP_PWR_SYSMEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x3} },
	{ EXYNOS5_LOGIC_RESET_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_OSCCLK_GATE_SYS_PWR_REG,		{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_LOGIC_RESET_SYSMEM_SYS_PWR_REG,	{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_OSCCLK_GATE_SYSMEM_SYS_PWR_REG,	{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_USBOTG_MEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_G2D_MEM_SYS_PWR_REG,			{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_USBDRD_MEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_SDMMC_MEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_CSSYS_MEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_SECSS_MEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_ROTATOR_MEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_INTRAM_MEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_INTROM_MEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_JPEG_MEM_SYS_PWR_REG,			{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_HSI_MEM_SYS_PWR_REG,			{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_MCUIOP_MEM_SYS_PWR_REG,		{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_SATA_MEM_SYS_PWR_REG,			{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_DRAM_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_MAU_SYS_PWR_REG,	{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_PAD_RETENTION_GPIO_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_UART_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_MMCA_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_MMCB_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_EBIA_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_EBIB_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_SPI_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_GPIO_SYSMEM_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_ISOLATION_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_ISOLATION_SYSMEM_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_ALV_SEL_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_XUSBXTI_SYS_PWR_REG,			{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_XXTI_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_EXT_REGULATOR_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_GPIO_MODE_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_GPIO_MODE_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_GPIO_MODE_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_TOP_ASB_RESET_SYS_PWR_REG,		{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_TOP_ASB_ISOLATION_SYS_PWR_REG,	{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_GSCL_SYS_PWR_REG,			{ 0x7, 0x0, 0x0} },
	{ EXYNOS5_ISP_SYS_PWR_REG,			{ 0x7, 0x0, 0x0} },
	{ EXYNOS5_MFC_SYS_PWR_REG,			{ 0x7, 0x0, 0x0} },
	{ EXYNOS5_G3D_SYS_PWR_REG,			{ 0x7, 0x0, 0x0} },
	{ EXYNOS5_DISP1_SYS_PWR_REG,			{ 0x7, 0x0, 0x0} },
	{ EXYNOS5_MAU_SYS_PWR_REG,			{ 0x7, 0x7, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_GSCL_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_ISP_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_MFC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_G3D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_DISP1_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_GSCL_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_ISP_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_MFC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_G3D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_CMU_RESET_GSCL_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	/* CMU_RESET_ISP_SYS_PWR_REG handled in exynos5250_disable_isp() */
	{ EXYNOS5_CMU_RESET_MFC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_G3D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ PMU_TABLE_END,},
};

static struct exynos_pmu_conf exynos5420_pmu_config[] = {
	/* { .reg = address, .val = { AFTR, LPA, SLEEP } */
	{ EXYNOS5_ARM_CORE0_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_ARM_CORE0_LOCAL_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_ARM_CORE0_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_ARM_CORE1_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_ARM_CORE1_LOCAL_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_ARM_CORE1_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_ARM_CORE2_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_ARM_CORE2_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_ARM_CORE2_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_ARM_CORE3_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_ARM_CORE3_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_ARM_CORE3_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_KFC_CORE0_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_KFC_CORE0_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_KFC_CORE0_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_KFC_CORE1_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_KFC_CORE1_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_KFC_CORE1_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_KFC_CORE2_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_KFC_CORE2_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_KFC_CORE2_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_KFC_CORE3_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_KFC_CORE3_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_DIS_IRQ_KFC_CORE3_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_ISP_ARM_SYS_PWR_REG,				{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_ISP_ARM_LOCAL_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_DIS_IRQ_ISP_ARM_CENTRAL_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_ARM_COMMON_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_KFC_COMMON_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_ARM_L2_SYS_PWR_REG,				{ 0x0, 0x0, 0x0} },
	{ EXYNOS5420_KFC_L2_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_ACLKSTOP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SCLKSTOP_SYS_PWR_REG,			{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_CMU_RESET_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_CMU_ACLKSTOP_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SCLKSTOP_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_CMU_RESET_SYSMEM_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_DRAM_FREQ_DOWN_SYS_PWR_REG,			{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_DDRPHY_DLLOFF_SYS_PWR_REG,			{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_DDRPHY_DLLLOCK_SYS_PWR_REG,			{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_APLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_MPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_VPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_EPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_BPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_DPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_IPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_KPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_MPLLUSER_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_BPLLUSER_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_RPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_SPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_TOP_BUS_SYS_PWR_REG,				{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_TOP_RETENTION_SYS_PWR_REG,			{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_TOP_PWR_SYS_PWR_REG,				{ 0x3, 0x3, 0x0} },
	{ EXYNOS5_TOP_BUS_SYSMEM_SYS_PWR_REG,			{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_TOP_RETENTION_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_TOP_PWR_SYSMEM_SYS_PWR_REG,			{ 0x3, 0x0, 0x0} },
	{ EXYNOS5_LOGIC_RESET_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_OSCCLK_GATE_SYS_PWR_REG,			{ 0x1, 0x0, 0x1} },
	{ EXYNOS5_LOGIC_RESET_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_OSCCLK_GATE_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_INTRAM_MEM_SYS_PWR_REG,			{ 0x3, 0x0, 0x3} },
	{ EXYNOS5420_INTROM_MEM_SYS_PWR_REG,			{ 0x3, 0x0, 0x3} },
	{ EXYNOS5_PAD_RETENTION_DRAM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_JTAG_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_DRAM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_UART_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_MMC0_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_MMC1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_MMC2_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_HSI_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_EBIA_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_EBIB_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_SPI_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_PAD_RETENTION_DRAM_COREBLK_SYS_PWR_REG,	{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_ISOLATION_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_PAD_ISOLATION_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_PAD_ALV_SEL_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_XUSBXTI_SYS_PWR_REG,				{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_XXTI_SYS_PWR_REG,				{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_EXT_REGULATOR_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_GPIO_MODE_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_GPIO_MODE_SYSMEM_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_GPIO_MODE_MAU_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_TOP_ASB_RESET_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_TOP_ASB_ISOLATION_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_GSCL_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5_ISP_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5_MFC_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5_G3D_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5420_DISP1_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5420_MAU_SYS_PWR_REG,				{ 0x7, 0x7, 0x0} },
	{ EXYNOS5420_G2D_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5420_MSC_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5420_FSYS_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5420_FSYS2_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5420_PSGEN_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5420_PERIC_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5420_WCORE_SYS_PWR_REG,				{ 0x7, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_GSCL_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_ISP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_MFC_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_G3D_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_CLKSTOP_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_CLKSTOP_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5420_CMU_CLKSTOP_G2D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_CLKSTOP_MSC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_CLKSTOP_FSYS_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_CLKSTOP_PSGEN_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_CLKSTOP_PERIC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_CLKSTOP_WCORE_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_GSCL_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_ISP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_MFC_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_G3D_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_SYSCLK_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_SYSCLK_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5420_CMU_SYSCLK_G2D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_SYSCLK_MSC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_SYSCLK_FSYS_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_SYSCLK_FSYS2_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_SYSCLK_PSGEN_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_SYSCLK_PERIC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_SYSCLK_WCORE_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_RESET_FSYS2_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_RESET_PSGEN_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_RESET_PERIC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_RESET_WCORE_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_GSCL_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_ISP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_MFC_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_G3D_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_RESET_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_RESET_MAU_SYS_PWR_REG,			{ 0x1, 0x1, 0x0} },
	{ EXYNOS5420_CMU_RESET_G2D_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_RESET_MSC_SYS_PWR_REG,			{ 0x1, 0x0, 0x0} },
	{ EXYNOS5420_CMU_RESET_FSYS_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ PMU_TABLE_END,},
};

static void __iomem *exynos5_list_both_cnt_feed[] = {
	EXYNOS5_ARM_CORE0_OPTION,
	EXYNOS5_ARM_CORE1_OPTION,
	EXYNOS5_ARM_COMMON_OPTION,
	EXYNOS5_GSCL_OPTION,
	EXYNOS5_ISP_OPTION,
	EXYNOS5_MFC_OPTION,
	EXYNOS5_G3D_OPTION,
	EXYNOS5_DISP1_OPTION,
	EXYNOS5_MAU_OPTION,
	EXYNOS5_TOP_PWR_OPTION,
	EXYNOS5_TOP_PWR_SYSMEM_OPTION,
};

static void __iomem *exynos5_list_diable_wfi_wfe[] = {
	EXYNOS5_ARM_CORE1_OPTION,
	EXYNOS5_FSYS_ARM_OPTION,
	EXYNOS5_ISP_ARM_OPTION,
};

void __iomem *exynos5_list_disable_pmu_reg[] = {
	EXYNOS5_CMU_CLKSTOP_ISP_SYS_PWR_REG,
	EXYNOS5_CMU_SYSCLK_ISP_SYS_PWR_REG,
	EXYNOS5_CMU_RESET_ISP_SYS_PWR_REG,
};

static void exynos5_power_off(void)
{
	unsigned int tmp;

	pr_info("Power down.\n");
	tmp = __raw_readl(EXYNOS5_PS_HOLD_CONTROL);
	tmp &= ~(1 << 8);
	__raw_writel(tmp, EXYNOS5_PS_HOLD_CONTROL);

	/* Wait a little so we don't give a false warning below */
	mdelay(100);

	pr_err("Power down failed, please power off system manually.\n");
	while (1)
		;
}

static void exynos5_debug_enable_uart_wakeup(void)
{
#ifdef CONFIG_SAMSUNG_PM_DEBUG
	unsigned int tmp;

	/* Enable UART automatic wakeup for resume console output */
	tmp = __raw_readl(S5P_PAD_RET_UART_OPTION);
	tmp |= EXYNOS5_PAD_RET_UART_AUTOMATIC_WAKEUP;
	__raw_writel(tmp, S5P_PAD_RET_UART_OPTION);
#endif
}

#define ISP_DISABLE_TRIES			10

/*
 * Disable the image signal processor.
 *
 * We currently have no code in the kernel to manage the state of the ISP.
 *
 * The ISP's power sequencing code needs to be run in a very specific order
 * and shouldn't necessarily be intertwined with the power on/power off code
 * of the main CPU core.  Until there is kernel code to manage the ISP, we'll
 * just hardcode powering off the ISP here.
 */
static void exynos5250_disable_isp(void)
{
	int i;

	/* Make sure ISP ARM is disabled; don't use WFI or WFE */
	__raw_writel(0, EXYNOS5_ISP_ARM_OPTION);

	/* Put the ISP ARM in reset */
	__raw_writel(0x0, EXYNOS5_ISP_ARM_CONFIGURATION);
	for (i = 0; i < ISP_DISABLE_TRIES; i++) {
		if (!(__raw_readl(EXYNOS5_ISP_ARM_STATUS) & 0x1))
			break;
		usleep_range(80, 100);
	}
	WARN_ON(i == ISP_DISABLE_TRIES);

	/* Reset the ISP CMU block in power-off/low power state */
	__raw_writel(0x0, EXYNOS5_CMU_RESET_ISP_SYS_PWR_REG);

	/* Turn off power to the ISP in normal mode */
	__raw_writel(0x0, EXYNOS5_ISP_CONFIGURATION);
	for (i = 0; i < ISP_DISABLE_TRIES; i++) {
		if (!(__raw_readl(EXYNOS5_ISP_STATUS) & 0x7))
			break;
		usleep_range(80, 100);
	}
	WARN_ON(i == ISP_DISABLE_TRIES);
}

static void exynos5_init_pmu(void)
{
	unsigned int i;
	unsigned int tmp;

	/*
	 * Enable both SC_FEEDBACK and SC_COUNTER
	 */
	for (i = 0 ; i < ARRAY_SIZE(exynos5_list_both_cnt_feed) ; i++) {
		tmp = __raw_readl(exynos5_list_both_cnt_feed[i]);
		tmp |= (EXYNOS5_USE_SC_FEEDBACK |
			EXYNOS5_USE_SC_COUNTER);
		__raw_writel(tmp, exynos5_list_both_cnt_feed[i]);
	}

	/*
	 * SKIP_DEACTIVATE_ACEACP_IN_PWDN_BITFIELD Enable
	 * MANUAL_L2RSTDISABLE_CONTROL_BITFIELD Enable
	 */
	tmp = __raw_readl(EXYNOS5_ARM_COMMON_OPTION);
	tmp |= (EXYNOS5_MANUAL_L2RSTDISABLE_CONTROL |
		EXYNOS5_SKIP_DEACTIVATE_ACEACP_IN_PWDN);
	__raw_writel(tmp, EXYNOS5_ARM_COMMON_OPTION);

	/*
	 * Disable WFI/WFE on XXX_OPTION
	 */
	for (i = 0 ; i < ARRAY_SIZE(exynos5_list_diable_wfi_wfe) ; i++) {
		tmp = __raw_readl(exynos5_list_diable_wfi_wfe[i]);
		tmp &= ~(EXYNOS5_OPTION_USE_STANDBYWFE |
			 EXYNOS5_OPTION_USE_STANDBYWFI);
		__raw_writel(tmp, exynos5_list_diable_wfi_wfe[i]);
	}

	exynos5_debug_enable_uart_wakeup();
}

/*
 * exynos_set_core_flag - set the cluster id to IROM register
 *			  to ensure that we wake up with the
 *			  current cluster.
 */
static void exynos_set_core_flag(void)
{
	int cluster_id = (read_cpuid_mpidr() >> 8) & 0xf;

	if (cluster_id)
		__raw_writel(1, EXYNOS_IROM_DATA2);
	else
		__raw_writel(0, EXYNOS_IROM_DATA2);
}

void exynos_sys_powerdown_conf(enum sys_powerdown mode)
{
	unsigned int i;

	if (soc_is_exynos5250())
		exynos5_init_pmu();

	if (soc_is_exynos5420())
		exynos_set_core_flag();

	for (i = 0; (exynos_pmu_config[i].reg != PMU_TABLE_END) ; i++)
		__raw_writel(exynos_pmu_config[i].val[mode],
				exynos_pmu_config[i].reg);

	if (soc_is_exynos4412()) {
		for (i = 0; exynos4412_pmu_config[i].reg != PMU_TABLE_END ; i++)
			__raw_writel(exynos4412_pmu_config[i].val[mode],
				exynos4412_pmu_config[i].reg);
	}
}

static int __init exynos_pmu_init(void)
{
	unsigned int value;

	exynos_pmu_config = exynos4210_pmu_config;

	if (soc_is_exynos4210()) {
		exynos_pmu_config = exynos4210_pmu_config;
		pr_info("EXYNOS4210 PMU Initialize\n");
	} else if (soc_is_exynos4212() || soc_is_exynos4412()) {
		exynos_pmu_config = exynos4x12_pmu_config;
		pr_info("EXYNOS4x12 PMU Initialize\n");
	} else if (soc_is_exynos5250()) {
		exynos5250_disable_isp();
		/*
		 * When SYS_WDTRESET is set, watchdog timer reset request
		 * is ignored by power management unit.
		 */
		value = __raw_readl(EXYNOS5_AUTO_WDTRESET_DISABLE);
		value &= ~EXYNOS5_SYS_WDTRESET;
		__raw_writel(value, EXYNOS5_AUTO_WDTRESET_DISABLE);

		value = __raw_readl(EXYNOS5_MASK_WDTRESET_REQUEST);
		value &= ~EXYNOS5_SYS_WDTRESET;
		__raw_writel(value, EXYNOS5_MASK_WDTRESET_REQUEST);

		exynos_pmu_config = exynos5250_pmu_config;
		pm_power_off = exynos5_power_off;
		pr_info("EXYNOS5250 PMU Initialize\n");
	} else if (soc_is_exynos5420()) {
		__raw_writel(0xfffff, EXYNOS5_XXTI_DURATION3);

		/* Mask and disable watchdog timer reset request */
		__raw_writel(0xffffffff, EXYNOS5_AUTO_WDTRESET_DISABLE);
		__raw_writel(0xffffffff, EXYNOS5_MASK_WDTRESET_REQUEST);

		/* Enable USE_STANDBY_WFI for all CORE */
		__raw_writel(EXYNOS5420_USE_STANDBY_WFI_ALL,
				S5P_CENTRAL_SEQ_OPTION);

		value = __raw_readl(EXYNOS_L2_OPTION(0));
		value &= ~EXYNOS5_USE_RETENTION;
		__raw_writel(value, EXYNOS_L2_OPTION(0));

		value = __raw_readl(EXYNOS_L2_OPTION(1));
		value &= ~EXYNOS5_USE_RETENTION;
		__raw_writel(value, EXYNOS_L2_OPTION(1));

		/*
		 * If L2_COMMON is turned off, clocks related to ATB async
		 * bridge are gated. Thus, when ISP power is gated, LPI
		 * may get stuck.
		 */
		value = __raw_readl(EXYNOS5420_LPI_MASK);
		value |= EXYNOS5420_ATB_ISP_ARM;
		__raw_writel(value, EXYNOS5420_LPI_MASK);
		value = __raw_readl(EXYNOS5420_LPI_MASK1);
		value |= EXYNOS5420_ATB_KFC;
		__raw_writel(value, EXYNOS5420_LPI_MASK1);

		/* Prevent issue of new bus request from L2 memory */
		value = __raw_readl(EXYNOS5420_ARM_COMMON_OPTION);
		value |= EXYNOS5_SKIP_DEACTIVATE_ACEACP_IN_PWDN;
		__raw_writel(value, EXYNOS5420_ARM_COMMON_OPTION);

		/*
		 * This setting is to reduce suspend/resume time.
		 */
		__raw_writel(DUR_WAIT_RESET, EXYNOS5420_LOGIC_RESET_DURATION3);

		/* Serialized CPU wakeup of Eagle */
		__raw_writel(SPREAD_ENABLE, EXYNOS5420_ARM_INTR_SPREAD_ENABLE);
		__raw_writel(SPREAD_USE_STANDWFI,
			EXYNOS5420_ARM_INTR_SPREAD_USE_STANDBYWFI);
		__raw_writel(0x1, EXYNOS5420_UP_SCHEDULER);

		exynos_pmu_config = exynos5420_pmu_config;
		pr_info("EXYNOS5420 PMU Initialized\n");
	} else {
		pr_info("EXYNOS: PMU not supported\n");
	}

	return 0;
}
arch_initcall(exynos_pmu_init);
