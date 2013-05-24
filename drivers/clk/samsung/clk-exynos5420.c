/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 * Author: Thomas Abraham <thomas.ab@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Common Clock Framework support for Exynos5420 SoC.
*/

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "clk.h"
#include "clk-pll.h"

#define SRC_CPU			0x200
#define DIV_CPU0		0x500
#define DIV_CPU1		0x504
#define GATE_BUS_CPU		0x700
#define GATE_SCLK_CPU		0x800
#define CLKOUT_CMU_CPU		0xa00
#define SRC_TOP0		0x10200
#define SRC_TOP1		0x10204
#define SRC_TOP2		0x10208
#define SRC_TOP3		0x1020c
#define SRC_TOP4		0x10210
#define SRC_TOP5		0x10214
#define SRC_TOP6		0x10218
#define SRC_TOP7		0x1021c
#define SRC_DISP10		0x1022c
#define SRC_MAU			0x10240
#define SRC_FSYS		0x10244
#define SRC_PERIC0		0x10250
#define SRC_PERIC1		0x10254
#define SRC_TOP10		0x10280
#define SRC_TOP11		0x10284
#define SRC_TOP12		0x10288
#define	SRC_MASK_DISP10		0x1032c
#define SRC_MASK_FSYS		0x10340
#define SRC_MASK_PERIC0		0x10350
#define SRC_MASK_PERIC1		0x10354
#define DIV_TOP0		0x10500
#define DIV_TOP1		0x10504
#define DIV_TOP2		0x10508
#define DIV_DISP10		0x1052c
#define DIV_MAU			0x10544
#define DIV_FSYS0		0x10548
#define DIV_FSYS1		0x1054c
#define DIV_FSYS2		0x10550
#define DIV_PERIC0		0x10558
#define DIV_PERIC1		0x1055c
#define DIV_PERIC2		0x10560
#define DIV_PERIC3		0x10564
#define GATE_BUS_TOP		0x10700
#define GATE_BUS_GSCL1		0x10720
#define GATE_BUS_FSYS0		0x10740
#define GATE_BUS_FSYS2		0x10748
#define GATE_BUS_PERIC		0x10750
#define GATE_BUS_PERIC1		0x10754
#define GATE_BUS_PERIS0		0x10760
#define GATE_BUS_PERIS1		0x10764
#define GATE_IP_GSCL0		0x10910
#define GATE_IP_GSCL1		0x10920
#define GATE_IP_MFC		0x1092c
#define GATE_IP_DISP1		0x10928
#define GATE_IP_G3D		0x10930
#define GATE_IP_GEN		0x10934
#define GATE_IP_FSYS		0x10944
#define GATE_IP_PERIC		0x10950
#define GATE_IP_PERIS		0x10960
#define GATE_IP_MSCL		0x10970
#define GATE_TOP_SCLK_GSCL	0x10820
#define GATE_TOP_SCLK_DISP1	0x10828
#define GATE_TOP_SCLK_MAU	0x1083c
#define GATE_TOP_SCLK_FSYS	0x10840
#define GATE_TOP_SCLK_PERIC	0x10850

enum exynos5420_clks {
	none,

	/* core clocks */
	oscclk /* AKA "xxti", AKA "fin_pll" */,

	/* gate for special clocks (sclk) */
	sclk_uart0 = 128, sclk_uart1, sclk_uart2, sclk_uart3, sclk_mmc0 = 132,
	sclk_mmc1, sclk_mmc2, sclk_spi0, sclk_spi1, sclk_spi2, sclk_i2s1,
	sclk_i2s2, sclk_pcm1, sclk_pcm2, sclk_spdif, sclk_hdmi, sclk_pixel,
	sclk_dp1, sclk_mipi1, sclk_fimd1, sclk_maudio0, sclk_maupcm0,
	sclk_usbd300, sclk_usbd301, sclk_usbphy300, sclk_usbphy301, sclk_unipro,
	sclk_pwm, sclk_gscl_wa, sclk_gscl_wb,

	/* gate clocks */
	aclk66_peric = 256, uart0, uart1, uart2, uart3, i2c0, i2c1, i2c2, i2c3,
	i2c4, i2c5, i2c6, i2c7, i2c_hdmi, tsadc, spi0, spi1, spi2, keyif, i2s1,
	i2s2, pcm1, pcm2, pwm, spdif, i2c8, i2c9, i2c10,
	aclk66_psgen = 300, chipid, sysreg, tzpc0, tzpc1, tzpc2, tzpc3, tzpc4,
	tzpc5, tzpc6, tzpc7, tzpc8, tzpc9, hdmi_cec, seckey, mct, wdt, rtc,
	tmu, tmu_gpu,
	pclk66_gpio = 330,
	aclk200_fsys2 = 350, mmc0, mmc1, mmc2, sromc, ufs,
	aclk200_fsys = 360, tsi, pdma0, pdma1, rtic, usbh20, usbd300, usbd301,
	aclk400_mscl = 380, mscl0, mscl1, mscl2, smmu_mscl0, smmu_mscl1,
	smmu_mscl2,
	aclk333 = 400, mfc, smmu_mfcl, smmu_mfcr,
	aclk200_disp1 = 410, dsim1, dp1, hdmi,
	aclk300_disp1 = 420, fimd1,
	aclk166 = 430, mixer,
	aclk266 = 440, rotator, mdma1, smmu_rotator, smmu_mdma1,
	aclk300_jpeg = 450, jpeg, jpeg2, smmu_jpeg,
	aclk300_gscl = 460, smmu_gscl0, smmu_gscl1, gscl_wa, gscl_wb, gscl0,
	gscl1, clk_3aa,
	aclk266_g2d = 470, sss, slim_sss, mdma0,
	aclk333_g2d = 480, g2d,
	aclk333_432_gscl = 490, smmu_3aa, smmu_fimcl0, smmu_fimcl1, smmu_fimcl3,
	fimc_lite3,
	aclk_g3d = 500, g3d,

	nr_clks,
};

/*
 * list of controller registers to be saved and restored during a
 * suspend/resume cycle.
 */
static __initdata unsigned long exynos5420_clk_regs[] = {
	SRC_CPU,
	DIV_CPU0,
	DIV_CPU1,
	GATE_BUS_CPU,
	GATE_SCLK_CPU,
	CLKOUT_CMU_CPU,
	SRC_TOP0,
	SRC_TOP1,
	SRC_TOP2,
	SRC_TOP3,
	SRC_TOP4,
	SRC_TOP5,
	SRC_TOP6,
	SRC_TOP7,
	SRC_DISP10,
	SRC_MAU,
	SRC_FSYS,
	SRC_PERIC0,
	SRC_PERIC1,
	SRC_TOP10,
	SRC_TOP11,
	SRC_TOP12,
	SRC_MASK_DISP10,
	SRC_MASK_FSYS,
	SRC_MASK_PERIC0,
	SRC_MASK_PERIC1,
	DIV_TOP0,
	DIV_TOP1,
	DIV_TOP2,
	DIV_DISP10,
	DIV_MAU,
	DIV_FSYS0,
	DIV_FSYS1,
	DIV_FSYS2,
	DIV_PERIC0,
	DIV_PERIC1,
	DIV_PERIC2,
	DIV_PERIC3,
	GATE_BUS_TOP,
	GATE_BUS_GSCL1,
	GATE_BUS_FSYS0,
	GATE_BUS_FSYS2,
	GATE_BUS_PERIC,
	GATE_BUS_PERIC1,
	GATE_BUS_PERIS0,
	GATE_BUS_PERIS1,
	GATE_IP_GSCL0,
	GATE_IP_GSCL1,
	GATE_IP_MFC,
	GATE_IP_DISP1,
	GATE_IP_G3D,
	GATE_IP_GEN,
	GATE_IP_FSYS,
	GATE_IP_MSCL,
	GATE_TOP_SCLK_GSCL,
	GATE_TOP_SCLK_DISP1,
	GATE_TOP_SCLK_MAU,
	GATE_TOP_SCLK_FSYS,
	GATE_TOP_SCLK_PERIC,
};

/* list of all parent clock list */
PNAME(mout_mspll_cpu_p)		= { "sclk_cpll", "sclk_dpll", "sclk_mpll",
					"sclk_spll" };
PNAME(mout_cpu_p)		= { "mout_apll" , "mout_mspll_cpu" };
PNAME(mout_apll_p)		= { "oscclk", "fout_apll", };
PNAME(mout_cpll_p)		= { "oscclk", "fout_cpll", };
PNAME(mout_dpll_p)		= { "oscclk", "fout_dpll", };
PNAME(mout_epll_p)		= { "oscclk", "fout_epll", };
PNAME(mout_rpll_p)		= { "oscclk", "fout_rpll", };
PNAME(mout_ipll_p)		= { "oscclk", "fout_ipll", };
PNAME(mout_spll_p)		= { "oscclk", "fout_spll", };
PNAME(mout_vpll_p)		= { "oscclk", "fout_vpll", };
PNAME(mout_mpll_p)		= { "oscclk", "fout_mpll", };
PNAME(mout_group1_p)		= { "sclk_cpll", "sclk_dpll", "sclk_mpll" };
PNAME(mout_group2_p)		= { "oscclk", "sclk_cpll", "sclk_dpll",
					"sclk_mpll", "sclk_spll", "sclk_ipll",
					"sclk_epll", "sclk_rpll" };
PNAME(mout_group3_p)		= { "sclk_rpll", "sclk_spll" };
PNAME(mout_group4_p)		= { "sclk_ipll", "sclk_dpll", "sclk_mpll" };
PNAME(mout_group5_p)		= { "sclk_vpll", "sclk_dpll" };

PNAME(mout_sw_aclk66_p)		= { "dout_aclk66", "sclk_spll" };
PNAME(mout_aclk66_peric_p)	= { "oscclk", "mout_sw_aclk66" };

PNAME(mout_sw_aclk200_fsys_p)	= { "dout_aclk200_fsys", "sclk_spll"};
PNAME(mout_user_aclk200_fsys_p)	= { "oscclk", "mout_sw_aclk200_fsys" };
PNAME(mout_sw_aclk200_fsys2_p)	= { "dout_aclk200_fsys2", "sclk_spll"};
PNAME(mout_user_aclk200_fsys2_p) = { "oscclk", "mout_sw_aclk200_fsys2" };

PNAME(mout_sw_aclk200_p)	= { "dout_aclk200", "sclk_spll"};
PNAME(mout_aclk200_disp1_p)	= { "oscclk", "mout_sw_aclk200" };

PNAME(mout_sw_aclk400_mscl_p)	= { "dout_aclk400_mscl", "sclk_spll"};
PNAME(mout_user_aclk400_mscl_p)	= { "oscclk", "mout_sw_aclk400_mscl" };

PNAME(mout_sw_aclk333_p)	= { "dout_aclk333", "sclk_spll"};
PNAME(mout_user_aclk333_p)	= { "oscclk", "mout_sw_aclk333" };

PNAME(mout_sw_aclk166_p)	= { "dout_aclk166", "sclk_spll"};
PNAME(mout_user_aclk166_p)	= { "oscclk", "mout_sw_aclk166" };

PNAME(mout_sw_aclk266_p)	= { "dout_aclk266", "sclk_spll"};
PNAME(mout_user_aclk266_p)	= { "oscclk", "mout_sw_aclk266" };

PNAME(mout_sw_aclk333_432_gscl_p) = { "dout_aclk333_432_gscl", "sclk_spll"};
PNAME(mout_user_aclk333_432_gscl_p) = { "oscclk", "mout_sw_aclk333_432_gscl" };

PNAME(mout_sw_aclk300_gscl_p)	= { "dout_aclk300_gscl", "sclk_spll"};
PNAME(mout_user_aclk300_gscl_p)	= { "oscclk", "mout_sw_aclk300_gscl" };

PNAME(mout_sw_aclk300_disp1_p)	= { "dout_aclk300_disp1", "sclk_spll"};
PNAME(mout_user_aclk300_disp1_p) = { "oscclk", "mout_sw_aclk300_disp1" };

PNAME(mout_sw_aclk300_jpeg_p) = { "dout_aclk300_jpeg", "sclk_spll"};
PNAME(mout_user_aclk300_jpeg_p)	= { "oscclk", "mout_sw_aclk300_jpeg" };

PNAME(mout_sw_aclk_g3d_p)	= { "dout_aclk_g3d", "sclk_spll"};
PNAME(mout_user_aclk_g3d_p)	= { "oscclk", "mout_sw_aclk_g3d" };

PNAME(mout_sw_aclk266_g2d_p)	= { "dout_aclk266_g2d", "sclk_spll"};
PNAME(mout_user_aclk266_g2d_p)	= { "oscclk", "mout_sw_aclk266_g2d" };

PNAME(mout_sw_aclk333_g2d_p)	= { "dout_aclk333_g2d", "sclk_spll"};
PNAME(mout_user_aclk333_g2d_p)	= { "oscclk", "mout_sw_aclk333_g2d" };

PNAME(mout_audio0_p)		= { "oscclk", "cdclk0", "sclk_dpll",
					"sclk_mpll", "sclk_spll", "sclk_ipll",
					"sclk_epll", "sclk_rpll" };
PNAME(mout_audio1_p)		= { "oscclk", "cdclk1", "sclk_dpll",
					"sclk_mpll", "sclk_spll", "sclk_ipll",
					"sclk_epll", "sclk_rpll" };
PNAME(mout_audio2_p)		= { "oscclk", "cdclk2", "sclk_dpll",
					"sclk_mpll", "sclk_spll", "sclk_ipll",
					"sclk_epll", "sclk_rpll" };
PNAME(mout_spdif_p)		= { "oscclk", "dout_audio0", "dout_audio1",
					"dout_audio2", "spdof_extclk",
					"sclk_ipll", "sclk_epll", "sclk_rpll" };
PNAME(mout_hdmi_p)		= { "sclk_hdmiphy", "dout_hdmi_pixel" };
PNAME(mout_maudio0_p)		= { "oscclk", "maudio_clk", "sclk_dpll",
					"sclk_mpll", "sclk_spll", "sclk_ipll",
					"sclk_epll", "sclk_rpll" };

/* fixed rate clocks generated outside the soc */
struct samsung_fixed_rate_clock exynos5420_fixed_rate_ext_clks[] __initdata = {
	FRATE(oscclk, "oscclk", NULL, CLK_IS_ROOT, 0),
};

/* fixed rate clocks generated inside the soc */
struct samsung_fixed_rate_clock exynos5420_fixed_rate_clks[] __initdata = {
	FRATE(none, "sclk_hdmiphy", NULL, CLK_IS_ROOT, 24000000),
	FRATE(none, "sclk_pwi", NULL, CLK_IS_ROOT, 24000000),
	FRATE(none, "sclk_usbh20", NULL, CLK_IS_ROOT, 48000000),
	FRATE(none, "mphy_refclk_ixtal24", NULL, CLK_IS_ROOT, 48000000),
	FRATE(none, "sclk_usbh20_scan_clk", NULL, CLK_IS_ROOT, 480000000),
};

struct samsung_fixed_factor_clock exynos5420_fixed_factor_clks[] __initdata = {
	FFACTOR(none, "sclk_hsic_12m", "oscclk", 1, 2, 0),
};

struct samsung_mux_clock exynos5420_mux_clks[] __initdata = {
	MUX(none, "mout_mspll_cpu", mout_mspll_cpu_p, SRC_TOP7, 12, 2),
	MUX(none, "mout_apll", mout_apll_p, SRC_CPU, 0, 1),
	MUX(none, "mout_cpu", mout_cpu_p, SRC_CPU, 16, 1),
	MUX(none, "sclk_cpll", mout_cpll_p, SRC_TOP6, 28, 1),
	MUX(none, "sclk_dpll", mout_dpll_p, SRC_TOP6, 24, 1),
	MUX(none, "sclk_epll", mout_epll_p, SRC_TOP6, 20, 1),
	MUX(none, "sclk_rpll", mout_rpll_p, SRC_TOP6, 16, 1),
	MUX(none, "sclk_ipll", mout_ipll_p, SRC_TOP6, 12, 1),
	MUX(none, "sclk_spll", mout_spll_p, SRC_TOP6, 8, 1),
	MUX(none, "sclk_vpll", mout_vpll_p, SRC_TOP6, 4, 1),
	MUX(none, "sclk_mpll", mout_mpll_p, SRC_TOP6, 0, 1),
	MUX(none, "mout_aclk200_fsys2", mout_group1_p, SRC_TOP0, 12, 2),
	MUX(none, "mout_sw_aclk200_fsys2", mout_sw_aclk200_fsys2_p,
							SRC_TOP10, 12, 1),
	MUX(none, "mout_user_aclk200_fsys2", mout_user_aclk200_fsys2_p,
							SRC_TOP3, 12, 1),
	MUX(none, "mout_aclk333_432_gscl", mout_group4_p, SRC_TOP1, 0, 2),
	MUX(none, "mout_sw_aclk333_432_gscl", mout_sw_aclk333_432_gscl_p,
							SRC_TOP11, 0, 1),
	MUX(none, "mout_user_aclk333_432_gscl", mout_user_aclk333_432_gscl_p,
							SRC_TOP4, 0, 1),
	MUX(none, "mout_aclk300_gscl", mout_group1_p, SRC_TOP2, 28, 2),
	MUX(none, "mout_sw_aclk300_gscl", mout_sw_aclk300_gscl_p,
							SRC_TOP12, 28, 1),
	MUX(none, "mout_user_aclk300_gscl", mout_user_aclk300_gscl_p,
							SRC_TOP5, 28, 1),
	MUX(none, "mout_aclk200", mout_group1_p, SRC_TOP0, 8, 2),
	MUX(none, "mout_sw_aclk200", mout_sw_aclk200_p, SRC_TOP10, 8, 1),
	MUX(none, "mout_aclk200_disp1", mout_aclk200_disp1_p, SRC_TOP3, 8, 1),
	MUX(none, "mout_aclk66", mout_group1_p, SRC_TOP1, 8, 2),
	MUX(none, "mout_sw_aclk66", mout_sw_aclk66_p, SRC_TOP11, 8, 1),
	MUX(none, "mout_aclk66_peric", mout_aclk66_peric_p, SRC_TOP4, 8, 1),
	MUX(none, "mout_aclk66_psgen", mout_aclk66_peric_p, SRC_TOP5, 4, 1),
	MUX(none, "mout_maudio0", mout_maudio0_p, SRC_MAU, 28, 3),
	MUX(none, "mout_aclk200_fsys", mout_group1_p, SRC_TOP0, 28, 2),
	MUX(none, "mout_sw_aclk200_fsys", mout_sw_aclk200_fsys_p,
							SRC_TOP10, 28, 1),
	MUX(none, "mout_user_aclk200_fsys", mout_user_aclk200_fsys_p,
							SRC_TOP3, 28, 1),
	MUX(none, "mout_aclk400_mscl", mout_group1_p, SRC_TOP0, 4, 2),
	MUX(none, "mout_sw_aclk400_mscl", mout_sw_aclk400_mscl_p,
							SRC_TOP10, 4, 1),
	MUX(none, "mout_user_aclk400_mscl", mout_user_aclk400_mscl_p,
							SRC_TOP3, 4, 1),
	MUX(none, "mout_aclk333", mout_group1_p, SRC_TOP1, 28, 2),
	MUX(none, "mout_sw_aclk333", mout_sw_aclk333_p, SRC_TOP11, 28, 1),
	MUX(none, "mout_user_aclk333", mout_user_aclk333_p, SRC_TOP4, 28, 1),
	MUX(none, "mout_aclk166", mout_group1_p, SRC_TOP1, 24, 2),
	MUX(none, "mout_sw_aclk166", mout_sw_aclk166_p, SRC_TOP11, 24, 1),
	MUX(none, "mout_user_aclk166", mout_user_aclk166_p, SRC_TOP4, 24, 1),
	MUX(none, "mout_aclk266", mout_group1_p, SRC_TOP1, 20, 2),
	MUX(none, "mout_sw_aclk266", mout_sw_aclk266_p, SRC_TOP11, 20, 1),
	MUX(none, "mout_user_aclk266", mout_user_aclk266_p, SRC_TOP4, 20, 1),
	MUX(none, "mout_aclk300_disp1", mout_group1_p, SRC_TOP2, 24, 2),
	MUX(none, "mout_sw_aclk300_disp1", mout_sw_aclk300_disp1_p,
							SRC_TOP12, 24, 1),
	MUX(none, "mout_user_aclk300_disp1", mout_user_aclk300_disp1_p,
							SRC_TOP5, 24, 1),
	MUX(none, "mout_aclk300_jpeg", mout_group1_p, SRC_TOP2, 20, 2),
	MUX(none, "mout_sw_aclk300_jpeg", mout_sw_aclk300_jpeg_p,
							SRC_TOP12, 20, 1),
	MUX(none, "mout_user_aclk300_jpeg", mout_user_aclk300_jpeg_p,
							SRC_TOP5, 20, 1),
	MUX(none, "mout_aclk_g3d", mout_group5_p, SRC_TOP2, 16, 2),
	MUX(none, "mout_sw_aclk_g3d", mout_sw_aclk_g3d_p, SRC_TOP12, 16, 1),
	MUX(aclk_g3d, "mout_user_aclk_g3d", mout_user_aclk_g3d_p,
							SRC_TOP5, 16, 1),
	MUX(none, "mout_aclk266_g2d", mout_group1_p, SRC_TOP2, 12, 2),
	MUX(none, "mout_sw_aclk266_g2d", mout_sw_aclk266_g2d_p,
							SRC_TOP12, 12, 1),
	MUX(none, "mout_user_aclk266_g2d", mout_user_aclk266_g2d_p,
							SRC_TOP5, 12, 1),
	MUX(none, "mout_aclk333_g2d", mout_group1_p, SRC_TOP2, 8, 2),
	MUX(none, "mout_sw_aclk333_g2d", mout_sw_aclk333_g2d_p,
							SRC_TOP12, 8, 1),
	MUX(none, "mout_user_aclk333_g2d", mout_user_aclk333_g2d_p,
							SRC_TOP5, 8, 1),
	/* PERIC Blk */
	MUX(none, "mout_uart0", mout_group2_p, SRC_PERIC0, 4, 3),
	MUX(none, "mout_uart1", mout_group2_p, SRC_PERIC0, 8, 3),
	MUX(none, "mout_uart2", mout_group2_p, SRC_PERIC0, 12, 3),
	MUX(none, "mout_uart3", mout_group2_p, SRC_PERIC0, 16, 3),
	MUX(none, "mout_pwm", mout_group2_p, SRC_PERIC0, 24, 3),
	MUX(none, "mout_spdif", mout_spdif_p, SRC_PERIC0, 28, 3),
	MUX(none, "mout_audio0", mout_audio0_p, SRC_PERIC1, 8, 3),
	MUX(none, "mout_audio1", mout_audio1_p, SRC_PERIC1, 12, 3),
	MUX(none, "mout_audio2", mout_audio2_p, SRC_PERIC1, 16, 3),
	MUX(none, "mout_spi0", mout_group2_p, SRC_PERIC1, 20, 3),
	MUX(none, "mout_spi1", mout_group2_p, SRC_PERIC1, 24, 3),
	MUX(none, "mout_spi2", mout_group2_p, SRC_PERIC1, 28, 3),
	/*FSYS Blk */
	MUX(none, "mout_mmc0", mout_group2_p, SRC_FSYS, 8, 3),
	MUX(none, "mout_mmc1", mout_group2_p, SRC_FSYS, 12, 3),
	MUX(none, "mout_mmc2", mout_group2_p, SRC_FSYS, 16, 3),
	MUX(none, "mout_unipro", mout_group2_p, SRC_FSYS, 24, 3),
	MUX(none, "mout_usbd300", mout_group2_p, SRC_FSYS, 20, 3),
	MUX(none, "mout_usbd301", mout_group2_p, SRC_FSYS, 4, 3),
	/* Disp1 Block*/
	MUX(none, "mout_fimd1", mout_group3_p, SRC_DISP10, 4, 3),
	MUX(none, "mout_mipi1", mout_group2_p, SRC_DISP10, 16, 3),
	MUX(none, "mout_dp1", mout_group2_p, SRC_DISP10, 20, 3),
	MUX(none, "mout_pixel", mout_group2_p, SRC_DISP10, 24, 3),
	MUX(none, "mout_hdmi", mout_hdmi_p, SRC_DISP10, 28, 1),
};

struct samsung_div_clock exynos5420_div_clks[] __initdata = {
	DIV(none, "div_arm", "mout_cpu", DIV_CPU0, 0, 3),
	DIV(none, "armclk2", "div_arm", DIV_CPU0, 28, 3),
	DIV(none, "sclk_apll", "mout_apll", DIV_CPU0, 24, 3),
	DIV(none, "dout_aclk400_mscl", "mout_aclk400_mscl", DIV_TOP0, 4, 3),
	DIV(none, "dout_aclk200", "mout_aclk200", DIV_TOP0, 8, 3),
	DIV(none, "dout_aclk200_fsys2", "mout_aclk200_fsys2", DIV_TOP0, 12, 3),
	DIV(none, "dout_pclk200_fsys", "mout_pclk200_fsys", DIV_TOP0, 24, 3),
	DIV(none, "dout_aclk200_fsys", "mout_aclk200_fsys", DIV_TOP0, 28, 3),
	DIV(none, "dout_aclk333_432_gscl", "mout_aclk333_432_gscl",
							DIV_TOP1, 0, 2),
	DIV(none, "dout_aclk66", "mout_aclk66", DIV_TOP1, 8, 6),
	DIV(none, "dout_aclk266", "mout_aclk266", DIV_TOP1, 20, 3),
	DIV(none, "dout_aclk166", "mout_aclk166", DIV_TOP1, 24, 3),
	DIV(none, "dout_aclk333", "mout_aclk333", DIV_TOP1, 28, 3),
	DIV(none, "dout_aclk333_g2d", "mout_aclk333_g2d", DIV_TOP2, 8, 2),
	DIV(none, "dout_aclk266_g2d", "mout_aclk266_g2d", DIV_TOP2, 12, 2),
	DIV(none, "dout_aclk_g3d", "mout_aclk_g3d", DIV_TOP2, 16, 2),
	DIV(none, "dout_aclk300_jpeg", "mout_aclk300_jpeg", DIV_TOP2, 20, 2),
	DIV(none, "dout_aclk300_disp1", "mout_aclk300_disp1", DIV_TOP2, 24, 2),
	DIV(none, "dout_aclk300_gscl", "mout_aclk300_gscl", DIV_TOP2, 28, 2),
	DIV(none, "dout_uart0", "mout_uart0", DIV_PERIC0, 8, 4),
	DIV(none, "dout_uart1", "mout_uart1", DIV_PERIC0, 12, 4),
	DIV(none, "dout_uart2", "mout_uart2", DIV_PERIC0, 16, 4),
	DIV(none, "dout_uart3", "mout_uart3", DIV_PERIC0, 20, 4),
	DIV(none, "dout_pwm", "mout_pwm", DIV_PERIC0, 28, 4),
	/* spi */
	DIV(none, "dout_spi0", "mout_spi0", DIV_PERIC1, 20, 4),
	DIV(none, "dout_spi1", "mout_spi1", DIV_PERIC1, 24, 4),
	DIV(none, "dout_spi2", "mout_spi2", DIV_PERIC1, 28, 4),
	/* audio -i2s */
	DIV(none, "dout_audio0", "mout_audio0", DIV_PERIC3, 20, 4),
	DIV(none, "dout_audio1", "mout_audio1", DIV_PERIC3, 24, 4),
	DIV(none, "dout_audio2", "mout_audio2", DIV_PERIC3, 28, 4),
	/* pcm */
	DIV(none, "dout_pcm1", "dout_audio1", DIV_PERIC2, 16, 8),
	DIV(none, "dout_pcm2", "dout_audio2", DIV_PERIC2, 24, 8),
	/* i2s */
	DIV(none, "dout_i2s1", "dout_audio1", DIV_PERIC3, 6, 6),
	DIV(none, "dout_i2s2", "dout_audio2", DIV_PERIC3, 12, 6),
	/* mmc */
	DIV(none, "dout_mmc0", "mout_mmc0", DIV_FSYS1, 0, 10),
	DIV(none, "dout_mmc1", "mout_mmc1", DIV_FSYS1, 10, 10),
	DIV(none, "dout_mmc2", "mout_mmc2", DIV_FSYS1, 20, 10),
	/* usbd - unipro*/
	DIV(none, "dout_usbphy300", "mout_usbd300", DIV_FSYS0, 16, 4),
	DIV(none, "dout_usbphy301", "mout_usbd301", DIV_FSYS0, 12, 4),
	DIV(none, "dout_usbd301", "mout_usbd301", DIV_FSYS0, 20, 4),
	DIV(none, "dout_usbd300", "mout_usbd300", DIV_FSYS0, 24, 4),
	DIV(none, "dout_unipro", "mout_unipro", DIV_FSYS2, 24, 4),
	/* Display */
	DIV(none, "dout_fimd1", "mout_fimd1", DIV_DISP10, 0, 4),
	DIV(none, "dout_mipi1", "mout_mipi1", DIV_DISP10, 16, 8),
	DIV(none, "dout_dp1", "mout_dp1", DIV_DISP10, 24, 4),
	DIV(none, "dout_hdmi_pixel", "mout_pixel", DIV_DISP10, 28, 4),
	/* Audio Blk */
	DIV(none, "dout_maudio0", "mout_maudio0", DIV_MAU, 20, 4),
	DIV(none, "dout_maupcm0", "dout_maudio0", DIV_MAU, 24, 8),
};

struct samsung_gate_clock exynos5420_gate_clks[] __initdata = {
	GATE(mct, "pclk_st", "aclk66_psgen", GATE_BUS_PERIS1, 2, 0, 0),
	/* Check how to use with CG_STATUS_bits */
	GATE(aclk200_disp1, "aclk200_disp1", "mout_aclk200_disp1",
					GATE_BUS_TOP, 18, CLK_IGNORE_UNUSED, 0),
	GATE(aclk200_fsys2, "aclk200_fsys2", "mout_user_aclk200_fsys2",
				GATE_BUS_FSYS0, 10, CLK_IGNORE_UNUSED, 0),
	GATE(aclk200_fsys, "aclk200_fsys", "mout_user_aclk200_fsys",
				GATE_BUS_FSYS0, 9, CLK_IGNORE_UNUSED, 0),
	GATE(aclk66_peric, "aclk66_peric", "mout_aclk66_peric",
					SRC_TOP4, 8, CLK_IGNORE_UNUSED, 0),
	GATE(aclk66_psgen, "aclk66_psgen", "mout_aclk66_psgen",
					GATE_BUS_TOP, 10, CLK_IGNORE_UNUSED, 0),
	GATE(pclk66_gpio, "pclk66_gpio", "mout_sw_aclk66",
					GATE_BUS_TOP, 9, CLK_IGNORE_UNUSED, 0),
	/* UART */
	GATE(uart0, "uart0", NULL, GATE_IP_PERIC, 0, CLK_IGNORE_UNUSED, 0),
	GATE(uart1, "uart1", NULL, GATE_IP_PERIC, 1, CLK_IGNORE_UNUSED, 0),
	GATE(uart2, "uart2", NULL, GATE_IP_PERIC, 2, CLK_IGNORE_UNUSED, 0),
	GATE(uart3, "uart3", NULL, GATE_IP_PERIC, 3, CLK_IGNORE_UNUSED, 0),
	GATE(sclk_uart0, "sclk_uart0", "dout_uart0", SRC_MASK_PERIC0, 4,
							CLK_IGNORE_UNUSED, 0),
	GATE(sclk_uart1, "sclk_uart1", "dout_uart1", SRC_MASK_PERIC0, 8,
							CLK_IGNORE_UNUSED, 0),
	GATE(sclk_uart2, "sclk_uart2", "dout_uart2", SRC_MASK_PERIC0, 12,
							CLK_IGNORE_UNUSED, 0),
	GATE(sclk_uart3, "sclk_uart3", "dout_uart3", SRC_MASK_PERIC0,
						16, CLK_IGNORE_UNUSED, 0),
	/*spdif*/
	GATE(spdif, "spdif", "aclk66_peric", GATE_BUS_PERIC,
						29, CLK_IGNORE_UNUSED, 0),
	GATE(sclk_spdif, "sclk_spdif", "mout_spdif", SRC_MASK_PERIC0,
						28, CLK_IGNORE_UNUSED, 0),
	GATE(sclk_unipro, "sclk_unipro", "dout_unipro",
		SRC_MASK_FSYS, 24, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_gscl_wa, "sclk_gscl_wa", "aclK333_432_gscl",
		GATE_TOP_SCLK_GSCL, 6, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_gscl_wb, "sclk_gscl_wb", "aclk333_432_gscl",
		GATE_TOP_SCLK_GSCL, 7, CLK_SET_RATE_PARENT, 0),
	/* Display */
	GATE(sclk_fimd1, "sclk_fimd1", "dout_fimd1",
		GATE_TOP_SCLK_DISP1, 0, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_mipi1, "sclk_mipi1", "dout_mipi1",
		GATE_TOP_SCLK_DISP1, 3, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_dp1, "sclk_dp1", "dout_dp1",
		GATE_TOP_SCLK_DISP1, 20, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_pixel, "sclk_pixel", "dout_hdmi_pixel",
		GATE_TOP_SCLK_DISP1, 10, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_hdmi, "sclk_hdmi", "mout_hdmi",
		GATE_TOP_SCLK_DISP1, 9, CLK_SET_RATE_PARENT, 0),
	/* i2c */
	GATE(i2c0, "i2c0", "aclk66_peric", GATE_IP_PERIC, 6, 0, 0),
	GATE(i2c1, "i2c1", "aclk66_peric", GATE_IP_PERIC, 7, 0, 0),
	GATE(i2c2, "i2c2", "aclk66_peric", GATE_IP_PERIC, 8, 0, 0),
	GATE(i2c3, "i2c3", "aclk66_peric", GATE_IP_PERIC, 9, 0, 0),
	GATE(i2c4, "i2c4", "aclk66_peric", GATE_IP_PERIC, 10, 0, 0),
	GATE(i2c5, "i2c5", "aclk66_peric", GATE_IP_PERIC, 11, 0, 0),
	GATE(i2c6, "i2c6", "aclk66_peric", GATE_IP_PERIC, 12, 0, 0),
	GATE(i2c7, "i2c7", "aclk66_peric", GATE_IP_PERIC, 13, 0, 0),
	GATE(i2c8, "i2c8", "aclk66_peric", GATE_IP_PERIC, 28, 0, 0),
	GATE(i2c9, "i2c9", "aclk66_peric", GATE_IP_PERIC, 30, 0, 0),
	GATE(i2c10, "i2c10", "aclk66_peric", GATE_IP_PERIC, 31, 0, 0),
	/* SPI */
	GATE(spi0, "spi0", "aclk66_peric", GATE_BUS_PERIC, 19, 0, 0),
	GATE(spi1, "spi1", "aclk66_peric", GATE_BUS_PERIC, 20, 0, 0),
	GATE(spi2, "spi2", "aclk66_peric", GATE_BUS_PERIC, 21, 0, 0),
	GATE(sclk_spi0, "sclk_spi0", "dout_spi0", SRC_MASK_PERIC1, 20,
						CLK_SET_RATE_PARENT, 0),
	GATE(sclk_spi1, "sclk_spi1", "dout_spi1", SRC_MASK_PERIC1, 24,
						CLK_SET_RATE_PARENT, 0),
	GATE(sclk_spi2, "sclk_spi2", "dout_spi2", SRC_MASK_PERIC1, 28,
						CLK_SET_RATE_PARENT, 0),
	/*i2s */
	GATE(i2s1, "i2s1", "aclk66_peric", GATE_BUS_PERIC, 23, 0, 0),
	GATE(i2s2, "i2s2", "aclk66_peric", GATE_BUS_PERIC, 24, 0, 0),
	GATE(sclk_i2s1, "sclk_i2s1", "dout_i2s1", SRC_MASK_PERIC1, 12, 0, 0),
	GATE(sclk_i2s2, "sclk_i2s2", "dout_i2s2", SRC_MASK_PERIC1, 16, 0, 0),
	/*pcm */
	GATE(pcm1, "pcm1", "aclk66_peric", GATE_BUS_PERIC, 26, 0, 0),
	GATE(pcm2, "pcm2", "aclk66_peric", GATE_BUS_PERIC, 27, 0, 0),
	GATE(sclk_pcm1, "sclk_pcm1", "dout_pcm1", SRC_MASK_PERIC1, 12, 0, 0),
	GATE(sclk_pcm2, "sclk_pcm2", "dout_pcm2", SRC_MASK_PERIC1, 16, 0, 0),
	GATE(wdt, "wdt", "aclk66_psgen", GATE_BUS_PERIS1, 3, 0, 0),
	GATE(rtc, "rtc", "aclk66_psgen", GATE_BUS_PERIS1, 4, 0, 0),

	/* MMC */
	GATE(mmc0, "mmc0", "aclk200_fsys2", GATE_BUS_FSYS0, 12,
							CLK_IGNORE_UNUSED, 0),
	GATE(mmc1, "mmc1", "aclk200_fsys2", GATE_BUS_FSYS0, 13,
							CLK_IGNORE_UNUSED, 0),
	GATE(mmc2, "mmc2", "aclk200_fsys2", GATE_BUS_FSYS0, 14,
							CLK_IGNORE_UNUSED, 0),
	GATE(sclk_mmc0, "sclk_mmc0", "dout_mmc0", SRC_MASK_FSYS, 8,
							CLK_IGNORE_UNUSED, 0),
	GATE(sclk_mmc1, "sclk_mmc1", "dout_mmc1", SRC_MASK_FSYS, 12,
							CLK_IGNORE_UNUSED, 0),
	GATE(sclk_mmc2, "sclk_mmc2", "dout_mmc2", SRC_MASK_FSYS, 16,
							CLK_IGNORE_UNUSED, 0),

	GATE(mscl0, "mscl0", "aclk400_mscl", GATE_IP_MSCL, 0, 0, 0),
	GATE(mscl1, "mscl1", "aclk400_mscl", GATE_IP_MSCL, 1, 0, 0),
	GATE(mscl2, "mscl2", "aclk400_mscl", GATE_IP_MSCL, 2, 0, 0),
	GATE(smmu_mscl0, "smmu_mscl0", "aclk400_mscl", GATE_IP_MSCL, 8, 0, 0),
	GATE(smmu_mscl1, "smmu_mscl1", "aclk400_mscl", GATE_IP_MSCL, 9, 0, 0),
	GATE(smmu_mscl2, "smmu_mscl2", "aclk400_mscl", GATE_IP_MSCL, 10, 0, 0),
	GATE(mfc, "mfc", "aclk333", GATE_IP_MFC, 0, 0, 0),
	GATE(smmu_mfcl, "smmu_mfcl", "aclk333", GATE_IP_MFC, 1, 0, 0),
	GATE(smmu_mfcr, "smmu_mfcr", "aclk333", GATE_IP_MFC, 2, 0, 0),
	GATE(smmu_3aa, "smmu_3aa", "aclk333_432_gscl", GATE_IP_GSCL1, 2, 0, 0),
	GATE(smmu_fimcl0, "smmu_fimcl0", "aclk333_432_gscl",
							GATE_IP_GSCL1, 3, 0, 0),
	GATE(smmu_fimcl1, "smmu_fimcl1", "aclk333_432_gscl",
							GATE_IP_GSCL1, 4, 0, 0),
	GATE(smmu_gscl0, "smmu_gscl0", "aclk300_gscl", GATE_IP_GSCL1, 6, 0, 0),
	GATE(smmu_gscl1, "smmu_gscl1", "aclk300_gscl", GATE_IP_GSCL1, 7, 0, 0),
	GATE(gscl_wa, "gscl_wa", "aclk300_gscl", GATE_IP_GSCL1, 12, 0, 0),
	GATE(gscl_wb, "gscl_wb", "aclk300_gscl", GATE_IP_GSCL1, 13, 0, 0),
	GATE(smmu_fimcl3, "smmu_fimcl3,", "aclk333_432_gscl",
						GATE_IP_GSCL1, 16, 0, 0),
	GATE(fimc_lite3, "fimc_lite3", "aclk333_432_gscl",
						GATE_IP_GSCL1, 17, 0, 0),
	GATE(gscl0, "gscl0", "aclk300_gscl", GATE_IP_GSCL0, 0, 0, 0),
	GATE(gscl1, "gscl1", "aclk300_gscl", GATE_IP_GSCL0, 1, 0, 0),
	GATE(clk_3aa, "clk_3aa", "aclk300_gscl", GATE_IP_GSCL0, 2, 0, 0),

	GATE(fimd1, "fimd1", "aclk300_disp1", GATE_IP_DISP1, 0, 0, 0),
	GATE(dsim1, "dsim1", "aclk200_disp1", GATE_IP_DISP1, 3, 0, 0),
	GATE(dp1, "dp1", "aclk200_disp1", GATE_IP_DISP1, 4, 0, 0),
	GATE(mixer, "mixer", "aclk166", GATE_IP_DISP1, 5, 0, 0),
	GATE(hdmi, "hdmi", "aclk200_disp1", GATE_IP_DISP1, 6, 0, 0),

	GATE(rotator, "rotator", "aclk266", GATE_IP_GEN, 1, 0, 0),
	GATE(jpeg, "jpeg", "aclk300_jpeg", GATE_IP_GEN, 2, 0, 0),
	GATE(jpeg2, "jpeg2", "aclk300_jpeg", GATE_IP_GEN, 3, 0, 0),
	GATE(mdma1, "mdma1", "aclk266", GATE_IP_GEN, 4, 0, 0),
	GATE(smmu_rotator, "smmu_rotator", "aclk266", GATE_IP_GEN, 6, 0, 0),
	GATE(smmu_jpeg, "smmu_jpeg", "aclk300_jpeg", GATE_IP_GEN, 7, 0, 0),
	GATE(smmu_mdma1, "smmu_mdma1", "aclk266", GATE_IP_GEN, 9, 0, 0),
	GATE(g3d, "g3d", "mout_user_aclk_g3d", GATE_IP_G3D, 9, 0, 0),
	GATE(usbh20, "usbh20", "aclk200_fsys", GATE_IP_FSYS, 18, 0, 0),
	GATE(usbd300, "usbd300", "aclk200_fsys", GATE_IP_FSYS, 19, 0, 0),
	GATE(usbd301, "usbd301", "aclk200_fsys", GATE_IP_FSYS, 20, 0, 0),
};

static __initdata struct of_device_id ext_clk_match[] = {
	{ .compatible = "samsung,clock-xxti", .data = (void *)0, },
	{ },
};

/* register exynox5420 clocks */
void __init exynos5420_clk_init(struct device_node *np)
{
	void __iomem *reg_base;
	struct clk *apll, *bpll, *cpll, *dpll, *ipll, *mpll, *spll, *vpll;

	if (np) {
		reg_base = of_iomap(np, 0);
		if (!reg_base)
			panic("%s: failed to map registers\n", __func__);
	} else {
		panic("%s: unable to determine soc\n", __func__);
	}

	samsung_clk_init(np, reg_base, nr_clks,
			exynos5420_clk_regs, ARRAY_SIZE(exynos5420_clk_regs),
			NULL, 0);
	samsung_clk_of_register_fixed_ext(exynos5420_fixed_rate_ext_clks,
			ARRAY_SIZE(exynos5420_fixed_rate_ext_clks),
			ext_clk_match);

	apll = samsung_clk_register_pll35xx("fout_apll", "oscclk",
			reg_base + 0x100);
	bpll = samsung_clk_register_pll35xx("fout_bpll", "oscclk",
			reg_base + 0x20110);
	cpll = samsung_clk_register_pll35xx("fout_cpll", "oscclk",
			reg_base + 0x10120);
	dpll = samsung_clk_register_pll35xx("fout_dpll", "oscclk",
			reg_base + 0x10128);
	ipll = samsung_clk_register_pll35xx("fout_ipll", "oscclk",
			reg_base + 0x10150);
	mpll = samsung_clk_register_pll35xx("fout_mpll", "oscclk",
			reg_base + 0x10180);
	spll = samsung_clk_register_pll35xx("fout_spll", "oscclk",
			reg_base + 0x10160);
	vpll = samsung_clk_register_pll35xx("fout_vpll", "oscclk",
			reg_base + 0x10170);

	samsung_clk_register_fixed_rate(exynos5420_fixed_rate_clks,
			ARRAY_SIZE(exynos5420_fixed_rate_clks));
	samsung_clk_register_fixed_factor(exynos5420_fixed_factor_clks,
			ARRAY_SIZE(exynos5420_fixed_factor_clks));
	samsung_clk_register_mux(exynos5420_mux_clks,
			ARRAY_SIZE(exynos5420_mux_clks));
	samsung_clk_register_div(exynos5420_div_clks,
			ARRAY_SIZE(exynos5420_div_clks));
	samsung_clk_register_gate(exynos5420_gate_clks,
			ARRAY_SIZE(exynos5420_gate_clks));

	pr_info("Exynos5420: clock setup completed\n");
}
CLK_OF_DECLARE(exynos5420_clk, "samsung,exynos5420-clock", exynos5420_clk_init);
