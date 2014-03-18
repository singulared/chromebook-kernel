/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/opp.h>
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/devfreq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/pm_qos.h>
#include <linux/reboot.h>
#include <linux/kobject.h>
#include <linux/of.h>

#include <mach/regs-clock.h>
#include <mach/devfreq.h>
#include <mach/asv-exynos.h>

#include <plat/pll.h>

#include "exynos5420_ppmu.h"

#define INT_VOLT_STEP_UV		12500

static struct device *int_dev;

static struct pm_qos_request exynos5_int_qos;
static struct pm_qos_request boot_int_qos;
static struct int_pm_clks **exynos5_int_pm_clks;

cputime64_t int_pre_time;

enum int_bus_idx {
	LV_0 = 0,
	LV_1,
	LV_2,
	LV_3,
	LV_4,
	LV_5,
	LV_6,
	LV_END,
};

enum int_bus_pll {
	S_PLL = 0,
	C_PLL,
	D_PLL,
	M_PLL,
	I_PLL,
};

struct int_bus_opp_table {
	unsigned int idx;
	unsigned long freq;
	unsigned long volt;
	cputime64_t time_in_state;
};

struct int_bus_opp_table int_bus_opp_list[] = {
	{LV_0, 600000, 1050000, 0},
	{LV_1, 500000, 1000000, 0},
	{LV_2, 400000, 1000000, 0},
	{LV_3, 333000,  950000, 0},
	{LV_4, 222000,  900000, 0},
	{LV_5, 111000,  850000, 0},
	{LV_6,  83000,  850000, 0},
};

struct int_clk_info {
	unsigned int idx;
	unsigned long target_freq;
	enum int_bus_pll src_pll;
};

struct int_pm_clks {
	struct list_head node;
	const char *mux_clk_name;   /* The parent of the div clock */
	struct clk *mux_clk;
	const char *div_clk_name;
	struct clk *div_clk;
	struct int_clk_info *clk_info;
};

struct busfreq_data_int {
	struct list_head list;
	struct device *dev;
	struct devfreq *devfreq;
	struct opp *curr_opp;
	struct mutex lock;
	struct clk *mout_mpll;
	struct clk *mout_dpll;
	struct clk *mout_cpll;
	struct clk *mout_spll;
	struct clk *mout_ipll;
	struct regulator *vdd_int;
	struct exynos5420_ppmu_handle *ppmu;
	int busy;
};

/* TOP 0 */
struct int_clk_info aclk_200_fsys[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 200000, D_PLL},
	{LV_1, 200000, D_PLL},
	{LV_2, 200000, D_PLL},
	{LV_3, 200000, D_PLL},
	{LV_4, 150000, D_PLL},
	{LV_5, 100000, D_PLL},
	{LV_6, 100000, D_PLL},
};

struct int_clk_info aclk_333_5422[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 400000, M_PLL},
	{LV_1, 400000, M_PLL},
	{LV_2, 333000, C_PLL},
	{LV_3, 222000, C_PLL},
	{LV_4, 167000, C_PLL},
	{LV_5, 111000, C_PLL},
	{LV_6,  96000, C_PLL},
};

struct int_clk_info aclk_300_gscl_5422[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 300000, D_PLL},
	{LV_1, 300000, D_PLL},
	{LV_2, 300000, D_PLL},
	{LV_3, 300000, D_PLL},
	{LV_4, 200000, D_PLL},
	{LV_5, 150000, D_PLL},
	{LV_6, 150000, D_PLL},
};

struct int_clk_info pclk_200_fsys[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 200000, D_PLL},
	{LV_1, 200000, D_PLL},
	{LV_2, 200000, D_PLL},
	{LV_3, 150000, D_PLL},
	{LV_4, 150000, D_PLL},
	{LV_5, 100000, D_PLL},
	{LV_6, 100000, D_PLL},
};

struct int_clk_info pclk_200_fsys_5422[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 200000, D_PLL},
	{LV_1, 200000, D_PLL},
	{LV_2, 200000, D_PLL},
	{LV_3, 100000, D_PLL},
	{LV_4, 100000, D_PLL},
	{LV_5, 100000, D_PLL},
	{LV_6, 100000, D_PLL},
};

struct int_clk_info aclk_100_noc[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 100000, D_PLL},
	{LV_1, 100000, D_PLL},
	{LV_2, 100000, D_PLL},
	{LV_3,  86000, D_PLL},
	{LV_4,  75000, D_PLL},
	{LV_5,  75000, D_PLL},
	{LV_6,  75000, D_PLL},
};

struct int_clk_info aclk_100_noc_5422[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 100000, D_PLL},
	{LV_1, 100000, D_PLL},
	{LV_2, 100000, D_PLL},
	{LV_3,  86000, D_PLL},
	{LV_4,  75000, D_PLL},
	{LV_5,  60000, D_PLL},
	{LV_6,  60000, D_PLL},
};

struct int_clk_info aclk_400_wcore[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 400000, S_PLL},
	{LV_1, 400000, S_PLL},
	{LV_2, 400000, S_PLL},
	{LV_3, 333000, C_PLL},
	{LV_4, 222000, C_PLL},
	{LV_5, 111000, C_PLL},
	{LV_6,  84000, C_PLL},
};

struct int_clk_info aclk_400_wcore_5422[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 400000, M_PLL},
	{LV_1, 400000, M_PLL},
	{LV_2, 400000, M_PLL},
	{LV_3, 333000, C_PLL},
	{LV_4, 222000, C_PLL},
	{LV_5, 111000, C_PLL},
	{LV_6,  84000, C_PLL},
};

struct int_clk_info aclk_200_fsys2[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 200000, D_PLL},
	{LV_1, 200000, D_PLL},
	{LV_2, 200000, D_PLL},
	{LV_3, 200000, D_PLL},
	{LV_4, 150000, D_PLL},
	{LV_5, 100000, D_PLL},
	{LV_6, 100000, D_PLL},
};

struct int_clk_info aclk_200_fsys2_5422[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 120000, D_PLL},
	{LV_1, 120000, D_PLL},
	{LV_2, 120000, D_PLL},
	{LV_3, 120000, D_PLL},
	{LV_4, 120000, D_PLL},
	{LV_5, 100000, D_PLL},
	{LV_6,  75000, D_PLL},
};

struct int_clk_info aclk_200_disp1[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 200000, D_PLL},
	{LV_1, 200000, D_PLL},
	{LV_2, 200000, D_PLL},
	{LV_3, 200000, D_PLL},
	{LV_4, 150000, D_PLL},
	{LV_5, 100000, D_PLL},
	{LV_6, 100000, D_PLL},
};

struct int_clk_info aclk_200_disp1_5422[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 200000, D_PLL},
	{LV_1, 200000, D_PLL},
	{LV_2, 200000, D_PLL},
	{LV_3, 150000, D_PLL},
	{LV_4, 150000, D_PLL},
	{LV_5, 100000, D_PLL},
	{LV_6, 100000, D_PLL},
};

struct int_clk_info aclk_400_mscl[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 400000, S_PLL},
	{LV_1, 400000, S_PLL},
	{LV_2, 400000, S_PLL},
	{LV_3, 333000, C_PLL},
	{LV_4, 222000, C_PLL},
	{LV_5, 167000, C_PLL},
	{LV_6,  84000, C_PLL},
};

struct int_clk_info aclk_400_mscl_5422[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 400000, M_PLL},
	{LV_1, 400000, M_PLL},
	{LV_2, 333000, C_PLL},
	{LV_3, 333000, C_PLL},
	{LV_4, 222000, C_PLL},
	{LV_5, 167000, C_PLL},
	{LV_6,  84000, C_PLL},
};

struct int_clk_info aclk_400_isp[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 400000, S_PLL},
	{LV_1, 400000, S_PLL},
	{LV_2, 100000, M_PLL},
	{LV_3, 100000, M_PLL},
	{LV_4, 100000, M_PLL},
	{LV_5, 100000, M_PLL},
	{LV_6, 100000, M_PLL},
};

/* TOP 1 */
struct int_clk_info aclk_166[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 167000, C_PLL},
	{LV_1, 167000, C_PLL},
	{LV_2, 167000, C_PLL},
	{LV_3, 134000, C_PLL},
	{LV_4, 111000, C_PLL},
	{LV_5,  84000, C_PLL},
	{LV_6,  84000, C_PLL},
};

struct int_clk_info aclk_266[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 267000, M_PLL},
	{LV_1, 267000, M_PLL},
	{LV_2, 267000, M_PLL},
	{LV_3, 160000, M_PLL},
	{LV_4, 134000, M_PLL},
	{LV_5, 134000, M_PLL},
	{LV_6,  86000, D_PLL},
};

struct int_clk_info aclk_266_5422[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 267000, M_PLL},
	{LV_1, 267000, M_PLL},
	{LV_2, 267000, M_PLL},
	{LV_3, 160000, M_PLL},
	{LV_4, 134000, M_PLL},
	{LV_5, 134000, M_PLL},
	{LV_6,  75000, D_PLL},
};

struct int_clk_info aclk_66[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0,  67000, C_PLL},
	{LV_1,  67000, C_PLL},
	{LV_2,  67000, C_PLL},
	{LV_3,  67000, C_PLL},
	{LV_4,  67000, C_PLL},
	{LV_5,  67000, C_PLL},
	{LV_6,  67000, C_PLL},
};

struct int_clk_info aclk_333_432_isp[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 432000, I_PLL},
	{LV_1, 144000, I_PLL},
	{LV_2,   3000, I_PLL},
	{LV_3,   3000, I_PLL},
	{LV_4,   3000, I_PLL},
	{LV_5,   3000, I_PLL},
	{LV_6,   3000, I_PLL},
};

struct int_clk_info aclk_333_432_gscl[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 432000, I_PLL},
	{LV_1, 432000, I_PLL},
	{LV_2,   3000, I_PLL},
	{LV_3,   3000, I_PLL},
	{LV_4,   3000, I_PLL},
	{LV_5,   3000, I_PLL},
	{LV_6,   3000, I_PLL},
};

/* TOP 2 */
struct int_clk_info aclk_300_disp1[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 200000, D_PLL},
	{LV_1, 200000, D_PLL},
	{LV_2, 200000, D_PLL},
	{LV_3, 200000, D_PLL},
	{LV_4, 200000, D_PLL},
	{LV_5, 200000, D_PLL},
	{LV_6, 120000, D_PLL},
};

struct int_clk_info aclk_400_disp1[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 300000, D_PLL},
	{LV_1, 300000, D_PLL},
	{LV_2, 300000, D_PLL},
	{LV_3, 300000, D_PLL},
	{LV_4, 200000, D_PLL},
	{LV_5, 200000, D_PLL},
	{LV_6, 120000, D_PLL},
};

struct int_clk_info aclk_300_jpeg[] = {
	/* Level, Freq, Parent_Pll */
	{LV_0, 300000, D_PLL},
	{LV_1, 300000, D_PLL},
	{LV_2, 300000, D_PLL},
	{LV_3, 300000, D_PLL},
	{LV_4, 200000, D_PLL},
	{LV_5, 150000, D_PLL},
	{LV_6,  75000, D_PLL},
};

#define EXYNOS5_INT_PM_CLK(NAME, CLK, PCLK, CLK_INFO)	\
static struct int_pm_clks int_pm_clks_##NAME = {	\
	.mux_clk_name = CLK,				\
	.div_clk_name = PCLK,				\
	.clk_info = CLK_INFO,				\
}

EXYNOS5_INT_PM_CLK(aclk_200_fsys, "aclk200_fsys",
			"aclk200_fsys_d", aclk_200_fsys);
EXYNOS5_INT_PM_CLK(pclk_200_fsys, "pclk200_fsys",
			"pclk200_fsys_d", pclk_200_fsys);
EXYNOS5_INT_PM_CLK(aclk_100_noc, "aclk100_noc",
			"aclk100_noc_d", aclk_100_noc);
EXYNOS5_INT_PM_CLK(aclk_400_wcore, "aclk400_wcore",
			"aclk400_wcore_d", aclk_400_wcore);
EXYNOS5_INT_PM_CLK(aclk_200_fsys2, "aclk200_fsys2",
			"aclk200_fsys2_d", aclk_200_fsys2);
EXYNOS5_INT_PM_CLK(aclk_400_mscl, "aclk400_mscl",
			"aclk400_mscl_d", aclk_400_mscl);
EXYNOS5_INT_PM_CLK(aclk_400_isp, "aclk400_isp",
			"aclk400_isp_d", aclk_400_isp);
EXYNOS5_INT_PM_CLK(aclk_166, "aclk166",
			"aclk166_d", aclk_166);
EXYNOS5_INT_PM_CLK(aclk_266, "aclk266",
			"aclk266_d", aclk_266);
EXYNOS5_INT_PM_CLK(aclk_66, "aclk66",
			"aclk66_d", aclk_66);
EXYNOS5_INT_PM_CLK(aclk_333_432_isp, "aclk333_432_isp",
			"aclk333432_ispd", aclk_333_432_isp);
EXYNOS5_INT_PM_CLK(aclk_333_432_gscl, "aclk333_432_gsc",
			"aclk333432_gscd", aclk_333_432_gscl);
EXYNOS5_INT_PM_CLK(aclk_300_disp1, "aclk300_disp1",
			"aclk300_disp1_d", aclk_300_disp1);
EXYNOS5_INT_PM_CLK(aclk_300_jpeg, "aclk300_jpeg",
			"aclk300_jpeg_d", aclk_300_jpeg);
EXYNOS5_INT_PM_CLK(aclk_400_disp1, "aclk400_disp1",
			"aclk400_disp1_d", aclk_400_disp1);

static struct int_pm_clks *exynos5420_int_pm_clks[] = {
	&int_pm_clks_aclk_200_fsys,
	&int_pm_clks_pclk_200_fsys,
	&int_pm_clks_aclk_100_noc,
	&int_pm_clks_aclk_400_wcore,
	&int_pm_clks_aclk_200_fsys2,
	&int_pm_clks_aclk_400_mscl,
	&int_pm_clks_aclk_400_isp,
	&int_pm_clks_aclk_166,
	&int_pm_clks_aclk_266,
	&int_pm_clks_aclk_66,
	&int_pm_clks_aclk_333_432_isp,
	&int_pm_clks_aclk_333_432_gscl,
	&int_pm_clks_aclk_300_disp1,
	&int_pm_clks_aclk_300_jpeg,
	&int_pm_clks_aclk_400_disp1,
	NULL,
};

EXYNOS5_INT_PM_CLK(aclk_300_gscl_5422, "aclk300_gscl",
			"aclk300_gscl_d", aclk_300_gscl_5422);
EXYNOS5_INT_PM_CLK(aclk_333_5422, "aclk333",
			"aclk333_d", aclk_333_5422);
EXYNOS5_INT_PM_CLK(aclk_200_disp1_5422, "aclk200_disp1",
			"aclk200_d", aclk_200_disp1_5422);
EXYNOS5_INT_PM_CLK(pclk_200_fsys_5422, "pclk200_fsys",
			"pclk200_fsys_d", pclk_200_fsys_5422);
EXYNOS5_INT_PM_CLK(aclk_100_noc_5422, "aclk100_noc",
			"aclk100_noc_d", aclk_100_noc_5422);
EXYNOS5_INT_PM_CLK(aclk_200_fsys2_5422, "aclk200_fsys2",
			"aclk200_fsys2_d", aclk_200_fsys2_5422);
EXYNOS5_INT_PM_CLK(aclk_400_mscl_5422, "aclk400_mscl",
			"aclk400_mscl_d", aclk_400_mscl_5422);
EXYNOS5_INT_PM_CLK(aclk_266_5422, "aclk266",
			"aclk266_d", aclk_266_5422);
EXYNOS5_INT_PM_CLK(aclk_400_wcore_5422, "aclk400_wcore",
			"aclk400_wcore_d", aclk_400_wcore_5422);

static struct int_pm_clks *exynos5422_int_pm_clks[] = {
	&int_pm_clks_aclk_200_fsys,
	&int_pm_clks_pclk_200_fsys_5422,
	&int_pm_clks_aclk_100_noc_5422,
	&int_pm_clks_aclk_400_wcore_5422,
	&int_pm_clks_aclk_200_fsys2_5422,
	&int_pm_clks_aclk_400_mscl_5422,
	&int_pm_clks_aclk_166,
	&int_pm_clks_aclk_266_5422,
	&int_pm_clks_aclk_66,
	&int_pm_clks_aclk_300_disp1,
	&int_pm_clks_aclk_300_jpeg,
	&int_pm_clks_aclk_400_disp1,
	&int_pm_clks_aclk_200_disp1_5422,
	&int_pm_clks_aclk_333_5422,
	&int_pm_clks_aclk_300_gscl_5422,
	NULL,
};

static struct clk *exynos5_find_pll(struct busfreq_data_int *data,
				    enum int_bus_idx target_pll)
{
	struct clk *target_src_clk = NULL;

	switch (target_pll) {
	case S_PLL:
		target_src_clk = data->mout_spll;
		break;
	case C_PLL:
		target_src_clk = data->mout_cpll;
		break;
	case M_PLL:
		target_src_clk = data->mout_mpll;
		break;
	case D_PLL:
		target_src_clk = data->mout_dpll;
		break;
	case I_PLL:
		target_src_clk = data->mout_ipll;
		break;
	default:
		break;
	}

	return target_src_clk;
}

static void exynos5_int_set_freq(struct busfreq_data_int *data,
		unsigned long target_freq, unsigned long pre_freq)
{
	unsigned int i;
	unsigned long tar_rate;
	int target_idx = -EINVAL;
	int pre_idx = -EINVAL;
	struct int_pm_clks *int_clk;
	struct clk *new_src_pll;
	struct clk *old_src_pll;
	unsigned long old_src_rate, new_src_rate;
	unsigned long rate1, rate2, rate3, rate4;

	/* Find the levels for target and previous frequencies */
	for (i = 0; i < LV_END; i++) {
		if (int_bus_opp_list[i].freq == target_freq)
			target_idx = int_bus_opp_list[i].idx;
		if (int_bus_opp_list[i].freq == pre_freq)
			pre_idx = int_bus_opp_list[i].idx;
	}

	list_for_each_entry(int_clk, &data->list, node) {
		tar_rate = int_clk->clk_info[target_idx].target_freq * 1000;

		old_src_pll = clk_get_parent(int_clk->mux_clk);
		new_src_pll = exynos5_find_pll(data,
				int_clk->clk_info[target_idx].src_pll);

		if (old_src_pll == new_src_pll) {
			/* No need to change pll */
			clk_set_rate(int_clk->div_clk, tar_rate);
			pr_debug("%s: %s now %lu (%lu)\n", __func__,
				 int_clk->mux_clk_name,
				 clk_get_rate(int_clk->div_clk), tar_rate);
			continue;
		}

		old_src_rate = clk_get_rate(old_src_pll);
		new_src_rate = clk_get_rate(new_src_pll);
		rate1 = clk_get_rate(int_clk->div_clk);

		/*
		 * If we're switching to a faster PLL we should bump up the
		 * divider before switching.
		 */
		if (new_src_rate > old_src_rate) {
			int new_div;
			unsigned long tmp_rate;

			new_div = DIV_ROUND_UP(new_src_rate, tar_rate);
			tmp_rate = DIV_ROUND_UP(old_src_rate, new_div);
			clk_set_rate(int_clk->div_clk, tmp_rate);
		}
		rate2 = clk_get_rate(int_clk->div_clk);

		/* We can safely change the mux now */
		clk_set_parent(int_clk->mux_clk, new_src_pll);
		rate3 = clk_get_rate(int_clk->div_clk);

		/*
		 * Give us a proper divider; technically not needed in the case
		 * where we pre-calculated the divider above (the new_src_rate >
		 * old_src_rate case), but let's be formal about it.
		 */
		clk_set_rate(int_clk->div_clk, tar_rate);
		rate4 = clk_get_rate(int_clk->div_clk);

		pr_debug("%s: %s => PLL %d; %lu=>%lu=>%lu=>%lu (%lu)\n",
			 __func__, int_clk->mux_clk_name,
			 int_clk->clk_info[target_idx].src_pll,
			 rate1, rate2, rate3, rate4, tar_rate);
	}
}

static void exynos5_int_update_state(unsigned int target_freq)
{
	cputime64_t cur_time = get_jiffies_64();
	cputime64_t tmp_cputime;
	unsigned int target_idx = LV_0;
	unsigned int i;

	/* Find level value with corresponding target frequency */
	for (i = LV_0; i < LV_END; i++) {
		if (int_bus_opp_list[i].freq == target_freq)
			target_idx = int_bus_opp_list[i].idx;
	}

	tmp_cputime = cur_time - int_pre_time;

	int_bus_opp_list[target_idx].time_in_state =
		int_bus_opp_list[target_idx].time_in_state + tmp_cputime;

	int_pre_time = cur_time;
}

static int exynos5_int_busfreq_target(struct device *dev,
				      unsigned long *_freq, u32 flags)
{
	int err = 0;
	struct platform_device *pdev = container_of(dev,
					struct platform_device, dev);
	struct busfreq_data_int *data = platform_get_drvdata(pdev);
	struct opp *opp;
	unsigned long freq;
	unsigned long old_freq;
	unsigned long target_volt;
	static bool did_run_once;

	mutex_lock(&data->lock);

	/* get available opp information */
	rcu_read_lock();
	opp = devfreq_recommended_opp(dev, _freq, flags);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		dev_err(dev, "%s: Invalid OPP.\n", __func__);
		mutex_unlock(&data->lock);
		return PTR_ERR(opp);
	}

	freq = opp_get_freq(opp);
	target_volt = opp_get_voltage(opp);
	rcu_read_unlock();

	/* get old opp information */
	rcu_read_lock();
	old_freq = opp_get_freq(data->curr_opp);
	rcu_read_unlock();

	exynos5_int_update_state(old_freq);

	/*
	 * Skip if no change, except on the first run where we need to deal
	 * with the fact that the BIOS may have left us in a strange state.
	 */
	if (old_freq == freq && did_run_once)
		goto out;
	did_run_once = true;

	/*
	 * If target frequency is higher than old frequency
	 * change the voltage before setting freq ratio
	 */
	if (old_freq <= freq) {
		regulator_set_voltage(data->vdd_int,
				target_volt, target_volt + INT_VOLT_STEP_UV);

		exynos5_int_set_freq(data, freq, old_freq);
	} else {
		exynos5_int_set_freq(data, freq, old_freq);

		regulator_set_voltage(data->vdd_int,
				target_volt, target_volt + INT_VOLT_STEP_UV);
	}

	data->curr_opp = opp;
out:
	mutex_unlock(&data->lock);

	return err;
}

static int exynos5_int_bus_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
	struct busfreq_data_int *data = dev_get_drvdata(dev);
	unsigned long busy_data;
	unsigned int int_ccnt;
	unsigned long int_pmcnt;

	rcu_read_lock();
	stat->current_frequency = opp_get_freq(data->curr_opp);
	rcu_read_unlock();

	/*
	 * Bandwidth of memory interface is 128bits
	 * So bus can transfer 16bytes per cycle
	 */
	busy_data = exynos5420_ppmu_get_busy(data->ppmu, PPMU_SET_DDR,
						&int_ccnt, &int_pmcnt);
	stat->total_time = int_ccnt;
	stat->busy_time = int_pmcnt;
	return 0;
}

#if defined(CONFIG_DEVFREQ_GOV_SIMPLE_ONDEMAND)
static struct devfreq_simple_ondemand_data exynos5_int_governor_data = {
	.upthreshold		= 15,
};
#endif

static struct devfreq_dev_profile exynos5_int_devfreq_profile = {
	.initial_freq	= 333000,
	.polling_ms	= 100,
	.target		= exynos5_int_busfreq_target,
	.get_dev_status	= exynos5_int_bus_get_dev_status,
};

static int get_vtiming(struct device *dev) {
	u32 ldata[4];
	struct device_node *dpc_node, *dp_node;

	dpc_node = of_find_compatible_node(NULL, NULL, "samsung,exynos5-dp");
	if (!dpc_node) {
		dev_err(dev, "samsung,exynos5-dp node does not exist\n");
		return -1;
	}

	dp_node = of_parse_phandle(dpc_node, "samsung,dp-display", 0);
	if (!dp_node) {
		dev_err(dev, "samsung,dp-display node does not exist\n");
		return -1;
	}

	if (of_property_read_u32_array(dp_node, "lcd-vtiming", ldata, 4)) {
		dev_err(dev, "invalid vertical timing\n");
		return -1;
	}

	return ldata[3];
}

static int exynos5420_init_int_table(struct busfreq_data_int *data)
{
	unsigned int i;
	unsigned int ret;
	unsigned int asv_volt;

	for (i = 0; i < ARRAY_SIZE(int_bus_opp_list); i++) {
#ifdef CONFIG_ARM_EXYNOS5420_ASV
		if (soc_is_exynos5420() && i > 2) {
			/* Lock INT to at least 400MHz levels */
			asv_volt = get_match_volt(ID_INT,
					int_bus_opp_list[2].freq);
		} else {
			asv_volt = get_match_volt(ID_INT,
					int_bus_opp_list[i].freq);
		}
#else
		asv_volt = int_bus_opp_list[i].volt;
#endif
		pr_debug("INT %luKhz ASV is %duV\n",
					int_bus_opp_list[i].freq, asv_volt);
		ret = opp_add(data->dev, int_bus_opp_list[i].freq, asv_volt);
		if (ret) {
			dev_err(data->dev, "Fail to add opp entries.\n");
			return ret;
		}
	}

	opp_disable(data->dev, 600000);
	if (soc_is_exynos5420()) {
		opp_disable(data->dev, 500000);
		opp_disable(data->dev, 400000);

		/* HACK: If we have a 5420 w/ high res we may need this */
		if (get_vtiming(data->dev) >= 1080) {
			opp_disable(data->dev, 111000);
			opp_disable(data->dev, 83000);
		}
	} else {
		/* Only 500Mhz is available so we can run at 2.1GHz */
		exynos5_int_devfreq_profile.initial_freq = 500000;
		opp_disable(data->dev, 400000);
		opp_disable(data->dev, 333000);
		opp_disable(data->dev, 222000);
		opp_disable(data->dev, 111000);
		opp_disable(data->dev, 83000);
	}

	return 0;
}

static ssize_t int_show_state(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int i;
	ssize_t len = 0;
	ssize_t write_cnt = (ssize_t)((PAGE_SIZE / LV_END) - 2);

	for (i = LV_0; i < LV_END; i++) {
		len += snprintf(buf + len,
			write_cnt, "%ld %llu\n", int_bus_opp_list[i].freq,
			(unsigned long long)int_bus_opp_list[i].time_in_state);
	}

	return len;
}

static DEVICE_ATTR(int_time_in_state, 0644, int_show_state, NULL);

static struct attribute *busfreq_int_entries[] = {
	&dev_attr_int_time_in_state.attr,
	NULL,
};
static struct attribute_group busfreq_int_attr_group = {
	.name	= "time_in_state",
	.attrs	= busfreq_int_entries,
};

static ssize_t show_freq_table(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, count = 0;
	struct opp *opp;
	ssize_t write_cnt = (ssize_t)((PAGE_SIZE /
					ARRAY_SIZE(int_bus_opp_list)) - 2);

	if (!unlikely(int_dev)) {
		pr_err("%s: device is not probed\n", __func__);
		return -ENODEV;
	}

	rcu_read_lock();
	for (i = 0; i < ARRAY_SIZE(int_bus_opp_list); i++) {
		opp = opp_find_freq_exact(int_dev,
					int_bus_opp_list[i].freq, true);
		if (!IS_ERR_OR_NULL(opp))
			count += snprintf(&buf[count], write_cnt, "%lu ",
							opp_get_freq(opp));
	}
	rcu_read_unlock();

	count += snprintf(&buf[count], 2, "\n");
	return count;
}

static DEVICE_ATTR(freq_table, S_IRUGO, show_freq_table, NULL);

static struct exynos_devfreq_platdata default_qos_int_pd = {
	.default_qos = 333000,
};

static int exynos5_int_reboot_notifier_call(struct notifier_block *this,
					   unsigned long code, void *_cmd)
{
	pm_qos_update_request(&exynos5_int_qos, 333000);
	return NOTIFY_DONE;
}

static struct notifier_block exynos5_int_reboot_notifier = {
	.notifier_call = exynos5_int_reboot_notifier_call,
};

static void exynos5_int_remove_clocks(struct busfreq_data_int *data)
{
	struct int_pm_clks *int_clk;

	list_for_each_entry(int_clk, &data->list, node) {
		clk_put(int_clk->mux_clk);
		clk_put(int_clk->div_clk);
	}
}

static int exynos5_busfreq_int_probe(struct platform_device *pdev)
{
	struct busfreq_data_int *data;
	struct opp *opp;
	struct device *dev = &pdev->dev;
	struct exynos_devfreq_platdata *pdata;
	int err = 0;
	int nr_clk;
	struct clk *mux_clk, *div_clk;
	struct int_pm_clks *int_clk;

	data = kzalloc(sizeof(struct busfreq_data_int), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory for INT.\n");
		return -ENOMEM;
	}

	data->dev = dev;
	INIT_LIST_HEAD(&data->list);
	mutex_init(&data->lock);

	/* Setting table for int */
	exynos5420_init_int_table(data);

	data->vdd_int = regulator_get(dev, "vdd_int");
	if (IS_ERR(data->vdd_int)) {
		dev_err(dev, "Cannot get the regulator \"vdd_int\"\n");
		err = PTR_ERR(data->vdd_int);
		goto err_regulator;
	}

	data->mout_ipll = devm_clk_get(dev, "mout_ipll");
	if (IS_ERR(data->mout_ipll)) {
		dev_err(dev, "Cannot get clock \"mout_ipll\"\n");
		err = PTR_ERR(data->mout_ipll);
		goto err_mout_mpll;
	}

	data->mout_mpll = clk_get(dev, "mout_mpll");
	if (IS_ERR(data->mout_mpll)) {
		dev_err(dev, "Cannot get clock \"mout_mpll\"\n");
		err = PTR_ERR(data->mout_mpll);
		goto err_mout_mpll;
	}

	data->mout_dpll = clk_get(dev, "mout_dpll");
	if (IS_ERR(data->mout_dpll)) {
		dev_err(dev, "Cannot get clock \"mout_dpll\"\n");
		err = PTR_ERR(data->mout_dpll);
		goto err_mout_dpll;
	}

	data->mout_spll = clk_get(dev, "mout_spll");
	if (IS_ERR(data->mout_spll)) {
		dev_err(dev, "Cannot get clock \"mout_spll\"\n");
		err = PTR_ERR(data->mout_spll);
		goto err_mout_spll;
	}

	data->mout_cpll = clk_get(dev, "mout_cpll");
	if (IS_ERR(data->mout_cpll)) {
		dev_err(dev, "Cannot get clock \"mout_cpll\"\n");
		err = PTR_ERR(data->mout_cpll);
		goto err_mout_cpll;
	}

	if (soc_is_exynos5420())
		exynos5_int_pm_clks = exynos5420_int_pm_clks;
	else
		exynos5_int_pm_clks = exynos5422_int_pm_clks;

	/* Register and add int clocks to list */
	for (nr_clk = 0; exynos5_int_pm_clks[nr_clk] != NULL; nr_clk++) {
		int_clk = exynos5_int_pm_clks[nr_clk];
		mux_clk = clk_get(NULL, int_clk->mux_clk_name);
		div_clk = clk_get(NULL, int_clk->div_clk_name);
		if (IS_ERR(mux_clk) || IS_ERR(div_clk)) {
			if (IS_ERR(div_clk))
				dev_err(dev, "Failed to get %s clock\n",
					int_clk->div_clk_name);
			else
				clk_put(div_clk);
			if (IS_ERR(mux_clk))
				dev_err(dev, "Failed to get %s clock\n",
					int_clk->mux_clk_name);
			else
				clk_put(mux_clk);
			goto err_int_clk;
		} else {
			int_clk->mux_clk = mux_clk;
			int_clk->div_clk = div_clk;
			list_add_tail(&int_clk->node, &data->list);
		}
	}

	rcu_read_lock();
	opp = opp_find_freq_floor(dev,
				&exynos5_int_devfreq_profile.initial_freq);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		dev_err(dev, "Invalid initial frequency %lu kHz.\n",
			       exynos5_int_devfreq_profile.initial_freq);
		err = PTR_ERR(opp);
		goto err_opp_add;
	}
	rcu_read_unlock();

	int_pre_time = get_jiffies_64();

	data->curr_opp = opp;

	platform_set_drvdata(pdev, data);

	data->ppmu = exynos5420_ppmu_get(PPMU_SET_DDR);
	if (!data->ppmu)
		goto err_opp_add;

#if defined(CONFIG_DEVFREQ_GOV_USERSPACE)
	data->devfreq = devfreq_add_device(dev, &exynos5_int_devfreq_profile,
						&devfreq_userspace, NULL);
#endif
#if defined(CONFIG_DEVFREQ_GOV_SIMPLE_ONDEMAND)
	data->devfreq = devfreq_add_device(dev, &exynos5_int_devfreq_profile,
			   "simple_ondemand", &exynos5_int_governor_data);
#endif
	if (IS_ERR(data->devfreq)) {
		err = PTR_ERR(data->devfreq);
		goto err_opp_add;
	}

	devfreq_register_opp_notifier(dev, data->devfreq);
	int_dev = data->dev;

	/* Create file for time_in_state */
	err = sysfs_create_group(&data->devfreq->dev.kobj,
				&busfreq_int_attr_group);

	/* Add sysfs for freq_table */
	err = device_create_file(&data->devfreq->dev, &dev_attr_freq_table);
	if (err)
		dev_err(dev, "%s: Fail to create sysfs file\n", __func__);

	pdata = pdev->dev.platform_data;
	if (!pdata)
		pdata = &default_qos_int_pd;

	pm_qos_add_request(&exynos5_int_qos, PM_QOS_DEVICE_THROUGHPUT,
							pdata->default_qos);
	pm_qos_add_request(&boot_int_qos, PM_QOS_DEVICE_THROUGHPUT,
							pdata->default_qos);
	/* After 40ms this qos request is cancelled automatically */
	pm_qos_update_request_timeout(&boot_int_qos, pdata->default_qos,
								40000 * 1000);

	register_reboot_notifier(&exynos5_int_reboot_notifier);
	return 0;

err_opp_add:
err_int_clk:
	exynos5_int_remove_clocks(data);
	clk_put(data->mout_cpll);
err_mout_cpll:
	clk_put(data->mout_spll);
err_mout_spll:
	clk_put(data->mout_dpll);
err_mout_dpll:
	clk_put(data->mout_mpll);
err_mout_mpll:
	regulator_put(data->vdd_int);
err_regulator:
	kfree(data);

	return err;
}

static int exynos5_busfreq_int_remove(struct platform_device *pdev)
{
	struct busfreq_data_int *data = platform_get_drvdata(pdev);

	devfreq_remove_device(data->devfreq);
	exynos5420_ppmu_put(data->ppmu);
	pm_qos_remove_request(&exynos5_int_qos);

	exynos5_int_remove_clocks(data);
	clk_put(data->mout_cpll);
	clk_put(data->mout_spll);
	clk_put(data->mout_dpll);
	clk_put(data->mout_mpll);

	regulator_put(data->vdd_int);
	kfree(data);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int exynos5_busfreq_int_suspend(struct device *dev)
{
	struct exynos_devfreq_platdata *pdata = dev->platform_data;

	if (pm_qos_request_active(&exynos5_int_qos))
		pm_qos_update_request(&exynos5_int_qos, pdata->default_qos);

	return 0;
}

static int exynos5_busfreq_int_resume(struct device *dev)
{
	struct exynos_devfreq_platdata *pdata = dev->platform_data;

	if (pm_qos_request_active(&exynos5_int_qos))
		pm_qos_update_request(&exynos5_int_qos, pdata->default_qos);

	return 0;
}

static const struct dev_pm_ops exynos5_busfreq_int_pm = {
	.suspend	= exynos5_busfreq_int_suspend,
	.resume		= exynos5_busfreq_int_resume,
};

static struct platform_driver exynos5_busfreq_int_driver = {
	.probe	= exynos5_busfreq_int_probe,
	.remove	= exynos5_busfreq_int_remove,
	.driver = {
		.name	= "exynos5420-devfreq-int",
		.owner	= THIS_MODULE,
		.pm	= &exynos5_busfreq_int_pm,
	},
};

static int __init exynos5_busfreq_int_init(void)
{
	return platform_driver_register(&exynos5_busfreq_int_driver);
}
device_initcall(exynos5_busfreq_int_init);

static void __exit exynos5_busfreq_int_exit(void)
{
	platform_driver_unregister(&exynos5_busfreq_int_driver);
}
module_exit(exynos5_busfreq_int_exit);
