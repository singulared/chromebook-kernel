/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS5420 - CPU frequency scaling support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>

#include <mach/map.h>
#include <mach/regs-clock.h>

#include <plat/clock.h>

#include <mach/cpufreq.h>

#define CPUFREQ_NUM_LEVELS	(L18 + 1)
#define EXYNOS5420_M_MASK	0x3ff
#define EXYNOS5420_P_MASK	0x3f
#define EXYNOS5420_S_MASK	0x7
#define EXYNOS5_CLKDIV_STATCPU0_MASK	0x11111111
#define EXYNOS5_CLKDIV_STATCPU1_MASK	0x111

static int max_support_idx;
static int min_support_idx = (CPUFREQ_NUM_LEVELS - 1);

static struct clk *mout_cpu;
static struct clk *mout_mspll_cpu;
static struct clk *mout_apll;
static struct clk *fout_apll;

struct cpufreq_clkdiv {
	unsigned int	index;
	unsigned int	clkdiv;
	unsigned int	clkdiv1;
};

static unsigned int exynos5420_volt_table[CPUFREQ_NUM_LEVELS];
static struct cpufreq_frequency_table exynos5420_freq_table[] = {
	{L0,  2000 * 1000},
	{L1,  1900 * 1000},
	{L2,  1800 * 1000},
	{L3,  1700 * 1000},
	{L4,  1600 * 1000},
	{L5,  1500 * 1000},
	{L6,  1400 * 1000},
	{L7,  1300 * 1000},
	{L8,  1200 * 1000},
	{L9,  1100 * 1000},
	{L10, 1000 * 1000},
	{L11,  900 * 1000},
	{L12,  800 * 1000},
	{L13,  700 * 1000},
	{L14,  600 * 1000},
	{L15,  500 * 1000},
	{L16,  400 * 1000},
	{L17,  300 * 1000},
	{L18,  200 * 1000},
	{0, CPUFREQ_TABLE_END},
};

static struct cpufreq_clkdiv exynos5420_clkdiv_table[CPUFREQ_NUM_LEVELS];

static unsigned int clkdiv_cpu0_5420[CPUFREQ_NUM_LEVELS][7] = {
	/*
	 *  Clock divider values for {CPUD, ATB, PCLK_DBG, APLL, ARM2}
	 */
	{ 2, 7, 7, 3, 0 }, /* ARM L0: 2.0GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L1: 1.9GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L2: 1.8GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L3: 1.7GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L4: 1.6GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L5: 1.5GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L6: 1.4GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L7: 1.3GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L8: 1.2GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L9: 1.1GHz */
	{ 2, 6, 6, 3, 0 }, /* ARM L10: 1GHz */
	{ 2, 6, 6, 3, 0 }, /* ARM L11: 900MHz */
	{ 2, 5, 5, 3, 0 }, /* ARM L12: 800MHz */
	{ 2, 5, 5, 3, 0 }, /* ARM L13: 700MHz */
	{ 2, 4, 4, 3, 0 }, /* ARM L14: 600MHz */
	{ 2, 3, 3, 3, 0 }, /* ARM L15: 500MHz */
	{ 2, 3, 3, 3, 0 }, /* ARM L16: 400MHz */
	{ 2, 3, 3, 3, 0 }, /* ARM L17: 300MHz */
	{ 2, 3, 3, 3, 0 }, /* ARM L18: 200MHz */
};

unsigned int clkdiv_cpu1_5420[CPUFREQ_NUM_LEVELS][2] = {
	/* Clock divider values for { copy, HPM } */
	{ 7, 7 }, /* ARM L0: 2.0GHz */
	{ 7, 7 }, /* ARM L1: 1.9GHz */
	{ 7, 7 }, /* ARM L2: 1.8GHz */
	{ 7, 7 }, /* ARM L3: 1.7GHz */
	{ 7, 7 }, /* ARM L4: 1.6GHz */
	{ 7, 7 }, /* ARM L5: 1.5GHz */
	{ 7, 7 }, /* ARM L6: 1.4GHz */
	{ 7, 7 }, /* ARM L7: 1.3GHz */
	{ 7, 7 }, /* ARM L8: 1.2GHz */
	{ 7, 7 }, /* ARM L9: 1.1GHz */
	{ 7, 7 }, /* ARM L10: 1GHz */
	{ 7, 7 }, /* ARM L11: 900MHz */
	{ 7, 7 }, /* ARM L12: 800MHz */
	{ 7, 7 }, /* ARM L13: 700MHz */
	{ 7, 7 }, /* ARM L14: 600MHz */
	{ 7, 7 }, /* ARM L15: 500MHz */
	{ 7, 7 }, /* ARM L16: 400MHz */
	{ 7, 7 }, /* ARM L17: 300MHz */
	{ 7, 7 }, /* ARM L18: 200MHz */
};

static unsigned int exynos5420_apll_pms_table[CPUFREQ_NUM_LEVELS] = {
	((250 << 16) | (3 << 8) | 0x0), /* APLL FOUT L0: 2GHz */
	((475 << 16) | (6 << 8) | 0x0), /* APLL FOUT L1: 1.9GHz */
	((225 << 16) | (3 << 8) | 0x0), /* APLL FOUT L2: 1.8GHz */
	((425 << 16) | (6 << 8) | 0x0), /* APLL FOUT L3: 1.7GHz */
	((200 << 16) | (3 << 8) | 0x0), /* APLL FOUT L4: 1.6GHz */
	((250 << 16) | (4 << 8) | 0x0), /* APLL FOUT L5: 1.5GHz */
	((175 << 16) | (3 << 8) | 0x0), /* APLL FOUT L6: 1.4GHz */
	((325 << 16) | (6 << 8) | 0x0), /* APLL FOUT L7: 1.3GHz */
	((200 << 16) | (2 << 8) | 0x1), /* APLL FOUT L8: 1.2GHz */
	((275 << 16) | (3 << 8) | 0x1), /* APLL FOUT L9: 1.1GHz */
	((250 << 16) | (3 << 8) | 0x1), /* APLL FOUT L10: 1GHz */
	((150 << 16) | (2 << 8) | 0x1), /* APLL FOUT L11: 900MHz */
	((200 << 16) | (3 << 8) | 0x1), /* APLL FOUT L12: 800MHz */
	((175 << 16) | (3 << 8) | 0x1), /* APLL FOUT L13: 700MHz */
	((200 << 16) | (2 << 8) | 0x2), /* APLL FOUT L14: 600MHz */
	((250 << 16) | (3 << 8) | 0x2), /* APLL FOUT L15: 500MHz */
	((200 << 16) | (3 << 8) | 0x2), /* APLL FOUT L16: 400MHz */
	((400 << 16) | (4 << 8) | 0x3), /* APLL FOUT L17: 300MHz */
	((200 << 16) | (3 << 8) | 0x3), /* APLL FOUT L18: 200MHz */
};

/*
 * Default ASV table
 */
static const unsigned int asv_voltage_5420[CPUFREQ_NUM_LEVELS] = {
	1300000,	/* L0  2000 */
	1300000,	/* L1  1900 */
	1200000,	/* L2  1800 */
	1200000,	/* L3  1700 */
	1200000,	/* L4  1600 */
	1175000,	/* L5  1500 */
	1150000,	/* L6  1400 */
	1125000,	/* L7  1300 */
	1100000,	/* L8  1200 */
	1075000,	/* L9  1100 */
	1050000,	/* L10 1000 */
	1000000,	/* L11  900 */
	 950000,	/* L12  800 */
	 925000,	/* L13  700 */
	 900000,	/* L14  600 */
	 900000,	/* L15  500 */
	 900000,	/* L16  400 */
	 900000,	/* L17  300 */
	 900000,	/* L18  200 */
};

static const unsigned int exynos5420_max_op_freq_b_evt0[NR_CPUS + 1] = {
	1400000,
	1400000,
	1400000,
	1400000,
	CPUFREQ_TABLE_END,
};

static void exynos5420_set_clkdiv(unsigned int div_index)
{
	unsigned int tmp;

	/* Change Divider - CPU0 for CMU_CPU */
	tmp = exynos5420_clkdiv_table[div_index].clkdiv;
	__raw_writel(tmp, EXYNOS5_CLKDIV_CPU0);

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKDIV_STATCPU0);
	} while (tmp & EXYNOS5_CLKDIV_STATCPU0_MASK);
	pr_debug("DIV_CPU0[0x%x]\n", __raw_readl(EXYNOS5_CLKDIV_CPU0));

	/* Change Divider - CPU1 for CMU_CPU */
	tmp = exynos5420_clkdiv_table[div_index].clkdiv1;
	__raw_writel(tmp, EXYNOS5_CLKDIV_CPU1);

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKDIV_STATCPU1);
	} while (tmp & EXYNOS5_CLKDIV_STATCPU1_MASK);
	pr_debug("DIV_CPU1[0x%x]\n", __raw_readl(EXYNOS5_CLKDIV_CPU1));
}

static void exynos5420_set_apll(unsigned int new_index,
				unsigned int old_index)
{
	unsigned int tmp, pdiv;

	/* 1. MUX_CORE_SEL = MOUT_MSPLL; ARMCLK uses MOUT_MSPLL for lock time */
	if (clk_set_parent(mout_cpu, mout_mspll_cpu)) {
		pr_err(KERN_ERR "Unable to set parent %s of clock %s.\n",
			mout_mspll_cpu->name, mout_cpu->name);
	}

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKMUX_STATCPU)
			>> EXYNOS5_CLKSRC_CPU_MUXCORE_SHIFT;
		tmp &= EXYNOS5420_S_MASK;
	} while (tmp != 0x2);

	/* 2. Set APLL Lock time */
	pdiv = ((exynos5420_apll_pms_table[new_index] >> 8) &
						EXYNOS5420_P_MASK);
	__raw_writel((pdiv * 250), EXYNOS5_APLL_LOCK);

	/* 3. Change PLL PMS values */
	tmp = __raw_readl(EXYNOS5_APLL_CON0);
	tmp &= ~((EXYNOS5420_M_MASK << 16) | (EXYNOS5420_P_MASK << 8) |
						(EXYNOS5420_S_MASK << 0));
	tmp |= exynos5420_apll_pms_table[new_index];
	 __raw_writel(tmp, EXYNOS5_APLL_CON0);

	/* 4. wait_lock_time */
	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_APLL_CON0);
	} while (!(tmp & EXYNOS5_APLLCON0_LOCKED_MASK));

	/* 5. MUX_CORE_SEL = APLL */
	if (clk_set_parent(mout_cpu, mout_apll)) {
		pr_err("Unable to set parent %s of clock %s.\n",
				mout_apll->name, mout_cpu->name);
	}

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKMUX_STATCPU);
		tmp &= EXYNOS5_CLKMUX_STATCPU_MUXCORE_MASK;
	} while (tmp != (0x1 << EXYNOS5_CLKSRC_CPU_MUXCORE_SHIFT));
}

static bool exynos5420_pms_change(unsigned int old_index,
				  unsigned int new_index)
{
	unsigned int old_pm = (exynos5420_apll_pms_table[old_index] >> 8);
	unsigned int new_pm = (exynos5420_apll_pms_table[new_index] >> 8);

	return (old_pm != new_pm);
}

static void exynos5420_set_frequency(unsigned int old_index,
				     unsigned int new_index)
{
	unsigned int tmp;

	if (old_index > new_index) {
		/* 1. Change the system clock divider values */
		exynos5420_set_clkdiv(new_index);
		if (!exynos5420_pms_change(old_index, new_index)) {
			/* 2. Change just s value in apll m,p,s value */
			tmp = __raw_readl(EXYNOS5_APLL_CON0);
			tmp &= ~(EXYNOS5420_S_MASK << 0);
			tmp |= (exynos5420_apll_pms_table[new_index] &
							EXYNOS5420_S_MASK);
			__raw_writel(tmp, EXYNOS5_APLL_CON0);
		} else {
			/* 2. Change the apll m,p,s value */
			exynos5420_set_apll(new_index, old_index);
		}
	} else if (old_index < new_index) {
		if (!exynos5420_pms_change(old_index, new_index)) {
			/* 1. Change just s value in apll m,p,s value */
			tmp = __raw_readl(EXYNOS5_APLL_CON0);
			tmp &= ~(EXYNOS5420_S_MASK << 0);
			tmp |= (exynos5420_apll_pms_table[new_index] &
							EXYNOS5420_S_MASK);
			__raw_writel(tmp, EXYNOS5_APLL_CON0);
		} else {
			/* 1. Change the apll m,p,s value */
			exynos5420_set_apll(new_index, old_index);
		}
		/* 2. Change the system clock divider values */
		exynos5420_set_clkdiv(new_index);
	}
	clk_set_rate(fout_apll,
			exynos5420_freq_table[new_index].frequency * 1000);
}

static void __init set_volt_table(void)
{
	unsigned int i;

	for (i = 0; i < CPUFREQ_NUM_LEVELS; i++) {
		exynos5420_volt_table[i] = asv_voltage_5420[i];
		pr_debug("CPUFREQ of CA15 L%d : %d uV\n", i,
				exynos5420_volt_table[i]);
	}
	min_support_idx = L18;
	for (i = L0; i < L6; i++)
		exynos5420_freq_table[i].frequency = CPUFREQ_ENTRY_INVALID;
	max_support_idx = L6;
}

int exynos5420_cpufreq_init(struct exynos_dvfs_info *info)
{
	int i;
	unsigned int tmp;
	unsigned long rate;

	set_volt_table();

	mout_cpu = clk_get(NULL, "mout_cpu");
	if (IS_ERR(mout_cpu))
		return PTR_ERR(mout_cpu);

	mout_mspll_cpu = clk_get(NULL, "mout_mspll_cpu");
	if (IS_ERR(mout_mspll_cpu))
		goto err_mout_mspll_cpu;
	rate = clk_get_rate(mout_mspll_cpu) / 1000;

	mout_apll = clk_get(NULL, "mout_apll");
	if (IS_ERR(mout_apll))
		goto err_mout_apll;

	fout_apll = clk_get(NULL, "fout_apll");
	if (IS_ERR(fout_apll))
		goto err_fout_apll;

	for (i = L0; i < CPUFREQ_NUM_LEVELS; i++) {
		exynos5420_clkdiv_table[i].index = i;

		tmp = __raw_readl(EXYNOS5_CLKDIV_CPU0);
		tmp &= ~(EXYNOS5_CLKDIV_CPU0_CPUD_MASK |
			EXYNOS5_CLKDIV_CPU0_ATB_MASK |
			EXYNOS5_CLKDIV_CPU0_PCLKDBG_MASK |
			EXYNOS5_CLKDIV_CPU0_APLL_MASK |
			EXYNOS5_CLKDIV_CPU0_CORE2_MASK);
		tmp |= ((clkdiv_cpu0_5420[i][0] <<
				EXYNOS5_CLKDIV_CPU0_CPUD_SHIFT) |
			(clkdiv_cpu0_5420[i][1] <<
				EXYNOS5_CLKDIV_CPU0_ATB_SHIFT) |
			(clkdiv_cpu0_5420[i][2] <<
				EXYNOS5_CLKDIV_CPU0_PCLKDBG_SHIFT) |
			(clkdiv_cpu0_5420[i][3] <<
				EXYNOS5_CLKDIV_CPU0_APLL_SHIFT) |
			(clkdiv_cpu0_5420[i][4] <<
				EXYNOS5_CLKDIV_CPU0_CORE2_SHIFT));

		exynos5420_clkdiv_table[i].clkdiv = tmp;

		tmp = __raw_readl(EXYNOS5_CLKDIV_CPU1);
		tmp &= ~(EXYNOS5_CLKDIV_CPU1_COPY_MASK |
			EXYNOS5_CLKDIV_CPU1_HPM_MASK);
		tmp |= ((clkdiv_cpu1_5420[i][0] <<
				EXYNOS5_CLKDIV_CPU1_COPY_SHIFT) |
			(clkdiv_cpu1_5420[i][1] <<
				EXYNOS5_CLKDIV_CPU1_HPM_SHIFT));

		exynos5420_clkdiv_table[i].clkdiv1 = tmp;
	}

	info->mpll_freq_khz = rate;
	info->pm_lock_idx = L0;
	info->pll_safe_idx = L12;
	info->max_support_idx = max_support_idx;
	info->min_support_idx = min_support_idx;
	info->cpu_clk = fout_apll;
	pr_debug("fout_apll[%lu]\n", clk_get_rate(fout_apll));
	info->max_op_freqs = exynos5420_max_op_freq_b_evt0;
	info->volt_table = exynos5420_volt_table;
	info->freq_table = exynos5420_freq_table;
	info->set_freq = exynos5420_set_frequency;
	info->need_apll_change = exynos5420_pms_change;

	tmp = __raw_readl(EXYNOS5_CLKOUT_CMU_CPU);
	tmp &= ~0xffff;
	tmp |= 0x1904;
	__raw_writel(tmp, EXYNOS5_CLKOUT_CMU_CPU);

	return 0;

err_fout_apll:
	clk_put(fout_apll);
err_mout_apll:
	clk_put(mout_apll);
err_mout_mspll_cpu:
	clk_put(mout_mspll_cpu);

	pr_err("%s: failed initialization\n", __func__);
	return -EINVAL;
}
