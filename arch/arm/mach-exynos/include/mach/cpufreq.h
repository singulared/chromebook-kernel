/* linux/arch/arm/mach-exynos/include/mach/cpufreq.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - CPUFreq support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

enum cpufreq_level_index {
	L0, L1, L2, L3, L4,
	L5, L6, L7, L8, L9,
	L10, L11, L12, L13, L14,
	L15, L16, L17, L18, L19,
	L20,
};

/**
 * struct exynos_dvfs_info - Exynos DVFS information structure
 *
 * @boot_freq: Initial frequency setup by the bootloader
 * @blocked: Prevents any frequency changes other than to the boot frequency
 * @regulator: Voltage regulator reference for each cluster
 *
 * This contains the clock/voltage information about the current configuration
 * mainly for the core cpufreq drivers.
 */
struct exynos_dvfs_info {
	unsigned long	mpll_freq_khz;
	unsigned int	pll_safe_idx;
	unsigned int	pm_lock_idx;
	unsigned int	max_support_idx;
	unsigned int	min_support_idx;
	unsigned int	boot_freq;
	struct clk	*cpu_clk;
	unsigned int	*volt_table;
	bool		blocked;
	struct cpufreq_frequency_table	*freq_table;
	const unsigned int	*max_op_freqs;
	struct regulator	*regulator;
	void (*set_freq)(unsigned int, unsigned int);
	bool (*need_apll_change)(unsigned int, unsigned int);
};

extern int exynos4210_cpufreq_init(struct exynos_dvfs_info *);
extern int exynos4x12_cpufreq_init(struct exynos_dvfs_info *);
extern int exynos5250_cpufreq_init(struct exynos_dvfs_info *);
extern const unsigned int exynos5250_cpufreq_get_asv(unsigned int);
extern int exynos5420_cpufreq_init(struct exynos_dvfs_info *);
extern int exynos5420_cpufreq_CA7_init(struct exynos_dvfs_info *);
extern int exynos_iks_cpufreq_init(void);

/* Exynos5420 comprises two clusters - Cortex A7 and A15 */
enum cluster_type {
	 CA7,
	 CA15,
	 CA_END,
};
