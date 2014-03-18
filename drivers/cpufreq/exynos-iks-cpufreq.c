/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - CPU frequency scaling support for EXYNOS series
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/cpufreq.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/cpu.h>

#include <asm/cputype.h>
#include <asm/bL_switcher.h>
#include <mach/regs-pmu.h>
#include <mach/asv-exynos.h>
#include <plat/cpu.h>

#include <mach/cpufreq.h>

/* For switcher */
/* Minimum (Big/Little) clock frequency */
static unsigned int freq_min[CA_END] __read_mostly;
/* Maximum (Big/Little) clock frequency */
static unsigned int freq_max[CA_END] __read_mostly;
/* cpu number on (Big/Little) cluster */
static struct cpumask cluster_cpus[CA_END];

#define ACTUAL_FREQ(x, cur)  ((cur == CA7) ? (x) << 1 : (x))
#define VIRT_FREQ(x, cur)    ((cur == CA7) ? (x) >> 1 : (x))

#define LIMIT_COLD_VOLTAGE	1250000

static struct exynos_dvfs_info *exynos_info[CA_END];
static struct exynos_dvfs_info exynos_info_CA7;
static struct exynos_dvfs_info exynos_info_CA15;

static struct cpufreq_frequency_table *merge_freq_table;

static struct regulator *arm_regulator;
static struct regulator *kfc_regulator;
static unsigned int volt_offset;

static struct cpufreq_freqs *freqs[CA_END];

static DEFINE_MUTEX(cpufreq_lock);
static DEFINE_MUTEX(cpufreq_scale_lock);

/* Include CPU mask of each cluster */
static enum cluster_type boot_cluster;

DEFINE_PER_CPU(enum cluster_type, cpu_cur_cluster);
static DEFINE_PER_CPU(unsigned int, req_freq);

static struct cpufreq_policy fake_policy[CA_END][NR_CPUS];

static bool use_cluster_switching;

static void switch_complete_cb(void *cookie)
{
	struct completion *switch_complete = (struct completion *) cookie;
	complete(switch_complete);
}

static unsigned int get_limit_voltage(unsigned int voltage)
{
	if (voltage > LIMIT_COLD_VOLTAGE)
		return voltage;

	if (voltage + volt_offset > LIMIT_COLD_VOLTAGE)
		return LIMIT_COLD_VOLTAGE;

	return voltage + volt_offset;
}

static void init_cpumask_cluster_set(unsigned int cluster)
{
	unsigned int i;

	for_each_online_cpu(i) {
		per_cpu(cpu_cur_cluster, i) = cluster;
		if (cluster == CA15)
			cpumask_set_cpu(i, &cluster_cpus[CA15]);
		else
			cpumask_set_cpu(i, &cluster_cpus[CA7]);
	}
}

static enum cluster_type get_cur_cluster(unsigned int cpu)
{
	return per_cpu(cpu_cur_cluster, cpu);
}

/* Get table size */
static unsigned int cpufreq_get_table_size(
				struct cpufreq_frequency_table *table,
				unsigned int cluster_id)
{
	int size = 0;

	for (; (table[size].frequency != CPUFREQ_TABLE_END); size++)
		;
	return size;
}

/*
 * copy entries of all the per-cluster cpufreq_frequency_table entries
 * into a single frequency table which is published to cpufreq core
 */
static int cpufreq_merge_tables(void)
{
	int cluster_id, i;
	unsigned int total_sz = 0, size[CA_END];
	struct cpufreq_frequency_table *freq_table;

	for (cluster_id = 0; cluster_id < CA_END; cluster_id++) {
		size[cluster_id] =  cpufreq_get_table_size(
		   exynos_info[cluster_id]->freq_table, cluster_id);
		total_sz += size[cluster_id];
	}

	freq_table = kzalloc(sizeof(struct cpufreq_frequency_table) *
						(total_sz + 1), GFP_KERNEL);
	merge_freq_table = freq_table;

	memcpy(freq_table, exynos_info[CA15]->freq_table,
			size[CA15] * sizeof(struct cpufreq_frequency_table));
	freq_table += size[CA15];
	memcpy(freq_table, exynos_info[CA7]->freq_table,
			size[CA7] * sizeof(struct cpufreq_frequency_table));

	for (i = size[CA15]; i <= total_sz; i++) {
		if (merge_freq_table[i].frequency != CPUFREQ_ENTRY_INVALID)
			merge_freq_table[i].frequency >>= 1;
	}

	merge_freq_table[total_sz].frequency = CPUFREQ_TABLE_END;

	for (i = 0; merge_freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		pr_info("merged_table index: %d freq: %d\n", i,
				merge_freq_table[i].frequency);
	}

	return 0;
}

static bool is_alive(unsigned int cluster)
{
	unsigned int tmp;

	tmp = __raw_readl(cluster == CA15 ? EXYNOS5420_ARM_COMMON_STATUS :
					EXYNOS5420_KFC_COMMON_STATUS) & 0x3;

	return tmp ? true : false;
}

/*
 * Requesting core switch to other cluster. It save the current status
 * and wakes up the core of new cluster. Then waking core restore status and
 * take a task of other cluster.
 */
static void switch_to_entry(unsigned int cpu, enum cluster_type target_cluster)
{
	struct completion complete;

	init_completion(&complete);
	bL_switch_request_cb(cpu, !target_cluster, switch_complete_cb,
				&complete);
	wait_for_completion(&complete);
	per_cpu(cpu_cur_cluster, cpu) = target_cluster;
}

int exynos_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, merge_freq_table);
}

unsigned int exynos_getspeed_cluster(enum cluster_type cluster)
{
	return VIRT_FREQ(clk_get_rate(exynos_info[cluster]->cpu_clk) / 1000,
				cluster);
}

unsigned int exynos_getspeed(unsigned int cpu)
{
	unsigned int cur = get_cur_cluster(cpu);

	return exynos_getspeed_cluster(cur);
}

static unsigned int get_max_req_freq(unsigned int cluster_id)
{
	unsigned int i, max_freq = 0, tmp = 0, cur;

	for_each_online_cpu(i) {
		cur = get_cur_cluster(i);
		if (cur == cluster_id) {
			tmp = per_cpu(req_freq, i);
			if (tmp > max_freq)
				max_freq = tmp;
		}
	}

	return max_freq;
}

static void set_req_freq(unsigned int cpu, unsigned int freq)
{
	per_cpu(req_freq, cpu) = freq;
}

static unsigned int exynos_get_safe_volt(unsigned int old_index,
			unsigned int new_index, unsigned int cur)
{
	unsigned int safe_arm_volt = 0;
	struct cpufreq_frequency_table *freq_table
					= exynos_info[cur]->freq_table;
	unsigned int *volt_table = exynos_info[cur]->volt_table;

	/*
	 * ARM clock source will be changed APLL to MPLL temporary
	 * To support this level, need to control regulator for
	 * reguired voltage level
	 */
	if (exynos_info[cur]->need_apll_change != NULL) {
		if (exynos_info[cur]->need_apll_change(old_index, new_index) &&
			(freq_table[new_index].frequency
					< exynos_info[cur]->mpll_freq_khz) &&
			(freq_table[old_index].frequency
					< exynos_info[cur]->mpll_freq_khz)) {
				safe_arm_volt
				  = volt_table[exynos_info[cur]->pll_safe_idx];
		}
	}

	return safe_arm_volt;
}

/* Determine valid target frequency using freq_table */
int exynos5_frequency_table_target(struct cpufreq_policy *policy,
				   struct cpufreq_frequency_table *table,
				   unsigned int target_freq,
				   unsigned int relation,
				   unsigned int *index)
{
	unsigned int i;

	if (!cpu_online(policy->cpu))
		return -EINVAL;

	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		unsigned int freq = table[i].frequency;

		if (freq == CPUFREQ_ENTRY_INVALID)
			continue;

		if (target_freq == freq) {
			*index = i;
			break;
		}
	}

	if (table[i].frequency == CPUFREQ_TABLE_END)
		return -EINVAL;

	return 0;
}

static int is_cpufreq_valid(int cpu)
{
	struct cpufreq_policy policy;

	return !cpufreq_get_policy(&policy, cpu);
}

static int exynos_cpufreq_scale(unsigned int target_freq,
				unsigned int curr_freq, unsigned int cpu)
{
	unsigned int cur = get_cur_cluster(cpu);
	struct cpufreq_frequency_table *freq_table
					= exynos_info[cur]->freq_table;
	unsigned int *volt_table = exynos_info[cur]->volt_table;
	struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);
	struct regulator *regulator = exynos_info[cur]->regulator;
	unsigned int new_index, old_index, j;
	unsigned int volt, safe_volt = 0;
	bool increased_frequency;
	int ret = 0, err;

	pr_debug("\ncpu %d cluster %d\n", cpu, cur);
	if (!policy)
		return ret;

	if (!is_alive(cur))
		goto out;

	freqs[cur]->cpu = cpu;
	freqs[cur]->new = target_freq;

	if (exynos5_frequency_table_target(policy, freq_table,
				ACTUAL_FREQ(curr_freq, cur),
				CPUFREQ_RELATION_L, &old_index)) {
		ret = -EINVAL;
		goto out;
	}

	if (exynos5_frequency_table_target(policy, freq_table,
				ACTUAL_FREQ(freqs[cur]->new, cur),
				CPUFREQ_RELATION_L, &new_index)) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * ARM clock source will be changed APLL to MPLL temporary
	 * To support this level, need to control regulator for
	 * required voltage level
	 */
	safe_volt = exynos_get_safe_volt(old_index, new_index, cur);
	if (safe_volt)
		safe_volt = get_limit_voltage(safe_volt);

	volt = get_limit_voltage(volt_table[new_index]);

	/* Update policy current frequency */
	for_each_cpu(j, &cluster_cpus[cur]) {
		if (is_cpufreq_valid(j)) {
			freqs[cur]->cpu = j;
			cpufreq_notify_transition(
					freqs[cur], CPUFREQ_PRECHANGE);
		}
	}

	increased_frequency = (ACTUAL_FREQ(freqs[cur]->new, cur) >
			ACTUAL_FREQ(freqs[cur]->old, cur));

	/* When the new frequency is higher than current frequency */
	if (increased_frequency && !safe_volt) {
		/* Firstly, voltage up to increase frequency */
		ret = regulator_set_voltage(regulator, volt, volt);
		if (ret) {
			pr_warn("Failed to increase CPU voltage: %d\n", ret);
			freqs[cur]->new = freqs[cur]->old;
			goto out_notify;
		}
	}

	if (safe_volt) {
		ret = regulator_set_voltage(regulator, safe_volt, safe_volt);
		if (ret) {
			pr_warn("Failed to increase CPU voltage: %d\n", ret);
			freqs[cur]->new = freqs[cur]->old;
			goto out_notify;
		}
	}

	if (old_index != new_index) {
		pr_debug("\nold_index %d new_index %d\n", old_index, new_index);
		exynos_info[cur]->set_freq(old_index, new_index);
	}

	/* When the new frequency is lower than current frequency */
	if (!increased_frequency || (increased_frequency && safe_volt)) {
		/* down the voltage after frequency change */
		err = regulator_set_voltage(regulator, volt, volt);
		if (err)
			/*
			 * Not actually fatal since we've already changed
			 * the frequency and are running at a high enough
			 * voltage.
			 */
			pr_warn("Failed to decrease CPU voltage: %d\n", err);
	}

out_notify:
	for_each_cpu(j, &cluster_cpus[cur]) {
		if (is_cpufreq_valid(j)) {
			freqs[cur]->cpu = j;
			cpufreq_notify_transition(
					freqs[cur], CPUFREQ_POSTCHANGE);
		}
	}

out:
	cpufreq_cpu_put(policy);
	return ret;
}

static enum cluster_type exynos_switch(struct cpufreq_policy *policy,
						enum cluster_type cur)
{
	unsigned int cpu;
	enum cluster_type new_cluster;

	new_cluster = !cur;

	for_each_cpu(cpu, policy->cpus) {
		switch_to_entry(cpu, new_cluster);
		/* set big/litte-cpu mask */
		cpumask_clear_cpu(cpu, &cluster_cpus[cur]);
		cpumask_set_cpu(cpu, &cluster_cpus[new_cluster]);
	}

	return new_cluster;
}

/* Set clock frequency */
static int exynos_target(struct cpufreq_policy *policy,
		unsigned int target_freq, unsigned int relation)
{
	/* read current cluster */
	enum cluster_type cur = get_cur_cluster(policy->cpu);
	unsigned int index, a15_freq, new_freq = 0, do_switch = 0;
	int ret = 0, volt, i;

	mutex_lock(&cpufreq_lock);

	if (exynos_info[cur]->blocked && (target_freq != freq_max[CA15]))
		goto out;

	/* get current frequency */
	freqs[cur]->old = exynos_getspeed(policy->cpu);

	/* save the frequency & cpu number */
	set_req_freq(policy->cpu, target_freq);

	if (cur == CA15 && target_freq < freq_min[CA15])
		do_switch = 1;	/* Switch to Little */
	else if (cur == CA7 && target_freq > freq_max[CA7]) {
		do_switch = 1;	/* Switch from LITTLE to big */
		if (cpumask_weight(&cluster_cpus[CA15]) == 0) {
			/*
			 * If the A15 cluster is being powered up again,
			 * restore the voltage to the frequency level it
			 * will be powered up at.
			 */
			a15_freq = ACTUAL_FREQ(exynos_getspeed_cluster(CA15),
						CA15);
			volt = get_match_volt(ID_ARM, a15_freq);
			ret = regulator_set_voltage(arm_regulator, volt, volt);
			if (ret) {
				pr_err("Failed to restore CPU voltge: %d\n",
					ret);
				goto out;
			}
		}
	}

	if (do_switch) {
		cur = exynos_switch(policy, cur);
		freqs[cur]->old = exynos_getspeed_cluster(cur);
		policy->cur = freqs[cur]->old;

		if (use_cluster_switching) {
			for_each_online_cpu(i) {
				struct cpufreq_policy *tmp_policy;

				tmp_policy = cpufreq_cpu_get(i);
				tmp_policy->cur = freqs[cur]->old;
				cpufreq_cpu_put(tmp_policy);
			}
		}

		if (cpumask_weight(&cluster_cpus[CA15]) == 0) {
			/*
			 * If the A15 cluster is no longer active, lower the
			 * ARM rail voltage to save power.
			 */
			a15_freq = ACTUAL_FREQ(freq_min[CA15], CA15);
			volt = get_match_volt(ID_ARM, a15_freq);
			regulator_set_voltage(arm_regulator, volt, volt);
		}
	}

	new_freq = max(get_max_req_freq(cur), target_freq);

	if (cpufreq_frequency_table_target(&fake_policy[cur][policy->cpu],
			exynos_info[cur]->freq_table,
			ACTUAL_FREQ(new_freq, cur), relation, &index)) {
		ret = -EINVAL;
		goto out;
	}
	new_freq = exynos_info[cur]->freq_table[index].frequency;

	/* frequency and volt scaling */
	ret = exynos_cpufreq_scale(VIRT_FREQ(new_freq, cur),
						freqs[cur]->old, policy->cpu);

out:
	mutex_unlock(&cpufreq_lock);

	return ret;
}

#ifdef CONFIG_PM
static int exynos_cpufreq_suspend(struct cpufreq_policy *policy)
{
	return 0;
}

static int exynos_cpufreq_resume(struct cpufreq_policy *policy)
{
	return 0;
}
#endif

/*
 * exynos_cpufreq_pm_notifier - block CPUFREQ's activities in suspend-resume
 *			context
 * @notifier
 * @pm_event
 * @v
 *
 * While cpufreq_disable == true, target() ignores every frequency but
 * max_freq. The max_freq value is the maximum frequency.
 * In order to eliminate possible inconsistency in clock values, we save
 * and restore frequencies during suspend and resume and block CPUFREQ
 * activities. Note that the standard suspend/resume cannot be used
 * as they are too deep (syscore_ops) for regulator actions.
 */
static int exynos_cpufreq_pm_notifier(struct notifier_block *notifier,
				       unsigned long pm_event, void *v)
{
	struct cpufreq_policy *policy;
	unsigned int maxfreq_ca7, maxfreq_ca15, saved_max;
	int volt;
	unsigned int i;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		mutex_lock(&cpufreq_lock);
		exynos_info[CA7]->blocked = true;
		exynos_info[CA15]->blocked = true;
		mutex_unlock(&cpufreq_lock);

		maxfreq_ca7 = freq_max[CA7];
		maxfreq_ca15 = freq_max[CA15];

		volt = get_match_volt(ID_KFC, ACTUAL_FREQ(maxfreq_ca7, CA7));
		volt = get_limit_voltage(volt);

		if (regulator_set_voltage(kfc_regulator, volt, volt))
			goto err;

		volt = get_match_volt(ID_ARM, ACTUAL_FREQ(maxfreq_ca15, CA15));
		volt = get_limit_voltage(volt);

		if (regulator_set_voltage(arm_regulator, volt, volt))
			goto err;

		/*
		 * Migrate all cores to the CA15 max frequency so that
		 * we may be on the A15 cluster when suspending the system.
		 */
		for_each_online_cpu(i) {
			policy = cpufreq_cpu_get(i);
			/* Save/restore maximum scaling frequency. */
			saved_max = policy->user_policy.max;
			policy->user_policy.max = maxfreq_ca15;
			cpufreq_update_policy(i);
			exynos_target(policy, maxfreq_ca15,
						CPUFREQ_RELATION_H);
			policy->user_policy.max = saved_max;
			cpufreq_update_policy(i);
			cpufreq_cpu_put(policy);
		}

		pr_debug("PM_SUSPEND_PREPARE for CPUFREQ\n");
		break;
	case PM_POST_SUSPEND:
		pr_debug("PM_POST_SUSPEND for CPUFREQ\n");

		mutex_lock(&cpufreq_lock);
		exynos_info[CA7]->blocked = false;
		exynos_info[CA15]->blocked = false;
		mutex_unlock(&cpufreq_lock);

		break;
	}
	return NOTIFY_OK;
err:
	pr_err("%s: failed to set voltage\n", __func__);

	return NOTIFY_BAD;
}

static struct notifier_block exynos_cpufreq_nb = {
	.notifier_call = exynos_cpufreq_pm_notifier,
};

static int exynos_cpufreq_reboot_notifier(struct notifier_block *this,
				unsigned long code, void *_cmd)
{
	struct cpufreq_policy *policy;
	unsigned int i;

	mutex_lock(&cpufreq_lock);
	exynos_info[CA7]->blocked = true;
	exynos_info[CA15]->blocked = true;
	mutex_unlock(&cpufreq_lock);

	for_each_online_cpu(i) {
		policy = cpufreq_cpu_get(i);
		policy->user_policy.max = freq_max[CA15];
		cpufreq_update_policy(i);
		exynos_target(policy, freq_max[CA15], CPUFREQ_RELATION_H);
		cpufreq_cpu_put(policy);
	}
	return NOTIFY_DONE;
}

static struct notifier_block exynos_cpufreq_reboot_nb = {
	.notifier_call = &exynos_cpufreq_reboot_notifier,
};

static int exynos_policy_notifier(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int cpu = policy->cpu;

	if (val != CPUFREQ_ADJUST)
		return 0;

	if (!cpu_online(cpu))
		return -EINVAL;

	if (policy->max <= freq_max[CA7]) {
		fake_policy[CA7][cpu].max = ACTUAL_FREQ(policy->max, CA7);
		fake_policy[CA15][cpu].max = freq_max[CA15];
	} else {
		fake_policy[CA7][cpu].max = ACTUAL_FREQ(freq_max[CA7], CA7);
		fake_policy[CA15][cpu].max = policy->max;
	}

	if (policy->min <= freq_max[CA7]) {
		fake_policy[CA7][cpu].min = ACTUAL_FREQ(policy->min, CA7);
		fake_policy[CA15][cpu].min = freq_min[CA15];
	} else {
		fake_policy[CA7][cpu].min = ACTUAL_FREQ(freq_min[CA7], CA7);
		fake_policy[CA15][cpu].min = policy->min;
	}

	return 0;
}

static struct notifier_block notifier_policy_block = {
	.notifier_call = exynos_policy_notifier,
};

static int exynos_bL_switcher_notifier(struct notifier_block *nb,
					unsigned long val, void *data)
{
	if (val == BL_NOTIFY_PRE_DISABLE) {
		pr_warn("Disabling the bL_switcher is not supported\n");
		return NOTIFY_BAD;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_bL_switcher_nb = {
	.notifier_call = exynos_bL_switcher_notifier,
};

static int exynos_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	struct cpufreq_policy *cpu0_policy;
	int i;

	policy->cur = policy->min = policy->max = exynos_getspeed(policy->cpu);
	freqs[CA7]->old = exynos_getspeed_cluster(CA7);
	freqs[CA15]->old = exynos_getspeed_cluster(CA15);

	cpufreq_frequency_table_get_attr(merge_freq_table, policy->cpu);

	/*
	 * set the time it takes on this CPU to switch between two frequencies
	 * in nano seconds.
	 */
	policy->cpuinfo.transition_latency = 100000;

	cpumask_clear(policy->cpus);
	if (use_cluster_switching) {
		for_each_online_cpu(i)
			cpumask_set_cpu(i, policy->cpus);
	} else {
		cpumask_set_cpu(policy->cpu, policy->cpus);
	}

	cpu0_policy = cpufreq_cpu_get(0);
	/*
	 * Secondary CPUs are disabled before suspend, so any policy settings
	 * are lost.  Instead of having the policy being reset to default
	 * values, copy the settings in CPU0's policy (which are preserved)
	 * in case the CPUs were throttled before suspend.
	 */
	if (policy->cpu != 0 && cpu0_policy) {
		policy->max = cpu0_policy->max;
		policy->min = cpu0_policy->min;
		policy->cpuinfo.max_freq = cpu0_policy->cpuinfo.max_freq;
		policy->cpuinfo.min_freq = cpu0_policy->cpuinfo.min_freq;
		cpufreq_cpu_put(cpu0_policy);
		return 0;
	} else {
		return cpufreq_frequency_table_cpuinfo(policy,
							merge_freq_table);
	}
}

static int exynos_cpufreq_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);
	return 0;
}

static struct freq_attr *exynos_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver exynos_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= exynos_verify_speed,
	.target		= exynos_target,
	.get		= exynos_getspeed,
	.init		= exynos_cpufreq_cpu_init,
	.exit		= exynos_cpufreq_cpu_exit,
	.name		= "exynos_cpufreq",
	.attr		= exynos_cpufreq_attr,
#ifdef CONFIG_PM
	.suspend	= exynos_cpufreq_suspend,
	.resume		= exynos_cpufreq_resume,
#endif

};

int __init exynos_iks_cpufreq_init(void)
{
	int ret = -EINVAL;
	int cpu;

	boot_cluster = 0;

	if (soc_is_exynos5420())
		use_cluster_switching = true;

	if (!bL_switcher_get_enabled()) {
		bL_switcher_put_enabled();
		ret = -EINVAL;
		goto err_bL_enable;
	}
	bL_switcher_register_notifier(&exynos_bL_switcher_nb);
	bL_switcher_put_enabled();

	exynos_info[CA7] = kzalloc(sizeof(struct exynos_dvfs_info),
					GFP_KERNEL);
	if (!exynos_info[CA7]) {
		ret = -ENOMEM;
		goto err_alloc_info_CA7;
	}

	exynos_info[CA15] = kzalloc(sizeof(struct exynos_dvfs_info),
					GFP_KERNEL);
	if (!exynos_info[CA15]) {
		ret = -ENOMEM;
		goto err_alloc_info_CA15;
	}

	freqs[CA7] = kzalloc(sizeof(struct cpufreq_freqs), GFP_KERNEL);
	if (!freqs[CA7]) {
		ret = -ENOMEM;
		goto err_alloc_freqs_CA7;
	}

	freqs[CA15] = kzalloc(sizeof(struct cpufreq_freqs), GFP_KERNEL);
	if (!freqs[CA15]) {
		ret = -ENOMEM;
		goto err_alloc_freqs_CA15;
	}

	/* Get to boot_cluster_num - 0 for CA7; 1 for CA15 */
	boot_cluster = !(read_cpuid(CPUID_MPIDR) >> 8 & 0xf);
	pr_debug("%s: boot_cluster is %s\n", __func__,
					boot_cluster == CA7 ? "CA7" : "CA15");

	init_cpumask_cluster_set(boot_cluster);

	ret = exynos5420_cpufreq_CA7_init(&exynos_info_CA7);
	if (ret)
		goto err_init_cpufreq;

	ret = exynos5420_cpufreq_init(&exynos_info_CA15);
	if (ret)
		goto err_init_cpufreq;

	arm_regulator = regulator_get(NULL, "vdd_arm");
	if (IS_ERR(arm_regulator)) {
		pr_err("%s: failed to get resource vdd_arm\n", __func__);
		goto err_init_cpufreq;
	}

	kfc_regulator = regulator_get(NULL, "vdd_kfc");
	if (IS_ERR(kfc_regulator)) {
		pr_err("%s:failed to get resource vdd_kfc\n", __func__);
		goto err_vdd_kfc;
	}

	memcpy(exynos_info[CA7], &exynos_info_CA7,
				sizeof(struct exynos_dvfs_info));
	exynos_info[CA7]->regulator = kfc_regulator;

	memcpy(exynos_info[CA15], &exynos_info_CA15,
				sizeof(struct exynos_dvfs_info));
	exynos_info[CA15]->regulator = arm_regulator;

	if (exynos_info[CA7]->set_freq == NULL) {
		pr_err("%s: No set_freq function (ERR)\n", __func__);
		goto err_set_freq;
	}

	freq_max[CA15] = exynos_info[CA15]->
		freq_table[exynos_info[CA15]->max_support_idx].frequency;
	freq_min[CA15] = exynos_info[CA15]->
		freq_table[exynos_info[CA15]->min_support_idx].frequency;
	freq_max[CA7] = VIRT_FREQ(exynos_info[CA7]->
		freq_table[exynos_info[CA7]->max_support_idx].frequency, CA7);
	freq_min[CA7] = VIRT_FREQ(exynos_info[CA7]->
		freq_table[exynos_info[CA7]->min_support_idx].frequency, CA7);
	cpufreq_merge_tables();

	register_pm_notifier(&exynos_cpufreq_nb);
	register_reboot_notifier(&exynos_cpufreq_reboot_nb);

	for_each_cpu(cpu, cpu_possible_mask) {
		fake_policy[CA15][cpu].cpu = cpu;
		fake_policy[CA15][cpu].max = freq_max[CA15];
		fake_policy[CA15][cpu].min = freq_min[CA15];
		fake_policy[CA7][cpu].max = ACTUAL_FREQ(freq_max[CA7], CA7);
		fake_policy[CA7][cpu].min = ACTUAL_FREQ(freq_min[CA7], CA7);
	}

	cpufreq_register_notifier(&notifier_policy_block,
					CPUFREQ_POLICY_NOTIFIER);

	if (cpufreq_register_driver(&exynos_driver)) {
		pr_err("%s: failed to register cpufreq driver\n", __func__);
		goto err_cpufreq;
	}

	return 0;

err_cpufreq:
	unregister_reboot_notifier(&exynos_cpufreq_reboot_nb);
	unregister_pm_notifier(&exynos_cpufreq_nb);
err_set_freq:
	regulator_put(kfc_regulator);
err_vdd_kfc:
	regulator_put(arm_regulator);
err_init_cpufreq:
	kfree(freqs[CA15]);
err_alloc_freqs_CA15:
	kfree(freqs[CA7]);
err_alloc_freqs_CA7:
	kfree(exynos_info[CA15]);
err_alloc_info_CA15:
	kfree(exynos_info[CA7]);
err_alloc_info_CA7:
	bL_switcher_unregister_notifier(&exynos_bL_switcher_nb);
err_bL_enable:
	pr_err("%s: failed initialization\n", __func__);

	return ret;
}
