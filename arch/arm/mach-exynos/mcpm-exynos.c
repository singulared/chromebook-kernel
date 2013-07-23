/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * arch/arm/mach-exynos/mcpm-exynos.c
 *
 * Based on arch/arm/mach-vexpress/dcscb.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/arm-cci.h>

#include <asm/mcpm.h>
#include <asm/cputype.h>
#include <asm/cp15.h>

#include <plat/cpu.h>
#include <mach/regs-pmu.h>

/* CPU boot mode flags */
#define EXYNOS_RESET		(1 << 0)
#define EXYNOS_HOTPLUG		(1 << 1)
#define EXYNOS_BOOT_MODE_MASK	0x1f
#define REG_CPU_STATE_ADDR	(S5P_VA_SYSRAM_NS + 0x28)

/* Secondary CPU entry point */
#define REG_ENTRY_ADDR		(S5P_VA_SYSRAM_NS + 0x1C)

static void set_boot_flag(unsigned int cpu, unsigned int mode)
{
	unsigned int tmp;

	tmp = __raw_readl(REG_CPU_STATE_ADDR + (cpu * 4));
	if (mode & EXYNOS_BOOT_MODE_MASK)
		tmp &= ~EXYNOS_BOOT_MODE_MASK;
	tmp |= mode;
	__raw_writel(tmp, REG_CPU_STATE_ADDR + (cpu * 4));
}

static void clear_boot_flag(unsigned int cpu, unsigned int mode)
{
	unsigned int tmp;

	tmp = __raw_readl(REG_CPU_STATE_ADDR + (cpu * 4));
	tmp &= ~mode;
	__raw_writel(tmp, REG_CPU_STATE_ADDR + (cpu * 4));
}

/*
 * We can't use regular spinlocks. In the switcher case, it is possible
 * for an outbound CPU to call power_down() after its inbound counterpart
 * is already live using the same logical CPU number which trips lockdep
 * debugging.
 */
static arch_spinlock_t bl_lock = __ARCH_SPIN_LOCK_UNLOCKED;
static int cpu_use_count[4][2];

static void exynos_core_power_control(unsigned int cpu, unsigned int cluster,
						int enable)
{
	unsigned int status, val = 0;

	if (cluster)
		cpu += 4;
	if (enable)
		val = EXYNOS_CORE_LOCAL_PWR_EN;

	status = __raw_readl(EXYNOS_ARM_CORE_STATUS(cpu));
	if ((status & EXYNOS_CORE_LOCAL_PWR_EN) == val)
		return;
	__raw_writel(val, EXYNOS_ARM_CORE_CONFIGURATION(cpu));
}

static void exynos_cluster_power_control(unsigned int cluster, int enable)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(20);
	unsigned int status, val = 0;

	if (enable)
		val = EXYNOS_CORE_LOCAL_PWR_EN;

	status = __raw_readl(EXYNOS_COMMON_STATUS(cluster));
	if ((status & EXYNOS_CORE_LOCAL_PWR_EN) == val)
		return;

	__raw_writel(val, EXYNOS_COMMON_CONFIGURATION(cluster));
	/* Wait until cluster power control is applied */
	while (time_before(jiffies, timeout)) {
		status = __raw_readl(EXYNOS_COMMON_STATUS(cluster));

		if ((status & EXYNOS_CORE_LOCAL_PWR_EN) == val)
			return;

		mdelay(1);
	}
	pr_warn("timed out waiting for cluster %u to power %s\n", cluster,
		enable ? "on" : "off");
}

static int exynos_power_up(unsigned int cpu, unsigned int cluster)
{
	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	if (cpu >= 4 || cluster >= 2)
		return -EINVAL;

	/*
	 * Since this is called with IRQs enabled, and no arch_spin_lock_irq
	 * variant exists, we need to disable IRQs manually here.
	 */
	local_irq_disable();
	arch_spin_lock(&bl_lock);

	cpu_use_count[cpu][cluster]++;
	if (cpu_use_count[cpu][cluster] == 1) {
		if (__mcpm_cluster_state(cluster) == CLUSTER_DOWN)
			exynos_cluster_power_control(cluster, 1);
		exynos_core_power_control(cpu, cluster, 1);
		set_boot_flag(cpu, EXYNOS_HOTPLUG);
		__raw_writel(virt_to_phys(mcpm_entry_point),
				REG_ENTRY_ADDR);

		/* CPU should be powered up there */
		/* Also setup Cluster Power Sequence here */
	} else if (cpu_use_count[cpu][cluster] != 2) {
		/*
		 * The only possible values are:
		 * 0 = CPU down
		 * 1 = CPU (still) up
		 * 2 = CPU requested to be up before it had a chance
		 *     to actually make itself down.
		 * Any other value is a bug.
		 */
		BUG();
	}

	arch_spin_unlock(&bl_lock);
	local_irq_enable();

	return 0;
}

static void clear_coherency(void)
{
	unsigned long v, u;

	asm volatile(
		"mrc	p15, 0, %0, c1, c0, 1\n"
		"bic	%0, %0, %2\n"
		"ldr	%1, [%3]\n"
		"and	%1, %1, #0\n"
		"orr	%0, %0, %1\n"
		"mcr	p15, 0, %0, c1, c0, 1\n"
		: "=&r" (v), "=&r" (u)
		: "Ir" (0x40), "Ir" (S5P_INFORM0)
		: "cc");
}

static void exynos_power_down(void)
{
	unsigned int mpidr, cpu, cluster, cpumask;
	bool last_man = false, skip_wfi = false;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	cpumask = (1 << cpu);

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	BUG_ON(cpu >= 4 || cluster >= 2);

	__mcpm_cpu_going_down(cpu, cluster);

	arch_spin_lock(&bl_lock);
	BUG_ON(__mcpm_cluster_state(cluster) != CLUSTER_UP);
	cpu_use_count[cpu][cluster]--;
	if (cpu_use_count[cpu][cluster] == 0) {
		int cnt = 0, i;

		exynos_core_power_control(cpu, cluster, 0);
		for (i = 0; i < 4; i++)
			cnt += cpu_use_count[i][cluster];
		if (cnt == 0)
			last_man = 1;
	} else if (cpu_use_count[cpu][cluster] == 1) {
		/*
		 * A power_up request went ahead of us.
		 * Even if we do not want to shut this CPU down,
		 * the caller expects a certain state as if the WFI
		 * was aborted.  So let's continue with cache cleaning.
		 */
		skip_wfi = true;
	} else {
		BUG();
	}

	if (last_man && __mcpm_outbound_enter_critical(cpu, cluster)) {
		arch_spin_unlock(&bl_lock);

		/*
		 * Flush all cache levels for this cluster.
		 *
		 * A15/A7 can hit in the cache with SCTLR.C=0, so we don't need
		 * a preliminary flush here for those CPUs.  At least, that's
		 * the theory -- without the extra flush, Linux explodes on
		 * RTSM (maybe not needed anymore, to be investigated).
		 */
		set_cr(get_cr() & ~CR_C);
		dsb();
		flush_cache_louis();
		flush_cache_all();

		/*
		 * This is a harmless no-op.  On platforms with a real
		 * outer cache this might either be needed or not,
		 * depending on where the outer cache sits.
		 */
		outer_flush_all();

		/* Disable local coherency by clearing the ACTLR "SMP" bit: */
		clear_coherency();

		/*
		 * Disable cluster-level coherency by masking
		 * incoming snoops and DVM messages:
		 */
		cci_control_port_by_cpu(mpidr, false);

		__mcpm_outbound_leave_critical(cluster, CLUSTER_DOWN);
	} else {
		arch_spin_unlock(&bl_lock);

		/*
		 * Flush the local CPU cache.
		 *
		 * A15/A7 can hit in the cache with SCTLR.C=0, so we don't need
		 * a preliminary flush here for those CPUs.  At least, that's
		 * the theory -- without the extra flush, Linux explodes on
		 * RTSM (maybe not needed anymore, to be investigated).
		 */
		set_cr(get_cr() & ~CR_C);
		dsb();
		flush_cache_louis();

		/* Disable local coherency by clearing the ACTLR "SMP" bit: */
		clear_coherency();
	}

	/* Now we are prepared for power-down, do it: */
	if (!skip_wfi) {
		dsb();
		dmb();
		if (last_man)
			cci_control_port_by_cpu(mpidr, false);
		__mcpm_cpu_down(cpu, cluster);
		wfi();
	} else {
		__mcpm_cpu_down(cpu, cluster);
	}

	/* Not dead at this point?  Let our caller cope. */
}

static void exynos_powered_up(bool switched)
{
	unsigned int mpidr, cpu, cluster;
	unsigned long timeout = jiffies + msecs_to_jiffies(10);

	mpidr = read_cpuid_mpidr();
	cpu = mpidr & 0xff;
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	cluster ^= 1;
	clear_boot_flag(cpu, EXYNOS_HOTPLUG);

	/*
	 * Only wait for the corresponding CPU in the other cluster to go
	 * down if we were powered up because of a switch.
	 */
	if (!switched)
		return;

	while (!__mcpm_is_cpu_down(cpu, cluster) &&
			time_before(jiffies, timeout))
		cpu_relax();

	if (!__mcpm_is_cpu_down(cpu, cluster))
		pr_warn("timed out waiting for CPU %u to power off\n",
			cpu + cluster * 4);

	udelay(10);

	local_irq_disable();
	arch_spin_lock(&bl_lock);
	if (__mcpm_cluster_state(cluster) == CLUSTER_DOWN)
		exynos_cluster_power_control(cluster, 0);
	arch_spin_unlock(&bl_lock);
	local_irq_enable();
}

static const struct mcpm_platform_ops exynos_power_ops = {
	.power_up	= exynos_power_up,
	.power_down	= exynos_power_down,
	.powered_up	= exynos_powered_up,
};

static void __init exynos_usage_count_init(void)
{
	unsigned int mpidr, cpu, cluster, i;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	BUG_ON(cpu >= 4 || cluster >= 2);

	for (i = 0; i < 4; i++) {
		cpu_use_count[i][0] = 0;
		cpu_use_count[i][1] = 0;
	}
	cpu_use_count[cpu][cluster] = 1;
}

static size_t bL_check_status(char *info)
{
	size_t len = 0;
	int i;
	void __iomem *cci_base;

	cci_base = ioremap(EXYNOS5_PA_CCI, SZ_64K);
	if (!cci_base)
		return -ENOMEM;

	len += sprintf(&info[len], "\t0 1 2 3 L2 COMMON CCI\n");
	len += sprintf(&info[len], "[A15]   ");
	for (i = 0; i < 4; i++) {
		len += sprintf(&info[len], "%d ",
			(readl(EXYNOS_ARM_CORE_STATUS(i)) & 0x3) == 3 ? 1 : 0);
	}
	len += sprintf(&info[len], " %d",
			(readl(EXYNOS_L2_STATUS(0)) & 0x3) == 3 ? 1 : 0);
	len += sprintf(&info[len], "    %d",
			(readl(EXYNOS_COMMON_STATUS(0)) & 0x3) == 3 ? 1 : 0);
	len += sprintf(&info[len], "    %d\n",
			(readl(cci_base + 0x4000 + 1 * 0x1000) & 0x3)
								== 3 ? 1 : 0);

	len += sprintf(&info[len], "[A7]    ");
	for (i = 4; i < 8; i++)
		len += sprintf(&info[len], "%d ",
			(readl(EXYNOS_ARM_CORE_STATUS(i)) & 0x3) == 3 ? 1 : 0);
	len += sprintf(&info[len], " %d",
			(readl(EXYNOS_L2_STATUS(1)) & 0x3) == 3 ? 1 : 0);
	len += sprintf(&info[len], "    %d",
			(readl(EXYNOS_COMMON_STATUS(1)) & 0x3) == 3 ? 1 : 0);
	len += sprintf(&info[len], "    %d\n\n",
			(readl(cci_base + 0x4000 + 0 * 0x1000) & 0x3)
								== 3 ? 1 : 0);

	iounmap(cci_base);

	return len;
}

static ssize_t bL_status_read(struct file *file, char __user *buf,
			size_t len, loff_t *pos)
{
	size_t count = 0;
	char info[100];

	count = bL_check_status(info);
	if (count < 0)
		return -EINVAL;

	return simple_read_from_buffer(buf, len, pos, info, count);
}

static const struct file_operations bL_status_fops = {
	.read		= bL_status_read,
};

static struct miscdevice bL_status_device = {
	MISC_DYNAMIC_MINOR,
	"bL_status",
	&bL_status_fops
};

extern void exynos_power_up_setup(unsigned int affinity_level);

static int __init exynos_mcpm_init(void)
{
	int i, ret = 0;
	unsigned long temp;

	if (!soc_is_exynos5420())
		return 0;

	clear_boot_flag(0, EXYNOS_RESET);

	/* disable ARM/KFC L2 Cache retention */
	for (i = 0; i < MAX_NR_CLUSTERS; i++)
		writel(0x0, EXYNOS_L2_OPTION(i));

	dsb();

	for (i = 0; i < MAX_NR_CLUSTERS; i++) {
		temp = readl(EXYNOS_COMMON_OPTION(i));
		temp &= ~0x1;
		temp |= 0x2;
		writel(temp, EXYNOS_COMMON_OPTION(i));
	}

	exynos_usage_count_init();

	ret = mcpm_platform_register(&exynos_power_ops);
	if (!ret)
		ret = mcpm_sync_init(exynos_power_up_setup);

	return ret;
}

early_initcall(exynos_mcpm_init);

static int __init exynos_bl_status_init(void)
{
	int ret;

	if (!soc_is_exynos5420())
		return 0;

	ret = misc_register(&bL_status_device);
	if (ret)
		pr_info("bl_status not available\n");
	return 0;
}

late_initcall(exynos_bl_status_init);
