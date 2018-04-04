/*
 *
 * (C) COPYRIGHT 2012-2016 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */



#define CREATE_TRACE_POINTS

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/semaphore.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>

#include <mach/map.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <mach/regs-clock.h>
#include <mach/pmu.h>
#include <mach/regs-pmu.h>
#include <asm/delay.h>
#include <mach/map.h>
#include <generated/autoconf.h>
#include <plat/cpu.h>
#if defined(CONFIG_MALI_MIDGARD_DVFS) && defined(CONFIG_CPU_FREQ)
#include <mach/asv-5250.h>
#include <mach/asv-exynos.h>
#endif

#include <linux/timer.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

#include <mali_kbase.h>
#include <mali_kbase_pm.h>
#include <mali_kbase_mem.h>
#include <mali_midg_regmap.h>
#include <mali_kbase_mem_linux.h>
#include <mali_kbase_defs.h>
#include <backend/gpu/mali_kbase_pm_internal.h>

#include "mali_linux_dvfs_trace.h"

#define MALI_DVFS_DEBUG 0
#define MALI_DVFS_STEP 8
#define MALI_DVFS_KEEP_STAY_CNT 10

#ifdef CONFIG_MALI_MIDGARD_DVFS
#define KBASE_PM_DVFS_FREQUENCY (100) /* 100ms */
#ifdef CONFIG_CPU_FREQ
#define MALI_DVFS_ASV_ENABLE
#endif
#endif

#ifdef MALI_DVFS_ASV_ENABLE
#define MALI_DVFS_ASV_GROUP_SPECIAL_NUM 10
#define MALI_DVFS_ASV_GROUP_NUM 13
#endif

#define HZ_IN_MHZ            (1000000)
#define MALI_RTPM_DEBUG      0
#define RUNTIME_PM_DELAY_TIME 10
#define CONFIG_MIDGARD_HWVER_R0P0 1
#define G3D_ASV_VOL_OFFSET	25000

struct regulator *kbase_platform_get_regulator(void);
int kbase_platform_regulator_init(void);
int kbase_platform_regulator_disable(void);
int kbase_platform_regulator_enable(void);
int kbase_platform_get_default_voltage(struct device *dev, int *vol);
int kbase_platform_get_voltage(struct device *dev, int *vol);
#if defined CONFIG_MALI_MIDGARD_DVFS
static int kbase_platform_set_voltage(struct device *dev, int vol);
static void kbase_platform_dvfs_set_clock(struct kbase_device *kbdev, int freq);
static void kbase_platform_dvfs_set_level(struct kbase_device *kbdev, int level);
static int kbase_platform_dvfs_get_level(int freq);
#endif

struct mali_dvfs_info {
	unsigned int voltage;
	unsigned int clock;
	int min_threshold;
	int max_threshold;
	int up_cnt_threshold;
	int down_cnt_threshold;
};
static struct mali_dvfs_info *mali_dvfs_infotbl;

#ifdef CONFIG_MALI_MIDGARD_DVFS
/*
 * Governor parameters.  The governor gets periodic samples of the
 * GPU utilisation (%busy) and maintains a weighted average over the
 * last DVFS_AVG_LPF_LEN values.  When the average is in the range
 * [min_threshold..max_threshold] we maintain the current clock+voltage.
 * If the utilisation drops below min for down_cnt_threshold samples
 * we step down.  If the utilisation exceeds max_threshold for
 * up_cnt_threshold samples we step up.
 *
 * The up/down thresholds are chosen to enable fast step up under
 * load with a longer step down; this optimizes for performance over
 * power consumption.  266MHz is the "sweet spot"; it has the best
 * performance/power ratio.  For this reason it has slightly extended
 * up/down thresholds to make it "sticky".
 */
/* TODO(sleffler) round or verify time is a multiple of frequency */
/* convert a time in milliseconds to a dvfs sample count */
#define	DVFS_TIME_TO_CNT(t)	((t) / KBASE_PM_DVFS_FREQUENCY)

/* TODO(sleffler) should be const but for voltage */
static struct mali_dvfs_info mali_dvfs_infotbl_exynos5250[MALI_DVFS_STEP] = {
/*
 * A dummy level is added in 5250 and 5422 to make 8 levels. In 5250 since the
 * max_threshold value of step 6 is 100, the dvfs code will never come to dummy
 * step 7.
 */
#if (MALI_DVFS_STEP == 8)
	{ 912500, 100000000,  0,   0, DVFS_TIME_TO_CNT(0), DVFS_TIME_TO_CNT(0)},
	{ 925000, 160000000,  0,   0, DVFS_TIME_TO_CNT(0), DVFS_TIME_TO_CNT(0)},
	{1025000, 266000000,  0,  85, DVFS_TIME_TO_CNT(1000), DVFS_TIME_TO_CNT(3000)},
	{1075000, 350000000, 65,  85, DVFS_TIME_TO_CNT(750), DVFS_TIME_TO_CNT(1500)},
	{1125000, 400000000, 65,  85, DVFS_TIME_TO_CNT(750), DVFS_TIME_TO_CNT(1500)},
	{1025000, 450000000, 65,  90, DVFS_TIME_TO_CNT(1000), DVFS_TIME_TO_CNT(1500)},
	{1250000, 533000000, 75, 100, DVFS_TIME_TO_CNT(750), DVFS_TIME_TO_CNT(1500)},
	{1250000, 533000000, 75, 100, DVFS_TIME_TO_CNT(750), DVFS_TIME_TO_CNT(1500)}

#else
#error no table
#endif
};

static struct mali_dvfs_info mali_dvfs_infotbl_exynos5420[MALI_DVFS_STEP] = {
#if (MALI_DVFS_STEP == 8)
	{ 862500, 100000000,  0,  60, DVFS_TIME_TO_CNT(750), DVFS_TIME_TO_CNT(2000)},
	{ 862500, 177000000, 40,  75, DVFS_TIME_TO_CNT(750), DVFS_TIME_TO_CNT(2000)},
	{ 862500, 266000000, 65,  85, DVFS_TIME_TO_CNT(1000), DVFS_TIME_TO_CNT(3000)},
	{ 900000, 350000000, 65,  85, DVFS_TIME_TO_CNT(750), DVFS_TIME_TO_CNT(1500)},
	{ 937500, 420000000, 65,  85, DVFS_TIME_TO_CNT(750), DVFS_TIME_TO_CNT(1500)},
	{ 950000, 480000000, 65, 100, DVFS_TIME_TO_CNT(1000), DVFS_TIME_TO_CNT(1500)},
	{ 987500, 533000000, 75, 100, DVFS_TIME_TO_CNT(750), DVFS_TIME_TO_CNT(1500)},
	{1025000, 600000000, 75, 100, DVFS_TIME_TO_CNT(750), DVFS_TIME_TO_CNT(1500)}

#else
#error no table
#endif
};

static struct mali_dvfs_info mali_dvfs_infotbl_exynos5422[MALI_DVFS_STEP] = {
#if (MALI_DVFS_STEP == 8)
	{  825000, 100000000,  0,  60, DVFS_TIME_TO_CNT(750),
					DVFS_TIME_TO_CNT(2000)},
	{  825000, 177000000, 40,  75, DVFS_TIME_TO_CNT(750),
					DVFS_TIME_TO_CNT(2000)},
	{  887500, 266000000, 65,  85, DVFS_TIME_TO_CNT(1000),
					DVFS_TIME_TO_CNT(3000)},
	{  912500, 350000000, 65,  85, DVFS_TIME_TO_CNT(750),
					DVFS_TIME_TO_CNT(1500)},
	{  937500, 420000000, 65,  85, DVFS_TIME_TO_CNT(750),
					DVFS_TIME_TO_CNT(1500)},
	{  975000, 480000000, 65, 100, DVFS_TIME_TO_CNT(1000),
					DVFS_TIME_TO_CNT(1500)},
	{ 1025000, 543000000, 75, 100, DVFS_TIME_TO_CNT(750),
					DVFS_TIME_TO_CNT(1500)},
	{ 1025000, 543000000, 75, 100, DVFS_TIME_TO_CNT(750),
					DVFS_TIME_TO_CNT(1500)},

#else
#error no table
#endif
};

int kbase_platform_dvfs_init(struct kbase_device *kbdev);
void kbase_platform_dvfs_term(void);
int kbase_platform_dvfs_get_control_status(void);
int kbase_platform_dvfs_get_utilisation(void);

#ifdef MALI_DVFS_ASV_ENABLE

static const unsigned int (*mali_dvfs_asv_vol_tbl_special)[MALI_DVFS_STEP];
static const unsigned int (*mali_dvfs_asv_vol_tbl)[MALI_DVFS_STEP];
static const unsigned int *mali_dvfs_vol_default;

static const unsigned int mali_dvfs_asv_vol_tbl_special_exynos5250
	[MALI_DVFS_ASV_GROUP_SPECIAL_NUM][MALI_DVFS_STEP] = {
	/*  100Mh   160Mh     266Mh   350Mh             400Mh   450Mh   533Mh*/
	{/*Group 1*/
		912500, 925000, 1025000, 1075000, 1100000, 1150000, 1225000,
		1225000,
	},
	{/*Group 2*/
		900000, 900000, 1000000, 1037500, 1087500, 1125000, 1200000,
		1200000,
	},
	{/*Group 3*/
		912500, 925000, 1025000, 1037500, 1100000, 1150000, 1225000,
		1225000,
	},
	{/*Group 4*/
		900000, 900000, 1000000, 1025000, 1087500, 1125000, 1200000,
		1200000,
	},
	{/*Group 5*/
		912500, 925000, 1000000, 1000000, 1125000, 1150000, 1250000,
		1250000,
	},
	{/*Group 6*/
		900000, 912500, 987500, 987500, 1112500, 1150000, 1237500,
		1237500,
	},
	{/*Group 7*/
		900000, 900000, 975000, 987500, 1100000, 1137500, 1225000,
		1225000,
	},
	{/*Group 8*/
		900000, 900000, 975000, 987500, 1100000, 1137500, 1225000,
		1225000,
	},
	{/*Group 9*/
		887500, 900000, 962500, 975000, 1087500, 1125000, 1212500,
		1212500,
	},
	{/*Group 10*/
		887500, 900000, 962500, 962500, 1087500, 1125000, 1212500,
		1212500,
	},
};

static const unsigned int mali_dvfs_asv_vol_tbl_exynos5250
	[MALI_DVFS_ASV_GROUP_NUM][MALI_DVFS_STEP] = {
       /*  100Mh       160Mh      266Mh        350Mh,  400Mh   450Mh   533Mh*/
	{/*Group 0*/
		925000, 925000, 1025000, 1075000, 1125000, 1150000, 1200000,
		1200000,
	},
	{/*Group 1*/
		900000, 900000, 1000000, 1037500, 1087500, 1137500, 1187500,
		1187500,
	},
	{/*Group 2*/
		900000, 900000, 950000, 1037500, 1075000, 1125000, 1187500,
		1187500,
	},
	{/*Group 3*/
		900000, 900000, 950000, 1037500, 1075000, 1125000, 1187500,
		1187500,
	},
	{/*Group 4*/
		900000, 900000, 937500, 1025000, 1075000, 1112500, 1175000,
		1175000,
	},
	{/*Group 5*/
		900000, 900000, 937500, 1000000, 1050000, 1100000, 1150000,
		1150000,
	},
	{/*Group 6*/
		900000, 900000, 925000, 987500, 1037500, 1087500, 1137500,
		1137500,
	},
	{/*Group 7*/
		900000, 900000, 912500, 987500, 1025000, 1075000, 1125000,
		1125000,
	},
	{/*Group 8*/
		900000, 900000, 912500, 987500, 1012500, 1075000, 1125000,
		1112500,
	},
	{/*Group 9*/
		900000, 900000, 900000, 975000, 1012500, 1050000, 1125000,
		1112500,
	},
	{/*Group 10*/
		875000, 900000, 900000, 962500, 1000000, 1050000, 1112500,
		1112500,
	},
	{/*Group 11*/
		875000, 900000, 900000, 962500, 1000000, 1050000, 1112500,
		1112500,
	},
	{/*Group 12*/ /*Same as Group 11 because other socs may need more ASV groups*/
		875000, 900000, 900000, 962500, 1000000, 1050000, 1112500,
		1112500,
	},
};

static const unsigned int mali_dvfs_vol_default_exynos5250[MALI_DVFS_STEP] = {
	 925000, 925000, 1025000, 1075000, 1125000, 1150000, 1200000, 1200000
};

static int kbase_platform_asv_set(int enable);
#endif /* MALI_DVFS_ASV_ENABLE */
#endif /* CONFIG_MALI_MIDGARD_DVFS */

int kbase_platform_cmu_pmu_control(struct kbase_device *kbdev, int control);
void kbase_platform_remove_sysfs_file(struct device *dev);
int kbase_platform_init(struct kbase_device *kbdev);
void kbase_platform_term(struct kbase_device *kbdev);
#ifdef CONFIG_MALI_MIDGARD_DVFS
static void kbase_platform_dvfs_set_max(struct kbase_device *kbdev);
#endif /* CONFIG_MALI_MIDGARD_DVFS */

static int kbase_platform_create_sysfs_file(struct device *dev);

#ifdef CONFIG_MALI_MIDGARD_DVFS
static struct mali_dvfs_status mali_dvfs_status_current;
#endif /* CONFIG_MALI_MIDGARD_DVFS */

struct exynos_context
{
	/** Indicator if system clock to mail-t604 is active */
	int cmu_pmu_status;
	/** cmd & pmu lock */
	spinlock_t cmu_pmu_lock;
	struct clk *sclk_g3d;
	int t6xx_default_clock;
};

/**
 * Read the CPU clock speed
 */
int get_cpu_clock_speed(u32* cpu_clock)
{
	struct clk * cpu_clk;
	u32 freq=0;
	cpu_clk = clk_get(NULL, "armclk");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);
	freq = clk_get_rate(cpu_clk);
	*cpu_clock = (freq/HZ_IN_MHZ);
	return 0;
}

/**
 * Power Management callback - power ON
 */
static int pm_callback_power_on(struct kbase_device *kbdev)
{
#ifdef KBASE_PM_RUNTIME
	pm_runtime_resume(kbdev->dev);
#endif /* KBASE_PM_RUNTIME */
	return 0;
}

/**
 * Power Management callback - power OFF
 */
static void pm_callback_power_off(struct kbase_device *kbdev)
{
#ifdef KBASE_PM_RUNTIME
	pm_schedule_suspend(kbdev->dev, RUNTIME_PM_DELAY_TIME);
#endif /* KBASE_PM_RUNTIME */
}

/**
 * Power Management callback - suspend
 */
static void pm_callback_suspend(struct kbase_device *kbdev)
{
#ifdef CONFIG_MALI_MIDGARD_DVFS
	kbase_platform_dvfs_set_max(kbdev);
#endif /* CONFIG_MALI_MIDGARD_DVFS */
}

#ifdef KBASE_PM_RUNTIME
static int pm_callback_runtime_idle(struct kbase_device *kbdev)
{
	/* Avoid pm_runtime_suspend being called. */
	return 1;
}
#endif /* KBASE_PM_RUNTIME */

struct kbase_pm_callback_conf pm_callbacks =
{
	.power_on_callback = pm_callback_power_on,
	.power_off_callback = pm_callback_power_off,
	.power_suspend_callback = pm_callback_suspend,
	.power_resume_callback = NULL,
#ifdef KBASE_PM_RUNTIME
	.power_runtime_idle_callback = pm_callback_runtime_idle,
#endif /* KBASE_PM_RUNTIME */
};

int kbase_platform_early_init(void)
{
	/* Nothing needed at this stage */
	return 0;
}

/**
 * Exynos5 hardware specific initialization
 */
int kbase_platform_exynos5_init(struct kbase_device *kbdev)
{
	int err;

	err = kbase_platform_init(kbdev);
	if (err)
		return err;

	err = kbase_platform_create_sysfs_file(kbdev->dev);
	if (err) {
		kbase_platform_term(kbdev);
		return err;
	}

	return 0;
}

/**
 * Exynos5 hardware specific termination
 */
void kbase_platform_exynos5_term(struct kbase_device *kbdev)
{
	kbase_platform_remove_sysfs_file(kbdev->dev);
	kbase_platform_term(kbdev);
}

struct kbase_platform_funcs_conf platform_funcs =
{
	.platform_init_func = &kbase_platform_exynos5_init,
	.platform_term_func = &kbase_platform_exynos5_term,
};

struct kbase_platform_config platform_config;

struct kbase_platform_config *kbase_get_platform_config(void)
{
	return &platform_config;
}

static struct clk *clk_g3d = NULL;

/**
 * Initialize GPU clocks
 */
static int kbase_platform_power_clock_init(struct kbase_device *kbdev)
{
	struct device *dev =  kbdev->dev;
	int timeout;
	struct exynos_context *platform;
	void *g3d_status_reg;

	platform = (struct exynos_context *) kbdev->platform_context;
	if(NULL == platform)
	{
		panic("oops");
	}

	/* Turn on G3D power */
	if (soc_is_exynos5250()) {
		g3d_status_reg = EXYNOS5_G3D_STATUS;
		__raw_writel(0x7, EXYNOS5_G3D_CONFIGURATION);
	}
	else if (soc_is_exynos542x()) {
		g3d_status_reg = EXYNOS5420_G3D_STATUS;
		__raw_writel(0x7, EXYNOS5420_G3D_CONFIGURATION);
	}

	/* Wait for G3D power stability for 1ms */
	timeout = 10;
	while((__raw_readl(g3d_status_reg) & 0x7) != 0x7) {
		if(timeout == 0) {
			/* need to call panic  */
			panic("failed to turn on g3d power\n");
			goto out;
		}
		timeout--;
		udelay(100);
	}

	/* Turn on G3D clock */
	clk_g3d = clk_get(dev, "g3d");
	if (IS_ERR(clk_g3d)) {
		clk_g3d = NULL;
		printk(KERN_ERR "failed to clk_get [clk_g3d]\n");
		/* chrome linux does not have this clock */
	}
	else
	{
		/* android_v4 support */
		clk_prepare_enable(clk_g3d);
		printk("v4 support\n");
	}

#ifdef CONFIG_MIDGARD_HWVER_R0P0
	platform->sclk_g3d = clk_get(dev, "aclk_g3d");
	if(IS_ERR(platform->sclk_g3d)) {
		printk(KERN_ERR "failed to clk_get [aclk_g3d]\n");
		goto out;
	}
#else /* CONFIG_MIDGARD_HWVER_R0P0 */
	{
		struct clk *mpll = NULL;
		mpll = clk_get(dev, "mout_mpll_user");
		if(IS_ERR(mpll)) {
			printk(KERN_ERR "failed to clk_get [mout_mpll_user]\n");
			goto out;
		}

		platform->sclk_g3d = clk_get(dev, "sclk_g3d");
		if(IS_ERR(platform->sclk_g3d)) {
			printk(KERN_ERR "failed to clk_get [sclk_g3d]\n");
			goto out;
		}

		clk_set_parent(platform->sclk_g3d, mpll);
		if(IS_ERR(platform->sclk_g3d)) {
			printk(KERN_ERR "failed to clk_set_parent\n");
			goto out;
		}

		clk_set_rate(platform->sclk_g3d, platform->t6xx_default_clock);
		if (IS_ERR(platform->sclk_g3d)) {
			printk(KERN_ERR "failed to clk_set_rate [sclk_g3d] = %d\n",
				platform->t6xx_default_clock);
			goto out;
		}
	}
#endif /*  CONFIG_MIDGARD_HWVER_R0P0 */
	(void) clk_prepare_enable(platform->sclk_g3d);
	return 0;
out:
	return -EPERM;
}

/**
 * Enable GPU clocks
 */
static int kbase_platform_clock_on(struct kbase_device *kbdev)
{
	struct exynos_context *platform;
	if (!kbdev)
		return -ENODEV;

	platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	if(clk_g3d)
	{
		/* android_v4 support */
		(void) clk_enable(clk_g3d);
	}
	else
	{
		/* chrome support */
		(void) clk_enable(platform->sclk_g3d);
	}

	return 0;
}

/**
 * Disable GPU clocks
 */
static int kbase_platform_clock_off(struct kbase_device *kbdev)
{
	struct exynos_context *platform;
	if (!kbdev)
		return -ENODEV;

	platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	if(clk_g3d)
	{
		/* android_v4 support */
		(void)clk_disable_unprepare(clk_g3d);
	}
	else
	{
		/* chrome support */
		(void)clk_disable_unprepare(platform->sclk_g3d);
	}
	return 0;
}

/**
 * Enable GPU power
 */
static int kbase_platform_power_on(void)
{
	int timeout;
	void *g3d_status_reg;

	/* Turn on G3D power */
	if (soc_is_exynos5250()) {
		g3d_status_reg = EXYNOS5_G3D_STATUS;
		__raw_writel(0x7, EXYNOS5_G3D_CONFIGURATION);
	}
	else if (soc_is_exynos542x()) {
		g3d_status_reg = EXYNOS5420_G3D_STATUS;
		__raw_writel(0x7, EXYNOS5420_G3D_CONFIGURATION);
	}

	/* Wait for G3D power stability */
	timeout = 1000;
	while((__raw_readl(g3d_status_reg) & 0x7) != 0x7) {
		if(timeout == 0) {
			/* need to call panic  */
			panic("failed to turn on g3d via g3d_configuration\n");
			return -ETIMEDOUT;
		}
		timeout--;
		udelay(10);
	}

	return 0;
}

/**
 * Disable GPU power
 */
static int kbase_platform_power_off(void)
{
	int timeout;
	void *g3d_status_reg;

	/* Turn off G3D  */
	if (soc_is_exynos5250()) {
		g3d_status_reg = EXYNOS5_G3D_STATUS;
		__raw_writel(0x0, EXYNOS5_G3D_CONFIGURATION);
	}
	else if (soc_is_exynos542x()) {
		g3d_status_reg = EXYNOS5420_G3D_STATUS;
		__raw_writel(0x0, EXYNOS5420_G3D_CONFIGURATION);
	}

	/* Wait for G3D power stability */
	timeout = 1000;
	while(__raw_readl(g3d_status_reg) & 0x7) {
		if(timeout == 0) {
			/* need to call panic */
			panic( "failed to turn off g3d via g3d_configuration\n");
			return -ETIMEDOUT;
		}
		timeout--;
		udelay(10);
	}

	return 0;
}

/**
 * Power Management unit control. Enable/disable power and clocks to GPU
 */
int kbase_platform_cmu_pmu_control(struct kbase_device *kbdev, int control)
{
	unsigned long flags;
	struct exynos_context *platform;
	if (!kbdev)
	{
		return -ENODEV;
	}

	platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform)
	{
		return -ENODEV;
	}

	spin_lock_irqsave(&platform->cmu_pmu_lock, flags);

	/* off */
	if(control == 0)
	{
		if(platform->cmu_pmu_status == 0)
		{
			spin_unlock_irqrestore(&platform->cmu_pmu_lock, flags);
			return 0;
		}

		if(kbase_platform_power_off())
			panic("failed to turn off g3d power\n");
		if(kbase_platform_clock_off(kbdev))

			panic("failed to turn off sclk_g3d\n");

		platform->cmu_pmu_status = 0;
#if MALI_RTPM_DEBUG
		printk( KERN_ERR "3D cmu_pmu_control - off\n" );
#endif /* MALI_RTPM_DEBUG */
	}
	else
	{
		/* on */
		if(platform->cmu_pmu_status == 1)
		{
			spin_unlock_irqrestore(&platform->cmu_pmu_lock, flags);
			return 0;
		}

		if(kbase_platform_clock_on(kbdev))
			panic("failed to turn on sclk_g3d\n");
		if(kbase_platform_power_on())
			panic("failed to turn on g3d power\n");

		platform->cmu_pmu_status = 1;
#if MALI_RTPM_DEBUG
		printk( KERN_ERR "3D cmu_pmu_control - on\n");
#endif /* MALI_RTPM_DEBUG */
	}

	spin_unlock_irqrestore(&platform->cmu_pmu_lock, flags);

	return 0;
}

/** The sysfs file @c clock, fbdev.
 *
 * This is used for obtaining information about the vithar
 * operating clock & framebuffer address,
 */

static ssize_t mali_sysfs_show_clock(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev;
	struct exynos_context *platform;
	unsigned int clkrate;

	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	platform = (struct exynos_context *) kbdev->platform_context;
	if(!platform)
		return -ENODEV;

	if(!platform->sclk_g3d)
		return -ENODEV;

	clkrate = clk_get_rate(platform->sclk_g3d);

	return scnprintf(buf, PAGE_SIZE, "%u\n", clkrate);
}

static ssize_t mali_sysfs_set_clock(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef CONFIG_MALI_MIDGARD_DVFS
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	struct exynos_context *platform;
	unsigned long freq;
	int level;

	if (!kbdev) {
		pr_err("%s: no kbdev\n", __func__);
		return -ENODEV;
	}

	platform = (struct exynos_context *) kbdev->platform_context;
	if (platform == NULL) {
		pr_err("%s: no platform\n", __func__);
		return -ENODEV;
	}
	if (!platform->sclk_g3d) {
		pr_info("%s: clkout not 3d\n", __func__);
		return -ENODEV;
	}

	/* TODO(dianders) need to be more careful fiddling voltage+clock */
	freq = simple_strtoul(buf, NULL, 10);
	if (!freq)
		return -EINVAL;

	level = kbase_platform_dvfs_get_level(freq);
	if (level < 0) {
		pr_err("%s: invalid value\n", __func__);
		return -ENOENT;
	}

	kbase_platform_dvfs_set_level(kbdev, level);
#endif /* CONFIG_MALI_MIDGARD_DVFS */
	return count;
}
DEVICE_ATTR(clock, S_IRUGO|S_IWUSR, mali_sysfs_show_clock,
	mali_sysfs_set_clock);

static ssize_t mali_sysfs_available_frequencies(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;

	for (i = 0; i < MALI_DVFS_STEP; i++) {
		 ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u\n",
				mali_dvfs_infotbl[i].clock);
	}

	return ret;
}
DEVICE_ATTR(available_frequencies, S_IRUGO, mali_sysfs_available_frequencies,
	NULL);

static ssize_t mali_sysfs_show_fbdev(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev;
	ssize_t ret = 0;
	int i;

	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	for(i = 0 ; i < num_registered_fb ; i++) {
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "fb[%d] xres=%d, yres=%d, addr=0x%lx\n", i, registered_fb[i]->var.xres, registered_fb[i]->var.yres, registered_fb[i]->fix.smem_start);
	}

	if (ret < PAGE_SIZE - 1)
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n");
	else
	{
		buf[PAGE_SIZE-2] = '\n';
		buf[PAGE_SIZE-1] = '\0';
		ret = PAGE_SIZE-1;
	}

	return ret;
}
DEVICE_ATTR(fbdev, S_IRUGO, mali_sysfs_show_fbdev, NULL);

static ssize_t mali_sysfs_show_memory(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev;
	int ret;

	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	ret = sprintf(buf, "%lu bytes\n",
		      atomic_read(&kbdev->memdev.used_pages) * PAGE_SIZE);

	return ret;
}
DEVICE_ATTR(memory, S_IRUGO, mali_sysfs_show_memory, NULL);

static ssize_t mali_sysfs_show_vol(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev;
	ssize_t ret = 0;
	int vol;

	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	kbase_platform_get_voltage(dev, &vol);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "Current operating voltage for mali t6xx = %d", vol);

	if (ret < PAGE_SIZE - 1)
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n");
	else
	{
		buf[PAGE_SIZE-2] = '\n';
		buf[PAGE_SIZE-1] = '\0';
		ret = PAGE_SIZE-1;
	}

	return ret;
}
DEVICE_ATTR(vol, S_IRUGO|S_IWUSR, mali_sysfs_show_vol, NULL);

static int get_clkout_cmu_top(int *val)
{
	*val = __raw_readl(/*EXYNOS5_CLKOUT_CMU_TOP*/EXYNOS_CLKREG(0x10A00));
	if((*val & 0x1f) == 0xB) /* CLKOUT is ACLK_400 in CLKOUT_CMU_TOP */
		return 1;
	else
		return 0;
}

static void set_clkout_for_3d(void)
{
	int tmp;

	tmp = 0x0;
	tmp |= 0x1000B; /* ACLK_400 selected */
	tmp |= 9 << 8;  /* divided by (9 + 1) */
	__raw_writel(tmp, /*EXYNOS5_CLKOUT_CMU_TOP*/EXYNOS_CLKREG(0x10A00));

#ifdef PMU_XCLKOUT_SET
	exynos5_pmu_xclkout_set(1, XCLKOUT_CMU_TOP);
#else /* PMU_XCLKOUT_SET */
	tmp = 0x0;
	tmp |= 7 << 8; /* CLKOUT_CMU_TOP selected */
	__raw_writel(tmp, /*S5P_PMU_DEBUG*/S5P_PMUREG(0x0A00));
#endif /* PMU_XCLKOUT_SET */
}

static ssize_t mali_sysfs_show_clkout(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev;
	ssize_t ret = 0;
	int val;

	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	if(get_clkout_cmu_top(&val))
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "Current CLKOUT is g3d divided by 10, CLKOUT_CMU_TOP=0x%x", val);
	else
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "Current CLKOUT is not g3d, CLKOUT_CMU_TOP=0x%x", val);

	if (ret < PAGE_SIZE - 1)
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n");
	else
	{
		buf[PAGE_SIZE-2] = '\n';
		buf[PAGE_SIZE-1] = '\0';
		ret = PAGE_SIZE-1;
	}

	return ret;
}

static ssize_t mali_sysfs_set_clkout(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	set_clkout_for_3d();
	pr_info("clkout set to 3d\n");
	return count;
}
DEVICE_ATTR(clkout, S_IRUGO|S_IWUSR, mali_sysfs_show_clkout,
	mali_sysfs_set_clkout);

#ifdef CONFIG_MALI_MIDGARD_DVFS
static ssize_t mali_sysfs_show_dvfs(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev;
	ssize_t ret = 0;

	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	if (!kbase_pm_metrics_is_active(kbdev))
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "off\n");
	else
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "on\n");

	return ret;
}

static ssize_t mali_sysfs_set_dvfs(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct kbase_device *kbdev;
	struct exynos_context *platform;
	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	platform = kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	if (sysfs_streq("off", buf)) {
		if (kbase_pm_metrics_is_active(kbdev)) {
			kbasep_pm_metrics_term(kbdev);
			kbase_platform_dvfs_set_level(kbdev,
				kbase_platform_dvfs_get_level(platform->t6xx_default_clock));
			pr_info("G3D DVFS is disabled\n");
		}
	} else if (sysfs_streq("on", buf)) {
		if (!kbase_pm_metrics_is_active(kbdev)) {
			ret = kbasep_pm_metrics_init(kbdev);
			if (ret)
				pr_warning("kbase_pm_metrics_init failed,"
				    " error %u\n", ret);
			else
				pr_info("G3D DVFS is enabled\n");
		}
	} else {
		pr_info("%s: invalid, only [on, off] is accepted\n", buf);
		return -EINVAL;
	}

	return count;
}
DEVICE_ATTR(dvfs, S_IRUGO|S_IWUSR, mali_sysfs_show_dvfs, mali_sysfs_set_dvfs);
#endif /* CONFIG_MALI_MIDGARD_DVFS */

#ifdef MALI_DVFS_ASV_ENABLE
static ssize_t mali_sysfs_show_asv(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	struct kbase_device *kbdev;
	ssize_t ret = 0;
	int i;
	int asv_group;

	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;
	if (!buf)
		return -EINVAL;

	asv_group = exynos_asv_group_get(ID_G3D);
	if (soc_is_exynos5250()) {
		ret += scnprintf(buf, PAGE_SIZE, "asv group:%d exynos_lot_id:%d\n",
				asv_group, exynos_lot_id);
	} else if (soc_is_exynos542x()) {
		ret += scnprintf(buf, PAGE_SIZE, "asv group:%d mp%d\n",
				asv_group, exynos5420_is_g3d_mp6() ? 6 : 4);
	}

	for (i = MALI_DVFS_STEP - 1; i >= 0; i--) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u:%d\n",
				mali_dvfs_infotbl[i].clock,
				mali_dvfs_infotbl[i].voltage);
	}
	return ret;
}

static ssize_t mali_sysfs_set_asv(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if (sysfs_streq("off", buf))
		kbase_platform_asv_set(0);
	else if (sysfs_streq("on", buf))
		kbase_platform_asv_set(1);
	else {
		printk(KERN_ERR "invalid val -only [on, off] is accepted\n");
		return -EINVAL;
	}

	return count;
}
DEVICE_ATTR(asv, S_IRUGO|S_IWUSR, mali_sysfs_show_asv, mali_sysfs_set_asv);
#endif

static struct device_attribute *attributes[] = {
	&dev_attr_clock,
	&dev_attr_available_frequencies,
	&dev_attr_fbdev,
	&dev_attr_memory,
	&dev_attr_vol,
	&dev_attr_clkout,
#ifdef CONFIG_MALI_MIDGARD_DVFS
	&dev_attr_dvfs,
#ifdef MALI_DVFS_ASV_ENABLE
	&dev_attr_asv,
#endif /* MALI_DVFS_ASV_ENABLE */
#endif /* CONFIG_MALI_MIDGARD_DVFS */
	NULL
};

static int kbase_platform_create_sysfs_file(struct device *dev)
{
	struct device_attribute **attr;
	int err;

	for (attr = attributes; *attr; attr++) {
		err = device_create_file(dev, *attr);
		if (err) {
			dev_err(dev, "Couldn't create sysfs file [%s]\n",
					(*attr)->attr.name);
			goto out;

		}
	}

	return 0;
out:
	for (; attr >= attributes; attr--)
		device_remove_file(dev, *attr);

	return err;
}

void kbase_platform_remove_sysfs_file(struct device *dev)
{
	struct device_attribute **attr;

	for (attr = attributes; *attr; attr++)
		device_remove_file(dev, *attr);
}

int kbase_platform_init(struct kbase_device *kbdev)
{
	struct exynos_context *platform;
	int err;

	platform = kmalloc(sizeof(struct exynos_context), GFP_KERNEL);

	if (!platform)
		return -ENOMEM;

	kbdev->platform_context = (void *) platform;

	if (soc_is_exynos542x())
		platform->t6xx_default_clock = 420000000;
	else
		platform->t6xx_default_clock = 533000000;

	platform->cmu_pmu_status = 0;
	spin_lock_init(&platform->cmu_pmu_lock);

	err = kbase_platform_power_clock_init(kbdev);
	if (err)
		goto clock_init_fail;

#ifdef CONFIG_REGULATOR
	err = kbase_platform_regulator_init();
	if (err)
		goto regulator_init_fail;
#endif /* CONFIG_REGULATOR */

#ifdef CONFIG_MALI_MIDGARD_DVFS
	err = kbase_platform_dvfs_init(kbdev);
	if (err)
		goto dvfs_init_fail;
#endif /* CONFIG_MALI_MIDGARD_DVFS */

	/* Enable power */
	kbase_platform_cmu_pmu_control(kbdev, 1);
	return 0;

#ifdef CONFIG_MALI_MIDGARD_DVFS
dvfs_init_fail:
#endif /* CONFIG_MALI_MIDGARD_DVFS */
#ifdef CONFIG_REGULATOR
	kbase_platform_regulator_disable();
regulator_init_fail:
#endif /* CONFIG_REGULATOR */
clock_init_fail:
	kfree(platform);
	kbdev->platform_context = NULL;
	return err;
}

void kbase_platform_term(struct kbase_device *kbdev)
{
	struct exynos_context *platform;

	platform = (struct exynos_context *) kbdev->platform_context;

#ifdef CONFIG_MALI_MIDGARD_DVFS
	kbase_platform_dvfs_term();
#endif /* CONFIG_MALI_MIDGARD_DVFS */

	/* Disable power */
	kbase_platform_cmu_pmu_control(kbdev, 0);
#ifdef CONFIG_REGULATOR
	kbase_platform_regulator_disable();
#endif /* CONFIG_REGULATOR */
	kfree(kbdev->platform_context);
	kbdev->platform_context = 0;
	return;
}

#ifdef CONFIG_REGULATOR
static struct regulator *g3d_regulator=NULL;
#ifdef CONFIG_MIDGARD_HWVER_R0P0
static int mali_gpu_vol = 1250000; /* 1.25V @ 533 MHz */
#else
static int mali_gpu_vol = 1050000; /* 1.05V @ 266 MHz */
#endif /*  CONFIG_MIDGARD_HWVER_R0P0 */
#endif /* CONFIG_REGULATOR */

#ifdef CONFIG_MALI_MIDGARD_DVFS
/*
 * Weighted moving average support for signed integer data
 * with 7-bits of precision (not currently used; all data
 * are integers).
 */
#define DVFS_AVG_DUMMY_MARKER	(~0)
#define DVFS_AVG_LPF_LEN	4	/* NB: best to be pow2 */
#define DVFS_AVG_EP_MULTIPLIER	(1<<7)	/* 7 fractional bits */

#define DVFS_AVG_RESET(x)	((x) = DVFS_AVG_DUMMY_MARKER)
#define _DVFS_AVG_IN(x)		((x) * DVFS_AVG_EP_MULTIPLIER)
#define _DVFS_LPF_UTIL(x, y, len) \
	((x != DVFS_AVG_DUMMY_MARKER) ? \
	 (((x) * ((len) - 1) + (y)) / (len)) : (y))
#define DVFS_AVG_LPF(x, y) do { \
	x = _DVFS_LPF_UTIL((x), _DVFS_AVG_IN((y)), DVFS_AVG_LPF_LEN); \
} while (0)
#define DVFS_TO_AVG(x)		DIV_ROUND_CLOSEST(x, DVFS_AVG_EP_MULTIPLIER)

#ifdef MALI_DVFS_ASV_ENABLE
enum asv_update_val {
	DVFS_NOT_UPDATE_ASV_TBL = 0,
	DVFS_UPDATE_ASV_TBL = 1,
	DVFS_UPDATE_ASV_DEFAULT_TBL = 2,
};
#endif /* MALI_DVFS_ASV_ENABLE */

struct mali_dvfs_status {
	struct kbase_device *kbdev;
	int step;
	int utilisation;
	uint nsamples;
	u32 avg_utilisation;
#ifdef MALI_DVFS_ASV_ENABLE
	enum asv_update_val asv_need_update;
	int asv_group;
#endif
};

static struct workqueue_struct *mali_dvfs_wq = 0;
int mali_dvfs_control=0;
spinlock_t mali_dvfs_spinlock;

#ifdef MALI_DVFS_ASV_ENABLE
static int mali_dvfs_update_asv(int group)
{
	int i;

	/* TODO(Shariq): Replace exynos_lot_id with api call once ASV code is rebased */
	if (exynos_lot_id && group == 0) {
		/* wrong group: Use default table to keep the system running smoothly */
		for (i = 0; i < MALI_DVFS_STEP; i++)
			mali_dvfs_infotbl[i].voltage = mali_dvfs_vol_default[i];
		pr_err("exynos_lot_id has group 0. Using default gpu asv table\n");
		return 1;
	}

	if (group == -1) {
		for (i = 0; i < MALI_DVFS_STEP; i++)
			mali_dvfs_infotbl[i].voltage = mali_dvfs_vol_default[i];
		pr_err("mali_dvfs_update_asv use default table\n");
		return 1;
	}
	if (group > MALI_DVFS_ASV_GROUP_NUM) {
		/* unknown group: Use default table to keep the system running smoothly */
		for (i = 0; i < MALI_DVFS_STEP; i++)
			mali_dvfs_infotbl[i].voltage = mali_dvfs_vol_default[i];
		printk(KERN_ERR "invalid asv group (%d)\n", group);
		return 1;
	}
	for (i = 0; i < MALI_DVFS_STEP; i++) {
		if (exynos_lot_id)
			mali_dvfs_infotbl[i].voltage =
				mali_dvfs_asv_vol_tbl_special[group-1][i];
		else
			mali_dvfs_infotbl[i].voltage =
				mali_dvfs_asv_vol_tbl[group][i];
	}

	return 0;
}
#endif /* MALI_DVFS_ASV_ENABLE */

static void mali_dvfs_event_proc(struct work_struct *w)
{
	struct mali_dvfs_status dvfs_status;
	const struct mali_dvfs_info *info;
	int avg_utilisation;
	unsigned long irqflags;

	spin_lock_irqsave(&mali_dvfs_spinlock, irqflags);
	dvfs_status = mali_dvfs_status_current;

	BUG_ON(dvfs_status.step >= MALI_DVFS_STEP);
#ifdef MALI_DVFS_ASV_ENABLE
	if (soc_is_exynos5250()) {
		if (dvfs_status.asv_need_update ==
				DVFS_UPDATE_ASV_DEFAULT_TBL) {
			mali_dvfs_update_asv(-1);
			dvfs_status.asv_need_update = DVFS_NOT_UPDATE_ASV_TBL;
		} else if (dvfs_status.asv_need_update == DVFS_UPDATE_ASV_TBL) {
			if (!mali_dvfs_update_asv(exynos_result_of_asv & 0xf))
				dvfs_status.asv_group =
					(exynos_result_of_asv & 0xf);
			dvfs_status.asv_need_update = DVFS_NOT_UPDATE_ASV_TBL;
		}
	}
#endif
	spin_unlock_irqrestore(&mali_dvfs_spinlock, irqflags);

	info = &mali_dvfs_infotbl[dvfs_status.step];

	BUG_ON(dvfs_status.utilisation > 100);
	dvfs_status.nsamples++;
	DVFS_AVG_LPF(dvfs_status.avg_utilisation, dvfs_status.utilisation);
	BUG_ON(dvfs_status.nsamples <= 0);
	avg_utilisation = DVFS_TO_AVG(dvfs_status.avg_utilisation);
#if MALI_DVFS_DEBUG
	pr_debug("%s: step %d utilisation %d avg %d max %d min %d "
	    "nsamples %u up_cnt %d down_cnt %d\n", __func__,
	    dvfs_status.step, dvfs_status.utilisation,
	    avg_utilisation, info->max_threshold, info->min_threshold,
	    dvfs_status.nsamples, allow_up_cnt, allow_down_cnt);
#endif
	trace_mali_dvfs_event(dvfs_status.utilisation, avg_utilisation);

	if (avg_utilisation > info->max_threshold &&
	    dvfs_status.nsamples >= info->up_cnt_threshold) {
		dvfs_status.step++;
		/*
		 * NB: max clock should have max_threshold of 100
		 * so this should never trip.
		 */
		BUG_ON(dvfs_status.step >= MALI_DVFS_STEP);
		DVFS_AVG_RESET(dvfs_status.avg_utilisation);
		dvfs_status.nsamples = 0;
	} else if (dvfs_status.step > 0 &&
	    avg_utilisation < info->min_threshold &&
	    dvfs_status.nsamples >= info->down_cnt_threshold) {
		BUG_ON(dvfs_status.step <= 0);
		dvfs_status.step--;
		DVFS_AVG_RESET(dvfs_status.avg_utilisation);
		dvfs_status.nsamples = 0;
	}

	if (kbase_pm_metrics_is_active(dvfs_status.kbdev))
		kbase_platform_dvfs_set_level(dvfs_status.kbdev,
		dvfs_status.step);

	spin_lock_irqsave(&mali_dvfs_spinlock, irqflags);
	mali_dvfs_status_current = dvfs_status;
	spin_unlock_irqrestore(&mali_dvfs_spinlock, irqflags);

}

static DECLARE_WORK(mali_dvfs_work, mali_dvfs_event_proc);

/**
 * Exynos5 alternative dvfs_callback imlpementation.
 * instead of:
 *    action = kbase_pm_get_dvfs_action(kbdev);
 * use this:
 *    kbase_platform_dvfs_event(kbdev);
 */

int kbase_platform_dvfs_get_control_status(void)
{
	return mali_dvfs_control;
}


static void mali_dvfs_infotbl_init_exynos542x(struct kbase_device *kbdev)
{
	int i;

	if (soc_is_exynos5420())
		mali_dvfs_infotbl = mali_dvfs_infotbl_exynos5420;
	else
		mali_dvfs_infotbl = mali_dvfs_infotbl_exynos5422;

	for (i = 0; i < MALI_DVFS_STEP; i++) {
		unsigned int asv_volt;
#ifdef CONFIG_ARM_EXYNOS5420_ASV
		asv_volt = get_match_volt(ID_G3D,
				mali_dvfs_infotbl[i].clock / 1000);
#else
		asv_volt = 0;
#endif
		if (asv_volt)
			mali_dvfs_infotbl[i].voltage = asv_volt;
	}
}

int kbase_platform_dvfs_init(struct kbase_device *kbdev)
{
	unsigned long irqflags;

	/*default status
	add here with the right function to get initilization value.
	*/
	mali_dvfs_wq = create_singlethread_workqueue("mali_dvfs");
	if (!mali_dvfs_wq)
		return -ENOMEM;

	spin_lock_init(&mali_dvfs_spinlock);

	/*add a error handling here*/
	spin_lock_irqsave(&mali_dvfs_spinlock, irqflags);
	mali_dvfs_status_current.kbdev = kbdev;
	mali_dvfs_status_current.step = MALI_DVFS_STEP-1;
	mali_dvfs_status_current.utilisation = 100;
	DVFS_AVG_RESET(mali_dvfs_status_current.avg_utilisation);
#ifdef MALI_DVFS_ASV_ENABLE
	mali_dvfs_status_current.asv_need_update = DVFS_UPDATE_ASV_TBL;
	mali_dvfs_status_current.asv_group = -1;
#endif
	mali_dvfs_control = 1;

	if (soc_is_exynos5250()) {
		mali_dvfs_infotbl = mali_dvfs_infotbl_exynos5250;
#ifdef MALI_DVFS_ASV_ENABLE
		mali_dvfs_vol_default = mali_dvfs_vol_default_exynos5250;
		mali_dvfs_asv_vol_tbl_special =
				mali_dvfs_asv_vol_tbl_special_exynos5250;
		mali_dvfs_asv_vol_tbl = mali_dvfs_asv_vol_tbl_exynos5250;
#endif
	} else if (soc_is_exynos542x())
		mali_dvfs_infotbl_init_exynos542x(kbdev);

	spin_unlock_irqrestore(&mali_dvfs_spinlock, irqflags);

	return 0;
}

void kbase_platform_dvfs_term(void)
{
	if (mali_dvfs_wq)
		destroy_workqueue(mali_dvfs_wq);

	mali_dvfs_wq = NULL;
}
#endif /* CONFIG_MALI_MIDGARD_DVFS */


int kbase_platform_dvfs_event(struct kbase_device *kbdev, u32 utilisation,
		u32 util_gl_share, u32 util_cl_share[2])
{
#ifdef CONFIG_MALI_MIDGARD_DVFS
	unsigned long irqflags;

	spin_lock_irqsave(&mali_dvfs_spinlock, irqflags);
	mali_dvfs_status_current.utilisation = utilisation;
	spin_unlock_irqrestore(&mali_dvfs_spinlock, irqflags);
	queue_work_on(0, mali_dvfs_wq, &mali_dvfs_work);

	/*add error handle here*/
	return 0;
#else
	return -ENODEV;
#endif
}

int kbase_platform_regulator_init(void)
{
#ifdef CONFIG_REGULATOR
	int err;

	g3d_regulator = regulator_get(NULL, "vdd_g3d");
	if (IS_ERR(g3d_regulator)) {
		pr_err("%s: failed to obtain g3d regulator (err %ld)\n",
				__func__, PTR_ERR(g3d_regulator));
		return PTR_ERR(g3d_regulator);
	}

	err = regulator_enable(g3d_regulator);
	if (err) {
		pr_err("%s: failed to enable g3d regulator (err %d)\n",
				__func__, err);
		g3d_regulator = NULL;
		return err;
	}

	err = regulator_set_voltage(g3d_regulator, mali_gpu_vol, mali_gpu_vol);
	if (err) {
		kbase_platform_regulator_disable();
		pr_err("%s: failed to set g3d regulator operating voltage [%d] (err %d)\n",
				__func__, mali_gpu_vol, err);
		return err;
	}
#endif /* CONFIG_REGULATOR */

	return 0;
}

int kbase_platform_regulator_disable(void)
{
#ifdef CONFIG_REGULATOR
	int err;

	if (!g3d_regulator) {
		pr_err("%s: g3d_regulator is not initialized\n", __func__);
		return -ENODEV;
	}

	err = regulator_disable(g3d_regulator);
	if (err) {
		pr_err("%s: failed to disable g3d regulator (err %d)\n",
				__func__, err);
		return err;
	}
#endif /* CONFIG_REGULATOR */
	return 0;
}

int kbase_platform_regulator_enable(void)
{
#ifdef CONFIG_REGULATOR
	int err;

	if (!g3d_regulator) {
		pr_err("%s: g3d_regulator is not initialized\n", __func__);
		return -ENODEV;
	}

	err = regulator_enable(g3d_regulator);
	if (err) {
		pr_err("%s: failed to enable g3d regulator (err %d)\n",
				__func__, err);
		return err;
	}
#endif /* CONFIG_REGULATOR */
	return 0;
}

int kbase_platform_get_default_voltage(struct device *dev, int *vol)
{
#ifdef CONFIG_REGULATOR
	*vol = mali_gpu_vol;
#else /* CONFIG_REGULATOR */
	*vol = 0;
#endif /* CONFIG_REGULATOR */
	return 0;
}

int kbase_platform_get_voltage(struct device *dev, int *vol)
{
#ifdef CONFIG_REGULATOR
	if (!g3d_regulator) {
		dev_err(dev, "%s: g3d_regulator is not initialized\n",
				__func__);
		return -ENODEV;
	}

	*vol = regulator_get_voltage(g3d_regulator);
#else /* CONFIG_REGULATOR */
	*vol = 0;
#endif /* CONFIG_REGULATOR */
	return 0;
}

#ifdef CONFIG_MALI_MIDGARD_DVFS
static int kbase_platform_set_voltage(struct device *dev, int vol)
{
#ifdef CONFIG_REGULATOR
	int err;

	if (!g3d_regulator) {
		dev_err(dev, "%s: g3d_regulator is not initialized\n",
				__func__);
		return -ENODEV;
	}

	err = regulator_set_voltage(g3d_regulator, vol, vol);
	if (err) {
		dev_err(dev, "%s failed to set voltage (err %d)\n",
				__func__, err);
		return err;
	}
#endif /* CONFIG_REGULATOR */
	return 0;
}

static void kbase_platform_dvfs_set_clock(struct kbase_device *kbdev, int freq)
{
	static struct clk * mout_gpll = NULL;
	static struct clk * fin_gpll = NULL;
	static struct clk * fout_gpll = NULL;
	static int _freq = -1;
	static unsigned long gpll_rate_prev = 0;
	unsigned long gpll_rate = 0;
	struct exynos_context *platform;
	unsigned int i = MALI_DVFS_STEP;

	if (!kbdev)
		panic("no kbdev");

	platform = (struct exynos_context *) kbdev->platform_context;
	if (NULL == platform)
		panic("no platform");

	if (platform->sclk_g3d == 0)
		return;

	if (mout_gpll == NULL) {
		if (soc_is_exynos5250()) {
			mout_gpll = clk_get(kbdev->dev, "mout_gpll");
			fin_gpll = clk_get(kbdev->dev, "ext_xtal");
			fout_gpll = clk_get(kbdev->dev, "fout_gpll");
		} else if (soc_is_exynos542x()) {
			mout_gpll = clk_get(kbdev->dev, "mout_vpll");
			fin_gpll = clk_get(kbdev->dev, "ext_xtal");
			fout_gpll = clk_get(kbdev->dev, "fout_vpll");
		}
		if (IS_ERR(mout_gpll) || IS_ERR(fin_gpll) || IS_ERR(fout_gpll))
			panic("clk_get ERROR");
	}

	if (freq == _freq)
		return;

	trace_mali_dvfs_set_clock(freq);

	for (i = 0; i < MALI_DVFS_STEP; i++)
		if (freq == mali_dvfs_infotbl[i].clock)
			break;
	if (i == MALI_DVFS_STEP)
		return;

	gpll_rate = freq;

	/* if changed the GPLL rate, set rate for GPLL and wait for lock time */
	if( gpll_rate != gpll_rate_prev) {
		clk_set_parent(mout_gpll, fin_gpll);

		/*change gpll*/
		clk_set_rate( fout_gpll, gpll_rate );

		/*restore parent*/
		clk_set_parent(mout_gpll, fout_gpll);
		gpll_rate_prev = gpll_rate;
	}

	_freq = freq;

#if MALI_DVFS_DEBUG
	printk(KERN_DEBUG "dvfs_set_clock GPLL : %lu, ACLK_400 : %luMhz\n",
			gpll_rate, clk_get_rate(platform->sclk_g3d));
#endif /* MALI_DVFS_DEBUG */
	return;
}

static void kbase_platform_dvfs_set_vol(unsigned int vol)
{
	static int _vol = -1;

	if (_vol == vol)
		return;
	trace_mali_dvfs_set_voltage(vol);

	kbase_platform_set_voltage(NULL, vol);

	_vol = vol;
#if MALI_DVFS_DEBUG
	printk(KERN_DEBUG "dvfs_set_vol %dmV\n", vol);
#endif
	return;
}

static int kbase_platform_dvfs_get_level(int freq)
{
	int i;
	for (i = 0; i < MALI_DVFS_STEP; i++) {
		if (mali_dvfs_infotbl[i].clock == freq)
			return i;
	}
	return -ENOENT;
}

static void kbase_platform_dvfs_set_level(struct kbase_device *kbdev, int level)
{
	static int level_prev = -1;

	if (level == level_prev)
		return;
	if (WARN_ON((level >= MALI_DVFS_STEP) || (level < 0)))
		panic("invalid level");

	if (level > level_prev) {
		kbase_platform_dvfs_set_vol(mali_dvfs_infotbl[level].voltage);
		kbase_platform_dvfs_set_clock(kbdev,
			mali_dvfs_infotbl[level].clock);
	} else{
		kbase_platform_dvfs_set_clock(kbdev,
			mali_dvfs_infotbl[level].clock);
		kbase_platform_dvfs_set_vol(mali_dvfs_infotbl[level].voltage);
	}
	level_prev = level;
}

static void kbase_platform_dvfs_set_max(struct kbase_device *kbdev)
{
	int i, level;

	/*
	 * Firmware may initialize the GPU clocks at a rate higher than the
	 * one we suspended at. Set the maximum frequency and voltage at
	 * suspend time so that we don't under-volt the GPU during resume.
	 */
	for (i = 0; i < MALI_DVFS_STEP; i++)
		if (mali_dvfs_infotbl[i].max_threshold == 100) {
			level = i;
			break;
		}

	kbase_platform_dvfs_set_level(kbdev, level);
	mali_dvfs_status_current.step = level;
}
#endif /* CONFIG_MALI_MIDGARD_DVFS */

#ifdef MALI_DVFS_ASV_ENABLE
static int kbase_platform_asv_set(int enable)
{
	unsigned long irqflags;

	spin_lock_irqsave(&mali_dvfs_spinlock, irqflags);
	if (enable) {
		mali_dvfs_status_current.asv_need_update = DVFS_UPDATE_ASV_TBL;
		mali_dvfs_status_current.asv_group = -1;
	} else{
		mali_dvfs_status_current.asv_need_update = DVFS_UPDATE_ASV_DEFAULT_TBL;
	}
	spin_unlock_irqrestore(&mali_dvfs_spinlock, irqflags);
	return 0;
}
#endif /* MALI_DVFS_ASV_ENABLE */
