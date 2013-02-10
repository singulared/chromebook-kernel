/*
 * linux/arch/arm/mach-s5pv310/charger-slp7.c
 * COPYRIGHT(C) 2011
 * MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * SLP7(NURI) Board Charger Support with Charger-Manager Framework
 *
 */

#include <asm/io.h>

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/platform_data/ntc_thermistor.h>
#include <linux/regulator/consumer.h>
#include <linux/power/charger-manager.h>
#include <linux/mfd/max8997.h>
#include <linux/mfd/max8997-private.h>

#include <mach/regs-pmu.h>
#include <mach/irqs.h>

#include <plat/gpio-core.h>
#include <plat/pm.h>

/* Returns true if the last wakeup was caused by the exynos rtc */
static bool s3c_wksrc_rtc_alarm(void)
{
	u32 reg = __raw_readl(S5P_WAKEUP_STAT) & S5P_WAKEUP_STAT_WKSRC_MASK;

	if ((reg & S5P_WAKEUP_STAT_RTCALARM) &&
	    !(reg & ~S5P_WAKEUP_STAT_RTCALARM))
		return true; /* yes, it is */

	return false;
}

/* Convert an external interrupt number (0-31) to a Linux irq number */
static int eint_to_gpio(int eint)
{
	if (eint < 8)
		return EXYNOS5_GPX0(eint & 7);
	else if (eint < 16)
		return EXYNOS5_GPX1(eint & 7);
	else if (eint < 24)
		return EXYNOS5_GPX2(eint & 7);
	else
		return EXYNOS5_GPX3(eint & 7);
}

/**
 * charger_check_wake_irq() - Check if the given irq caused the wakeup
 * @wake_irq to check
 * @return true if that irq (and only that irq) caused the most recent wakeup
 * false otherwise (not that irq, or multiple irqs)
 */
static bool charger_check_wake_irq(int wake_irq)
{
	unsigned long bit = IRQ_EINT_BIT(wake_irq);
	bool s3c_rtc_wakeup;
	uint value = 0;
	uint mask;
	int i;

	/* Check which of the enabled wakeup souces are active */
	for (i = 0, mask = 1; i < 32; i++, mask <<= 1) {
		int gpio;

		if (!(s3c_irqwake_eintmask & mask)) {
			gpio = eint_to_gpio(i);

			if (!gpio_get_value(gpio))
				value |= mask;
		}
	}
	s3c_rtc_wakeup = s3c_wksrc_rtc_alarm();
	pr_debug("%s: s3c_rtc_wakeup=%d, eint=%#x\n", __func__,
		 s3c_rtc_wakeup, value);

	/*
	 * If we are looking for the exynos rtc and it is the only wakeup
	 * cause source, then we have a match
	 */
	if (wake_irq == EXYNOS5_IRQ_RTC_ALARM) {
		if (s3c_rtc_wakeup && !value)
			return true;

	/*
	 * We also have a match if the exynos rtc did not cause the wakeup
	 * and the select eint did
	 */
	} else if (!s3c_rtc_wakeup && value == (1U << bit)) {
		return true;
	}

	return false;
}

static char *nuri_charger_stats[] = { "max8997_pmic", "max8903_charger", NULL };
static struct regulator_bulk_data nuri_chargers[] = {
	{ .supply = "vinchg1", },
	{ .supply = "vinchg2", },
};

static struct charger_desc nuri_charger_desc = {
	.psy_name		= "battery",
	.polling_interval_ms	= 30000,
	.polling_mode		= CM_POLL_CHARGING_ONLY,
	.fullbatt_vchkdrop_ms	= 30000,
	.fullbatt_vchkdrop_uV	= 100000,
	.fullbatt_uV		= 4200000,
	.battery_present	= CM_FUEL_GAUGE,
	.psy_charger_stat	= nuri_charger_stats,
	.charger_regulators	= nuri_chargers,
	.num_charger_regulators	= ARRAY_SIZE(nuri_chargers),
	.psy_fuel_gauge		= "max17042_battery",

	.measure_battery_temp	= false,

	.temperature_high	= 60 * 1000,
	.temperature_high_recovery = 40 * 1000,
	.temperature_low	= 0 * 1000,
	.temperature_low_recovery = 5 * 1000,
};

struct charger_global_desc nuri_charger_g_desc = {
	.rtc_name = "rtc0",
	.rtc_only_wakeup = charger_check_wake_irq,
	.assume_timer_stops_in_suspend	= true,
};

struct platform_device nuri_charger_manager = {
	.name			= "charger-manager",
	.dev			= {
		.platform_data = &nuri_charger_desc,
	},
};
