/*
 * SAMSUNG EXYNOS5250 Flattened Device Tree enabled machine
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/memblock.h>
#include <linux/io.h>
#include <linux/pwm_backlight.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/memblock.h>
#include <linux/of_fdt.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <mach/regs-pmu.h>

#include <plat/cpu.h>
#include <plat/mfc.h>
#include <plat/backlight.h>
#include <plat/adc.h>	/* for s3c_adc_register and friends */
#include <plat/gpio-cfg.h>
#include <plat/fb.h>

#include "common.h"
#include <video/platform_lcd.h>

static unsigned long ramoops_dt_start, ramoops_dt_size;
static int __init init_dt_scan_ramoops(unsigned long node, const char *uname,
					int depth, void *data)
{
	__be32 *reg, *endp;
	unsigned long l;

	if (!of_flat_dt_is_compatible(node, "ramoops"))
		return 0;

	reg = of_get_flat_dt_prop(node, "reg", &l);
	if (!reg)
		return 0;
	endp = reg + (l / sizeof(__be32));

	/* This architecture uses single cells for address and size.
	 * Other architectures may differ. */
	ramoops_dt_start = be32_to_cpu(reg[0]);
	ramoops_dt_size = be32_to_cpu(reg[1]);
	return 0;
}

static void __init exynos5_ramoops_reserve(void)
{
	unsigned long start, size;

	of_scan_flat_dt(init_dt_scan_ramoops, NULL);

	/* If necessary, lower start and raise size to align to 1M. */
	start = round_down(ramoops_dt_start, SZ_1M);
	size = ramoops_dt_size + ramoops_dt_start - start;
	size = round_up(size, SZ_1M);

	if (memblock_remove(start, size)) {
		pr_err("Failed to remove ramoops %08lx@%08lx from memory\n",
			size, start);
	} else {
		pr_info("Ramoops: %08lx - %08lx\n", start, start + size - 1);
	}
}

static int smdk5250_bl_notify(struct device *unused, int brightness)
{
	/* manage lcd_bl_en signal */
	if (brightness)
		gpio_set_value(EXYNOS5_GPX3(0), 1);
	else
		gpio_set_value(EXYNOS5_GPX3(0), 0);

	return brightness;
}

/* LCD Backlight data */
static struct samsung_bl_gpio_info smdk5250_bl_gpio_info = {
	.no	= EXYNOS5_GPB2(0),
	.func	= S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdk5250_bl_data = {
	.max_brightness = 2800,
	.dft_brightness = 2800,
	.pwm_period_ns	= 1000000,
	.notify		= smdk5250_bl_notify,
};

/*
 * Hack to do power-on sequence after the 32khz SLP_CLK is
 * running as doing this entirely in the DT is problematic.
 * We attach an auxdata record to dw_mmc2 with a set_power
 * method that frobs the regulators.  This work is not board
 * specific but must be done before the device is recognized
 * so cannot go in the driver.
 */
static bool enable_mwi87xx(void)
{
	struct regulator *wifi_en, *wifi_rst;
	bool ok;

	wifi_en = regulator_get(NULL, "wifi-en");
	wifi_rst = regulator_get(NULL, "wifi-rst-l");
	ok = !IS_ERR(wifi_en) && !IS_ERR(wifi_rst);
	if (ok) {
		/*
		 * This assumes SLP_CLK is enabled and stable by the
		 * time we get here.  Also any delay required between
		 * RESETn and PDn should be set in startup-delay-us
		 * in the DT.
		 */
		regulator_enable(wifi_rst);
		regulator_enable(wifi_en);
	}

	if (!IS_ERR(wifi_rst))
		regulator_put(wifi_rst);
	if (!IS_ERR(wifi_en))
		regulator_put(wifi_en);
	return ok;
}

static void exynos_wifi_bt_set_power(u32 slot_id, u32 volt)
{
	if (volt == 0 || (!of_machine_is_compatible("google,snow") &&
			  !of_machine_is_compatible("google,spring") &&
			  !of_machine_is_compatible("google,daisy")))
		return;
	if (!enable_mwi87xx())
		pr_err("%s: problem enabling WiFi+BT\n", __func__);
	/* NB: bt-reset-l is tied to wifi-rst-l so BT should be ready too */
}

static int lcd_probe(struct plat_lcd_data *pd);
static void lcd_set_power(struct plat_lcd_data *pd, unsigned int power);

static struct regulator *lcd_fet, *backlight_fet;

static struct plat_lcd_data exynos5_lcd_data = {
	.probe		= lcd_probe,
	.set_power	= lcd_set_power,
};

static struct platform_device exynos5_lcd = {
	.name			= "platform-lcd",
	.dev.platform_data	= &exynos5_lcd_data,
};

static int lcd_probe(struct plat_lcd_data *pd)
{
	struct device *dev = &exynos5_lcd.dev;

	lcd_fet = regulator_get(dev, "lcd_vdd");
	if (IS_ERR(lcd_fet))
		return PTR_ERR(lcd_fet);

	backlight_fet = regulator_get(dev, "vcd_led");
	if (IS_ERR(backlight_fet)) {
		regulator_put(lcd_fet);
		return PTR_ERR(backlight_fet);
	}

	return 0;
}

static void lcd_set_power(struct plat_lcd_data *pd,
			unsigned int power)
{
	struct device *dev = &exynos5_lcd.dev;

	if (IS_ERR_OR_NULL(lcd_fet) || IS_ERR_OR_NULL(backlight_fet)) {
		dev_err(dev, "fet(s) not initialized\n");
		return;
	}

	if (power) {
			regulator_enable(lcd_fet);
			regulator_enable(backlight_fet);
	} else {
			regulator_disable(lcd_fet);
			regulator_disable(backlight_fet);
	}

	/* Wait 10 ms between regulator on and PWM start per spec */
	mdelay(10);

	exynos_wifi_bt_set_power(0, power);	/* TODO(sleffler) hack */
}

static void __init exynos5_dt_map_io(void)
{
	exynos_init_io(NULL, 0);
}

#define S5P_PMU_DEBUG				S5P_PMUREG(0x0A00)
/* PMU_DEBUG bits [12:8] = 0x10000 selects XXTI clock source */
#define PMU_DEBUG_XXTI				(0x10 << 8)
/* Mask bit[12:8] for xxti clock selection */
#define PMU_DEBUG_CLKOUT_SEL_MASK		0x1f00

static void __init enable_xclkout(void)
{
       unsigned int tmp;

       tmp = readl(S5P_PMU_DEBUG);
       tmp &= ~PMU_DEBUG_CLKOUT_SEL_MASK;
       tmp |= PMU_DEBUG_XXTI;
       writel(tmp, S5P_PMU_DEBUG);
}

static void __init exynos5_dt_machine_init(void)
{
	struct device_node *i2c_np;
	const char *i2c_compat = "samsung,s3c2440-i2c";
	unsigned int tmp;

	/* XCLKOUT needs to be moved over to the clock interface, but enable it
	 * here for now.
	 */
	enable_xclkout();

	/*
	 * Exynos5's legacy i2c controller and new high speed i2c
	 * controller have muxed interrupt sources. By default the
	 * interrupts for 4-channel HS-I2C controller are enabled.
	 * If node for first four channels of legacy i2c controller
	 * are available then re-configure the interrupts via the
	 * system register.
	 */
	if (of_machine_is_compatible("samsung,exynos5250")) {
		for_each_compatible_node(i2c_np, NULL, i2c_compat) {
			if (of_device_is_available(i2c_np)) {
				if (of_alias_get_id(i2c_np, "i2c") < 4) {
					tmp = readl(EXYNOS5_SYS_I2C_CFG);
					writel(tmp & ~(0x1 <<
						of_alias_get_id(i2c_np, "i2c")),
							EXYNOS5_SYS_I2C_CFG);
				}
			}
		}
	}

	if (of_machine_is_compatible("samsung,exynos5250")) {
		gpio_request_one(EXYNOS5_GPX3(0), GPIOF_OUT_INIT_HIGH,
								"lcd_bl_en");
		samsung_bl_set(&smdk5250_bl_gpio_info, &smdk5250_bl_data);
		exynos5_fimd1_gpio_setup_24bpp();
		of_platform_populate(NULL, of_default_bus_match_table,
				     NULL, NULL);
		platform_device_register(&exynos5_lcd);
	} else if (of_machine_is_compatible("samsung,exynos5420")) {
		of_platform_populate(NULL, of_default_bus_match_table,
				     NULL, NULL);
	} else if (of_machine_is_compatible("samsung,exynos5440"))
		of_platform_populate(NULL, of_default_bus_match_table,
				     NULL, NULL);
}

static char const *exynos5_dt_compat[] __initdata = {
	"samsung,exynos5250",
	"samsung,exynos5420",
	"samsung,exynos5440",
	NULL
};

static void __init exynos5_reserve(void)
{
#ifdef CONFIG_S5P_DEV_MFC
	struct s5p_mfc_dt_meminfo mfc_mem[] = {
		{ .compatible = "samsung,mfc-v6" },
		{ .compatible = "samsung,mfc-v7" },
	};
	int i;

	/* Reserve memory for MFC only if it's available */
	for (i = 0; i < ARRAY_SIZE(mfc_mem); i++)
		if (of_scan_flat_dt(s5p_fdt_find_mfc_mem, &mfc_mem[i])) {
			s5p_mfc_reserve_mem(mfc_mem[i].roff, mfc_mem[i].rsize,
					    mfc_mem[i].loff, mfc_mem[i].lsize);
			break;
		}
#endif

	exynos5_ramoops_reserve();
}

struct muxing_entry {
	const char *clock;
	const char *parent;
};

/**
 * peach_muxing_table - Peach static muxing table
 *
 * Contains clock muxing that should happen at startup.
 */
static const struct muxing_entry peach_muxing_table[] __initconst = {
	{ .clock = "mout_spi0", .parent = "sclk_mpll" },
	{ .clock = "mout_spi1", .parent = "sclk_mpll" },
	{ .clock = "mout_spi2", .parent = "sclk_mpll" },
	{ /* sentinel */ },
};

/**
 * apply_clock_muxing - apply a clock muxing table
 *
 * This will walk through the given table and set clocks to have the given
 * parent.
 */

static void __init apply_clock_muxing(const struct muxing_entry *muxing_table)
{
	const struct muxing_entry *entry;
	int ret;

	/*
	 * UGLY: We use __clk_lookup() here to avoid adding aliases
	 * for every last clock in clk-exynos5yyy.c
	 */
	for (entry = muxing_table; entry->parent; entry++) {
		struct clk *parent = __clk_lookup(entry->parent);
		struct clk *clock = __clk_lookup(entry->clock);

		pr_debug("Set parent of '%s' to '%s'\n",
			 entry->clock, entry->parent);

		if (!parent) {
			pr_err("Missing parent clock: '%s'\n", entry->parent);
			continue;
		} else if (!clock) {
			pr_err("Missing child clock: '%s'\n", entry->clock);
			continue;
		}

		ret = clk_set_parent(clock, parent);
		if (ret)
			pr_err("Couldn't set parent of '%s' to '%s'\n",
			       entry->clock, entry->parent);
	}
}

/**
 * exynos5_dt_setup_clock_muxing - static initialization of clock muxing
 *
 * We somehow need to setup clock muxing and would rather not rely on U-Boot
 * setting it up properly for us (it makes us fragile against U-Boot changes).
 *
 * The setup of clock muxing is a mix of the design of the board and the way
 * that the software is currently setup.  Because it's not purely defined by
 * the board, it can't live in the device tree.  That means that (at the moment)
 * the best place for it is here.
 *
 * We don't setup clock rates here and probably shouldn't add that because:
 * - If changing the muxing makes a clock violate its allowable frequency it
 *   seems like we should add the appropriate callbacks to the exynos5yyy clock
 *   driver to disallow that.
 * - If a given driver needs a certain frequency it should ask for it, probably
 *   by using a device tree node like "clock-frequency".
 */
static int __init exynos5_dt_setup_clock_muxing(void)
{
	if (of_machine_is_compatible("google,peach"))
		apply_clock_muxing(peach_muxing_table);

	return 0;
}
core_initcall(exynos5_dt_setup_clock_muxing);

DT_MACHINE_START(EXYNOS5_DT, "SAMSUNG EXYNOS5 (Flattened Device Tree)")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.init_irq	= exynos5_init_irq,
	.smp		= smp_ops(exynos_smp_ops),
	.map_io		= exynos5_dt_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= exynos5_dt_machine_init,
	.init_late	= exynos_init_late,
	.init_time	= exynos_init_time,
	.dt_compat	= exynos5_dt_compat,
	.restart        = exynos5_restart,
	.reserve	= exynos5_reserve,
MACHINE_END
