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
#include <linux/serial_core.h>
#include <linux/memblock.h>
#include <linux/io.h>
#include <linux/pwm_backlight.h>
#include <linux/platform_data/ntc_thermistor.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/memblock.h>
#include <linux/of_fdt.h>
#include <linux/spi/spi.h>
#include <linux/platform_data/spi-s3c64xx.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <mach/map.h>
#include <mach/regs-pmu.h>

#include <plat/cpu.h>
#include <plat/regs-serial.h>
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

static struct s3c64xx_spi_csinfo spi1_csi[] = {
	[0] = {
		.line		= EXYNOS5_GPA2(5),
		.fb_delay	= 0x2,
	},
};

static struct spi_board_info spi1_board_info[] __initdata = {
	{
		.modalias		= "spidev",
		.platform_data		= NULL,
		.max_speed_hz		= 10*1000*1000,
		.bus_num		= 1,
		.chip_select		= 0,
		.mode			= SPI_MODE_0,
		.controller_data	= spi1_csi,
	}
};

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
	.pwm_period_ns	= 1000000,
	.notify		= smdk5250_bl_notify,
};

static void lcd_set_power(struct plat_lcd_data *pd,
			unsigned int power)
{
	struct regulator *lcd_fet;
	struct regulator *backlight_fet;

	lcd_fet = regulator_get(NULL, "lcd_vdd");
	if (!IS_ERR(lcd_fet)) {
		if (power)
			regulator_enable(lcd_fet);
		else
			regulator_disable(lcd_fet);

		regulator_put(lcd_fet);
	}

	/* Turn on regulator for backlight */
	backlight_fet = regulator_get(NULL, "vcd_led");
	if (!IS_ERR(backlight_fet)) {
		if (power)
			regulator_enable(backlight_fet);
		else
			regulator_disable(backlight_fet);

		regulator_put(backlight_fet);
	}
	/* Wait 10 ms between regulator on and PWM start per spec */
	mdelay(10);
}

static struct plat_lcd_data smdk5250_lcd_data = {
	.set_power	= lcd_set_power,
};

static struct platform_device smdk5250_lcd = {
	.name			= "platform-lcd",
	.dev.platform_data	= &smdk5250_lcd_data,
};

/*
 * The following lookup table is used to override device names when devices
 * are registered from device tree. This is temporarily added to enable
 * device tree support addition for the EXYNOS5 architecture.
 *
 * For drivers that require platform data to be provided from the machine
 * file, a platform data pointer can also be supplied along with the
 * devices names. Usually, the platform data elements that cannot be parsed
 * from the device tree by the drivers (example: function pointers) are
 * supplied. But it should be noted that this is a temporary mechanism and
 * at some point, the drivers should be capable of parsing all the platform
 * data from the device tree.
 */
static const struct of_dev_auxdata exynos5250_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("samsung,exynos4210-uart", EXYNOS5_PA_UART0,
				"exynos4210-uart.0", NULL),
	OF_DEV_AUXDATA("samsung,exynos4210-uart", EXYNOS5_PA_UART1,
				"exynos4210-uart.1", NULL),
	OF_DEV_AUXDATA("samsung,exynos4210-uart", EXYNOS5_PA_UART2,
				"exynos4210-uart.2", NULL),
	OF_DEV_AUXDATA("samsung,exynos4210-uart", EXYNOS5_PA_UART3,
				"exynos4210-uart.3", NULL),
	OF_DEV_AUXDATA("samsung,s3c2440-i2c", EXYNOS5_PA_IIC(0),
				"s3c2440-i2c.0", NULL),
	OF_DEV_AUXDATA("samsung,s3c2440-i2c", EXYNOS5_PA_IIC(1),
				"s3c2440-i2c.1", NULL),
	OF_DEV_AUXDATA("samsung,s3c2440-i2c", EXYNOS5_PA_IIC(2),
				"s3c2440-i2c.2", NULL),
	OF_DEV_AUXDATA("samsung,s3c2440-i2c", EXYNOS5_PA_IIC(3),
				"s3c2440-i2c.3", NULL),
	OF_DEV_AUXDATA("samsung,s3c2440-i2c", EXYNOS5_PA_IIC(4),
				"s3c2440-i2c.4", NULL),
	OF_DEV_AUXDATA("samsung,s3c2440-i2c", EXYNOS5_PA_IIC(5),
				"s3c2440-i2c.5", NULL),
	OF_DEV_AUXDATA("samsung,s3c2440-i2c", EXYNOS5_PA_IIC(6),
				"s3c2440-i2c.6", NULL),
	OF_DEV_AUXDATA("samsung,s3c2440-i2c", EXYNOS5_PA_IIC(7),
				"s3c2440-i2c.7", NULL),
	OF_DEV_AUXDATA("samsung,s3c2440-hdmiphy-i2c", EXYNOS5_PA_IIC(8),
				"s3c2440-hdmiphy-i2c", NULL),
	OF_DEV_AUXDATA("samsung,exynos5250-dw-mshc", EXYNOS5_PA_DWMCI0,
				"dw_mmc.0", NULL),
	OF_DEV_AUXDATA("samsung,exynos5250-dw-mshc", EXYNOS5_PA_DWMCI1,
				"dw_mmc.1", NULL),
	OF_DEV_AUXDATA("samsung,exynos5250-dw-mshc", EXYNOS5_PA_DWMCI2,
				"dw_mmc.2", NULL),
	OF_DEV_AUXDATA("samsung,exynos5250-dw-mshc", EXYNOS5_PA_DWMCI3,
				"dw_mmc.3", NULL),
	OF_DEV_AUXDATA("samsung,exynos4210-spi", EXYNOS5_PA_SPI0,
				"exynos4210-spi.0", NULL),
	OF_DEV_AUXDATA("samsung,exynos4210-spi", EXYNOS5_PA_SPI1,
				"exynos4210-spi.1", NULL),
	OF_DEV_AUXDATA("samsung,exynos4210-spi", EXYNOS5_PA_SPI2,
				"exynos4210-spi.2", NULL),
	OF_DEV_AUXDATA("samsung,exynos5-sata-ahci", 0x122F0000,
				"exynos5-sata", NULL),
	OF_DEV_AUXDATA("samsung,exynos5-sata-phy", 0x12170000,
				"exynos5-sata-phy", NULL),
	OF_DEV_AUXDATA("samsung,exynos5-sata-phy-i2c", 0x121D0000,
				"exynos5-sata-phy-i2c", NULL),
	OF_DEV_AUXDATA("arm,pl330", EXYNOS5_PA_PDMA0, "dma-pl330.0", NULL),
	OF_DEV_AUXDATA("arm,pl330", EXYNOS5_PA_PDMA1, "dma-pl330.1", NULL),
	OF_DEV_AUXDATA("arm,pl330", EXYNOS5_PA_MDMA1, "dma-pl330.2", NULL),
	OF_DEV_AUXDATA("samsung,exynos5-gsc", EXYNOS5_PA_GSC0,
				"exynos-gsc.0", NULL),
	OF_DEV_AUXDATA("samsung,exynos5-gsc", EXYNOS5_PA_GSC1,
				"exynos-gsc.1", NULL),
	OF_DEV_AUXDATA("samsung,exynos5-gsc", EXYNOS5_PA_GSC2,
				"exynos-gsc.2", NULL),
	OF_DEV_AUXDATA("samsung,exynos5-gsc", EXYNOS5_PA_GSC3,
				"exynos-gsc.3", NULL),
	OF_DEV_AUXDATA("samsung,exynos5-hdmi", 0x14530000,
				"exynos5-hdmi", NULL),
	OF_DEV_AUXDATA("samsung,exynos5-mixer", 0x14450000,
				"exynos5-mixer", NULL),
	OF_DEV_AUXDATA("samsung,mfc-v6", 0x11000000, "s5p-mfc-v6", NULL),
	OF_DEV_AUXDATA("samsung,exynos5250-tmu", 0x10060000,
				"exynos-tmu", NULL),
	{},
};

static const struct of_dev_auxdata exynos5440_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("samsung,exynos4210-uart", EXYNOS5440_PA_UART0,
				"exynos4210-uart.0", NULL),
	{},
};


/* NTC Thermistor. Attached to S3C-ADC in some Samsung SoC Devices */
struct s3c_adc_client *ntc_adc_clients[4];
struct mutex ntc_adc_locks[] = {
	__MUTEX_INITIALIZER(ntc_adc_locks[0]),
	__MUTEX_INITIALIZER(ntc_adc_locks[1]),
	__MUTEX_INITIALIZER(ntc_adc_locks[2]),
	__MUTEX_INITIALIZER(ntc_adc_locks[3]),
};

static int __init s3c_adc_ntc_init(struct platform_device *pdev)
{
	struct s3c_adc_client *ntc_adc_client;

	ntc_adc_client = s3c_adc_register(pdev, NULL, NULL, 0);
	if (IS_ERR(ntc_adc_client))
		return PTR_ERR(ntc_adc_client);

	ntc_adc_clients[pdev->id] = ntc_adc_client;

	return 0;
}

/*
 * read_thermistor_uV: is a call back from ntc_thermistor driver.
 * returns the temperature of the thermistor in celsius
 */
static int read_thermistor_uV(struct platform_device *pdev)
{
	static unsigned int ntc_adc_ports[] = {3, 4, 5, 6};
	s64 converted;
	unsigned int port = ntc_adc_ports[pdev->id];
	struct s3c_adc_client *client = ntc_adc_clients[pdev->id];
	struct ntc_thermistor_platform_data *pdata = pdev->dev.platform_data;
	struct mutex *lock = ntc_adc_locks + pdev->id;
	int ret;

	/* Arrays are sized; make sure we haven't blown over */
	BUG_ON(pdev->id >= ARRAY_SIZE(ntc_adc_locks));

	/*
	 * s3c_adc_read() assumes two processes aren't using the same client
	 * at the same time (yes, it's a bad design), so grab a per-client
	 * mutex to ensure this is OK.
	 */
	mutex_lock(lock);
	ret = s3c_adc_read(client, port);
	mutex_unlock(lock);

	if (ret < 0) {
		dev_warn(&pdev->dev, "Thermistor read err: 0x%08x\n", ret);
		return ret;
	}

	converted = pdata->pullup_uV * (s64) ret;
	converted >>= 12;

	return (int) converted;
}

static struct ntc_thermistor_platform_data ntc_adc_pdata = {
	.read_uV	= read_thermistor_uV,
	.pullup_uV	= 1800000, /* voltage of vdd for ADC */
	.pullup_ohm	= 47000,
	.pulldown_ohm	= 0,
	.connect	= NTC_CONNECTED_GROUND,
};

struct platform_device s3c_device_adc_ntc_thermistor0 = {
	.name			= "ncp15wb473",
	.id			= 0,
	.dev			= {
		.platform_data = &ntc_adc_pdata,
	},
};

struct platform_device s3c_device_adc_ntc_thermistor1 = {
	.name			= "ncp15wb473",
	.id			= 1,
	.dev			= {
	.platform_data = &ntc_adc_pdata,
	},
};

struct platform_device s3c_device_adc_ntc_thermistor2 = {
	.name			= "ncp15wb473",
	.id			= 2,
	.dev			= {
		.platform_data = &ntc_adc_pdata,
	},
};

struct platform_device s3c_device_adc_ntc_thermistor3 = {
	.name			= "ncp15wb473",
	.id			= 3,
	.dev			= {
		.platform_data = &ntc_adc_pdata,
	},
};

static void __init exynos5_dt_map_io(void)
{
	unsigned long root = of_get_flat_dt_root();

	exynos_init_io(NULL, 0);

	if (of_flat_dt_is_compatible(root, "samsung,exynos5250"))
		s3c24xx_init_clocks(24000000);
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

static struct platform_device *smdk5250_devices[] __initdata = {
	&s3c_device_adc_ntc_thermistor0,
	&s3c_device_adc_ntc_thermistor1,
	&s3c_device_adc_ntc_thermistor2,
	&s3c_device_adc_ntc_thermistor3,
};

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
	for_each_compatible_node(i2c_np, NULL, i2c_compat) {
		if (of_device_is_available(i2c_np)) {
			if (of_alias_get_id(i2c_np, "i2c") < 4) {
				tmp = readl(EXYNOS5_SYS_I2C_CFG);
				writel(tmp & ~(0x1 << of_alias_get_id(i2c_np, "i2c")),
						EXYNOS5_SYS_I2C_CFG);
			}
		}
	}

	if (of_machine_is_compatible("samsung,exynos5250")) {
		gpio_request_one(EXYNOS5_GPX3(0), GPIOF_OUT_INIT_HIGH,
								"lcd_bl_en");
		samsung_bl_set(&smdk5250_bl_gpio_info, &smdk5250_bl_data);
		exynos5_fimd1_gpio_setup_24bpp();
		if (gpio_request_one(EXYNOS5_GPA2(5),
			GPIOF_OUT_INIT_HIGH, "SPI1_CS")) {
			printk(KERN_ERR "Spidev ChipSelect unavailable\n");
		} else {
			s3c_gpio_cfgpin(EXYNOS5_GPA2(5), S3C_GPIO_SFN(0x1));
			s3c_gpio_setpull(EXYNOS5_GPA2(5), S3C_GPIO_PULL_NONE);
			s5p_gpio_set_drvstr(EXYNOS5_GPA2(5),
					S5P_GPIO_DRVSTR_LV4);
			spi_register_board_info(spi1_board_info,
						ARRAY_SIZE(spi1_board_info));
		}
		of_platform_populate(NULL, of_default_bus_match_table,
				     exynos5250_auxdata_lookup, NULL);
		platform_device_register(&smdk5250_lcd);
		/* MAX77686 PMIC interrupt setup code */
		s3c_gpio_setpull(EXYNOS5_GPX3(2), S3C_GPIO_PULL_NONE);
	}
	else if (of_machine_is_compatible("samsung,exynos5440"))
		of_platform_populate(NULL, of_default_bus_match_table,
				     exynos5440_auxdata_lookup, NULL);

	/* Enable power to ADC */
	__raw_writel(0x1, S5P_ADC_PHY_CONTROL);

	s3c_adc_ntc_init(&s3c_device_adc_ntc_thermistor0);
	s3c_adc_ntc_init(&s3c_device_adc_ntc_thermistor1);
	s3c_adc_ntc_init(&s3c_device_adc_ntc_thermistor2);
	s3c_adc_ntc_init(&s3c_device_adc_ntc_thermistor3);

	platform_add_devices(smdk5250_devices, ARRAY_SIZE(smdk5250_devices));
}

static char const *exynos5_dt_compat[] __initdata = {
	"samsung,exynos5250",
	"samsung,exynos5440",
	NULL
};

static void __init exynos5_reserve(void)
{
#ifdef CONFIG_S5P_DEV_MFC
	struct s5p_mfc_dt_meminfo mfc_mem;

	/* Reserve memory for MFC only if it's available */
	mfc_mem.compatible = "samsung,mfc-v6";
	if (of_scan_flat_dt(s5p_fdt_find_mfc_mem, &mfc_mem))
		s5p_mfc_reserve_mem(mfc_mem.roff, mfc_mem.rsize, mfc_mem.loff,
				mfc_mem.lsize);
#endif

	exynos5_ramoops_reserve();
}

DT_MACHINE_START(EXYNOS5_DT, "SAMSUNG EXYNOS5 (Flattened Device Tree)")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.init_irq	= exynos5_init_irq,
	.smp		= smp_ops(exynos_smp_ops),
	.map_io		= exynos5_dt_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= exynos5_dt_machine_init,
	.init_late	= exynos_init_late,
	.timer		= &exynos4_timer,
	.dt_compat	= exynos5_dt_compat,
	.restart        = exynos5_restart,
	.reserve	= exynos5_reserve,
MACHINE_END
