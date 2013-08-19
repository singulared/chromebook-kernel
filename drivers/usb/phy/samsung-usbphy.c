/* linux/drivers/usb/phy/samsung-usbphy.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com
 *
 * Author: Praveen Paneri <p.paneri@samsung.com>
 *
 * Samsung USB-PHY helper driver with common function calls;
 * interacts with Samsung USB 2.0 PHY controller driver and later
 * with Samsung USB 3.0 PHY driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include "samsung-usbphy.h"

int samsung_usbphy_parse_dt(struct samsung_usbphy *sphy)
{
	struct device_node *usbphy_sys;
	struct device_node *node = sphy->dev->of_node;

	/* Get the Channel number in case of multiple PHY controllers */
	sphy->channel = of_alias_get_id(node, "usb3phy");
	if (sphy->channel < 0)
		dev_info(sphy->dev, "Not a multi controller PHY\n");

	/* Getting node for system controller interface for usb-phy */
	usbphy_sys = of_get_child_by_name(node, "usbphy-sys");
	if (!usbphy_sys) {
		dev_err(sphy->dev, "No sys-controller interface for usb-phy\n");
		return -ENODEV;
	}

	sphy->pmureg = of_iomap(usbphy_sys, 0);

	if (sphy->pmureg == NULL) {
		dev_err(sphy->dev, "Can't get usb-phy pmu control register\n");
		goto err0;
	}

	sphy->sysreg = of_iomap(usbphy_sys, 1);

	/*
	 * Not returning error code here, since this situation is not fatal.
	 * Few SoCs may not have this switch available
	 */
	if (sphy->sysreg == NULL)
		dev_warn(sphy->dev, "Can't get usb-phy sysreg cfg register\n");

	/*
	 * Some boards have a separate active-low reset GPIO for their HSIC USB
	 * devices. If they don't, this will just stay at an invalid value and
	 * the init code will ignore it.
	 */
	sphy->hsic_reset_gpio = of_get_named_gpio(node,
						  "samsung,hsic-reset-gpio", 0);
	if (gpio_is_valid(sphy->hsic_reset_gpio)) {
		if (devm_gpio_request_one(sphy->dev, sphy->hsic_reset_gpio,
				GPIOF_OUT_INIT_LOW, "samsung_hsic_reset")) {
			dev_err(sphy->dev, "can't request hsic reset gpio %d\n",
				sphy->hsic_reset_gpio);
			sphy->hsic_reset_gpio = -EINVAL;
		}
	}

	if (of_property_read_u32(node, "samsung,txpreempamptune",
				 &sphy->tx_preemp_amptune))
		dev_info(sphy->dev, "No HS Tx Pre-Emp current control\n");

	of_node_put(usbphy_sys);

	return 0;

err0:
	of_node_put(usbphy_sys);
	return -ENXIO;
}
EXPORT_SYMBOL_GPL(samsung_usbphy_parse_dt);

/*
 * Set isolation here for phy.
 * Here 'on = true' would mean USB PHY block is isolated, hence
 * de-activated and vice-versa.
 */
void samsung_usbphy_set_isolation(struct samsung_usbphy *sphy, bool on)
{
	void __iomem *reg = NULL;
	u32 reg_val;
	u32 en_mask = 0;

	if (!sphy->pmureg) {
		dev_warn(sphy->dev, "Can't set pmu isolation\n");
		return;
	}

	reg = sphy->pmureg;
	en_mask = sphy->drv_data->phy_en_mask;

	switch (sphy->drv_data->cpu_type) {
	case TYPE_S3C64XX:
		/*
		 * Do nothing: We will add here once S3C64xx goes for DT support
		 */
		return;
	case TYPE_EXYNOS4210:
		/*
		 * Exynos4 uses different registers for the HOST and DEVICE part
		 * of the PHY, so add an offset here if needed and fall through.
		 */
		if (sphy->phy.otg->host)
			reg += EXYNOS4_USBPHY_HOST_OFFSET;
	case TYPE_EXYNOS5250:
	case TYPE_EXYNOS5420:
		break;
	default:
		dev_err(sphy->dev, "Invalid SoC type\n");
		return;
	}

	if (reg) {
		reg_val = readl(reg);

		if (on)
			reg_val &= ~en_mask;
		else
			reg_val |= en_mask;
		writel(reg_val, reg);
	}
}
EXPORT_SYMBOL_GPL(samsung_usbphy_set_isolation);

/*
 * Configure the mode of working of usb-phy here: HOST/DEVICE.
 */
void samsung_usbphy_cfg_sel(struct samsung_usbphy *sphy, bool device_mode)
{
	u32 reg;

	if (!sphy->sysreg) {
		dev_warn(sphy->dev, "Can't configure specified phy mode\n");
		return;
	}

	reg = readl(sphy->sysreg);

	if (device_mode)
		reg &= ~EXYNOS_USB20PHY_CFG_HOST_LINK;
	else
		reg |= EXYNOS_USB20PHY_CFG_HOST_LINK;

	writel(reg, sphy->sysreg);
}
EXPORT_SYMBOL_GPL(samsung_usbphy_cfg_sel);

/*
 * Returns reference clock frequency selection value
 */
int samsung_usbphy_get_refclk_freq(struct samsung_usbphy *sphy)
{
	struct clk *ref_clk;
	int refclk_freq = 0;
	int cpu_type = sphy->drv_data->cpu_type;

	/*
	 * In exynos5250 USB host and device PHY use
	 * external crystal clock XXTI
	 */
	switch (cpu_type) {
	case TYPE_EXYNOS5420:
		/* fall through */
	case TYPE_EXYNOS5250:
		ref_clk = clk_get(sphy->dev, "ext_xtal");
		break;
	case TYPE_EXYNOS4210:
		/* fall through */
	case TYPE_S3C64XX:
		ref_clk = clk_get(sphy->dev, "xusbxti");
		break;
	default:
		break;
	}

	if (IS_ERR(ref_clk)) {
		dev_err(sphy->dev, "Failed to get reference clock\n");
		return PTR_ERR(ref_clk);
	}

	if (cpu_type == TYPE_EXYNOS5250 || cpu_type == TYPE_EXYNOS5420) {
		/* set clock frequency for PLL */
		switch (clk_get_rate(ref_clk)) {
		case 9600 * KHZ:
			refclk_freq = FSEL_CLKSEL_9600K;
			break;
		case 10 * MHZ:
			refclk_freq = FSEL_CLKSEL_10M;
			break;
		case 12 * MHZ:
			refclk_freq = FSEL_CLKSEL_12M;
			break;
		case 19200 * KHZ:
			refclk_freq = FSEL_CLKSEL_19200K;
			break;
		case 20 * MHZ:
			refclk_freq = FSEL_CLKSEL_20M;
			break;
		case 50 * MHZ:
			refclk_freq = FSEL_CLKSEL_50M;
			break;
		case 24 * MHZ:
		default:
			/* default reference clock */
			refclk_freq = FSEL_CLKSEL_24M;
			break;
		}
	} else {
		switch (clk_get_rate(ref_clk)) {
		case 12 * MHZ:
			refclk_freq = PHYCLK_CLKSEL_12M;
			break;
		case 24 * MHZ:
			refclk_freq = PHYCLK_CLKSEL_24M;
			break;
		case 48 * MHZ:
			refclk_freq = PHYCLK_CLKSEL_48M;
			break;
		default:
			if (sphy->drv_data->cpu_type == TYPE_S3C64XX)
				refclk_freq = PHYCLK_CLKSEL_48M;
			else
				refclk_freq = PHYCLK_CLKSEL_24M;
			break;
		}
	}
	clk_put(ref_clk);

	return refclk_freq;
}
EXPORT_SYMBOL_GPL(samsung_usbphy_get_refclk_freq);
