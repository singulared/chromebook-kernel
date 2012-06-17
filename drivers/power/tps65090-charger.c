/*
 * tps65090_charger.c - Power supply consumer driver for the TI TPS65090
 *
 * Copyright (c) 2012 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/mfd/tps65090.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>

#include "tps65090-private.h"

struct charger_data {
	struct device *dev;
	struct power_supply *charger;
	/* used by regulator core */
	struct regulator_desc desc;
	struct regulator_dev *rdev;
};

static enum power_supply_property tps65090_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS, /* "FULL" or "NOT FULL" only. */
	POWER_SUPPLY_PROP_PRESENT, /* the presence of battery */
	POWER_SUPPLY_PROP_ONLINE, /* charger is active or not */
};

static inline struct device *to_tps65090_dev(struct charger_data *charger_data)
{
	return charger_data->dev->parent;
}

static int tps65090_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct platform_device *pdev = to_platform_device(psy->dev->parent);
	struct charger_data *charger_data = platform_get_drvdata(pdev);
	struct device *parent = to_tps65090_dev(charger_data);
	int ret;
	u8 reg;

	ret = tps65090_read(parent, TPS65090_REG_IRQ1, &reg);
	if (ret)
		return ret;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (reg & TPS65090_IRQ1_VACG_MASK) {
			if (reg & TPS65090_IRQ1_CGACT_MASK)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else {
			if (reg & TPS65090_IRQ1_VBATG_MASK)
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !!(reg & TPS65090_IRQ1_VSYSG_MASK);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(reg & TPS65090_IRQ1_VACG_MASK);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct power_supply tps65090_charger = {
	.name = "tps65090-charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.get_property = tps65090_charger_get_property,
	.properties = tps65090_charger_props,
	.num_properties = ARRAY_SIZE(tps65090_charger_props),
};

static const struct of_device_id charger_data_match[] = {
	{ .compatible = "ti,tps65090" },
	{ }
};
MODULE_DEVICE_TABLE(of, charger_data_match);

static int tps65090_charger_is_enabled(struct regulator_dev *rdev)
{
	struct charger_data *cd = rdev_get_drvdata(rdev);
	struct device *parent = to_tps65090_dev(cd);
	uint8_t control;
	int ret;

	ret = tps65090_read(parent, TPS65090_REG_CG_CTRL0, &control);
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in checking charge state\n");
		return ret;
	}

	return (control & TPS65090_CG_CTRL0_ENC) != 0;
}

static int tps65090_charger_enable(struct regulator_dev *rdev)
{
	struct charger_data *cd = rdev_get_drvdata(rdev);
	struct device *parent = to_tps65090_dev(cd);
	int ret;

	ret = tps65090_set_bits(parent, TPS65090_REG_CG_CTRL0,
				TPS65090_CG_CTRL0_ENC);
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in enabling charging\n");
		return ret;
	}

	return ret;
}

static int tps65090_charger_disable(struct regulator_dev *rdev)
{
	struct charger_data *cd = rdev_get_drvdata(rdev);
	struct device *parent = to_tps65090_dev(cd);
	int ret;

	ret = tps65090_clr_bits(parent, TPS65090_REG_CG_CTRL0,
				TPS65090_CG_CTRL0_ENC);
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in disabling charging\n");
		return ret;
	}

	return ret;
}

static int tps65090_set_charger_voltage(struct regulator_dev *rdev, int min,
				       int max, unsigned *sel)
{
	/*
	 * Only needed for the core code to set constraints; the voltage
	 * isn't actually adjustable on tps65090.
	 */
	return 0;
}

static struct regulator_ops tps65090_charger_ops = {
	.enable		= tps65090_charger_enable,
	.disable	= tps65090_charger_disable,
	.set_voltage	= tps65090_set_charger_voltage,
	.is_enabled	= tps65090_charger_is_enabled,
};

static __devinit int tps65090_charger_probe(struct platform_device *pdev)
{
	struct device_node *mfd_node, *np;
	struct charger_data *charger_data;
	struct regulator_init_data *ri;
	int ret = 0;

	mfd_node = pdev->dev.parent->of_node;
	if (!mfd_node) {
		dev_err(&pdev->dev, "no device tree data available\n");
		return -EINVAL;
	}

	np = of_find_node_by_name(mfd_node, "charger");
	if (!np) {
		dev_err(&pdev->dev, "no OF charger data found at %s\n",
			mfd_node->full_name);
		return -EINVAL;
	}

	charger_data = devm_kzalloc(&pdev->dev, sizeof(struct charger_data),
				    GFP_KERNEL);
	if (!charger_data) {
		dev_err(&pdev->dev, "Failed to alloc driver structure\n");
		ret = -ENOMEM;
		goto err_nomem;
	}

	/* Register the driver. */
	charger_data->dev = &pdev->dev;
	charger_data->charger = &tps65090_charger;
	platform_set_drvdata(pdev, charger_data);
	pdev->dev.of_node = np;
	ret = power_supply_register(&pdev->dev, &tps65090_charger);
	if (ret) {
		dev_err(charger_data->dev, "failed: power supply register\n");
		goto err;
	}

	ri = of_get_regulator_init_data(&pdev->dev, np);
	if (!ri) {
		dev_err(&pdev->dev, "regulator_init_data failed for %s\n",
			np->full_name);
		goto err_reg;
	}

	if (ri->constraints.name) {
		charger_data->desc.name = ri->constraints.name;
		charger_data->desc.id = 0;
		charger_data->desc.ops = &tps65090_charger_ops;
		charger_data->desc.type = REGULATOR_VOLTAGE;
		charger_data->desc.owner = THIS_MODULE;
		charger_data->rdev = regulator_register(&charger_data->desc,
					&pdev->dev, ri, charger_data, np);
		if (IS_ERR(charger_data->rdev)) {
			dev_err(&pdev->dev, "Unable to regsister regulator\n");
			goto err_reg;
		}
	}

	of_node_put(np);
	return 0;
err_reg:
	power_supply_unregister(charger_data->charger);
err:
err_nomem:
	of_node_put(np);
	return ret;
}

static int __devexit tps65090_charger_remove(struct platform_device *pdev)
{
	struct charger_data *charger_data = platform_get_drvdata(pdev);

	power_supply_unregister(charger_data->charger);
	regulator_unregister(charger_data->rdev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tps65090_charger_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct charger_data *charger_data = platform_get_drvdata(pdev);

	power_supply_changed(charger_data->charger);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(tps65090_charger_pm_ops, NULL,
			 tps65090_charger_resume);

static struct platform_driver tps65090_charger_driver = {
	.driver = {
		.name = "tps65090-charger",
		.owner = THIS_MODULE,
		.pm = &tps65090_charger_pm_ops,
		.of_match_table = of_match_ptr(charger_data_match),
	},
	.probe = tps65090_charger_probe,
	.remove = __devexit_p(tps65090_charger_remove),
};

static int __init tps65090_charger_init(void)
{
	return platform_driver_register(&tps65090_charger_driver);
}
subsys_initcall(tps65090_charger_init);

static void __exit tps65090_charger_exit(void)
{
	platform_driver_unregister(&tps65090_charger_driver);
}
module_exit(tps65090_charger_exit);

MODULE_DESCRIPTION("TI tps65090 battery charger driver");
MODULE_AUTHOR("Simon Glass <sjg@chromium.org>");
MODULE_LICENSE("GPL");
