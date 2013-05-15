/*
 * sec-core.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/irq.h>
#include <linux/mfd/samsung/rtc.h>
#include <linux/mfd/samsung/s5m8767.h>
#include <linux/regmap.h>

static struct mfd_cell s5m8751_devs[] = {
	{
		.name = "s5m8751-pmic",
	}, {
		.name = "s5m-charger",
	}, {
		.name = "s5m8751-codec",
	},
};

static struct mfd_cell s5m8763_devs[] = {
	{
		.name = "s5m8763-pmic",
	}, {
		.name = "s5m-rtc",
	}, {
		.name = "s5m-charger",
	},
};

static struct mfd_cell s5m8767_devs[] = {
	{
		.name = "s5m8767-pmic",
	}, {
		.name = "s5m-rtc",
	},
};

static struct mfd_cell s2mps11_devs[] = {
	{
		.name = "s2mps11-pmic",
	},
};

#ifdef CONFIG_OF
static struct of_device_id sec_dt_match[] = {
	{	.compatible = "samsung,s5m8767-pmic",
		.data = (void *)S5M8767X,
	},
	{	.compatible = "samsung,s2mps11-pmic",
		.data = (void *)S2MPS11X,
	},
	{},
};
#endif

static struct regmap_config sec_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};


#ifdef CONFIG_OF
/*
 * Only the common platform data elements for s5m8767 are parsed here from the
 * device tree. Other sub-modules of s5m8767 such as pmic, rtc , charger and
 * others have to parse their own platform data elements from device tree.
 *
 * The s5m8767 platform data structure is instantiated here and the drivers for
 * the sub-modules need not instantiate another instance while parsing their
 * platform data.
 */
static struct sec_platform_data *sec_pmic_i2c_parse_dt_pdata(
					struct device *dev)
{
	struct sec_platform_data *pd;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		dev_err(dev, "could not allocate memory for pdata\n");
		return ERR_PTR(-ENOMEM);
	}

	/*
	 * ToDo: the 'wakeup' member in the platform data is more of a linux
	 * specfic information. Hence, there is no binding for that yet and
	 * not parsed here.
	 */

	if (of_get_property(dev->of_node, "s5m-core,enable-low-jitter", NULL)) {
		if (!(pd->device_type == S5M8767X))
			dev_warn(dev, "no low-jitter for this PMIC type\n");
		else
			pd->low_jitter = true;
	}
	dev_dbg(dev, "OF low-jitter property: %u\n", pd->low_jitter);

	return pd;
}
#else
static struct sec_platform_data *sec_pmic_i2c_parse_dt_pdata(
					struct device *dev)
{
	return 0;
}
#endif

static inline int sec_i2c_get_driver_data(struct i2c_client *i2c,
						const struct i2c_device_id *id)
{
#ifdef CONFIG_OF
	if (i2c->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_node(sec_dt_match, i2c->dev.of_node);
		return (int)match->data;
	}
#endif
	return (int)id->driver_data;
}

static int sec_pmic_set_low_jitter(struct sec_pmic_dev *sec_pmic)
{
	if (!sec_pmic->pdata->low_jitter)
		return 0;

	return regmap_update_bits(sec_pmic->pmic, S5M8767_REG_CTRL1,
				  S5M8767_LOW_JITTER_MASK,
				  S5M8767_LOW_JITTER_MASK);
}

static int sec_pmic_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct sec_platform_data *pdata = i2c->dev.platform_data;
	struct sec_pmic_dev *sec_pmic;
	struct i2c_client *rtc_i2c = NULL;
	int ret;

	sec_pmic = devm_kzalloc(&i2c->dev, sizeof(struct sec_pmic_dev),
				GFP_KERNEL);
	if (sec_pmic == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, sec_pmic);
	sec_pmic->dev = &i2c->dev;
	sec_pmic->irq = i2c->irq;
	sec_pmic->type = sec_i2c_get_driver_data(i2c, id);

	if (sec_pmic->dev->of_node) {
		pdata = sec_pmic_i2c_parse_dt_pdata(sec_pmic->dev);
		if (IS_ERR(pdata)) {
			ret = PTR_ERR(pdata);
			return ret;
		}
		pdata->device_type = sec_pmic->type;
	}
	if (pdata) {
		sec_pmic->device_type = pdata->device_type;
		sec_pmic->ono = pdata->ono;
		sec_pmic->irq_base = pdata->irq_base;
		sec_pmic->wakeup = pdata->wakeup;
		sec_pmic->pdata = pdata;
	}

	sec_pmic->pmic = devm_regmap_init_i2c(i2c, &sec_regmap_config);
	if (IS_ERR(sec_pmic->pmic)) {
		ret = PTR_ERR(sec_pmic->pmic);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	rtc_i2c = i2c_new_dummy(i2c->adapter, RTC_I2C_ADDR);
	i2c_set_clientdata(rtc_i2c, sec_pmic);
	sec_pmic->rtc = devm_regmap_init_i2c(rtc_i2c, &sec_regmap_config);
	if (IS_ERR(sec_pmic->rtc)) {
		ret = PTR_ERR(sec_pmic->rtc);
		dev_err(&rtc_i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		goto err;
	}

	if (pdata && pdata->cfg_pmic_irq)
		pdata->cfg_pmic_irq();

	sec_irq_init(sec_pmic);

	pm_runtime_set_active(sec_pmic->dev);

	switch (sec_pmic->device_type) {
	case S5M8751X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s5m8751_devs,
				      ARRAY_SIZE(s5m8751_devs), NULL, 0, NULL);
		break;
	case S5M8763X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s5m8763_devs,
				      ARRAY_SIZE(s5m8763_devs), NULL, 0, NULL);
		break;
	case S5M8767X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s5m8767_devs,
				      ARRAY_SIZE(s5m8767_devs), NULL, 0, NULL);
		break;
	case S2MPS11X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s2mps11_devs,
				      ARRAY_SIZE(s2mps11_devs), NULL, 0, NULL);
		break;
	default:
		/* If this happens the probe function is problem */
		BUG();
	}

	if (ret < 0)
		goto err;

	if (sec_pmic_set_low_jitter(sec_pmic) < 0) {
		dev_err(sec_pmic->dev, "failed to configure low-jitter\n");
		ret = -EIO;
		goto err;
	}

	return ret;

err:
	mfd_remove_devices(sec_pmic->dev);
	sec_irq_exit(sec_pmic);
	if (rtc_i2c)
		i2c_unregister_device(rtc_i2c);
	return ret;
}

static int sec_pmic_remove(struct i2c_client *i2c)
{
	struct sec_pmic_dev *sec_pmic = i2c_get_clientdata(i2c);

	device_init_wakeup(sec_pmic->dev, 0);
	pm_runtime_set_suspended(sec_pmic->dev);
	mfd_remove_devices(sec_pmic->dev);
	sec_irq_exit(sec_pmic);
	return 0;
}

static const struct i2c_device_id sec_pmic_id[] = {
	{ "sec_pmic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sec_pmic_id);

#ifdef CONFIG_PM
static int sec_pmic_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct sec_pmic_dev *sec_pmic = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		enable_irq_wake(sec_pmic->irq);

	return 0;
}

static int sec_pmic_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct sec_pmic_dev *sec_pmic = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		disable_irq_wake(sec_pmic->irq);

	return 0;
}

const struct dev_pm_ops sec_pmic_pm = {
	.suspend = sec_pmic_suspend,
	.resume = sec_pmic_resume,
};
#endif

static struct i2c_driver sec_pmic_driver = {
	.driver = {
		   .name = "sec_pmic",
		   .owner = THIS_MODULE,
#ifdef CONFIG_PM
		   .pm = &sec_pmic_pm,
#endif
		   .of_match_table = of_match_ptr(sec_dt_match),
	},
	.probe = sec_pmic_probe,
	.remove = sec_pmic_remove,
	.id_table = sec_pmic_id,
};

static int __init sec_pmic_init(void)
{
	return i2c_add_driver(&sec_pmic_driver);
}

subsys_initcall(sec_pmic_init);

static void __exit sec_pmic_exit(void)
{
	i2c_del_driver(&sec_pmic_driver);
}
module_exit(sec_pmic_exit);

MODULE_AUTHOR("Sangbeom Kim <sbkim73@samsung.com>");
MODULE_DESCRIPTION("Core support for the S5M MFD");
MODULE_LICENSE("GPL");
