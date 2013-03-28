/*
 *  Copyright (C) 2012 The Chromium OS Authors
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define pr_fmt(fmt) "chromeos_vbc_ec: " fmt

#include <linux/err.h>
#include <linux/mfd/chromeos_ec.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#include "chromeos.h"

static phandle ec_phandle;

static int match_of_node(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static struct device *get_ec_dev(phandle phandle)
{
	struct device_node *dn;
	struct device *dev;

	dn = of_find_node_by_phandle(phandle);
	if (!dn)
		return ERR_PTR(-ENODEV);

	dev = bus_find_device(&platform_bus_type, NULL, dn, match_of_node);
	of_node_put(dn);
	if (!dev)
		return ERR_PTR(-ENODEV);

	return dev;
}

static int chromeos_vbc_ec_read(void *buf, size_t count)
{
	struct device *dev;
	struct chromeos_ec_device *ec;
	struct ec_params_vbnvcontext param;
	struct ec_response_vbnvcontext resp;
	int err;

	if (!ec_phandle)
		return -ENODEV;

	dev = get_ec_dev(ec_phandle);
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	ec = dev_get_drvdata(dev->parent);

	param.op = EC_VBNV_CONTEXT_OP_READ;
	err = ec->command_sendrecv(ec,
			EC_CMD_VBNV_CONTEXT | (EC_VER_VBNV_CONTEXT << 8),
			&param, sizeof(param.op),
			&resp, sizeof(resp));
	put_device(dev);
	if (err < 0)
		return err;

	count = min(count, sizeof(resp.block));
	memcpy(buf, resp.block, count);

	return count;
}

static int chromeos_vbc_ec_write(const void *buf, size_t count)
{
	struct device *dev;
	struct chromeos_ec_device *ec;
	struct ec_params_vbnvcontext param;
	int err;

	if (!ec_phandle)
		return -ENODEV;

	dev = get_ec_dev(ec_phandle);
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	ec = dev_get_drvdata(dev->parent);

	count = min(count, sizeof(param.block));

	param.op = EC_VBNV_CONTEXT_OP_WRITE;
	memcpy(param.block, buf, count);
	err = ec->command_send(ec,
			EC_CMD_VBNV_CONTEXT | (EC_VER_VBNV_CONTEXT << 8),
			&param, sizeof(param));
	put_device(dev);
	if (err < 0)
		return err;

	return count;
}

static int __devinit chromeos_vbc_ec_probe(struct platform_device *pdev)
{
	if (!ec_phandle)
		return -ENODEV;
	pdev->dev.of_node = of_find_node_by_phandle(ec_phandle);
	return 0;
}

static struct chromeos_vbc chromeos_vbc_ec = {
	.name = "chromeos_vbc_ec",
	.read = chromeos_vbc_ec_read,
	.write = chromeos_vbc_ec_write,
};

static struct platform_driver chromeos_vbc_ec_driver = {
	.probe = chromeos_vbc_ec_probe,
	.driver = {
		.name = "chromeos_vbc_ec",
	},
};

static int __init chromeos_vbc_ec_init(void)
{
	struct device_node *of_node;
	const char *vbc_type;
	int err;

	of_node = of_find_compatible_node(NULL, NULL, "chromeos-firmware");
	if (!of_node)
		return -ENODEV;

	err = of_property_read_string(of_node, "nonvolatile-context-storage",
			&vbc_type);
	if (err)
		goto exit;

	if (strcmp(vbc_type, "mkbp")) {
		err = 0;  /* not configured to use vbc_ec, exit normally. */
		goto exit;
	}

	err = of_property_read_u32(of_node, "chromeos-vbc-ec", &ec_phandle);
	if (err) {
		pr_err("missing chromeos-vbc-ec property\n");
		goto exit;
	}

	err = chromeos_vbc_register(&chromeos_vbc_ec);
	if (err < 0)
		goto exit;

	err = platform_driver_register(&chromeos_vbc_ec_driver);
	if (err < 0)
		goto exit;

	err = 0;
exit:
	of_node_put(of_node);
	return err;
}
module_init(chromeos_vbc_ec_init);

static void __exit chromeos_vbc_ec_exit(void)
{
	platform_driver_unregister(&chromeos_vbc_ec_driver);
}
module_exit(chromeos_vbc_ec_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ChromeOS vboot context on EC accessor");
