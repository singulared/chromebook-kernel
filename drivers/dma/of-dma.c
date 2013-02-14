/*
 * Device tree helpers for DMA request / controller
 *
 * Based on of_gpio.c
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/rculist.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_dma.h>

static LIST_HEAD(of_dma_list);
static DEFINE_SPINLOCK(of_dma_lock);

/**
 * of_dma_get_controller - Get a DMA controller in DT DMA helpers list
 * @dma_spec:	pointer to DMA specifier as found in the device tree
 *
 * Finds a DMA controller with matching device node and number for dma cells
 * in a list of registered DMA controllers. If a match is found the use_count
 * variable is increased and a valid pointer to the DMA data stored is retuned.
 * A NULL pointer is returned if no match is found.
 */
static struct of_dma *of_dma_get_controller(struct of_phandle_args *dma_spec)
{
	struct of_dma *ofdma;

	spin_lock(&of_dma_lock);

	if (list_empty(&of_dma_list)) {
		spin_unlock(&of_dma_lock);
		return NULL;
	}

	list_for_each_entry(ofdma, &of_dma_list, of_dma_controllers)
		if ((ofdma->of_node == dma_spec->np) &&
		    (ofdma->of_dma_nbcells == dma_spec->args_count)) {
			ofdma->use_count++;
			spin_unlock(&of_dma_lock);
			return ofdma;
		}

	spin_unlock(&of_dma_lock);

	pr_debug("%s: can't find DMA controller %s\n", __func__,
		 dma_spec->np->full_name);

	return NULL;
}

/**
 * of_dma_put_controller - Decrement use count for a registered DMA controller
 * @of_dma:	pointer to DMA controller data
 *
 * Decrements the use_count variable in the DMA data structure. This function
 * should be called only when a valid pointer is returned from
 * of_dma_get_controller() and no further accesses to data referenced by that
 * pointer are needed.
 */
static void of_dma_put_controller(struct of_dma *ofdma)
{
	spin_lock(&of_dma_lock);
	ofdma->use_count--;
	spin_unlock(&of_dma_lock);
}

/**
 * of_dma_controller_register - Register a DMA controller to DT DMA helpers
 * @np:			device node of DMA controller
 * @of_dma_xlate:	translation function which converts a phandle
 *			arguments list into a dma_chan structure
 * @data		pointer to controller specific data to be used by
 *			translation function
 *
 * Returns 0 on success or appropriate errno value on error.
 *
 * Allocated memory should be freed with appropriate of_dma_controller_free()
 * call.
 */
int of_dma_controller_register(struct device_node *np,
				struct dma_chan *(*of_dma_xlate)
				(struct of_phandle_args *, struct of_dma *),
				void *data)
{
	struct of_dma	*ofdma;
	int		nbcells;

	if (!np || !of_dma_xlate) {
		pr_err("%s: not enough information provided\n", __func__);
		return -EINVAL;
	}

	ofdma = kzalloc(sizeof(*ofdma), GFP_KERNEL);
	if (!ofdma)
		return -ENOMEM;

	nbcells = be32_to_cpup(of_get_property(np, "#dma-cells", NULL));
	if (!nbcells) {
		pr_err("%s: #dma-cells property is missing or invalid\n",
		       __func__);
		kfree(ofdma);
		return -EINVAL;
	}

	ofdma->of_node = np;
	ofdma->of_dma_nbcells = nbcells;
	ofdma->of_dma_xlate = of_dma_xlate;
	ofdma->of_dma_data = data;
	ofdma->use_count = 0;

	/* Now queue of_dma controller structure in list */
	list_add_tail(&ofdma->of_dma_controllers, &of_dma_list);

	return 0;
}
EXPORT_SYMBOL_GPL(of_dma_controller_register);

/**
 * of_dma_controller_free - Remove a DMA controller from DT DMA helpers list
 * @np:		device node of DMA controller
 *
 * Memory allocated by of_dma_controller_register() is freed here.
 */
int of_dma_controller_free(struct device_node *np)
{
	struct of_dma *ofdma;

	spin_lock(&of_dma_lock);

	if (list_empty(&of_dma_list)) {
		spin_unlock(&of_dma_lock);
		return -ENODEV;
	}

	list_for_each_entry(ofdma, &of_dma_list, of_dma_controllers)
		if (ofdma->of_node == np) {
			if (ofdma->use_count) {
				spin_unlock(&of_dma_lock);
				return -EBUSY;
			}

			list_del(&ofdma->of_dma_controllers);
			spin_unlock(&of_dma_lock);
			kfree(ofdma);
			return 0;
		}

	spin_unlock(&of_dma_lock);
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(of_dma_controller_free);

/**
 * of_dma_find_channel - Find a DMA channel by name
 * @np:		device node to look for DMA channels
 * @name:	name of desired channel
 * @dma_spec:	pointer to DMA specifier as found in the device tree
 *
 * Find a DMA channel by the name. Returns 0 on success or appropriate
 * errno value on error.
 */
static int of_dma_find_channel(struct device_node *np, char *name,
			       struct of_phandle_args *dma_spec)
{
	int count, i;
	const char *s;

	count = of_property_count_strings(np, "dma-names");
	if (count < 0)
		return count;

	for (i = 0; i < count; i++) {
		if (of_property_read_string_index(np, "dma-names", i, &s))
			continue;

		if (strcmp(name, s))
			continue;

		if (!of_parse_phandle_with_args(np, "dmas", "#dma-cells", i,
						dma_spec))
			return 0;
	}

	return -ENODEV;
}

/**
 * of_dma_request_slave_channel - Get the DMA slave channel
 * @np:		device node to get DMA request from
 * @name:	name of desired channel
 *
 * Returns pointer to appropriate dma channel on success or NULL on error.
 */
struct dma_chan *of_dma_request_slave_channel(struct device_node *np,
					      char *name)
{
	struct of_phandle_args	dma_spec;
	struct of_dma		*ofdma;
	struct dma_chan		*chan;
	int			r;

	if (!np || !name) {
		pr_err("%s: not enough information provided\n", __func__);
		return NULL;
	}

	do {
		r = of_dma_find_channel(np, name, &dma_spec);
		if (r) {
			pr_err("%s: can't find DMA channel\n", np->full_name);
			return NULL;
		}

		ofdma = of_dma_get_controller(&dma_spec);

		if (!ofdma)
			continue;

		chan = ofdma->of_dma_xlate(&dma_spec, ofdma);

		of_dma_put_controller(ofdma);

		of_node_put(dma_spec.np);

	} while (!chan);

	return chan;
}

/**
 * of_dma_simple_xlate - Simple DMA engine translation function
 * @dma_spec:	pointer to DMA specifier as found in the device tree
 * @of_dma:	pointer to DMA controller data
 *
 * A simple translation function for devices that use a 32-bit value for the
 * filter_param when calling the DMA engine dma_request_channel() function.
 * Note that this translation function requires that #dma-cells is equal to 1
 * and the argument of the dma specifier is the 32-bit filter_param. Returns
 * pointer to appropriate dma channel on success or NULL on error.
 */
struct dma_chan *of_dma_simple_xlate(struct of_phandle_args *dma_spec,
						struct of_dma *ofdma)
{
	int count = dma_spec->args_count;
	struct of_dma_filter_info *info = ofdma->of_dma_data;

	if (!info || !info->filter_fn)
		return NULL;

	if (count != 1)
		return NULL;

	return dma_request_channel(info->dma_cap, info->filter_fn,
			&dma_spec->args[0]);
}
EXPORT_SYMBOL_GPL(of_dma_simple_xlate);
