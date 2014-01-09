/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS5 - ASV(Adaptive Supply Voltage) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <mach/map.h>
#include <mach/asv-exynos.h>
#include <mach/asv-5250.h>

#include <plat/cpu.h>

static LIST_HEAD(asv_list);
static DEFINE_MUTEX(asv_mutex);

void exynos_add_asv_member(struct asv_info *exynos_asv_info)
{
	mutex_lock(&asv_mutex);
	list_add_tail(&exynos_asv_info->node, &asv_list);
	mutex_unlock(&asv_mutex);
}

struct asv_info *asv_get(enum asv_type_id exynos_asv_type_id)
{
	struct asv_info *match_asv_info;

	list_for_each_entry(match_asv_info, &asv_list, node)
		if (exynos_asv_type_id == match_asv_info->asv_type)
			return match_asv_info;

	return match_asv_info;
}

int exynos_asv_group_get(enum asv_type_id asv_type_id)
{
	int asv_group = -1;

	if (soc_is_exynos5250())
		asv_group = exynos_result_of_asv & 0xf;
	else if (soc_is_exynos542x())
		asv_group = asv_get(asv_type_id)->result_asv_grp;

	return asv_group;
}

unsigned int get_match_volt(enum asv_type_id target_type,
				unsigned int target_freq)
{
	struct asv_info *match_asv_info = asv_get(target_type);
	unsigned int target_dvfs_level;
	unsigned int i;

	if (!match_asv_info) {
		pr_warn("EXYNOS ASV: failed to get_match_volt(type: %d)\n",
								target_type);
		return 0;
	}

	target_dvfs_level = match_asv_info->dvfs_level_nr;

	for (i = 0; i < target_dvfs_level; i++) {
		if (match_asv_info->asv_volt[i].asv_freq == target_freq)
			return match_asv_info->asv_volt[i].asv_value;
	}

	/* If there is no matched freq, return max supplied voltage */
	return match_asv_info->max_volt_value;
}

unsigned int get_match_abb(enum asv_type_id target_type,
				unsigned int target_freq)
{
	struct asv_info *match_asv_info = asv_get(target_type);
	unsigned int target_dvfs_level;
	unsigned int i;

	if (!match_asv_info) {
		pr_warn("EXYNOS ASV: failed to get_match_abb(type: %d)\n",
								target_type);
		return 0;
	}

	target_dvfs_level = match_asv_info->dvfs_level_nr;

	if (!match_asv_info->asv_abb) {
		pr_warn("EXYNOS ASV: request for nonexistent asv type \
						\(type: %d)\n", target_type);
		return 0;
	}

	for (i = 0; i < target_dvfs_level; i++) {
		if (match_asv_info->asv_abb[i].asv_freq == target_freq)
			return match_asv_info->asv_abb[i].asv_value;
	}

	/* If there is no matched freq, return default BB value */
	return ABB_X100;
}

unsigned int set_match_abb(enum asv_type_id target_type,
				unsigned int target_abb)
{
	struct asv_info *match_asv_info = asv_get(target_type);

	if (!match_asv_info) {
		pr_warn("EXYNOS ASV: failed to set_match_abb(type: %d)\n",
								target_type);
		return 0;
	}

	if (!match_asv_info->abb_info) {
		pr_warn("EXYNOS ASV: request for nonexist abb(type: %d)\n",
								target_type);
		return 0;
	}

	match_asv_info->abb_info->target_abb = target_abb;
	match_asv_info->abb_info->set_target_abb(match_asv_info);

	return 0;
}

static unsigned int set_asv_info(struct asv_common *exynos_asv_common,
							bool show_volt)
{
	struct asv_info *exynos_asv_info;
	unsigned int match_grp_nr;
	unsigned int result;

	list_for_each_entry(exynos_asv_info, &asv_list, node) {
		match_grp_nr =
			exynos_asv_info->ops->get_asv_group(exynos_asv_common,
						exynos_asv_info->asv_type);
		exynos_asv_info->result_asv_grp = match_grp_nr;
		pr_info("%s ASV group is %d\n", exynos_asv_info->name,
					exynos_asv_info->result_asv_grp);
		result = exynos_asv_info->ops->set_asv_info
						(exynos_asv_info, show_volt);

		if (result)
			return -ENOMEM;
		/* If need to set abb, call abb set function */
		if (exynos_asv_info->abb_info)
			exynos_asv_info->abb_info->set_target_abb
						(exynos_asv_info);
	}
	return 0;
}

static int __init asv_init(void)
{
	struct asv_common *exynos_asv_common;
	int ret;

	exynos_asv_common = kzalloc(sizeof(struct asv_common), GFP_KERNEL);
	if (!exynos_asv_common) {
		pr_err("ASV : Allocation failed\n");
		ret = -EINVAL;
		goto out1;
	}

	/* Define init function for each SoC types */
	if (soc_is_exynos542x())
		ret = exynos5420_init_asv(exynos_asv_common);
	else {
		pr_err("ASV : Unknown SoC type\n");
		ret = -EINVAL;
		goto out2;
	}

	if (ret) {
		pr_err("ASV : ASV initialize failed\n");
		ret = -EINVAL;
		goto out2;
	}

	/* If it is need to initialize, run init function */
	if (exynos_asv_common->init) {
		if (exynos_asv_common->init()) {
			pr_err("ASV : Cannot run init function\n");
			ret = -EINVAL;
			goto out2;
		}
	}

	/* Register ASV member for each SoC */
	if (exynos_asv_common->register_asv_member) {
		ret = exynos_asv_common->register_asv_member();
	} else {
		pr_err("ASV : There is no register_asv_member function\n");
		ret = -EINVAL;
		goto out2;
	}

	ret = set_asv_info(exynos_asv_common, false);
	if (ret) {
		ret = -EINVAL;
		goto out2;
	}

out2:
	kfree(exynos_asv_common);
out1:
	return ret;
}
arch_initcall_sync(asv_init);
