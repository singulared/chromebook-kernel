/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS5420 - ASV(Adaptive Supply Voltage) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/bitrev.h>

#include <mach/asv-exynos.h>
#include <mach/asv-exynos5420.h>
#include <mach/map.h>
#include <mach/regs-pmu.h>

#include <plat/cpu.h>

#define CHIP_ID3_REG		(S5P_VA_CHIPID + 0x04)
#define EXYNOS5420_HPM_OFFSET	12
#define EXYNOS5420_IDS_OFFSET	24
#define EXYNOS5420_HPM_MASK	0x1F
#define EXYNOS5420_IDS_MASK	0xFF

#define CHIP_ID4_REG		(S5P_VA_CHIPID + 0x1C)
#define EXYNOS5420_TMCB_OFFSET	0
#define EXYNOS5420_TMCB_MASK	0x7F

#define LOT_ID_REG		(S5P_VA_CHIPID + 0x14)

static bool is_special_lot;

static const char * const special_lot_list[] = {
	"NZXK8",
	"NZXKR",
	"NZXT6",
};

void exynos5420_set_abb(struct asv_info *asv_inform)
{
	void __iomem *target_reg;
	unsigned int target_value;

	switch (asv_inform->asv_type) {
	case ID_ARM:
	case ID_KFC:
		target_reg = EXYNOS5420_BB_CON0;
		target_value = arm_asv_abb_info[asv_inform->result_asv_grp];
		break;
	default:
		return;
	}

	set_abb(target_reg, target_value);
}

static unsigned int exynos5420_get_asv_group(struct asv_common *asv_comm,
						enum asv_type_id asv_type)
{
	unsigned int i;
	struct asv_info *target_asv_info = asv_get(asv_type);

	/* If sample is from special lot, must apply ASV group 0 */
	if (is_special_lot)
		return 0;

	for (i = 0; i < target_asv_info->asv_group_nr; i++) {
		if (refer_use_table_get_asv[0][i] &&
			asv_comm->ids_value <= refer_table_get_asv[0][i])
			return i;

		if (refer_use_table_get_asv[1][i] &&
			asv_comm->hpm_value <= refer_table_get_asv[1][i])
			return i;
	}

	return 0;
}

static unsigned int exynos5420_set_asv_info(struct asv_info *asv_inform,
							bool show_value)
{
	unsigned int i;
	unsigned int target_asv_grp_nr = asv_inform->result_asv_grp;
	unsigned int level = asv_inform->dvfs_level_nr;

	exynos5420_set_abb(asv_inform);

	asv_inform->asv_volt = kmalloc((sizeof(struct asv_freq_table) * level),
			GFP_KERNEL);
	if (!asv_inform->asv_volt) {
		pr_err("Set ASV Info: Allocation failed\n");
		goto out1;
	}
	asv_inform->asv_abb = kmalloc((sizeof(struct asv_freq_table) * level),
			GFP_KERNEL);
	if (!asv_inform->asv_abb) {
		pr_err("Set ASV Info: Allocation failed\n");
		goto out2;
	}

	if (asv_inform->asv_type == ID_ARM) {
		for (i = 0; i < level; i++) {
			asv_inform->asv_volt[i].asv_freq =
						arm_asv_volt_info[i][0];
			asv_inform->asv_volt[i].asv_value =
				arm_asv_volt_info[i][target_asv_grp_nr + 1];
		}
	} else if (asv_inform->asv_type == ID_KFC) {
		for (i = 0; i < level; i++) {
			asv_inform->asv_volt[i].asv_freq =
						kfc_asv_volt_info[i][0];
			asv_inform->asv_volt[i].asv_value =
				kfc_asv_volt_info[i][target_asv_grp_nr + 1];
		}
	}

	if (show_value) {
		for (i = 0; i < level; i++)
			pr_debug("%s LV%d freq : %d volt : %d abb : %d\n",
					asv_inform->name, i,
					asv_inform->asv_volt[i].asv_freq,
					asv_inform->asv_volt[i].asv_value,
					asv_inform->asv_abb[i].asv_value);
	}
	return 0;
out2:
	kfree(asv_inform->asv_volt);
out1:
	return -ENOMEM;
}

struct asv_ops exynos5420_asv_ops_arm = {
	.get_asv_group	= exynos5420_get_asv_group,
	.set_asv_info	= exynos5420_set_asv_info,
};

struct asv_ops exynos5420_asv_ops_kfc = {
	.get_asv_group	= exynos5420_get_asv_group,
	.set_asv_info	= exynos5420_set_asv_info,
};

struct asv_info exynos5420_asv_member[] = {
	{
		.asv_type	= ID_ARM,
		.name		= "VDD_ARM",
		.ops		= &exynos5420_asv_ops_arm,
		.asv_group_nr	= ASV_GRP_NR(ARM),
		.dvfs_level_nr	= DVFS_LEVEL_NR(ARM),
		.max_volt_value = MAX_VOLT(ARM),
	}, {
		.asv_type	= ID_KFC,
		.name		= "VDD_KFC",
		.ops		= &exynos5420_asv_ops_kfc,
		.asv_group_nr	= ASV_GRP_NR(KFC),
		.dvfs_level_nr	= DVFS_LEVEL_NR(KFC),
		.max_volt_value = MAX_VOLT(KFC),
	},
};

unsigned int exynos5420_register_asv_member(void)
{
	unsigned int i;

	/* Register asv member into list */
	for (i = 0; i < ARRAY_SIZE(exynos5420_asv_member); i++)
		exynos_add_asv_member(&exynos5420_asv_member[i]);

	return 0;
}

static bool exynos5420_check_lot_id(struct asv_common *asv_info)
{
	unsigned int lid_reg;
	unsigned int rev_lid;
	unsigned int i;
	unsigned int tmp;

	is_special_lot = false;
	lid_reg = __raw_readl(LOT_ID_REG);
	rev_lid = bitrev32(lid_reg);

	asv_info->lot_name[0] = 'N';
	lid_reg = (rev_lid >> 11) & 0x1FFFFF;

	for (i = 4; i >= 1; i--) {
		tmp = lid_reg % 36;
		lid_reg /= 36;
		asv_info->lot_name[i] =
				(tmp < 10) ? (tmp + '0') : ((tmp - 10) + 'A');
	}

	for (i = 0; i < ARRAY_SIZE(special_lot_list); i++) {
		if (!strncmp(asv_info->lot_name, special_lot_list[i],
								LOT_ID_LEN)) {
			is_special_lot = true;
			goto out;
		}
	}
out:
	pr_info("Exynos5420 ASV : Lot ID is %s[%s]\n", asv_info->lot_name,
				is_special_lot ? "Special" : "Non Special");
	return is_special_lot;
}

int exynos5420_init_asv(struct asv_common *asv_info)
{
	unsigned int chip_id3_value;
	unsigned int chip_id4_value;

	pr_debug("EXYNOS5420: Adaptive Supply Voltage init\n");

	is_special_lot = exynos5420_check_lot_id(asv_info);
	if (is_special_lot)
		goto set_asv_info;

	chip_id3_value = __raw_readl(CHIP_ID3_REG);
	chip_id4_value = __raw_readl(CHIP_ID4_REG);
	asv_info->hpm_value = (chip_id4_value >> EXYNOS5420_TMCB_OFFSET) &
							EXYNOS5420_TMCB_MASK;
	asv_info->ids_value = (chip_id3_value >> EXYNOS5420_IDS_OFFSET) &
							EXYNOS5420_IDS_MASK;
	if (!asv_info->hpm_value) {
		is_special_lot = true;
		pr_err("Exynos5420 ASV : invalid IDS value\n");
	}

	pr_debug("EXYNOS5420 ASV : %s IDS : %d HPM : %d\n", asv_info->lot_name,
				asv_info->ids_value, asv_info->hpm_value);

set_asv_info:
	asv_info->register_asv_member = exynos5420_register_asv_member;
	return 0;
}
