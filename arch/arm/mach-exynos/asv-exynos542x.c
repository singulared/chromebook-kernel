/*
 * Copyright (c) 2013-2014 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS542X - ASV(Adaptive Supply Voltage) driver
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
#include <linux/regulator/consumer.h>

#include <mach/asv-exynos.h>
#include <mach/asv-exynos542x.h>
#include <mach/map.h>
#include <mach/regs-pmu.h>
#include <mach/regs-clock.h>

#include <plat/cpu.h>

#define CHIP_ID3_REG		(S5P_VA_CHIPID + 0x04)
#define EXYNOS5420_IDS_OFFSET	24
#define EXYNOS5420_IDS_MASK	0xFF

#define CHIP_ID4_REG		(S5P_VA_CHIPID + 0x1C)
#define EXYNOS5420_TMCB_OFFSET	0
#define EXYNOS5420_TMCB_MASK	0x7F

#define LOT_ID_REG		(S5P_VA_CHIPID + 0x14)

#define G3D_ID_REG		(S5P_VA_CHIPID + 0x28)
#define G3D_SHADERCONFIG_MASK	0x3C00

static bool is_special_lot;

static const unsigned int *exynos542x_arm_asv_abb_info;
static const unsigned int *exynos542x_kfc_asv_abb_info;
static const unsigned int *exynos542x_int_asv_abb_info;
static const unsigned int *exynos542x_g3d_asv_abb_info;
static const unsigned int *exynos542x_mif_asv_abb_info;

static const unsigned int
(*exynos542x_refer_use_table_get_asv)[EXYNOS542X_MAX_ASV_GRP_NR];
static const unsigned int
(*exynos542x_refer_table_get_asv)[EXYNOS542X_MAX_ASV_GRP_NR];

static const unsigned int
(*exynos542x_arm_asv_volt_info)[EXYNOS542X_MAX_ASV_GRP_NR + 1];
static const unsigned int
(*exynos542x_kfc_asv_volt_info)[EXYNOS542X_MAX_ASV_GRP_NR + 1];
static const unsigned int
(*exynos542x_int_asv_volt_info)[EXYNOS542X_MAX_ASV_GRP_NR + 1];
static const unsigned int
(*exynos542x_g3d_asv_volt_info)[EXYNOS542X_MAX_ASV_GRP_NR + 1];
static const unsigned int
(*exynos542x_g3d_sram_asv_volt_info)[EXYNOS542X_MAX_ASV_GRP_NR + 1];
static const unsigned int
(*exynos542x_mif_asv_volt_info)[EXYNOS542X_MAX_ASV_GRP_NR + 1];
static const unsigned int
(*exynos542x_mif_sram_asv_volt_info)[EXYNOS542X_MAX_ASV_GRP_NR + 1];

static struct asv_info *exynos5420_g3d_asv_member;

static const char * const special_lot_list[] = {
	"NZXK8",
	"NZXKR",
	"NZXT6",
};

void exynos542x_set_abb(struct asv_info *asv_inform)
{
	void __iomem *target_reg;
	unsigned int target_value;

	switch (asv_inform->asv_type) {
	case ID_ARM:
		target_reg = EXYNOS5420_BIAS_CON_ARM;
		target_value =
			exynos542x_arm_asv_abb_info[asv_inform->result_asv_grp];
		break;
	case ID_KFC:
		target_reg = EXYNOS5420_BIAS_CON_KFC;
		target_value =
			exynos542x_kfc_asv_abb_info[asv_inform->result_asv_grp];
		break;
	case ID_INT:
		target_reg = EXYNOS5420_BIAS_CON_INT;
		target_value =
			exynos542x_int_asv_abb_info[asv_inform->result_asv_grp];
		break;
	case ID_G3D:
		target_reg = EXYNOS5420_BIAS_CON_G3D;
		target_value =
			exynos542x_g3d_asv_abb_info[asv_inform->result_asv_grp];
		break;
	case ID_MIF:
		target_reg = EXYNOS5420_BIAS_CON_MIF;
		target_value =
			exynos542x_mif_asv_abb_info[asv_inform->result_asv_grp];
		break;
	default:
		return;
	}

	set_abb(target_reg, target_value);
}

static struct abb_common exynos542x_abb_common = {
	.set_target_abb = exynos542x_set_abb,
};

static unsigned int exynos542x_get_asv_group(struct asv_common *asv_comm,
						enum asv_type_id asv_type)
{
	unsigned int i;
	struct asv_info *target_asv_info = asv_get(asv_type);
	const unsigned int (*refer_use_table_get_asv)
				[EXYNOS542X_MAX_ASV_GRP_NR];
	const unsigned int (*refer_table_get_asv)
				[EXYNOS542X_MAX_ASV_GRP_NR];

	/* If sample is from special lot, must apply ASV group 0 */
	if (is_special_lot)
		return 0;

	refer_use_table_get_asv = exynos542x_refer_use_table_get_asv;
	refer_table_get_asv = exynos542x_refer_table_get_asv;

	for (i = 0; i < target_asv_info->asv_group_nr; i++) {
		if (refer_use_table_get_asv[0][i] &&
			asv_comm->ids_value <= refer_table_get_asv[0][i])
			return i;

		if (refer_use_table_get_asv[1][i] &&
			asv_comm->hpm_value <= refer_table_get_asv[1][i])
			return i;
	}
	pr_warn("No matching ASV group for IDS:%d HPM:%d\n",
			asv_comm->ids_value, asv_comm->hpm_value);
	return 0;
}

static unsigned int exynos542x_set_asv_info(struct asv_info *asv_inform,
							bool show_value)
{
	unsigned int i;
	unsigned int target_asv_grp_nr = asv_inform->result_asv_grp;
	unsigned int level = asv_inform->dvfs_level_nr;
	const unsigned int (*asv_volt_info)[EXYNOS542X_MAX_ASV_GRP_NR + 1];

	asv_inform->asv_volt = kmalloc((sizeof(struct asv_freq_table) * level),
			GFP_KERNEL);
	if (!asv_inform->asv_volt) {
		pr_err("Set ASV Info: Allocation failed\n");
		goto out1;
	}
	/*
	 * TODO(a.kesavan) As there are no users of get/set match_abb yet,
	 * we are not populating asv_abb. get_match_abb would be used when
	 * we have dynamic ABB that varies with frequency.
	 */
	asv_inform->asv_abb = kmalloc((sizeof(struct asv_freq_table) * level),
			GFP_KERNEL);
	if (!asv_inform->asv_abb) {
		pr_err("Set ASV Info: Allocation failed\n");
		goto out2;
	}

	switch (asv_inform->asv_type) {
	case ID_ARM:
		asv_volt_info = exynos542x_arm_asv_volt_info;
		break;
	case ID_KFC:
		asv_volt_info = exynos542x_kfc_asv_volt_info;
		break;
	case ID_G3D:
		asv_volt_info = exynos542x_g3d_asv_volt_info;
		break;
	case ID_INT:
		asv_volt_info = exynos542x_int_asv_volt_info;
		break;
	case ID_MIF:
		asv_volt_info = exynos542x_mif_asv_volt_info;
		break;
	case ID_MIF_SRAM:
		asv_volt_info = exynos542x_mif_sram_asv_volt_info;
		break;
	case ID_G3D_SRAM:
		asv_volt_info = exynos542x_g3d_sram_asv_volt_info;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < level; i++) {
		asv_inform->asv_volt[i].asv_freq =
					asv_volt_info[i][0];
		asv_inform->asv_volt[i].asv_value =
					asv_volt_info[i][target_asv_grp_nr + 1];
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

static const struct asv_ops exynos542x_asv_ops = {
	.get_asv_group	= exynos542x_get_asv_group,
	.set_asv_info	= exynos542x_set_asv_info,
};

struct asv_info exynos5420_asv_member[] = {
	{
		.asv_type	= ID_ARM,
		.name		= "VDD_ARM",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5420_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5420_ARM),
		.max_volt_value = MAX_VOLT(EXYNOS5420_ARM),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_KFC,
		.name		= "VDD_KFC",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5420_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5420_KFC),
		.max_volt_value = MAX_VOLT(EXYNOS5420_KFC),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_INT,
		.name		= "VDD_INT",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5420_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5420_INT),
		.max_volt_value = MAX_VOLT(EXYNOS5420_INT),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_MIF,
		.name		= "VDD_MIF",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5420_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5420_MIF),
		.max_volt_value = MAX_VOLT(EXYNOS5420_MIF),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_MIF_SRAM,
		.name		= "VDD_MIF_SRAM",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5420_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5420_MIF_SRAM),
		.max_volt_value = MAX_VOLT(EXYNOS5420_MIF_SRAM),
	},
};

#define NR_G3D_ASV_MEMBER	2

static struct
asv_info exynos5420_g3d_mp4_asv_member[NR_G3D_ASV_MEMBER] = {
	{
		.asv_type	= ID_G3D,
		.name		= "VDD_G3D_MP4",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5420_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5420_G3D_MP4),
		.max_volt_value = MAX_VOLT(EXYNOS5420_G3D_MP4),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_G3D_SRAM,
		.name		= "VDD_G3D_MP4_SRAM",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5420_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5420_G3D_MP4_SRAM),
		.max_volt_value = MAX_VOLT(EXYNOS5420_G3D_MP4_SRAM),
	},
};

static struct
asv_info exynos5420_g3d_mp6_asv_member[NR_G3D_ASV_MEMBER] = {
	{
		.asv_type	= ID_G3D,
		.name		= "VDD_G3D_MP6",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5420_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5420_G3D_MP6),
		.max_volt_value = MAX_VOLT(EXYNOS5420_G3D_MP6),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_G3D_SRAM,
		.name		= "VDD_G3D_MP6_SRAM",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5420_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5420_G3D_MP6_SRAM),
		.max_volt_value = MAX_VOLT(EXYNOS5420_G3D_MP6_SRAM),
	},
};

struct asv_info exynos5422_asv_member[] = {
	{
		.asv_type	= ID_ARM,
		.name		= "VDD_ARM",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5422_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5422_ARM),
		.max_volt_value = MAX_VOLT(EXYNOS5422_ARM),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_KFC,
		.name		= "VDD_KFC",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5422_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5422_KFC),
		.max_volt_value = MAX_VOLT(EXYNOS5422_KFC),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_INT,
		.name		= "VDD_INT",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5422_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5422_INT),
		.max_volt_value = MAX_VOLT(EXYNOS5422_INT),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_MIF,
		.name		= "VDD_MIF",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5422_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5422_MIF),
		.max_volt_value = MAX_VOLT(EXYNOS5422_MIF),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_MIF_SRAM,
		.name		= "VDD_MIF_SRAM",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5422_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5422_MIF_SRAM),
		.max_volt_value = MAX_VOLT(EXYNOS5422_MIF_SRAM),
	}, {
		.asv_type	= ID_G3D,
		.name		= "VDD_G3D",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5422_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5422_G3D),
		.max_volt_value = MAX_VOLT(EXYNOS5422_G3D),
		.abb_info	= &exynos542x_abb_common,
	}, {
		.asv_type	= ID_G3D_SRAM,
		.name		= "VDD_G3D_SRAM",
		.ops		= &exynos542x_asv_ops,
		.asv_group_nr	= ASV_GRP_NR(EXYNOS5422_MAX),
		.dvfs_level_nr	= DVFS_LEVEL_NR(EXYNOS5422_G3D_SRAM),
		.max_volt_value = MAX_VOLT(EXYNOS5422_G3D_SRAM),
	},
};

unsigned int exynos542x_register_asv_member(void)
{
	unsigned int i;

	/* Register asv member into list */
	if (soc_is_exynos5420()) {
		for (i = 0; i < ARRAY_SIZE(exynos5420_asv_member); i++)
			exynos_add_asv_member(&exynos5420_asv_member[i]);

		for (i = 0; i < NR_G3D_ASV_MEMBER; i++)
			exynos_add_asv_member(&exynos5420_g3d_asv_member[i]);
	} else {
		for (i = 0; i < ARRAY_SIZE(exynos5422_asv_member); i++)
			exynos_add_asv_member(&exynos5422_asv_member[i]);
	}
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

static int __init exynos542x_set_asv_volt_g3d_sram(void)
{
	unsigned int g3d_sram_volt;
	struct regulator *g3d_sram_regulator;

	if (!(soc_is_exynos542x()))
		return 0;

	g3d_sram_regulator = regulator_get(NULL, "vdd_g3ds");

	/* Set the voltages based on the ASV group */
	g3d_sram_volt = get_match_volt(ID_G3D_SRAM, 0);

	pr_info("G3D_SRAM ASV is %d\n", g3d_sram_volt);

	if (!IS_ERR(g3d_sram_regulator)) {
		regulator_set_voltage(g3d_sram_regulator, g3d_sram_volt,
							g3d_sram_volt);
		regulator_put(g3d_sram_regulator);
	} else {
		pr_err("Regulator get error : g3d_sram\n");
	}

	return 0;
}
late_initcall(exynos542x_set_asv_volt_g3d_sram);

static void exynos542x_set_ema(void)
{
	unsigned int ema_val;

	/* iSRAM EMA Setting */
	ema_val = __raw_readl(EXYNOS5420_EMA_CON0);
	ema_val = (ema_val & ~(0x7 << 21)) | (0x5 << 21);
	__raw_writel(ema_val, EXYNOS5420_EMA_CON0);

	/* ARM EMA - Enable WAS */
	ema_val = __raw_readl(EXYNOS5420_ARM_EMA_CTRL);
	ema_val |= EXYNOS5420_ARM_WAS_ENABLE;
	__raw_writel(ema_val, EXYNOS5420_ARM_EMA_CTRL);

	/* G3D EMA Setting */
	ema_val = __raw_readl(EXYNOS5420_EMA_CON1);
	ema_val = (ema_val & ~(0xfff << 9)) | (0x5 << 18) | (0x5 << 15) |
						(0x5 << 12) | (0x5 << 9);
	__raw_writel(ema_val, EXYNOS5420_EMA_CON1);
}

bool exynos5420_is_g3d_mp6(void)
{
	return !(__raw_readl(G3D_ID_REG) & G3D_SHADERCONFIG_MASK);
}

int exynos542x_init_asv(struct asv_common *asv_info)
{
	unsigned int chip_id3_value;
	unsigned int chip_id4_value;

	pr_debug("EXYNOS542X: Adaptive Supply Voltage init\n");

	exynos542x_set_ema();

	if (soc_is_exynos5420()) {
		is_special_lot = exynos5420_check_lot_id(asv_info);
		if (is_special_lot)
			goto set_asv_info;
	}

	chip_id3_value = __raw_readl(CHIP_ID3_REG);
	chip_id4_value = __raw_readl(CHIP_ID4_REG);
	asv_info->hpm_value = (chip_id4_value >> EXYNOS5420_TMCB_OFFSET) &
							EXYNOS5420_TMCB_MASK;
	asv_info->ids_value = (chip_id3_value >> EXYNOS5420_IDS_OFFSET) &
							EXYNOS5420_IDS_MASK;
	if (!asv_info->hpm_value) {
		is_special_lot = true;
		pr_err("Exynos542X ASV : invalid IDS value\n");
	}

	pr_debug("EXYNOS542X ASV : %s IDS : %d HPM : %d\n", asv_info->lot_name,
				asv_info->ids_value, asv_info->hpm_value);


	if (soc_is_exynos5420()) {
		exynos542x_arm_asv_abb_info = exynos5420_arm_asv_abb_info;
		exynos542x_kfc_asv_abb_info = exynos5420_kfc_asv_abb_info;
		exynos542x_int_asv_abb_info = exynos5420_int_asv_abb_info;

		exynos542x_mif_asv_abb_info = exynos5420_mif_asv_abb_info;

		exynos542x_arm_asv_volt_info = exynos5420_arm_asv_volt_info;
		exynos542x_kfc_asv_volt_info = exynos5420_kfc_asv_volt_info;
		exynos542x_int_asv_volt_info = exynos5420_int_asv_volt_info;
		exynos542x_mif_asv_volt_info = exynos5420_mif_asv_volt_info;
		exynos542x_mif_sram_asv_volt_info =
			exynos5420_mif_sram_asv_volt_info;

		exynos542x_refer_use_table_get_asv =
			exynos5420_refer_use_table_get_asv;
		exynos542x_refer_table_get_asv =
			exynos5420_refer_table_get_asv;

		/* MP6 variant has different ASV tables */
		if (exynos5420_is_g3d_mp6()) {
			exynos542x_g3d_asv_abb_info =
				exynos5420_g3d_mp6_asv_abb_info;
			exynos542x_g3d_asv_volt_info =
				exynos5420_g3d_mp6_asv_volt_info;
			exynos542x_g3d_sram_asv_volt_info =
				exynos5420_g3d_mp6_sram_asv_volt_info;
			exynos5420_g3d_asv_member =
				exynos5420_g3d_mp6_asv_member;
		} else {
			exynos542x_g3d_asv_abb_info =
				exynos5420_g3d_mp4_asv_abb_info;
			exynos542x_g3d_asv_volt_info =
				exynos5420_g3d_mp4_asv_volt_info;
			exynos542x_g3d_sram_asv_volt_info =
				exynos5420_g3d_mp4_sram_asv_volt_info;
			exynos5420_g3d_asv_member =
				exynos5420_g3d_mp4_asv_member;
		}
	} else {
		exynos542x_arm_asv_abb_info = exynos5422_arm_asv_abb_info;
		exynos542x_kfc_asv_abb_info = exynos5422_kfc_asv_abb_info;
		exynos542x_int_asv_abb_info = exynos5422_int_asv_abb_info;
		exynos542x_g3d_asv_abb_info = exynos5422_g3d_asv_abb_info;
		exynos542x_mif_asv_abb_info = exynos5422_mif_asv_abb_info;

		exynos542x_arm_asv_volt_info = exynos5422_arm_asv_volt_info;
		exynos542x_kfc_asv_volt_info = exynos5422_kfc_asv_volt_info;
		exynos542x_int_asv_volt_info = exynos5422_int_asv_volt_info;
		exynos542x_g3d_asv_volt_info = exynos5422_g3d_asv_volt_info;
		exynos542x_mif_asv_volt_info = exynos5422_mif_asv_volt_info;
		exynos542x_g3d_sram_asv_volt_info =
			exynos5422_g3d_sram_asv_volt_info;
		exynos542x_mif_sram_asv_volt_info =
			exynos5422_mif_sram_asv_volt_info;

		exynos542x_refer_use_table_get_asv =
			exynos5422_refer_use_table_get_asv;
		exynos542x_refer_table_get_asv =
			exynos5422_refer_table_get_asv;
	}
set_asv_info:
	asv_info->register_asv_member = exynos542x_register_asv_member;
	return 0;
}
