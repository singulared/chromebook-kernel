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
#include <linux/regulator/consumer.h>

#include <mach/asv-exynos.h>
#include <mach/asv-exynos5420.h>
#include <mach/map.h>
#include <mach/regs-pmu.h>
#include <mach/regs-clock.h>

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

#define G3D_ID_REG		(S5P_VA_CHIPID + 0x28)
#define G3D_SHADERCONFIG_MASK	0x3C00

static bool is_special_lot;

static unsigned int *g3d_asv_abb_info;
static unsigned int (*g3d_asv_volt_info)[ARM_ASV_GRP_NR + 1];
static unsigned int (*g3d_sram_asv_volt_info)[ARM_ASV_GRP_NR + 1];
static struct asv_info *g3d_asv_member;

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
		target_reg = EXYNOS5420_BIAS_CON_ARM;
		target_value = arm_asv_abb_info[asv_inform->result_asv_grp];
		break;
	case ID_KFC:
		target_reg = EXYNOS5420_BIAS_CON_KFC;
		target_value = kfc_asv_abb_info[asv_inform->result_asv_grp];
		break;
	case ID_INT:
		target_reg = EXYNOS5420_BIAS_CON_INT;
		target_value = int_asv_abb_info[asv_inform->result_asv_grp];
		break;
	case ID_G3D:
		target_reg = EXYNOS5420_BIAS_CON_G3D;
		target_value = g3d_asv_abb_info[asv_inform->result_asv_grp];
		break;
	case ID_MIF:
		target_reg = EXYNOS5420_BIAS_CON_MIF;
		target_value = mif_asv_abb_info[asv_inform->result_asv_grp];
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
	unsigned int (*asv_volt_info)[ARM_ASV_GRP_NR + 1];

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

	switch (asv_inform->asv_type) {
	case ID_ARM:
		asv_volt_info = arm_asv_volt_info;
		break;
	case ID_KFC:
		asv_volt_info = kfc_asv_volt_info;
		break;
	case ID_G3D:
		asv_volt_info = g3d_asv_volt_info;
		break;
	case ID_INT:
		asv_volt_info = int_asv_volt_info;
		break;
	case ID_MIF:
		asv_volt_info = mif_asv_volt_info;
		break;
	case ID_MIF_SRAM:
		asv_volt_info = mif_sram_asv_volt_info;
		break;
	case ID_G3D_SRAM:
		asv_volt_info = g3d_sram_asv_volt_info;
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

struct asv_ops exynos5420_asv_ops_arm = {
	.get_asv_group	= exynos5420_get_asv_group,
	.set_asv_info	= exynos5420_set_asv_info,
};

struct asv_ops exynos5420_asv_ops_kfc = {
	.get_asv_group	= exynos5420_get_asv_group,
	.set_asv_info	= exynos5420_set_asv_info,
};

static struct asv_ops exynos5420_asv_ops_g3d = {
	.get_asv_group	= exynos5420_get_asv_group,
	.set_asv_info	= exynos5420_set_asv_info,
};

static struct asv_ops exynos5420_asv_ops_int = {
	.get_asv_group	= exynos5420_get_asv_group,
	.set_asv_info	= exynos5420_set_asv_info,
};

static struct asv_ops exynos5420_asv_ops_mif = {
	.get_asv_group	= exynos5420_get_asv_group,
	.set_asv_info	= exynos5420_set_asv_info,
};

static struct asv_ops exynos5420_asv_ops_mif_sram = {
	.get_asv_group	= exynos5420_get_asv_group,
	.set_asv_info	= exynos5420_set_asv_info,
};

static struct asv_ops exynos5420_asv_ops_g3d_sram = {
	.get_asv_group	= exynos5420_get_asv_group,
	.set_asv_info	= exynos5420_set_asv_info,
};

static struct asv_ops exynos5420_asv_ops_g3d_mp6 = {
	.get_asv_group	= exynos5420_get_asv_group,
	.set_asv_info	= exynos5420_set_asv_info,
};

static struct asv_ops exynos5420_asv_ops_g3d_mp6_sram = {
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
	}, {
		.asv_type	= ID_INT,
		.name		= "VDD_INT",
		.ops		= &exynos5420_asv_ops_int,
		.asv_group_nr	= ASV_GRP_NR(INT),
		.dvfs_level_nr	= DVFS_LEVEL_NR(INT),
		.max_volt_value = MAX_VOLT(INT),
	}, {
		.asv_type	= ID_MIF,
		.name		= "VDD_MIF",
		.ops		= &exynos5420_asv_ops_mif,
		.asv_group_nr	= ASV_GRP_NR(MIF),
		.dvfs_level_nr	= DVFS_LEVEL_NR(MIF),
		.max_volt_value = MAX_VOLT(MIF),
	}, {
		.asv_type	= ID_MIF_SRAM,
		.name		= "VDD_MIF_SRAM",
		.ops		= &exynos5420_asv_ops_mif_sram,
		.asv_group_nr	= ASV_GRP_NR(MIF_SRAM),
		.dvfs_level_nr	= DVFS_LEVEL_NR(MIF_SRAM),
		.max_volt_value = MAX_VOLT(MIF_SRAM),
	},
};

#define NR_G3D_ASV_MEMBER	2

static struct asv_info g3d_mp4_asv_member[NR_G3D_ASV_MEMBER] = {
	{
		.asv_type	= ID_G3D,
		.name		= "VDD_G3D_MP4",
		.ops		= &exynos5420_asv_ops_g3d,
		.asv_group_nr	= ASV_GRP_NR(G3D_MP4),
		.dvfs_level_nr	= DVFS_LEVEL_NR(G3D_MP4),
		.max_volt_value = MAX_VOLT(G3D_MP4),
	}, {
		.asv_type	= ID_G3D_SRAM,
		.name		= "VDD_G3D_MP4_SRAM",
		.ops		= &exynos5420_asv_ops_g3d_sram,
		.asv_group_nr	= ASV_GRP_NR(G3D_MP4_SRAM),
		.dvfs_level_nr	= DVFS_LEVEL_NR(G3D_MP4_SRAM),
		.max_volt_value = MAX_VOLT(G3D_MP4_SRAM),
	},
};

static struct asv_info g3d_mp6_asv_member[NR_G3D_ASV_MEMBER] = {
	{
		.asv_type	= ID_G3D,
		.name		= "VDD_G3D_MP6",
		.ops		= &exynos5420_asv_ops_g3d_mp6,
		.asv_group_nr	= ASV_GRP_NR(G3D_MP6),
		.dvfs_level_nr	= DVFS_LEVEL_NR(G3D_MP6),
		.max_volt_value = MAX_VOLT(G3D_MP6),
	}, {
		.asv_type	= ID_G3D_SRAM,
		.name		= "VDD_G3D_MP6_SRAM",
		.ops		= &exynos5420_asv_ops_g3d_mp6_sram,
		.asv_group_nr	= ASV_GRP_NR(G3D_MP6_SRAM),
		.dvfs_level_nr	= DVFS_LEVEL_NR(G3D_MP6_SRAM),
		.max_volt_value = MAX_VOLT(G3D_MP6_SRAM),
	},
};

unsigned int exynos5420_register_asv_member(void)
{
	unsigned int i;

	/* Register asv member into list */
	for (i = 0; i < ARRAY_SIZE(exynos5420_asv_member); i++)
		exynos_add_asv_member(&exynos5420_asv_member[i]);

	for (i = 0; i < NR_G3D_ASV_MEMBER; i++)
		exynos_add_asv_member(&g3d_asv_member[i]);

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

static int __init exynos5420_set_asv_volt_mif_sram(void)
{
	unsigned int mif_volt;
	unsigned int mif_sram_volt;
	unsigned int g3d_sram_volt;
	struct regulator *mif_regulator;
	struct regulator *mif_sram_regulator;
	struct regulator *g3d_sram_regulator;

	if (!(soc_is_exynos5420()))
		return 0;

	mif_regulator = regulator_get(NULL, "vdd_mif");
	mif_sram_regulator = regulator_get(NULL, "vdd_mifs");
	g3d_sram_regulator = regulator_get(NULL, "vdd_g3ds");

	/* Set the voltages based on the ASV group */
	mif_volt = get_match_volt(ID_MIF, 0);
	mif_sram_volt = get_match_volt(ID_MIF_SRAM, 0);
	g3d_sram_volt = get_match_volt(ID_G3D_SRAM, 0);

	pr_info("MIF, MIF_SRAM, G3D_SRAM ASV is %d, %d, %d\n",
			mif_volt, mif_sram_volt, g3d_sram_volt);
	if (!IS_ERR(mif_regulator))
		regulator_set_voltage(mif_regulator, mif_volt, mif_volt);
	else {
		pr_err("Regulator get error : mif\n");
		goto err_mif;
	}

	if (!IS_ERR(mif_sram_regulator))
		regulator_set_voltage(mif_sram_regulator, mif_sram_volt,
							mif_sram_volt);
	else {
		pr_err("Regulator get error : mif_sram\n");
		goto err_mif_sram;
	}

	if (!IS_ERR(g3d_sram_regulator))
		regulator_set_voltage(g3d_sram_regulator, g3d_sram_volt,
							g3d_sram_volt);
	else {
		pr_err("Regulator get error : g3d_sram\n");
		goto err_g3d_sram;
	}

	regulator_put(g3d_sram_regulator);
err_g3d_sram:
	regulator_put(mif_sram_regulator);
err_mif_sram:
	regulator_put(mif_regulator);
err_mif:
	return 0;
}
late_initcall(exynos5420_set_asv_volt_mif_sram);

static void exynos5420_set_ema(void)
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

int exynos5420_init_asv(struct asv_common *asv_info)
{
	unsigned int chip_id3_value;
	unsigned int chip_id4_value;
	bool is_g3d_mp6;

	pr_debug("EXYNOS5420: Adaptive Supply Voltage init\n");

	exynos5420_set_ema();
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

	/* MP6 variant has different ASV tables */
	is_g3d_mp6 = !(__raw_readl(G3D_ID_REG) & G3D_SHADERCONFIG_MASK);
	if (is_g3d_mp6) {
		g3d_asv_abb_info = g3d_mp6_asv_abb_info;
		g3d_asv_volt_info = g3d_mp6_asv_volt_info;
		g3d_sram_asv_volt_info = g3d_mp6_sram_asv_volt_info;
		g3d_asv_member = g3d_mp6_asv_member;
	} else {
		g3d_asv_abb_info = g3d_mp4_asv_abb_info;
		g3d_asv_volt_info = g3d_mp4_asv_volt_info;
		g3d_sram_asv_volt_info = g3d_mp4_sram_asv_volt_info;
		g3d_asv_member = g3d_mp4_asv_member;
	}

set_asv_info:
	asv_info->register_asv_member = exynos5420_register_asv_member;
	return 0;
}
