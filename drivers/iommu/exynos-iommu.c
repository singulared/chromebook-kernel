/* linux/drivers/iommu/exynos-iommu.c
 *
 * Copyright (c) 2011-2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_EXYNOS_IOMMU_DEBUG
#define DEBUG
#endif

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/iommu.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/memblock.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <asm/cacheflush.h>

/* We does not consider super section mapping (16MB) */
#define SECT_ORDER 20
#define LPAGE_ORDER 16
#define SPAGE_ORDER 12

#define SECT_SIZE (1 << SECT_ORDER)
#define LPAGE_SIZE (1 << LPAGE_ORDER)
#define SPAGE_SIZE (1 << SPAGE_ORDER)

#define SECT_MASK (~(SECT_SIZE - 1))
#define LPAGE_MASK (~(LPAGE_SIZE - 1))
#define SPAGE_MASK (~(SPAGE_SIZE - 1))

#define lv1ent_fault(sent) (((*(sent) & 3) == 0) || ((*(sent) & 3) == 3))
#define lv1ent_page(sent) ((*(sent) & 3) == 1)
#define lv1ent_section(sent) ((*(sent) & 3) == 2)

#define lv2ent_fault(pent) ((*(pent) & 3) == 0)
#define lv2ent_small(pent) ((*(pent) & 2) == 2)
#define lv2ent_large(pent) ((*(pent) & 3) == 1)

#define section_phys(sent) (*(sent) & SECT_MASK)
#define section_offs(iova) ((iova) & 0xFFFFF)
#define lpage_phys(pent) (*(pent) & LPAGE_MASK)
#define lpage_offs(iova) ((iova) & 0xFFFF)
#define spage_phys(pent) (*(pent) & SPAGE_MASK)
#define spage_offs(iova) ((iova) & 0xFFF)

#define lv1ent_offset(iova) ((iova) >> SECT_ORDER)
#define lv2ent_offset(iova) (((iova) & 0xFF000) >> SPAGE_ORDER)

#define NUM_LV1ENTRIES 4096
#define NUM_LV2ENTRIES 256

#define LV2TABLE_SIZE (NUM_LV2ENTRIES * sizeof(long))

#define SPAGES_PER_LPAGE (LPAGE_SIZE / SPAGE_SIZE)

#define lv2table_base(sent) (*(sent) & 0xFFFFFC00)

#define mk_lv1ent_sect(pa) ((pa) | 2)
#define mk_lv1ent_page(pa) ((pa) | 1)
#define mk_lv2ent_lpage(pa) ((pa) | 1)
#define mk_lv2ent_spage(pa) ((pa) | 2)

#define CTRL_ENABLE	0x5
#define CTRL_BLOCK	0x7
#define CTRL_DISABLE	0x0

#define REG_MMU_CTRL		0x000
#define REG_MMU_CFG		0x004
#define REG_MMU_STATUS		0x008
#define REG_MMU_FLUSH		0x00C
#define REG_MMU_FLUSH_ENTRY	0x010
#define REG_PT_BASE_ADDR	0x014
#define REG_INT_STATUS		0x018
#define REG_INT_CLEAR		0x01C

#define REG_PAGE_FAULT_ADDR	0x024
#define REG_AW_FAULT_ADDR	0x028
#define REG_AR_FAULT_ADDR	0x02C
#define REG_DEFAULT_SLAVE_ADDR	0x030

#define REG_MMU_VERSION		0x034

#define REG_PB0_SADDR		0x04C
#define REG_PB0_EADDR		0x050
#define REG_PB1_SADDR		0x054
#define REG_PB1_EADDR		0x058

static void *sysmmu_placeholder; /* Inidcate if a device is System MMU */

#define is_sysmmu(sysmmu) (sysmmu->archdata.iommu == &sysmmu_placeholder)
#define has_sysmmu(dev)							\
	(dev->parent && dev->archdata.iommu && is_sysmmu(dev->parent))
#define for_each_sysmmu(dev, sysmmu)					\
	for (sysmmu = dev->parent; sysmmu && is_sysmmu(sysmmu);		\
			sysmmu = sysmmu->parent)
#define for_each_sysmmu_until(dev, sysmmu, until)			\
	for (sysmmu = dev->parent; sysmmu != until; sysmmu = sysmmu->parent)

static struct kmem_cache *lv2table_kmem_cache;

static unsigned long *section_entry(unsigned long *pgtable, unsigned long iova)
{
	return pgtable + lv1ent_offset(iova);
}

static unsigned long *page_entry(unsigned long *sent, unsigned long iova)
{
	return (unsigned long *)__va(lv2table_base(sent)) + lv2ent_offset(iova);
}

enum exynos_sysmmu_inttype {
	SYSMMU_PAGEFAULT,
	SYSMMU_AR_MULTIHIT,
	SYSMMU_AW_MULTIHIT,
	SYSMMU_BUSERROR,
	SYSMMU_AR_SECURITY,
	SYSMMU_AR_ACCESS,
	SYSMMU_AW_SECURITY,
	SYSMMU_AW_PROTECTION, /* 7 */
	SYSMMU_FAULT_UNKNOWN,
	SYSMMU_FAULTS_NUM
};

static unsigned short fault_reg_offset[SYSMMU_FAULTS_NUM] = {
	REG_PAGE_FAULT_ADDR,
	REG_AR_FAULT_ADDR,
	REG_AW_FAULT_ADDR,
	REG_DEFAULT_SLAVE_ADDR,
	REG_AR_FAULT_ADDR,
	REG_AR_FAULT_ADDR,
	REG_AW_FAULT_ADDR,
	REG_AW_FAULT_ADDR
};

static char *sysmmu_fault_name[SYSMMU_FAULTS_NUM] = {
	"PAGE FAULT",
	"AR MULTI-HIT FAULT",
	"AW MULTI-HIT FAULT",
	"BUS ERROR",
	"AR SECURITY PROTECTION FAULT",
	"AR ACCESS PROTECTION FAULT",
	"AW SECURITY PROTECTION FAULT",
	"AW ACCESS PROTECTION FAULT",
	"UNKNOWN FAULT"
};

struct exynos_iommu_domain {
	struct list_head clients; /* list of sysmmu_drvdata.node */
	unsigned long *pgtable; /* lv1 page table, 16KB */
	short *lv2entcnt; /* free lv2 entry counter for each section */
	spinlock_t lock; /* lock for this structure */
	spinlock_t pgtablelock; /* lock for modifying page table @ pgtable */
};

/* exynos_iommu_owner
 * Metadata attached to the owner of a group of System MMUs that belong
 * to the same owner device.
 */
struct exynos_iommu_owner {
	struct list_head client; /* entry of exynos_iommu_domain.clients */
	struct device *dev;
	spinlock_t lock;	/* Lock to preserve consistency of System MMU */
};

struct sysmmu_drvdata {
	struct device *sysmmu;	/* System MMU's device descriptor */
	struct device *master;	/* Client device that needs System MMU */
	int nsfrs;
	struct clk *clk;
	int activations;
	spinlock_t lock;
	struct iommu_domain *domain;
	unsigned long pgtable;
	void __iomem *sfrbases[0];
};

static bool set_sysmmu_active(struct sysmmu_drvdata *data)
{
	/* return true if the System MMU was not active previously
	   and it needs to be initialized */
	return ++data->activations == 1;
}

static bool set_sysmmu_inactive(struct sysmmu_drvdata *data)
{
	/* return true if the System MMU is needed to be disabled */
	BUG_ON(data->activations < 1);
	return --data->activations == 0;
}

static bool is_sysmmu_active(struct sysmmu_drvdata *data)
{
	return data->activations > 0;
}

static void sysmmu_unblock(void __iomem *sfrbase)
{
	__raw_writel(CTRL_ENABLE, sfrbase + REG_MMU_CTRL);
}

static bool sysmmu_block(void __iomem *sfrbase)
{
	int i = 120;

	__raw_writel(CTRL_BLOCK, sfrbase + REG_MMU_CTRL);
	while ((i > 0) && !(__raw_readl(sfrbase + REG_MMU_STATUS) & 1))
		--i;

	if (!(__raw_readl(sfrbase + REG_MMU_STATUS) & 1)) {
		sysmmu_unblock(sfrbase);
		return false;
	}

	return true;
}

static void __sysmmu_tlb_invalidate(void __iomem *sfrbase)
{
	__raw_writel(0x1, sfrbase + REG_MMU_FLUSH);
}

static void __sysmmu_tlb_invalidate_entry(void __iomem *sfrbase,
						unsigned long iova)
{
	__raw_writel((iova & SPAGE_MASK) | 1, sfrbase + REG_MMU_FLUSH_ENTRY);
}

static void __sysmmu_set_ptbase(void __iomem *sfrbase,
				       unsigned long pgd)
{
	__raw_writel(0x1, sfrbase + REG_MMU_CFG); /* 16KB LV1, LRU */
	__raw_writel(pgd, sfrbase + REG_PT_BASE_ADDR);

	__sysmmu_tlb_invalidate(sfrbase);
}

static void __sysmmu_set_prefbuf(void __iomem *sfrbase, unsigned long base,
						unsigned long size, int idx)
{
	__raw_writel(base, sfrbase + REG_PB0_SADDR + idx * 8);
	__raw_writel(size - 1 + base,  sfrbase + REG_PB0_EADDR + idx * 8);
}

void exynos_sysmmu_set_prefbuf(struct device *dev,
				unsigned long base0, unsigned long size0,
				unsigned long base1, unsigned long size1)
{
	struct device *sysmmu;

	for_each_sysmmu(dev, sysmmu) {
		int i;
		unsigned long flags;
		struct sysmmu_drvdata *data = dev_get_drvdata(sysmmu);

		BUG_ON((base0 + size0) <= base0);
		BUG_ON((size1 > 0) && ((base1 + size1) <= base1));

		spin_lock_irqsave(&data->lock, flags);
		if (!is_sysmmu_active(data)) {
			spin_unlock_irqrestore(&data->lock, flags);
			continue;
		}

		for (i = 0; i < data->nsfrs; i++) {
			if ((readl(data->sfrbases[i] + REG_MMU_VERSION) >> 28)
									== 3) {
				if (!sysmmu_block(data->sfrbases[i]))
					continue;

				if (size1 == 0) {
					if (size0 <= SZ_128K) {
						base1 = base0;
						size1 = size0;
					} else {
						size1 = size0 -
						ALIGN(size0 / 2, SZ_64K);
						size0 = size0 - size1;
						base1 = base0 + size0;
					}
				}

				__sysmmu_set_prefbuf(
					data->sfrbases[i], base0, size0, 0);
				__sysmmu_set_prefbuf(
					data->sfrbases[i], base1, size1, 1);

				sysmmu_unblock(data->sfrbases[i]);
			}
		}
		spin_unlock_irqrestore(&data->lock, flags);
	}
}

static void __show_fault_information(unsigned long *pgtable, unsigned long iova,
				     int flags)
{
	unsigned long *ent;

	pr_err("%s occurred at 0x%lx(Page table base: 0x%lx)\n",
			sysmmu_fault_name[flags], iova, __pa(pgtable));

	ent = section_entry(pgtable, iova);
	pr_err("\tLv1 entry: 0x%lx\n", *ent);

	if (lv1ent_page(ent)) {
		ent = page_entry(ent, iova);
		pr_err("\t Lv2 entry: 0x%lx\n", *ent);
	}

	pr_err("Generating Kernel OOPS... because it is unrecoverable.\n");
}

static irqreturn_t exynos_sysmmu_irq(int irq, void *dev_id)
{
	/* SYSMMU is in blocked when interrupt occurred. */
	struct sysmmu_drvdata *data = dev_id;
	struct resource *irqres;
	struct platform_device *pdev;
	enum exynos_sysmmu_inttype itype = SYSMMU_FAULT_UNKNOWN;
	unsigned long addr = -1;

	int i, ret = -ENOSYS;

	spin_lock(&data->lock);

	WARN_ON(!is_sysmmu_active(data));

	pdev = to_platform_device(data->sysmmu);
	for (i = 0; i < (pdev->num_resources / 2); i++) {
		irqres = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		if (irqres && ((int)irqres->start == irq))
			break;
	}

	if (i < pdev->num_resources) {
		itype = (enum exynos_sysmmu_inttype)
			__ffs(__raw_readl(data->sfrbases[i] + REG_INT_STATUS));
		if (WARN_ON(!((itype >= 0) && (itype < SYSMMU_FAULT_UNKNOWN))))
			itype = SYSMMU_FAULT_UNKNOWN;
		else
			addr = __raw_readl(
				data->sfrbases[i] + fault_reg_offset[itype]);
	}

	if (data->domain)
		ret = report_iommu_fault(data->domain,
					data->master, addr, itype);
	else
		__show_fault_information(__va(data->pgtable), itype, addr);

	if (ret == -ENOSYS)
		pr_err("NO SYSTEM MMU FAULT HANDLER REGISTERED FOR %s\n",
						dev_name(data->master));
	else if (ret < 0)
		pr_err("SYSTEM MMU FAULT HANDLER FOR %s RETURNED ERROR, %d\n",
						dev_name(data->master), ret);
	else if (itype != SYSMMU_FAULT_UNKNOWN)
		__raw_writel(1 << itype, data->sfrbases[i] + REG_INT_CLEAR);
	else
		ret = -ENOSYS;

	if (ret)
		BUG();

	sysmmu_unblock(data->sfrbases[i]);

	spin_unlock(&data->lock);

	return IRQ_HANDLED;
}

static void __sysmmu_disable_nocount(struct sysmmu_drvdata *data)
{
	int i;

	for (i = 0; i < data->nsfrs; i++)
		__raw_writel(CTRL_DISABLE,
			data->sfrbases[i] + REG_MMU_CTRL);

	clk_disable(data->clk);
}

static bool __sysmmu_disable(struct sysmmu_drvdata *data)
{
	bool disabled;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	disabled = set_sysmmu_inactive(data);

	if (disabled) {
		data->pgtable = 0;
		data->domain = NULL;

		__sysmmu_disable_nocount(data);

		dev_dbg(data->sysmmu, "Disabled\n");
	} else  {
		dev_dbg(data->sysmmu, "%d times left to be disabled\n",
						data->activations);
	}

	spin_unlock_irqrestore(&data->lock, flags);

	return disabled;
}

static bool __exynos_sysmmu_disable(struct device *dev)
{
	unsigned long flags;
	bool disabled = true;
	struct exynos_iommu_owner *owner = dev->archdata.iommu;
	struct device *sysmmu;

	BUG_ON(!has_sysmmu(dev));

	spin_lock_irqsave(&owner->lock, flags);

	/* Every call to __sysmmu_disable() must return same result */
	for_each_sysmmu(dev, sysmmu) {
		struct sysmmu_drvdata *data = dev_get_drvdata(sysmmu);
		disabled = __sysmmu_disable(data);
		if (disabled)
			data->master = NULL;
	}

	spin_unlock_irqrestore(&owner->lock, flags);

	return disabled;
}

static void __sysmmu_enable_nocount(struct sysmmu_drvdata *data)
{
	int i;

	clk_enable(data->clk);

	for (i = 0; i < data->nsfrs; i++) {
		unsigned long cfg = 1;

		__sysmmu_set_ptbase(data->sfrbases[i], data->pgtable);

		if ((readl(data->sfrbases[i] + REG_MMU_VERSION) >> 28) == 3) {
			/* System MMU version is 3.x */
			cfg |= (1 << 12) | (2 << 28);
			__sysmmu_set_prefbuf(data->sfrbases[i], 0, -1, 0);
			__sysmmu_set_prefbuf(data->sfrbases[i], 0, -1, 1);
		}

		__raw_writel(cfg, data->sfrbases[i] + REG_MMU_CFG);

		__raw_writel(CTRL_ENABLE, data->sfrbases[i] + REG_MMU_CTRL);
	}
}

static int __sysmmu_enable(struct sysmmu_drvdata *data,
			unsigned long pgtable, struct iommu_domain *domain)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	if (set_sysmmu_active(data)) {
		data->pgtable = pgtable;
		data->domain = domain;

		__sysmmu_enable_nocount(data);

		dev_dbg(data->sysmmu, "Enabled\n");
	} else {
		ret = (pgtable == data->pgtable) ? 1 : -EBUSY;

		dev_dbg(data->sysmmu, "Already enabled\n");
	}

	if (WARN_ON(ret < 0))
		set_sysmmu_inactive(data); /* decrement count */

	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}

/* __exynos_sysmmu_enable: Enables System MMU
 *
 * returns -error if an error occurred and System MMU is not enabled,
 * 0 if the System MMU has been just enabled and 1 if System MMU was already
 * enabled before.
 */
static int __exynos_sysmmu_enable(struct device *dev, unsigned long pgtable,
				struct iommu_domain *domain)
{
	int ret = 0;
	unsigned long flags;
	struct exynos_iommu_owner *owner = dev->archdata.iommu;
	struct device *sysmmu;

	BUG_ON(!has_sysmmu(dev));

	spin_lock_irqsave(&owner->lock, flags);

	for_each_sysmmu(dev, sysmmu) {
		struct sysmmu_drvdata *data = dev_get_drvdata(sysmmu);
		ret = __sysmmu_enable(data, pgtable, domain);
		if (ret < 0) {
			struct device *iter;
			for_each_sysmmu_until(dev, iter, sysmmu) {
				data = dev_get_drvdata(iter);
				__sysmmu_disable(data);
			}
		} else {
			data->master = dev;
		}
	}

	spin_unlock_irqrestore(&owner->lock, flags);

	return ret;
}

int exynos_sysmmu_enable(struct device *dev, unsigned long pgtable)
{
	int ret;
	struct device *sysmmu;

	BUG_ON(!memblock_is_memory(pgtable));

	for_each_sysmmu(dev, sysmmu) {
		ret = pm_runtime_get_sync(sysmmu);
		if (ret < 0)
			break;
	}

	if (ret < 0) {
		struct device *start;
		for_each_sysmmu_until(dev, start, sysmmu)
			pm_runtime_put(start);

		return ret;
	}

	ret = __exynos_sysmmu_enable(dev, pgtable, NULL);
	if (ret < 0)
		for_each_sysmmu(dev, sysmmu)
			pm_runtime_put(sysmmu);

	return ret;
}

bool exynos_sysmmu_disable(struct device *dev)
{
	bool disabled;
	struct device *sysmmu;

	disabled = __exynos_sysmmu_disable(dev);

	for_each_sysmmu(dev, sysmmu)
		pm_runtime_put(sysmmu);

	return disabled;
}

static void sysmmu_tlb_invalidate_entry(struct device *dev, unsigned long iova)
{
	struct device *sysmmu;

	for_each_sysmmu(dev, sysmmu) {
		unsigned long flags;
		struct sysmmu_drvdata *data;

		data = dev_get_drvdata(sysmmu);

		spin_lock_irqsave(&data->lock, flags);
		if (is_sysmmu_active(data)) {
			int i;
			for (i = 0; i < data->nsfrs; i++) {
				if (sysmmu_block(data->sfrbases[i])) {
					__sysmmu_tlb_invalidate_entry(
						data->sfrbases[i], iova);
					sysmmu_unblock(data->sfrbases[i]);
				} else {
					dev_err(dev,
					"%s failed due to blocking timeout\n",
					__func__);
				}
			}
		} else {
			dev_dbg(dev,
			"Disabled. Skipping TLB invalidation for %#lx\n", iova);
		}
		spin_unlock_irqrestore(&data->lock, flags);
	}
}

void exynos_sysmmu_tlb_invalidate(struct device *dev)
{
	struct device *sysmmu;

	for_each_sysmmu(dev, sysmmu) {
		unsigned long flags;
		struct sysmmu_drvdata *data;

		data = dev_get_drvdata(sysmmu);

		spin_lock_irqsave(&data->lock, flags);
		if (is_sysmmu_active(data)) {
			int i;
			for (i = 0; i < data->nsfrs; i++) {
				if (sysmmu_block(data->sfrbases[i])) {
					__sysmmu_tlb_invalidate(
							data->sfrbases[i]);
					sysmmu_unblock(data->sfrbases[i]);
				} else {
					dev_err(dev,
					"%s failed due to blocking timeout\n",
					__func__);
				}
			}
		} else {
			dev_dbg(dev, "Disabled. Skipping TLB invalidation\n");
		}
		spin_unlock_irqrestore(&data->lock, flags);
	}
}

static int __init __sysmmu_init_clock(struct device *sysmmu,
					struct sysmmu_drvdata *drvdata,
					struct device *master)
{
	char *conid;
	struct clk *parent_clk;
	int ret;

	drvdata->clk = clk_get(sysmmu, "sysmmu");
	if (IS_ERR(drvdata->clk)) {
		dev_dbg(sysmmu, "No gating clock found.\n");
		drvdata->clk = NULL;
		return 0;
	}

	if (!master)
		return 0;

	conid = dev_get_platdata(sysmmu);
	if (!conid) {
		dev_dbg(sysmmu, "No parent clock specified.\n");
		return 0;
	}

	parent_clk = clk_get(master, conid);
	if (IS_ERR(parent_clk)) {
		parent_clk = clk_get(NULL, conid);
		if (IS_ERR(parent_clk)) {
			clk_put(drvdata->clk);
			dev_err(sysmmu, "No parent clock '%s,%s' found\n",
				dev_name(master), conid);
			return PTR_ERR(parent_clk);
		}
	}

	ret = clk_set_parent(drvdata->clk, parent_clk);
	if (ret) {
		clk_put(drvdata->clk);
		dev_err(sysmmu, "Failed to set parent clock '%s,%s'\n",
				dev_name(master), conid);
	}

	clk_put(parent_clk);

	return ret;
}

static int __init __sysmmu_setup(struct device *sysmmu,
				struct sysmmu_drvdata *drvdata)
{
	struct device_node *master_node;
	struct device *child;
	const char *compat;
	struct platform_device *pmaster = NULL;
	u32 master_inst_no = -1;
	int ret;

	master_node = of_parse_phandle(sysmmu->of_node, "mmu-master", 0);
	if (!master_node && !of_property_read_string(
			sysmmu->of_node, "mmu-master-compat", &compat)) {
		of_property_read_u32_array(sysmmu->of_node,
					"mmu-master-no", &master_inst_no, 1);
		for_each_compatible_node(master_node, NULL, compat) {
			pmaster = of_find_device_by_node(master_node);
			if (pmaster && (pmaster->id == master_inst_no))
				break;
			of_dev_put(pmaster);
			pmaster = NULL;
		}
	} else if (master_node) {
		pmaster = of_find_device_by_node(master_node);
	}

	if (!pmaster) {
		dev_dbg(sysmmu, "No master device is specified.\n");
		return __sysmmu_init_clock(sysmmu, drvdata, NULL);
	}

	child = &pmaster->dev;

	while (child->parent && is_sysmmu(child->parent))
		child = child->parent;

	ret = device_move(child, sysmmu, DPM_ORDER_PARENT_BEFORE_DEV);
	if (ret) {
		dev_err(sysmmu, "Failed to set parent of %s\n",
						dev_name(child));
		goto err_dev_put;
	}

	if (!pmaster->dev.archdata.iommu) {
		struct exynos_iommu_owner *owner;
		owner = devm_kzalloc(sysmmu, sizeof(*owner), GFP_KERNEL);
		if (!owner) {
			ret = -ENOMEM;
			dev_err(sysmmu, "Failed to allocate iommu data\n");
			goto err_dev_put;
		}

		INIT_LIST_HEAD(&owner->client);
		owner->dev = &pmaster->dev;
		spin_lock_init(&owner->lock);

		pmaster->dev.archdata.iommu = owner;
	}

	ret = __sysmmu_init_clock(sysmmu, drvdata, &pmaster->dev);
	if (ret)
		dev_err(sysmmu, "Failed to initialize gating clocks\n");
	else
		dev_dbg(sysmmu, "Assigned master device %s\n",
						dev_name(&pmaster->dev));
err_dev_put:
	of_dev_put(pmaster);

	return ret;
}

static int __init exynos_sysmmu_probe(struct platform_device *pdev)
{
	int i, ret;
	struct device *dev = &pdev->dev;
	struct sysmmu_drvdata *data;

	if (pdev->num_resources == 0) {
		dev_err(dev, "No System MMU resource defined\n");
		return -ENODEV;
	}

	data = devm_kzalloc(dev,
			sizeof(*data)
			+ sizeof(*data->sfrbases) * (pdev->num_resources / 2),
			GFP_KERNEL);
	if (!data) {
		dev_err(dev, "Not enough memory\n");
		return -ENOMEM;
	}

	data->nsfrs = pdev->num_resources / 2;

	for (i = 0; i < data->nsfrs; i++) {
		struct resource *res;
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			dev_err(dev, "Unable to find IOMEM region\n");
			return -ENOENT;
		}

		data->sfrbases[i] = devm_request_and_ioremap(dev, res);
		if (!data->sfrbases[i]) {
			dev_err(dev, "Unable to map IOMEM @ PA:%#x\n",
							res->start);
			return -EBUSY;
		}
	}

	for (i = 0; i < data->nsfrs; i++) {
		ret = platform_get_irq(pdev, i);
		if (ret <= 0) {
			dev_err(dev, "Unable to find IRQ resource\n");
			return ret;
		}

		ret = devm_request_irq(dev, ret, exynos_sysmmu_irq, 0,
					dev_name(dev), data);
		if (ret) {
			dev_err(dev, "Unabled to register interrupt handler\n");
			return ret;
		}
	}


	ret = __sysmmu_setup(dev, data);
	if (!ret) {
		data->sysmmu = dev;
		spin_lock_init(&data->lock);

		pm_runtime_enable(dev);

		platform_set_drvdata(pdev, data);

		dev->archdata.iommu = &sysmmu_placeholder;
		dev_dbg(dev, "Initialized successfully!\n");
	}

	return ret;
}

/*
 * Descriptions of Device Tree node for System MMU
 *
 * A System MMU should be described by a single tree node.
 *
 * A System MMU node should have the following properties:
 * - reg: tuples of the base address and the size of the IO region of System MMU
 * - compatible: it must be "samsung,exynos-sysmmu".
 * - interrupt-parent = specify if the interrupt of System MMU is generated by
 *   interrupt combiner or interrupt controller.
 * - interrupts: tuples of interrupt numbers. a tuple has 2 elements if
 *   @interrupt-parent is '<&combiner>', 3 elements otherwise.
 *
 * 'mmuname', 'reg' and 'interrupts' properties can be an array if the System
 * MMU driver controls several number of System MMUs at the same time. Note that
 * the number of elements in those three properties must be the same.
 *
 * The following properties are optional:
 * - mmuname: name of the System MMU for debugging purpose
 * - mmu-master: reference to the node of the master device.
 * - mmu-master-compat: 'compatible' proberty of the node of the master device
 *    of System MMU. This is ignored if @mmu-master is currectly specified.
 * - mmu-master-no: instance number of the master device of System MMU. This is
 *    ignored if @mmu-master is correctly specified. This is '0' by default.
 */
#ifdef CONFIG_OF
static struct of_device_id sysmmu_of_match[] __initconst = {
	{ .compatible = "samsung,exynos-sysmmu", },
	{ },
};
#endif

static struct platform_driver exynos_sysmmu_driver __refdata = {
	.probe		= exynos_sysmmu_probe,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "exynos-sysmmu",
		.of_match_table = of_match_ptr(sysmmu_of_match),
	}
};

static inline void pgtable_flush(void *vastart, void *vaend)
{
	dmac_flush_range(vastart, vaend);
	outer_flush_range(virt_to_phys(vastart),
				virt_to_phys(vaend));
}

static int exynos_iommu_fault_handler(struct iommu_domain *domain,
		struct device *dev, unsigned long iova, int flags, void *token)
{
	struct exynos_iommu_domain *priv = domain->priv;

	dev_err(dev, "System MMU Generated FAULT!\n\n");
	__show_fault_information(priv->pgtable, iova, flags);

	return -ENOSYS;
}

static int exynos_iommu_domain_init(struct iommu_domain *domain)
{
	struct exynos_iommu_domain *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pgtable = (unsigned long *)__get_free_pages(
						GFP_KERNEL | __GFP_ZERO, 2);
	if (!priv->pgtable)
		goto err_pgtable;

	priv->lv2entcnt = (short *)__get_free_pages(
						GFP_KERNEL | __GFP_ZERO, 1);
	if (!priv->lv2entcnt)
		goto err_counter;

	pgtable_flush(priv->pgtable, priv->pgtable + NUM_LV1ENTRIES);

	spin_lock_init(&priv->lock);
	spin_lock_init(&priv->pgtablelock);
	INIT_LIST_HEAD(&priv->clients);

	domain->geometry.aperture_start = 0;
	domain->geometry.aperture_end   = ~0UL;
	domain->geometry.force_aperture = true;

	iommu_set_fault_handler(domain, &exynos_iommu_fault_handler, NULL);

	domain->priv = priv;
	return 0;

err_counter:
	free_pages((unsigned long)priv->pgtable, 2);
err_pgtable:
	kfree(priv);
	return -ENOMEM;
}

static void exynos_iommu_domain_destroy(struct iommu_domain *domain)
{
	struct exynos_iommu_domain *priv = domain->priv;
	struct exynos_iommu_owner *owner, *n;
	unsigned long flags;
	int i;

	WARN_ON(!list_empty(&priv->clients));

	spin_lock_irqsave(&priv->lock, flags);

	list_for_each_entry_safe(owner, n, &priv->clients, client) {
		struct device *sysmmu;
		while (!__exynos_sysmmu_disable(owner->dev))
			; /* until System MMU is actually disabled */
		list_del_init(&owner->client);

		for_each_sysmmu(owner->dev, sysmmu)
			pm_runtime_put(sysmmu);
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	for (i = 0; i < NUM_LV1ENTRIES; i++)
		if (lv1ent_page(priv->pgtable + i))
			kmem_cache_free(lv2table_kmem_cache,
					__va(lv2table_base(priv->pgtable + i)));

	free_pages((unsigned long)priv->pgtable, 2);
	free_pages((unsigned long)priv->lv2entcnt, 1);
	kfree(domain->priv);
	domain->priv = NULL;
}

static int exynos_iommu_attach_device(struct iommu_domain *domain,
				   struct device *dev)
{
	struct exynos_iommu_owner *owner = dev->archdata.iommu;
	struct exynos_iommu_domain *priv = domain->priv;
	unsigned long flags;
	int ret;
	struct device *sysmmu;

	if (WARN_ON(!list_empty(&owner->client))) {
		bool found = false;
		struct exynos_iommu_owner *tmpowner;

		spin_lock_irqsave(&priv->lock, flags);
		list_for_each_entry(tmpowner, &priv->clients, client) {
			if (tmpowner == owner) {
				found = true;
				break;
			}
		}
		spin_unlock_irqrestore(&priv->lock, flags);

		if (!found) {
			dev_err(dev, "%s: Already attached to another domain\n",
								__func__);
			return -EBUSY;
		}

		dev_dbg(dev, "%s: Already attached to this domain\n", __func__);
		return 0;
	}

	for_each_sysmmu(dev, sysmmu) {
		ret = pm_runtime_get_sync(sysmmu);
		if (ret < 0)
			break;
	}

	if (ret < 0) {
		struct device *start;
		for_each_sysmmu_until(dev, start, sysmmu)
			pm_runtime_put(start);

		return ret;
	}

	spin_lock_irqsave(&priv->lock, flags);

	ret = __exynos_sysmmu_enable(dev, __pa(priv->pgtable), domain);

	/*
	 * __exynos_sysmmu_enable() returns 1
	 * if the System MMU of dev is already enabled
	 */
	BUG_ON(ret > 0);

	list_add_tail(&owner->client, &priv->clients);

	spin_unlock_irqrestore(&priv->lock, flags);

	if (ret < 0) {
		dev_err(dev, "%s: Failed to attach IOMMU with pgtable %#lx\n",
				__func__, __pa(priv->pgtable));
		for_each_sysmmu(dev, sysmmu)
			pm_runtime_put(sysmmu);
	} else {
		dev_dbg(dev, "%s: Attached new IOMMU with pgtable 0x%lx\n",
					__func__, __pa(priv->pgtable));
	}

	return ret;
}

static void exynos_iommu_detach_device(struct iommu_domain *domain,
				    struct device *dev)
{
	struct exynos_iommu_owner *owner, *n;
	struct exynos_iommu_domain *priv = domain->priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	list_for_each_entry_safe(owner, n, &priv->clients, client) {
		if (owner == dev->archdata.iommu) {
			if (__exynos_sysmmu_disable(dev))
				list_del_init(&owner->client);
			else
				BUG();
			break;
		}
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	if (owner == dev->archdata.iommu) {
		struct device *sysmmu;
		dev_dbg(dev, "%s: Detached IOMMU with pgtable %#lx\n",
					__func__, __pa(priv->pgtable));
		for_each_sysmmu(dev, sysmmu)
			pm_runtime_put(sysmmu);

	} else
		dev_dbg(dev, "%s: No IOMMU is attached\n", __func__);
}

static unsigned long *alloc_lv2entry(unsigned long *sent, unsigned long iova,
					short *pgcounter)
{
	if (lv1ent_fault(sent)) {
		unsigned long *pent;

		pent = kmem_cache_zalloc(lv2table_kmem_cache, GFP_ATOMIC);
		BUG_ON((unsigned long)pent & (LV2TABLE_SIZE - 1));
		if (!pent)
			return NULL;

		*sent = mk_lv1ent_page(__pa(pent));
		*pgcounter = NUM_LV2ENTRIES;
		pgtable_flush(pent, pent + NUM_LV2ENTRIES);
		pgtable_flush(sent, sent + 1);
	}

	return page_entry(sent, iova);
}

static int lv1set_section(unsigned long *sent, phys_addr_t paddr, short *pgcnt)
{
	if (lv1ent_section(sent))
		return -EADDRINUSE;

	if (lv1ent_page(sent)) {
		if (*pgcnt != NUM_LV2ENTRIES)
			return -EADDRINUSE;

		kmem_cache_free(lv2table_kmem_cache, page_entry(sent, 0));

		*pgcnt = 0;
	}

	*sent = mk_lv1ent_sect(paddr);

	pgtable_flush(sent, sent + 1);

	return 0;
}

static int lv2set_page(unsigned long *pent, phys_addr_t paddr, size_t size,
								short *pgcnt)
{
	if (size == SPAGE_SIZE) {
		if (!lv2ent_fault(pent))
			return -EADDRINUSE;

		*pent = mk_lv2ent_spage(paddr);
		pgtable_flush(pent, pent + 1);
		*pgcnt -= 1;
	} else { /* size == LPAGE_SIZE */
		int i;
		for (i = 0; i < SPAGES_PER_LPAGE; i++, pent++) {
			if (!lv2ent_fault(pent)) {
				memset(pent, 0, sizeof(*pent) * i);
				return -EADDRINUSE;
			}

			*pent = mk_lv2ent_lpage(paddr);
		}
		pgtable_flush(pent - SPAGES_PER_LPAGE, pent);
		*pgcnt -= SPAGES_PER_LPAGE;
	}

	return 0;
}

static int exynos_iommu_map(struct iommu_domain *domain, unsigned long iova,
			 phys_addr_t paddr, size_t size, int prot)
{
	struct exynos_iommu_domain *priv = domain->priv;
	unsigned long *entry;
	unsigned long flags;
	int ret = -ENOMEM;

	BUG_ON(priv->pgtable == NULL);

	spin_lock_irqsave(&priv->pgtablelock, flags);

	entry = section_entry(priv->pgtable, iova);

	if (size == SECT_SIZE) {
		ret = lv1set_section(entry, paddr,
					&priv->lv2entcnt[lv1ent_offset(iova)]);
	} else {
		unsigned long *pent;

		pent = alloc_lv2entry(entry, iova,
					&priv->lv2entcnt[lv1ent_offset(iova)]);

		if (!pent)
			ret = -ENOMEM;
		else
			ret = lv2set_page(pent, paddr, size,
					&priv->lv2entcnt[lv1ent_offset(iova)]);
	}

	if (ret) {
		pr_debug("%s: Failed to map iova 0x%lx/0x%x bytes\n",
							__func__, iova, size);
	}

	spin_unlock_irqrestore(&priv->pgtablelock, flags);

	return ret;
}

static size_t exynos_iommu_unmap(struct iommu_domain *domain,
					       unsigned long iova, size_t size)
{
	struct exynos_iommu_domain *priv = domain->priv;
	unsigned long flags;
	unsigned long *ent;

	BUG_ON(priv->pgtable == NULL);

	spin_lock_irqsave(&priv->pgtablelock, flags);

	ent = section_entry(priv->pgtable, iova);

	if (lv1ent_section(ent)) {
		BUG_ON(size < SECT_SIZE);

		*ent = 0;
		pgtable_flush(ent, ent + 1);
		size = SECT_SIZE;
		goto done;
	}

	if (unlikely(lv1ent_fault(ent))) {
		if (size > SECT_SIZE)
			size = SECT_SIZE;
		goto done;
	}

	/* lv1ent_page(sent) == true here */

	ent = page_entry(ent, iova);

	if (unlikely(lv2ent_fault(ent))) {
		size = SPAGE_SIZE;
		goto done;
	}

	if (lv2ent_small(ent)) {
		*ent = 0;
		pgtable_flush(ent, ent + 1);
		size = SPAGE_SIZE;
		priv->lv2entcnt[lv1ent_offset(iova)] += 1;
		goto done;
	}

	/* lv1ent_large(ent) == true here */
	BUG_ON(size < LPAGE_SIZE);

	memset(ent, 0, sizeof(*ent) * SPAGES_PER_LPAGE);
	pgtable_flush(ent, ent + SPAGES_PER_LPAGE);

	size = LPAGE_SIZE;
	priv->lv2entcnt[lv1ent_offset(iova)] += SPAGES_PER_LPAGE;
done:
	spin_unlock_irqrestore(&priv->pgtablelock, flags);

	spin_lock_irqsave(&priv->lock, flags);
	{
		struct exynos_iommu_owner *owner;
		list_for_each_entry(owner, &priv->clients, client)
			sysmmu_tlb_invalidate_entry(owner->dev, iova);
	}
	spin_unlock_irqrestore(&priv->lock, flags);


	return size;
}

static phys_addr_t exynos_iommu_iova_to_phys(struct iommu_domain *domain,
					  unsigned long iova)
{
	struct exynos_iommu_domain *priv = domain->priv;
	unsigned long *entry;
	unsigned long flags;
	phys_addr_t phys = 0;

	spin_lock_irqsave(&priv->pgtablelock, flags);

	entry = section_entry(priv->pgtable, iova);

	if (lv1ent_section(entry)) {
		phys = section_phys(entry) + section_offs(iova);
	} else if (lv1ent_page(entry)) {
		entry = page_entry(entry, iova);

		if (lv2ent_large(entry))
			phys = lpage_phys(entry) + lpage_offs(iova);
		else if (lv2ent_small(entry))
			phys = spage_phys(entry) + spage_offs(iova);
	}

	spin_unlock_irqrestore(&priv->pgtablelock, flags);

	return phys;
}

static struct iommu_ops exynos_iommu_ops = {
	.domain_init = &exynos_iommu_domain_init,
	.domain_destroy = &exynos_iommu_domain_destroy,
	.attach_dev = &exynos_iommu_attach_device,
	.detach_dev = &exynos_iommu_detach_device,
	.map = &exynos_iommu_map,
	.unmap = &exynos_iommu_unmap,
	.iova_to_phys = &exynos_iommu_iova_to_phys,
	.pgsize_bitmap = SECT_SIZE | LPAGE_SIZE | SPAGE_SIZE,
};

static int __init exynos_iommu_init(void)
{
	int ret;

	lv2table_kmem_cache = kmem_cache_create("exynos-iommu-lv2table",
		LV2TABLE_SIZE, LV2TABLE_SIZE, 0, NULL);
	if (!lv2table_kmem_cache) {
		pr_err("%s: failed to create kmem cache\n", __func__);
		return -ENOMEM;
	}

	ret = platform_driver_register(&exynos_sysmmu_driver);

	if (ret == 0)
		ret = bus_set_iommu(&platform_bus_type, &exynos_iommu_ops);

	if (ret) {
		pr_err("%s: Failed to register exynos-iommu driver.\n",
								__func__);
		kmem_cache_destroy(lv2table_kmem_cache);
	}

	return ret;
}
subsys_initcall(exynos_iommu_init);
