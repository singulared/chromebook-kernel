/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS - PPMU polling support
 *	This version supports EXYNOS5420.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <mach/map.h>

#include "exynos5420_ppmu.h"

static DEFINE_SPINLOCK(exynos5420_ppmu_lock);
static LIST_HEAD(exynos5420_ppmu_handle_list);

struct exynos5420_ppmu_handle {
	struct list_head node;
	struct exynos5420_ppmu ppmu[PPMU_END];
};

static struct exynos5420_ppmu ppmu[PPMU_END] = {
	[PPMU_DMC_0_0] = {
		.hw_base = S5P_VA_PPMU_DREX0_0,
		.event[PPMU_PMNCNT3] = RDWR_DATA_COUNT,
	},
	[PPMU_DMC_0_1] = {
		.hw_base = S5P_VA_PPMU_DREX0_1,
		.event[PPMU_PMNCNT3] = RDWR_DATA_COUNT,
	},
	[PPMU_DMC_1_0] = {
		.hw_base = S5P_VA_PPMU_DREX1_0,
		.event[PPMU_PMNCNT3] = RDWR_DATA_COUNT,
	},
	[PPMU_DMC_1_1] = {
		.hw_base = S5P_VA_PPMU_DREX1_1,
		.event[PPMU_PMNCNT3] = RDWR_DATA_COUNT,
	},
};

static void exynos5420_ppmu_setevent(void __iomem *ppmu_base,
					unsigned int ch,
					unsigned int evt)
{
	__raw_writel(evt, ppmu_base + PPMU_BEVTSEL(ch));
}

static void exynos5420_ppmu_start(void __iomem *ppmu_base)
{
	__raw_writel(PPMU_ENABLE, ppmu_base);
}

static void exynos5420_ppmu_stop(void __iomem *ppmu_base)
{
	__raw_writel(PPMU_DISABLE, ppmu_base);
}

static void exynos5420_ppmu_reset(struct exynos5420_ppmu *ppmu)
{
	void __iomem *ppmu_base = ppmu->hw_base;
	unsigned int cntens_val, flag_val, pmnc_val;
	unsigned long flags;

	cntens_val = PPMU_ENABLE_CYCLE | PPMU_ENABLE_COUNT0 |
			PPMU_ENABLE_COUNT1 | PPMU_ENABLE_COUNT2 |
			PPMU_ENABLE_COUNT3;

	flag_val = cntens_val;

	pmnc_val = PPMU_CC_RESET | PPMU_COUNT_RESET;

	/* Reset PPMU */
	__raw_writel(cntens_val, ppmu_base + PPMU_CNTENS);
	__raw_writel(flag_val, ppmu_base + PPMU_FLAG);
	__raw_writel(pmnc_val, ppmu_base);
	__raw_writel(0x0, ppmu_base + PPMU_CCNT);

	/* Set PPMU Event */
	ppmu->event[PPMU_PMNCNT0] = RD_DATA_COUNT;
	exynos5420_ppmu_setevent(ppmu_base, PPMU_PMNCNT0,
				ppmu->event[PPMU_PMNCNT0]);

	ppmu->event[PPMU_PMCCNT1] = WR_DATA_COUNT;
	exynos5420_ppmu_setevent(ppmu_base, PPMU_PMCCNT1,
				ppmu->event[PPMU_PMCCNT1]);

	ppmu->event[PPMU_PMNCNT3] = RDWR_DATA_COUNT;
	exynos5420_ppmu_setevent(ppmu_base, PPMU_PMNCNT3,
				ppmu->event[PPMU_PMNCNT3]);

	/* Start PPMU */
	local_irq_save(flags);
	exynos5420_ppmu_start(ppmu_base);
	local_irq_restore(flags);
}

static void exynos5420_ppmu_read(struct exynos5420_ppmu *ppmu)
{
	void __iomem *ppmu_base = ppmu->hw_base;
	u32 overflow;
	int i;
	unsigned long flags;

	/* Stop PPMU */
	local_irq_save(flags);
	exynos5420_ppmu_stop(ppmu_base);
	local_irq_restore(flags);

	/* Update local data from PPMU */
	ppmu->ccnt = __raw_readl(ppmu_base + PPMU_CCNT);
	overflow = __raw_readl(ppmu_base + PPMU_FLAG);
	ppmu->ccnt_overflow = overflow & PPMU_CCNT_OVERFLOW;

	for (i = PPMU_PMNCNT0; i < PPMU_PMNCNT_MAX; i++) {
		if (ppmu->event[i] == 0) {
			ppmu->count[i] = 0;
		} else {
			if (i == PPMU_PMNCNT3) {
				/* RDWR data count */
				ppmu->count[i] = ((__raw_readl(ppmu_base +
						PMCNT_OFFSET(i)) << 8) |
						__raw_readl(ppmu_base +
						PMCNT_OFFSET(i + 1)));
			} else {
				ppmu->count[i] = __raw_readl(ppmu_base +
						PMCNT_OFFSET(i));
			}
		}
	}
}

static void exynos5420_ppmu_add(struct exynos5420_ppmu *to,
				struct exynos5420_ppmu *from)
{
	int i, j;

	for (i = 0; i < PPMU_END; i++) {
		for (j = PPMU_PMNCNT0; j < PPMU_PMNCNT_MAX; j++)
			to[i].count[j] += from[i].count[j];

		to[i].ccnt += from[i].ccnt;
		/*
		 * Check for overflow; when to[i].ccnt is greater than
		 * its highest possible value will wrap around,
		 * indicating an overflow.
		 */
		if (to[i].ccnt < from[i].ccnt)
			to[i].ccnt_overflow = true;

		to[i].ns += from[i].ns;

		if (from[i].ccnt_overflow)
			to[i].ccnt_overflow = true;
	}
}

static void exynos5420_ppmu_handle_clear(struct exynos5420_ppmu_handle *handle)
{
	memset(&handle->ppmu, 0, sizeof(struct exynos5420_ppmu) * PPMU_END);
}

int exynos5420_ppmu_update(enum exynos5420_ppmu_sets filter)
{
	int i, ret;
	struct exynos5420_ppmu_handle *handle;
	enum exynos5420_ppmu_list start, end;

	ret = exynos5420_ppmu_get_filter(filter, &start, &end);
	if (ret < 0)
		return ret;

	for (i = start; i <= end; i++) {
		exynos5420_ppmu_read(&ppmu[i]);
		exynos5420_ppmu_reset(&ppmu[i]);
	}

	/*
	 * Update the ccnt(no. of clock cycles) and count(number of events).
	 * Also check if the ccnt has overflowed since occurence of the
	 * last event.
	 */
	list_for_each_entry(handle, &exynos5420_ppmu_handle_list, node)
		exynos5420_ppmu_add(handle->ppmu, ppmu);

	return 0;
}

int exynos5420_ppmu_get_filter(enum exynos5420_ppmu_sets filter,
	enum exynos5420_ppmu_list *start, enum exynos5420_ppmu_list *end)
{
	switch (filter) {
	case PPMU_SET_DDR:
		*start = PPMU_DMC_0_1;
		*end = PPMU_DMC_1_1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int exynos5420_ppmu_get_busy(struct exynos5420_ppmu_handle *handle,
				enum exynos5420_ppmu_sets filter,
				unsigned int *ccnt, unsigned long *pmcnt)
{
	unsigned long flags;
	int i, ret, temp;
	int busy = 0;
	enum exynos5420_ppmu_list start, end;
	unsigned int max_ccnt = 0, temp_ccnt;
	unsigned long max_pmcnt = 0, temp_pmcnt;

	ret = exynos5420_ppmu_get_filter(filter, &start, &end);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&exynos5420_ppmu_lock, flags);

	exynos5420_ppmu_update(filter);

	for (i = start; i <= end; i++) {
		if (handle->ppmu[i].ccnt_overflow) {
			busy = -EOVERFLOW;
			break;
		}

		temp_ccnt = handle->ppmu[i].ccnt;
		if (temp_ccnt > max_ccnt)
			max_ccnt = temp_ccnt;

		temp_pmcnt = handle->ppmu[i].count[PPMU_PMNCNT3];
		if (temp_pmcnt > max_pmcnt)
			max_pmcnt = temp_pmcnt;

		temp = handle->ppmu[i].count[PPMU_PMNCNT3] * 100;
		if (handle->ppmu[i].count > 0 && temp > 0 &&
				handle->ppmu[i].ccnt > 0)
			temp /= handle->ppmu[i].ccnt;

		if (temp > busy)
			busy = temp;
	}

	*ccnt = max_ccnt;
	*pmcnt = max_pmcnt;
	exynos5420_ppmu_handle_clear(handle);
	spin_unlock_irqrestore(&exynos5420_ppmu_lock, flags);
	return busy;
}

void exynos5420_ppmu_put(struct exynos5420_ppmu_handle *handle)
{
	unsigned long flags;

	spin_lock_irqsave(&exynos5420_ppmu_lock, flags);
	list_del(&handle->node);
	spin_unlock_irqrestore(&exynos5420_ppmu_lock, flags);
	kfree(handle);
}

struct exynos5420_ppmu_handle *exynos5420_ppmu_get(enum exynos5420_ppmu_sets
							filter)
{
	struct exynos5420_ppmu_handle *handle;
	unsigned long flags;

	handle = kzalloc(sizeof(struct exynos5420_ppmu_handle), GFP_KERNEL);
	if (!handle)
		return ERR_PTR(-ENOMEM);

	spin_lock_irqsave(&exynos5420_ppmu_lock, flags);

	exynos5420_ppmu_update(filter);
	list_add_tail(&handle->node, &exynos5420_ppmu_handle_list);

	spin_unlock_irqrestore(&exynos5420_ppmu_lock, flags);

	return handle;
}
