/*
 * drivers/media/platform/s5p-mfc/s5p_mfc_opr.c
 *
 * Samsung MFC (Multi Function Codec - FIMV) driver
 * This file contains hw related functions.
 *
 * Kamil Debski, Copyright (c) 2012 Samsung Electronics Co., Ltd.
 * http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "s5p_mfc_debug.h"
#include "s5p_mfc_opr.h"
#include "s5p_mfc_opr_v5.h"
#include "s5p_mfc_opr_v6.h"
#include "s5p_mfc_ctrl.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_pm.h"

void s5p_mfc_init_hw_ops(struct s5p_mfc_dev *dev)
{
	if (IS_MFCV6(dev))
		dev->mfc_ops = s5p_mfc_init_hw_ops_v6();
	else
		dev->mfc_ops = s5p_mfc_init_hw_ops_v5();
}

void s5p_mfc_init_regs(struct s5p_mfc_dev *dev)
{
	if (IS_MFCV6(dev))
		dev->mfc_regs = s5p_mfc_init_regs_v6_plus(dev);
}

void s5p_mfc_init_hw_ctrls(struct s5p_mfc_dev *dev)
{
	if (IS_MFCV6(dev))
		dev->mfc_ctrl_ops = s5p_mfc_init_hw_ctrl_ops_v6_plus();
	else
		dev->mfc_ctrl_ops = s5p_mfc_init_hw_ctrl_ops_v5();
}

int s5p_mfc_alloc_priv_buf(struct device *dev, struct s5p_mfc_priv_buf *b,
				size_t size, bool needs_cpu_access)
{
	void *token;
	dma_addr_t dma;
	DEFINE_DMA_ATTRS(attrs);

	mfc_debug(3, "Allocating priv buffer of size: %d\n", size);

	/* We shouldn't normally allocate on top of previously-allocated buffer,
	 * but if we happen to, at least free it first. */
	WARN_ON(b->token);
	s5p_mfc_release_priv_buf(dev, b);

	if (!needs_cpu_access)
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);

	token = dma_alloc_attrs(dev, size, &dma, GFP_KERNEL, &attrs);
	if (!token) {
		mfc_err("Allocating private buffer of size %d failed\n", size);
		return -ENOMEM;
	}

	if (needs_cpu_access)
		b->virt = token;

	b->dma = dma;
	b->size = size;
	b->attrs = attrs;
	b->token = token;

	mfc_debug(3, "Allocated dma_addr %08x mapped to %p\n", b->dma, b->virt);
	return 0;
}

void s5p_mfc_release_priv_buf(struct device *dev, struct s5p_mfc_priv_buf *b)
{
	if (b->token)
		dma_free_attrs(dev, b->size, b->token, b->dma, &b->attrs);

	memset(b, 0, sizeof(*b));
}

void s5p_mfc_fatal_error(struct s5p_mfc_dev *dev, struct s5p_mfc_ctx *ctx)
{
	assert_spin_locked(&dev->irqlock);

	mfc_err("Got a fatal error, will clean up context if present.\n");

	if (!ctx) {
		/* Unrecoverable error in low level initialization. */
		set_bit(0, &dev->hw_error);
		return;
	}

	ctx->state = MFCINST_ERROR;

	/*
	 * All remaining clean up will happen in s5p_mfc_stop_streaming()
	 * and/or s5p_mfc_release().
	 */
}

/**
 * s5p_mfc_get_new_ctx() - check if we can proceed with running a context
 * @dev: MFC device to run.
 *
 * This function tries to acquire hardware lock and select next context to
 * run in a race-free fashion.
 *
 * Return:	Pointer to found context and hardware lock acquired
 *		or NULL and hardware lock not acquired.
 */
static struct s5p_mfc_ctx *s5p_mfc_get_new_ctx(struct s5p_mfc_dev *dev)
{
	struct s5p_mfc_ctx *ctx = NULL;
	unsigned long flags;

	if (test_bit(0, &dev->enter_suspend)) {
		mfc_debug(1, "Entering suspend so do not schedule any jobs\n");
		return NULL;
	}
	if (test_bit(0, &dev->hw_error)) {
		mfc_debug(1, "Hardware error recovery in progress, not scheduling\n");
		return NULL;
	}
	spin_lock_irqsave(&dev->irqlock, flags);
	if (list_empty(&dev->ready_ctx_list)) {
		mfc_debug(1, "No ctx is scheduled to be run\n");
		goto done;
	}
	/* Check whether hardware is not running */
	if (test_and_set_bit(0, &dev->hw_lock) != 0) {
		/* This is perfectly ok, the scheduled ctx should wait */
		mfc_debug(1, "Couldn't lock HW\n");
		goto done;
	}
	ctx = list_first_entry(&dev->ready_ctx_list, struct s5p_mfc_ctx,
				ready_ctx_list);
	dev->curr_ctx = ctx;

done:
	spin_unlock_irqrestore(&dev->irqlock, flags);

	return ctx;
}

/* Try running an operation on hardware */
static int s5p_mfc_try_once(struct s5p_mfc_dev *dev)
{
	struct s5p_mfc_ctx *ctx;
	unsigned long flags;

	mfc_debug(1, "Try run dev: %p\n", dev);

	/* Choose the context to run */
	ctx = s5p_mfc_get_new_ctx(dev);
	if (!ctx)
		return 0;

	/*
	 * At this point we already hold hardware lock and has a context
	 * to run. We can power on the hardware and program the operation.
	 */
	mfc_debug(1, "New context: %p\n", ctx);
	mfc_debug(1, "Seting new context to %p\n", ctx);
	/* Got context to run in ctx */
	mfc_debug(1, "ctx->dst_queue_cnt=%d ctx->dpb_count=%d ctx->src_queue_cnt=%d\n",
		ctx->dst_queue_cnt, ctx->dpb_count, ctx->src_queue_cnt);
	mfc_debug(1, "ctx->state=%d\n", ctx->state);
	/* Last frame has already been sent to MFC
	 * Now obtaining frames from MFC buffer */

	if (test_and_set_bit(0, &dev->clk_flag) == 0)
		s5p_mfc_clock_on(dev);

	s5p_mfc_clean_ctx_int_flags(ctx);

	if (!s5p_mfc_hw_call(dev->mfc_ops, run, ctx))
		return 0;

	/*
	 * Running this context failed.
	 * We need to signal an error, recover and try another one.
	 * We do it exactly the same as in case of hardware error interrupt.
	 */
	spin_lock_irqsave(&dev->irqlock, flags);
	s5p_mfc_fatal_error(dev, ctx);
	s5p_mfc_ctx_done_locked(ctx);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	/* Stop the clock in case we don't have more ready contexts left. */
	if (test_and_clear_bit(0, &dev->clk_flag))
		s5p_mfc_clock_off(dev);
	/* Free hardware lock */
	if (test_and_clear_bit(0, &dev->hw_lock) == 0)
		mfc_err("Failed to unlock hardware\n");

	s5p_mfc_wake_up_ctx(ctx, S5P_MFC_R2H_CMD_ERR_RET, 0);

	return -EAGAIN;
}

/* Try running an operation on hardware */
void s5p_mfc_try_run(struct s5p_mfc_dev *dev)
{
	int ret;

	do {
		ret = s5p_mfc_try_once(dev);
	} while (ret == -EAGAIN);
}

static void s5p_mfc_try_ctx_locked(struct s5p_mfc_dev *dev,
				   struct s5p_mfc_ctx *ctx)
{
	assert_spin_locked(&dev->irqlock);

	if (list_empty(&ctx->ready_ctx_list)
	    && s5p_mfc_hw_call(ctx->c_ops, ctx_ready, ctx))
		list_add_tail(&ctx->ready_ctx_list, &dev->ready_ctx_list);
}

void s5p_mfc_try_ctx(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned long flags;

	spin_lock_irqsave(&dev->irqlock, flags);
	s5p_mfc_try_ctx_locked(dev, ctx);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	s5p_mfc_try_run(dev);
}

void s5p_mfc_ctx_done_locked(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;

	assert_spin_locked(&dev->irqlock);

	dev->curr_ctx = NULL;
	list_del_init(&ctx->ready_ctx_list);
	s5p_mfc_try_ctx_locked(dev, ctx);
}
