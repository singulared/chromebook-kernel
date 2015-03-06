/*
 * drivers/media/platform/samsung/mfc5/s5p_mfc_intr.c
 *
 * C file for Samsung MFC (Multi Function Codec - FIMV) driver
 * This file contains functions used to wait for command completion.
 *
 * Kamil Debski, Copyright (C) 2011 Samsung Electronics Co., Ltd.
 * http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include "s5p_mfc_common.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_intr.h"

int s5p_mfc_wait_for_done_dev(struct s5p_mfc_dev *dev)
{
	int ret;

	ret = wait_event_timeout(dev->queue, !test_bit(0, &dev->hw_lock),
					msecs_to_jiffies(MFC_INT_TIMEOUT));
	if (!ret || test_bit(0, &dev->hw_error)) {
		mfc_err("Waiting for dev failed\n");
		return -EIO;
	}
	mfc_debug(1, "Finished waiting for dev\n");
	return 0;
}

static inline bool s5p_mfc_ctx_idle(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned long flags;
	bool idle;

	spin_lock_irqsave(&dev->irqlock, flags);
	idle = list_empty(&ctx->ready_ctx_list)
		|| test_bit(0, &dev->hw_error);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	return idle;
}

int s5p_mfc_wait_for_done_ctx(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	bool error;

	WARN_ON(!mutex_is_locked(&dev->mfc_mutex));

	wait_event(dev->queue, s5p_mfc_ctx_idle(ctx));

	error = ctx->state == MFCINST_ERROR
		|| test_bit(0, &dev->hw_error);

	if (error) {
		mfc_err("Waiting for ctx %p ended with error\n", ctx);
		return -EIO;
	}
	mfc_debug(1, "Finished waiting for ctx %p\n", ctx);
	return 0;
}

void s5p_mfc_wake_up(struct s5p_mfc_dev *dev)
{
	wake_up_all(&dev->queue);
}

