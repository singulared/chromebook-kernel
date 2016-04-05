/*
 * Samsung S5P Multi Format Codec v 5.1
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Kamil Debski, <k.debski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-event.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <media/videobuf2-core.h>
#include "s5p_mfc_common.h"
#include "s5p_mfc_ctrl.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_dec.h"
#include "s5p_mfc_enc.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_opr.h"
#include "s5p_mfc_cmd.h"
#include "s5p_mfc_pm.h"
#ifdef CONFIG_EXYNOS_IOMMU
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/kref.h>
#include <linux/of_platform.h>
#include <asm/dma-iommu.h>
#endif

#define S5P_MFC_NAME		"s5p-mfc"
#define S5P_MFC_DEC_NAME	"s5p-mfc-dec"
#define S5P_MFC_ENC_NAME	"s5p-mfc-enc"

#ifdef CONFIG_EXYNOS_IOMMU
static struct dma_iommu_mapping *mapping = NULL;
#endif

int debug;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug level - higher value produces more verbose messages");

/*
 * NOTE: This function is carefully crafted to avoid deadlocks and race
 * conditions with other processes. The comments should hopefully explain
 * why this particular sequence is necessary.
 */
static void s5p_mfc_watchdog_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct s5p_mfc_dev *dev = container_of(dwork, struct s5p_mfc_dev,
						watchdog_work);
	struct s5p_mfc_ctx *ctx;
	unsigned long flags;
	int ret;

	/*
	 * We might have the following running in parallel with
	 * this work:
	 * - s5p_mfc_open(),
	 * - s5p_mfc_release(),
	 * - various V4L2 IOCTLs.
	 *
	 * All of them acquire dev->mfc_mutex, so they are serialized.
	 * However we cannot acquire this lock from here (yet), because
	 * they might be waiting for hardware to complete certain task.
	 *
	 * s5p_mfc_irq() is assumed not to be running, since this is
	 * a timeout handler.
	 */

	mfc_err("Driver timeout error handling\n");

	/* No need for the clock running. */
	if (test_and_clear_bit(0, &dev->clk_flag))
		s5p_mfc_clock_off(dev);

	/*
	 * No further commands allowed in error state, so we can freely
	 * release dev->hw_lock and wake up global waitqueue after
	 * setting dev->hw_error.
	 *
	 * This will unblock and fail anyone potentially waiting on
	 * dev->hw_lock and make further waits non-blocking.
	 */
	set_bit(0, &dev->hw_error);
	s5p_mfc_hw_unlock(dev);

	/*
	 * Clean up all existing contexts, put them into error state
	 * and wake up anyone waiting for their completions, so they can
	 * propagate the error status and release dev->mfc_mutex.
	 */
	spin_lock_irqsave(&dev->irqlock, flags);

	dev->curr_ctx = NULL;

	list_for_each_entry(ctx, &dev->ctx_list, ctx_list) {
		/*
		 * We can spare new instances, which have not created any
		 * hardware state yet.
		 */
		if (ctx->inst_no == MFC_NO_INSTANCE_SET)
			continue;

		s5p_mfc_fatal_error(dev, ctx);
		ctx->inst_no = MFC_NO_INSTANCE_SET;
		s5p_mfc_ctx_done_locked(ctx);
	}

	s5p_mfc_wake_up(dev);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	/*
	 * At this point we should not have anyone sleeping with
	 * dev->mfc_mutex held, so we are safe to acquire it ourselves.
	 *
	 * Lock the mutex that protects open and release.
	 * This is necessary to prevent dev->num_inst from changing
	 * and serialize deinitializations and initializations.
	 */
	mutex_lock(&dev->mfc_mutex);

	/*
	 * Userspace releasing all instances might have already deinitialized
	 * the hardware (and cleared hw_error). In this case we don't have
	 * anything else to do here.
	 *
	 * Similarly, if deinit happened before we set hw_error and ended up
	 * powering down the device.
	 */
	if (!test_bit(0, &dev->hw_error) || !dev->num_inst) {
		clear_bit(0, &dev->hw_error);
		goto out;
	}

	/* De-init MFC. Will clear dev->hw_error. */
	s5p_mfc_ctrl_ops_call(dev, deinit_hw, dev);
	ret = s5p_mfc_ctrl_ops_call(dev, init_hw, dev);
	if (ret) {
		/*
		 * This can only mean that hardware has gone totally
		 * bonghits and we cannot do anything about it anymore.
		 *
		 * *SHOULD NOT* really happen, unless some external
		 * factors are affecting the MFC (e.g. buggy clock or
		 * PM domain driver).
		 */
		mfc_err("Failed to reinit FW\n");
		__s5p_mfc_hw_unlock(dev);
	}

	s5p_mfc_try_run(dev);

out:
	/* Let them go... */
	mutex_unlock(&dev->mfc_mutex);
}

static enum s5p_mfc_node_type s5p_mfc_get_node_type(struct file *file)
{
	struct video_device *vdev = video_devdata(file);

	if (!vdev) {
		mfc_err("failed to get video_device");
		return MFCNODE_INVALID;
	}
	if (vdev->index == 0)
		return MFCNODE_DECODER;
	else if (vdev->index == 1)
		return MFCNODE_ENCODER;
	return MFCNODE_INVALID;
}

static void s5p_mfc_handle_frame(struct s5p_mfc_ctx *ctx, unsigned int reason,
					unsigned int err)
{
	assert_spin_locked(&ctx->dev->irqlock);

	if (s5p_mfc_hw_call(ctx->c_ops, post_frame_start, ctx, reason, err))
		mfc_err("post_frame_start() failed\n");
}

static void s5p_mfc_handle_irq_error(struct s5p_mfc_dev *dev,
		struct s5p_mfc_ctx *ctx, unsigned int reason, unsigned int err)
{
	struct s5p_mfc_buf *src_buf;
	enum s5p_mfc_error decode_error;

	assert_spin_locked(&dev->irqlock);

	mfc_err("Interrupt Error: %08x\n", err);

	decode_error = s5p_mfc_hw_call(dev->mfc_ops, translate_error,
				s5p_mfc_hw_call(dev->mfc_ops, err_dec, err));

	/* If an error is recoverable, try to recover */
	switch (decode_error) {
	case ERR_HEADER_NOT_FOUND:
		if (ctx->state != MFCINST_GOT_INST
			&& ctx->state != MFCINST_RES_CHANGE_END) {
			mfc_err("Invalid header error in unexpected state\n");
			break;
		}

		mfc_debug(2, "Stream header not found.\n");

		/* Current source buffer did not have stream header information.
		 * Return it to userspace and continue.
		 */
		if (!list_empty(&ctx->src_queue)) {
			src_buf = list_entry(ctx->src_queue.next,
				     struct s5p_mfc_buf, list);
			list_del(&src_buf->list);
			ctx->src_queue_cnt--;
			vb2_buffer_done(src_buf->b,
					VB2_BUF_STATE_DONE);
		}
		return;

	case ERR_WARNING:
		if (ctx->state == MFCINST_RUNNING) {
			mfc_debug(2, "Continuing with decode warning\n");
			s5p_mfc_handle_frame(ctx, reason, err);
			return;
		}
		break;

	default:
		break;
	}

	/* Unrecoverable error. */
	s5p_mfc_fatal_error(dev, ctx);
}

/* Header parsing interrupt handling */
static void s5p_mfc_handle_seq_done(struct s5p_mfc_ctx *ctx)
{
	if (ctx == NULL)
		return;

	assert_spin_locked(&ctx->dev->irqlock);

	if (s5p_mfc_hw_call(ctx->c_ops, post_seq_start, ctx))
		mfc_err("post_seq_start() failed\n");
}

/* Header parsing interrupt handling */
static void s5p_mfc_handle_init_buffers(struct s5p_mfc_ctx *ctx,
					unsigned int err)
{
	struct s5p_mfc_buf *src_buf;

	if (ctx == NULL)
		return;

	assert_spin_locked(&ctx->dev->irqlock);

	if (err == 0) {
		ctx->state = MFCINST_RUNNING;
		if (!ctx->dpb_flush_flag && ctx->head_processed) {
			if (!list_empty(&ctx->src_queue)) {
				src_buf = list_entry(ctx->src_queue.next,
					     struct s5p_mfc_buf, list);
				list_del(&src_buf->list);
				ctx->src_queue_cnt--;
				vb2_buffer_done(src_buf->b,
						VB2_BUF_STATE_DONE);
			}
		} else {
			ctx->dpb_flush_flag = 0;
		}
	}
}

static void s5p_mfc_handle_stream_complete(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_buf *mb_entry;

	assert_spin_locked(&ctx->dev->irqlock);

	mfc_debug(2, "Stream completed\n");

	ctx->state = MFCINST_FINISHED;

	if (!list_empty(&ctx->dst_queue)) {
		mb_entry = list_entry(ctx->dst_queue.next, struct s5p_mfc_buf,
									list);
		list_del(&mb_entry->list);
		ctx->dst_queue_cnt--;
		vb2_set_plane_payload(mb_entry->b, 0, 0);
		vb2_buffer_done(mb_entry->b, VB2_BUF_STATE_DONE);
	}
}

static void s5p_mfc_handle_open_instance(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;

	assert_spin_locked(&ctx->dev->irqlock);

	ctx->inst_no = s5p_mfc_hw_call(dev->mfc_ops, get_inst_no, dev);
	ctx->state = MFCINST_GOT_INST;
}

static void s5p_mfc_handle_close_instance(struct s5p_mfc_ctx *ctx)
{
	assert_spin_locked(&ctx->dev->irqlock);

	ctx->inst_no = MFC_NO_INSTANCE_SET;
	ctx->state = MFCINST_FREE;
}

static void s5p_mfc_handle_dpb_flush(struct s5p_mfc_ctx *ctx)
{
	assert_spin_locked(&ctx->dev->irqlock);

	ctx->state = MFCINST_RUNNING;
}

static void s5p_mfc_handle_sys_init(struct s5p_mfc_dev *dev,
				    unsigned int err)
{
	assert_spin_locked(&dev->irqlock);

	if (err)
		set_bit(0, &dev->hw_error);
	s5p_mfc_hw_unlock(dev);

	s5p_mfc_wake_up(dev);
}

/* Interrupt processing */
static irqreturn_t s5p_mfc_irq(int irq, void *priv)
{
	struct s5p_mfc_dev *dev = priv;
	struct s5p_mfc_ctx *ctx;
	unsigned long flags;
	unsigned int reason;
	unsigned int err;

	mfc_debug_enter();

	spin_lock_irqsave(&dev->irqlock, flags);

	/* Reset the timeout watchdog */
	cancel_delayed_work(&dev->watchdog_work);
	ctx = dev->curr_ctx;
	/* Get the reason of interrupt and the error code */
	reason = s5p_mfc_hw_call(dev->mfc_ops, get_int_reason, dev);
	err = s5p_mfc_hw_call(dev->mfc_ops, get_int_err, dev);
	s5p_mfc_hw_call(dev->mfc_ops, clear_int_flags, dev);
	mfc_debug(1, "Int reason: %d (err: %08x)\n", reason, err);
	switch (reason) {
	case S5P_MFC_R2H_CMD_ERR_RET:
		s5p_mfc_handle_irq_error(dev, ctx, reason, err);
		break;

	case S5P_MFC_R2H_CMD_SLICE_DONE_RET:
	case S5P_MFC_R2H_CMD_FIELD_DONE_RET:
	case S5P_MFC_R2H_CMD_FRAME_DONE_RET:
		s5p_mfc_handle_frame(ctx, reason, err);
		break;

	case S5P_MFC_R2H_CMD_SEQ_DONE_RET:
		s5p_mfc_handle_seq_done(ctx);
		break;

	case S5P_MFC_R2H_CMD_OPEN_INSTANCE_RET:
		s5p_mfc_handle_open_instance(ctx);
		break;

	case S5P_MFC_R2H_CMD_CLOSE_INSTANCE_RET:
		s5p_mfc_handle_close_instance(ctx);
		break;

	case S5P_MFC_R2H_CMD_SYS_INIT_RET:
	case S5P_MFC_R2H_CMD_FW_STATUS_RET:
	case S5P_MFC_R2H_CMD_SLEEP_RET:
	case S5P_MFC_R2H_CMD_WAKEUP_RET:
		s5p_mfc_handle_sys_init(dev, err);
		spin_unlock_irqrestore(&dev->irqlock, flags);
		mfc_debug_leave();
		return IRQ_HANDLED;

	case S5P_MFC_R2H_CMD_INIT_BUFFERS_RET:
		s5p_mfc_handle_init_buffers(ctx, err);
		break;

	case S5P_MFC_R2H_CMD_COMPLETE_SEQ_RET:
		s5p_mfc_handle_stream_complete(ctx);
		break;

	case S5P_MFC_R2H_CMD_DPB_FLUSH_RET:
		s5p_mfc_handle_dpb_flush(ctx);
		break;

	default:
		mfc_debug(2, "Unknown int reason\n");
		spin_unlock_irqrestore(&dev->irqlock, flags);
		mfc_debug_leave();
		return IRQ_HANDLED;
	}

	if (ctx)
		s5p_mfc_ctx_done_locked(ctx);

	spin_unlock_irqrestore(&dev->irqlock, flags);

	if (test_and_clear_bit(0, &dev->clk_flag))
		s5p_mfc_clock_off(dev);
	s5p_mfc_hw_unlock(dev);
	s5p_mfc_wake_up(dev);
	s5p_mfc_try_run(dev);

	mfc_debug_leave();
	return IRQ_HANDLED;
}

static int s5p_mfc_init_vb2_queue(struct vb2_queue *q, void *priv,
		enum v4l2_buf_type buf_type, enum s5p_mfc_node_type node_type)
{
	int ret = 0;

	q->type = buf_type;
	q->drv_priv = priv;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	if (node_type == MFCNODE_DECODER)
		q->ops = get_dec_queue_ops();
	else if (node_type == MFCNODE_ENCODER)
		q->ops = get_enc_queue_ops();

	q->mem_ops = (struct vb2_mem_ops *)&vb2_dma_contig_memops;
	ret = vb2_queue_init(q);
	if (ret) {
		mfc_err("Failed to initialize videobuf2 queue type %d\n",
				buf_type);
	}
	return ret;
}

static int s5p_mfc_startup(struct s5p_mfc_dev *dev)
{
	int ret;

	ret = s5p_mfc_power_on();
	if (ret < 0) {
		mfc_err("power on failed\n");
		return ret;
	}

	/* Init the FW */
	ret = s5p_mfc_ctrl_ops_call(dev, init_hw, dev);
	if (ret)
		goto err_init_hw;

	return 0;

err_init_hw:
	if (s5p_mfc_power_off() < 0)
		mfc_err("power off failed\n");
	s5p_mfc_clock_off(dev);
	__s5p_mfc_hw_unlock(dev);
	clear_bit(0, &dev->hw_error);

	return ret;
}

static void s5p_mfc_shutdown(struct s5p_mfc_dev *dev)
{
	s5p_mfc_ctrl_ops_call(dev, deinit_hw, dev);
	if (s5p_mfc_power_off() < 0)
		mfc_err("Power off failed\n");
}

/* Open an MFC node */
static int s5p_mfc_open(struct file *file)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	enum s5p_mfc_node_type node_type;
	struct s5p_mfc_ctx *ctx = NULL;
	unsigned long flags;
	int ret = 0;

	mfc_debug_enter();

	node_type = s5p_mfc_get_node_type(file);
	if (node_type != MFCNODE_DECODER && node_type != MFCNODE_ENCODER)
		return -ENOENT;

	if (mutex_lock_interruptible(&dev->mfc_mutex))
		return -ERESTARTSYS;
	if (dev->num_inst == MFC_NUM_CONTEXTS) {
		mfc_err("Too many open contexts\n");
		ret = -EBUSY;
		goto err_no_ctx;
	}
	dev->num_inst++;	/* It is guarded by mfc_mutex in vfd */
	/* Allocate memory for context */
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		mfc_err("Not enough memory\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);
	ctx->dev = dev;
	INIT_LIST_HEAD(&ctx->src_queue);
	INIT_LIST_HEAD(&ctx->dst_queue);
	INIT_LIST_HEAD(&ctx->ready_ctx_list);
	ctx->src_queue_cnt = 0;
	ctx->dst_queue_cnt = 0;
	/* Mark context as idle */
	if (node_type == MFCNODE_DECODER) {
		ctx->type = MFCINST_DECODER;
		ctx->c_ops = get_dec_codec_ops();
		ret = s5p_mfc_dec_init(ctx);
		if (ret) {
			mfc_err("Failed to init mfc decoder\n");
			goto err_fh_del;
		}
		/* Setup ctrl handler */
		ret = s5p_mfc_dec_ctrls_setup(ctx);
		if (ret) {
			mfc_err("Failed to setup mfc controls\n");
			goto err_ctrls_setup;
		}
	} else if (node_type == MFCNODE_ENCODER) {
		ctx->type = MFCINST_ENCODER;
		ctx->c_ops = get_enc_codec_ops();
		/* only for encoder */
		INIT_LIST_HEAD(&ctx->ref_queue);
		ctx->ref_queue_cnt = 0;
		ret = s5p_mfc_enc_init(ctx);
		if (ret) {
			mfc_err("Failed to init mfc encoder\n");
			goto err_fh_del;
		}
		/* Setup ctrl handler */
		ret = s5p_mfc_enc_ctrls_setup(ctx);
		if (ret) {
			mfc_err("Failed to setup mfc controls\n");
			goto err_ctrls_setup;
		}
	}

	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	ctx->inst_no = MFC_NO_INSTANCE_SET;
	/* Load firmware if this is the first instance */
	if (dev->num_inst == 1) {
		ret = s5p_mfc_startup(dev);
		if (ret)
			goto err_ctrls_setup;
	}
	/* Init videobuf2 queues */
	if (s5p_mfc_init_vb2_queue(&ctx->vq_dst, &ctx->fh,
			V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, node_type)
		|| s5p_mfc_init_vb2_queue(&ctx->vq_src, &ctx->fh,
			V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, node_type))
		goto err_shutdown;

	spin_lock_irqsave(&dev->irqlock, flags);
	list_add_tail(&ctx->ctx_list, &dev->ctx_list);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	mutex_unlock(&dev->mfc_mutex);
	mfc_debug_leave();
	return ret;

	/* Deinit when failure occured */
err_shutdown:
	if (dev->num_inst == 1)
		s5p_mfc_shutdown(dev);
err_ctrls_setup:
	s5p_mfc_dec_ctrls_delete(ctx);
err_fh_del:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
err_alloc:
	dev->num_inst--;
err_no_ctx:
	mutex_unlock(&dev->mfc_mutex);
	mfc_debug_leave();
	return ret;
}

/* Release MFC context */
static int s5p_mfc_release(struct file *file)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(file->private_data);
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned long flags;

	mfc_debug_enter();
	mutex_lock(&dev->mfc_mutex);
	vb2_queue_release(&ctx->vq_src);
	vb2_queue_release(&ctx->vq_dst);
	/* If instance was initialised and not yet freed,
	 * return instance and free resources */
	s5p_mfc_close_mfc_inst(dev, ctx);

	spin_lock_irqsave(&dev->irqlock, flags);
	if (WARN_ON(dev->curr_ctx == ctx)) {
		/*
		 * This should not happen, but in case there is a bug still
		 * hiding somewhere in the code let's try to keep the system
		 * kind of running.
		 */
		s5p_mfc_hw_unlock(dev);
		dev->curr_ctx = NULL;
	}
	list_del(&ctx->ctx_list);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	dev->num_inst--;
	if (dev->num_inst == 0) {
		mfc_debug(2, "Last instance\n");
		s5p_mfc_shutdown(dev);
	}
	mfc_debug(2, "Shutting down clock\n");
	s5p_mfc_dec_ctrls_delete(ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	mfc_debug_leave();
	mutex_unlock(&dev->mfc_mutex);
	return 0;
}

/* Poll */
static unsigned int s5p_mfc_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(file->private_data);
	struct s5p_mfc_dev *dev = ctx->dev;
	struct vb2_queue *src_q, *dst_q;
	struct vb2_buffer *src_vb = NULL, *dst_vb = NULL;
	unsigned int rc = 0;
	unsigned long flags;

	mutex_lock(&dev->mfc_mutex);
	src_q = &ctx->vq_src;
	dst_q = &ctx->vq_dst;
	/*
	 * There has to be at least one buffer queued on each queued_list, which
	 * means either in driver already or waiting for driver to claim it
	 * and start processing.
	 */
	if ((!vb2_is_streaming(src_q) || list_empty(&src_q->queued_list)) &&
	    (!vb2_is_streaming(dst_q) || list_empty(&dst_q->queued_list))) {
		rc = POLLERR;
		goto end;
	}
	mutex_unlock(&dev->mfc_mutex);
	poll_wait(file, &ctx->fh.wait, wait);
	poll_wait(file, &src_q->done_wq, wait);
	poll_wait(file, &dst_q->done_wq, wait);
	mutex_lock(&dev->mfc_mutex);
	if (v4l2_event_pending(&ctx->fh))
		rc |= POLLPRI;
	spin_lock_irqsave(&src_q->done_lock, flags);
	if (!list_empty(&src_q->done_list))
		src_vb = list_first_entry(&src_q->done_list, struct vb2_buffer,
								done_entry);
	if (src_vb && (src_vb->state == VB2_BUF_STATE_DONE
				|| src_vb->state == VB2_BUF_STATE_ERROR))
		rc |= POLLOUT | POLLWRNORM;
	spin_unlock_irqrestore(&src_q->done_lock, flags);
	spin_lock_irqsave(&dst_q->done_lock, flags);
	if (!list_empty(&dst_q->done_list))
		dst_vb = list_first_entry(&dst_q->done_list, struct vb2_buffer,
								done_entry);
	if (dst_vb && (dst_vb->state == VB2_BUF_STATE_DONE
				|| dst_vb->state == VB2_BUF_STATE_ERROR))
		rc |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&dst_q->done_lock, flags);
end:
	mutex_unlock(&dev->mfc_mutex);
	return rc;
}

/* Mmap */
static int s5p_mfc_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(file->private_data);
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret;

	if (mutex_lock_interruptible(&dev->mfc_mutex))
		return -ERESTARTSYS;
	if (offset < DST_QUEUE_OFF_BASE) {
		mfc_debug(2, "mmaping source\n");
		ret = vb2_mmap(&ctx->vq_src, vma);
	} else {		/* capture */
		mfc_debug(2, "mmaping destination\n");
		vma->vm_pgoff -= (DST_QUEUE_OFF_BASE >> PAGE_SHIFT);
		ret = vb2_mmap(&ctx->vq_dst, vma);
	}
	mutex_unlock(&dev->mfc_mutex);
	return ret;
}

/* v4l2 ops */
static const struct v4l2_file_operations s5p_mfc_fops = {
	.owner = THIS_MODULE,
	.open = s5p_mfc_open,
	.release = s5p_mfc_release,
	.poll = s5p_mfc_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = s5p_mfc_mmap,
};

static int match_child(struct device *dev, void *data)
{
	if (!dev_name(dev))
		return 0;
	return !strcmp(dev_name(dev), (char *)data);
}

static void *mfc_get_drv_data(struct platform_device *pdev);

#ifdef CONFIG_EXYNOS_IOMMU
static int s5p_mfc_alloc_memdevs_iommu(struct s5p_mfc_dev *dev)
{
	struct device *mdev = &dev->plat_dev->dev;
	int ret;

	mapping = arm_iommu_create_mapping(&platform_bus_type, 0x20000000,
			SZ_512M, 4);
	if (IS_ERR_OR_NULL(mapping)) {
		mfc_err("IOMMU mapping failed\n");
		ret = mapping ? PTR_ERR(mapping) : -ENOMEM;
		goto out;
	}
	mdev->dma_parms = devm_kzalloc(&dev->plat_dev->dev,
			sizeof(*mdev->dma_parms), GFP_KERNEL);
	dma_set_max_seg_size(mdev, 0xffffffffu);
	ret = arm_iommu_attach_device(mdev, mapping);
out:
	return ret;
}

#else
static int s5p_mfc_alloc_memdevs_noiommu(struct s5p_mfc_dev *dev)
{
	unsigned int mem_info[2] = { };

	dev->mem_dev_l = devm_kzalloc(&dev->plat_dev->dev,
			sizeof(struct device), GFP_KERNEL);
	if (!dev->mem_dev_l) {
		mfc_err("Not enough memory\n");
		return -ENOMEM;
	}
	device_initialize(dev->mem_dev_l);
	of_property_read_u32_array(dev->plat_dev->dev.of_node,
			"samsung,mfc-l", mem_info, 2);
	if (dma_declare_coherent_memory(dev->mem_dev_l, mem_info[0],
				mem_info[0], mem_info[1],
				DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE) == 0) {
		mfc_err("Failed to declare coherent memory for\n"
		"MFC device\n");
		return -ENOMEM;
	}

	dev->mem_dev_r = devm_kzalloc(&dev->plat_dev->dev,
			sizeof(struct device), GFP_KERNEL);
	if (!dev->mem_dev_r) {
		mfc_err("Not enough memory\n");
		return -ENOMEM;
	}
	device_initialize(dev->mem_dev_r);
	of_property_read_u32_array(dev->plat_dev->dev.of_node,
			"samsung,mfc-r", mem_info, 2);
	if (dma_declare_coherent_memory(dev->mem_dev_r, mem_info[0],
				mem_info[0], mem_info[1],
				DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE) == 0) {
		pr_err("Failed to declare coherent memory for\n"
		"MFC device\n");
		return -ENOMEM;
	}
	return 0;
}
#endif

static int s5p_mfc_alloc_memdevs(struct s5p_mfc_dev *dev)
{
#ifdef CONFIG_EXYNOS_IOMMU
	return s5p_mfc_alloc_memdevs_iommu(dev);
#else
	return s5p_mfc_alloc_memdevs_noiommu(dev);
#endif
}

#ifdef CONFIG_EXYNOS_IOMMU
void s5p_mfc_cleanup_memdevs_iommu(struct platform_device *pdev)
{
	if (mapping) {
		arm_iommu_detach_device(&pdev->dev);
		arm_iommu_release_mapping(mapping);
	}
}

#else
void s5p_mfc_cleanup_memdevs_noiommu(struct platform_device *pdev)
{
	struct s5p_mfc_dev *dev = platform_get_drvdata(pdev);
	if (pdev->dev.of_node) {
		put_device(dev->mem_dev_l);
		put_device(dev->mem_dev_r);
	}
}
#endif

void s5p_mfc_cleanup_memdevs(struct platform_device *pdev)
{
#ifdef CONFIG_EXYNOS_IOMMU
	s5p_mfc_cleanup_memdevs_iommu(pdev);
#else
	s5p_mfc_cleanup_memdevs_noiommu(pdev);
#endif
}

/* MFC probe function */
static int s5p_mfc_probe(struct platform_device *pdev)
{
	DEFINE_DMA_ATTRS(attrs);
	struct s5p_mfc_dev *dev;
	struct video_device *vfd;
	struct resource *res;
	int ret;

	pr_debug("%s++\n", __func__);
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Not enough memory for MFC device\n");
		return -ENOMEM;
	}

	spin_lock_init(&dev->irqlock);
	INIT_LIST_HEAD(&dev->ready_ctx_list);
	INIT_LIST_HEAD(&dev->ctx_list);
	dev->plat_dev = pdev;
	if (!dev->plat_dev) {
		dev_err(&pdev->dev, "No platform data specified\n");
		return -ENODEV;
	}

	dev->variant = mfc_get_drv_data(pdev);

	ret = s5p_mfc_init_pm(dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get mfc clock source\n");
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	dev->regs_base = devm_request_and_ioremap(&pdev->dev, res);
	if (dev->regs_base == NULL) {
		dev_err(&pdev->dev, "Failed to obtain io memory\n");
		ret = -ENOENT;
		goto err_res;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get irq resource\n");
		ret = -ENOENT;
		goto err_res;
	}
	dev->irq = res->start;
	ret = devm_request_irq(&pdev->dev, dev->irq, s5p_mfc_irq,
					IRQF_DISABLED, pdev->name, dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to install irq (%d)\n", ret);
		goto err_res;
	}

	if (pdev->dev.of_node) {
		ret = s5p_mfc_alloc_memdevs(dev);
		if (ret) {
			dev_err(&pdev->dev, "Failed to create device memory\n");
			goto err_res;
		}
	} else {
		dev->mem_dev_l = device_find_child(&dev->plat_dev->dev,
				"s5p-mfc-l", match_child);
		if (!dev->mem_dev_l) {
			mfc_err("Mem child (L) device get failed\n");
			ret = -ENODEV;
			goto err_res;
		}
		dev->mem_dev_r = device_find_child(&dev->plat_dev->dev,
				"s5p-mfc-r", match_child);
		if (!dev->mem_dev_r) {
			mfc_err("Mem child (R) device get failed\n");
			ret = -ENODEV;
			goto err_res;
		}
	}

#ifdef CONFIG_EXYNOS_IOMMU
	dev->mem_dev_l = &dev->plat_dev->dev;
	dev->mem_dev_r = &dev->plat_dev->dev;
#endif
	/*
	 * We'll do mostly sequential access, so sacrifice TLB efficiency for
	 * faster allocation.
	 */
	dma_set_attr(DMA_ATTR_ALLOC_SINGLE_PAGES, &attrs);

	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);

	dev->alloc_ctx[0] = vb2_dma_contig_init_ctx_attrs(dev->mem_dev_l,
							  &attrs);
	if (IS_ERR(dev->alloc_ctx[0])) {
		ret = PTR_ERR(dev->alloc_ctx[0]);
		goto err_cleanupmemdevs;
	}

	dev->alloc_ctx[1] = vb2_dma_contig_init_ctx_attrs(dev->mem_dev_r,
							  &attrs);
	if (IS_ERR(dev->alloc_ctx[1])) {
		ret = PTR_ERR(dev->alloc_ctx[1]);
		goto err_mem_init_ctx_1;
	}
	mutex_init(&dev->mfc_mutex);

	ret = s5p_mfc_alloc_firmware(dev);
	if (ret)
		goto err_alloc_fw;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto err_v4l2_dev_reg;
	init_waitqueue_head(&dev->queue);

	/* decoder */
	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&dev->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto err_dec_alloc;
	}
	vfd->fops	= &s5p_mfc_fops,
	vfd->ioctl_ops	= get_dec_v4l2_ioctl_ops();
	vfd->release	= video_device_release,
	vfd->lock	= &dev->mfc_mutex;
	vfd->v4l2_dev	= &dev->v4l2_dev;
	vfd->vfl_dir	= VFL_DIR_M2M;
	snprintf(vfd->name, sizeof(vfd->name), "%s", S5P_MFC_DEC_NAME);
	dev->vfd_dec	= vfd;
	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 10);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		video_device_release(vfd);
		goto err_dec_reg;
	}
	v4l2_info(&dev->v4l2_dev,
		  "decoder registered as /dev/video%d\n", vfd->num);
	video_set_drvdata(vfd, dev);

	/* encoder */
	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&dev->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto err_enc_alloc;
	}
	vfd->fops	= &s5p_mfc_fops,
	vfd->ioctl_ops	= get_enc_v4l2_ioctl_ops();
	vfd->release	= video_device_release,
	vfd->lock	= &dev->mfc_mutex;
	vfd->v4l2_dev	= &dev->v4l2_dev;
	vfd->vfl_dir	= VFL_DIR_M2M;
	snprintf(vfd->name, sizeof(vfd->name), "%s", S5P_MFC_ENC_NAME);
	dev->vfd_enc	= vfd;
	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 10);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		video_device_release(vfd);
		goto err_enc_reg;
	}
	v4l2_info(&dev->v4l2_dev,
		  "encoder registered as /dev/video%d\n", vfd->num);
	video_set_drvdata(vfd, dev);
	platform_set_drvdata(pdev, dev);

	dev->clk_flag = 0;
	INIT_DELAYED_WORK(&dev->watchdog_work, s5p_mfc_watchdog_worker);

	/* Initialize HW ops and commands based on MFC version */
	s5p_mfc_init_hw_ops(dev);
	s5p_mfc_init_hw_cmds(dev);
	s5p_mfc_init_regs(dev);
	s5p_mfc_init_hw_ctrls(dev);

	pr_debug("%s--\n", __func__);
	return 0;

/* Deinit MFC if probe had failed */
err_enc_reg:
	video_device_release(dev->vfd_enc);
err_enc_alloc:
	video_unregister_device(dev->vfd_dec);
err_dec_reg:
	video_device_release(dev->vfd_dec);
err_dec_alloc:
	v4l2_device_unregister(&dev->v4l2_dev);
err_v4l2_dev_reg:
	s5p_mfc_release_firmware(dev);
err_alloc_fw:
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx[1]);
err_mem_init_ctx_1:
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx[0]);
err_cleanupmemdevs:
	s5p_mfc_cleanup_memdevs(pdev);
err_res:
	s5p_mfc_final_pm(dev);
	pr_debug("%s-- with error\n", __func__);
	return ret;

}

/* Remove the driver */
static int s5p_mfc_remove(struct platform_device *pdev)
{
	struct s5p_mfc_dev *dev = platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing %s\n", pdev->name);

	cancel_delayed_work_sync(&dev->watchdog_work);

	video_unregister_device(dev->vfd_enc);
	video_unregister_device(dev->vfd_dec);
	v4l2_device_unregister(&dev->v4l2_dev);
	s5p_mfc_release_firmware(dev);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx[0]);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx[1]);

	s5p_mfc_cleanup_memdevs(pdev);
	s5p_mfc_final_pm(dev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int s5p_mfc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s5p_mfc_dev *m_dev = platform_get_drvdata(pdev);
	int ret;

	if (m_dev->num_inst == 0)
		return 0;

	if (test_and_set_bit(0, &m_dev->enter_suspend) != 0) {
		mfc_err("Error: going to suspend for a second time\n");
		return -EIO;
	}

	/*
	 * Wait for run being currently processed if necessary.
	 *
	 * Since s5p_mfc_try_run() checks status of enter_suspend flag,
	 * if we acquire hw_lock no other thread will be able to schedule
	 * further runs.
	 *
	 * Timeout is handled by watchdog timer.
	 */
	wait_event(m_dev->queue, !s5p_mfc_hw_trylock(m_dev));
	s5p_mfc_hw_unlock(m_dev);

	ret = s5p_mfc_ctrl_ops_call(m_dev, sleep, m_dev);
	if (ret) {
		clear_bit(0, &m_dev->enter_suspend);
		__s5p_mfc_hw_unlock(m_dev);
		return ret;
	}

	return 0;
}

static int s5p_mfc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s5p_mfc_dev *m_dev = platform_get_drvdata(pdev);
	int ret;

	if (m_dev->num_inst == 0)
		return 0;

	ret = s5p_mfc_ctrl_ops_call(m_dev, wakeup, m_dev);
	clear_bit(0, &m_dev->enter_suspend);
	if (ret) {
		__s5p_mfc_hw_unlock(m_dev);
		return ret;
	}

	s5p_mfc_try_run(m_dev);
	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int s5p_mfc_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s5p_mfc_dev *m_dev = platform_get_drvdata(pdev);

	atomic_set(&m_dev->pm.power, 0);
	return 0;
}

static int s5p_mfc_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s5p_mfc_dev *m_dev = platform_get_drvdata(pdev);
	int pre_power;

	if (!m_dev->alloc_ctx)
		return 0;
	pre_power = atomic_read(&m_dev->pm.power);
	atomic_set(&m_dev->pm.power, 1);
	return 0;
}
#endif

/* Power management */
static const struct dev_pm_ops s5p_mfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(s5p_mfc_suspend, s5p_mfc_resume)
	SET_RUNTIME_PM_OPS(s5p_mfc_runtime_suspend, s5p_mfc_runtime_resume,
			   NULL)
};

struct s5p_mfc_buf_size_v5 mfc_buf_size_v5 = {
	.h264_ctx	= MFC_H264_CTX_BUF_SIZE,
	.non_h264_ctx	= MFC_CTX_BUF_SIZE,
	.dsc		= DESC_BUF_SIZE,
	.shm		= SHARED_BUF_SIZE,
};

struct s5p_mfc_buf_size buf_size_v5 = {
	.fw	= MAX_FW_SIZE,
	.cpb	= MAX_CPB_SIZE,
	.priv	= &mfc_buf_size_v5,
};

struct s5p_mfc_buf_align mfc_buf_align_v5 = {
	.base = MFC_BASE_ALIGN_ORDER,
};

static u32 mfc_def_fmt_v5[4] = {
	[SRC_FMT_DEC] = V4L2_PIX_FMT_H264,
	[DST_FMT_DEC] = V4L2_PIX_FMT_NV12MT,
	[SRC_FMT_ENC] = V4L2_PIX_FMT_NV12MT,
	[DST_FMT_ENC] = V4L2_PIX_FMT_H264,
};

static struct s5p_mfc_variant mfc_drvdata_v5 = {
	.version	= MFC_VERSION,
	.version_bit	= MFC_V5_BIT,
	.port_num	= MFC_NUM_PORTS,
	.buf_size	= &buf_size_v5,
	.buf_align	= &mfc_buf_align_v5,
	.def_fmt	= mfc_def_fmt_v5,
	.fw_name	= "s5p-mfc.fw",
};

struct s5p_mfc_buf_size_v6 mfc_buf_size_v6 = {
	.dev_ctx	= MFC_CTX_BUF_SIZE_V6,
	.h264_dec_ctx	= MFC_H264_DEC_CTX_BUF_SIZE_V6,
	.other_dec_ctx	= MFC_OTHER_DEC_CTX_BUF_SIZE_V6,
	.h264_enc_ctx	= MFC_H264_ENC_CTX_BUF_SIZE_V6,
	.other_enc_ctx	= MFC_OTHER_ENC_CTX_BUF_SIZE_V6,
};

struct s5p_mfc_buf_size buf_size_v6 = {
	.fw	= MAX_FW_SIZE_V6,
	.cpb	= MAX_CPB_SIZE_V6,
	.priv	= &mfc_buf_size_v6,
};

struct s5p_mfc_buf_align mfc_buf_align_v6 = {
	.base = 0,
};

static u32 mfc_def_fmt_v6[4] = {
	[SRC_FMT_DEC] = V4L2_PIX_FMT_H264,
	[DST_FMT_DEC] = V4L2_PIX_FMT_NV12M,
	[SRC_FMT_ENC] = V4L2_PIX_FMT_NV12M,
	[DST_FMT_ENC] = V4L2_PIX_FMT_H264,
};

static struct s5p_mfc_variant mfc_drvdata_v6 = {
	.version	= MFC_VERSION_V6,
	.version_bit	= MFC_V6_BIT,
	.port_num	= MFC_NUM_PORTS_V6,
	.buf_size	= &buf_size_v6,
	.buf_align	= &mfc_buf_align_v6,
	.def_fmt	= mfc_def_fmt_v6,
	.fw_name        = "s5p-mfc-v6.fw",
};

struct s5p_mfc_buf_size_v6 mfc_buf_size_v7 = {
	.dev_ctx	= MFC_CTX_BUF_SIZE_V7,
	.h264_dec_ctx	= MFC_H264_DEC_CTX_BUF_SIZE_V7,
	.other_dec_ctx	= MFC_OTHER_DEC_CTX_BUF_SIZE_V7,
	.h264_enc_ctx	= MFC_H264_ENC_CTX_BUF_SIZE_V7,
	.other_enc_ctx	= MFC_OTHER_ENC_CTX_BUF_SIZE_V7,
};

struct s5p_mfc_buf_size buf_size_v7 = {
	.fw	= MAX_FW_SIZE_V7,
	.cpb	= MAX_CPB_SIZE_V7,
	.priv	= &mfc_buf_size_v7,
};

struct s5p_mfc_buf_align mfc_buf_align_v7 = {
	.base = 0,
};

static struct s5p_mfc_variant mfc_drvdata_v7 = {
	.version	= MFC_VERSION_V7,
	.version_bit	= MFC_V7_BIT,
	.port_num	= MFC_NUM_PORTS_V7,
	.buf_size	= &buf_size_v7,
	.buf_align	= &mfc_buf_align_v7,
	.def_fmt	= mfc_def_fmt_v6,
	.fw_name        = "s5p-mfc-v7.fw",
};

struct s5p_mfc_buf_size_v6 mfc_buf_size_v8 = {
	.dev_ctx	= MFC_CTX_BUF_SIZE_V8,
	.h264_dec_ctx	= MFC_H264_DEC_CTX_BUF_SIZE_V8,
	.other_dec_ctx	= MFC_OTHER_DEC_CTX_BUF_SIZE_V8,
	.h264_enc_ctx	= MFC_H264_ENC_CTX_BUF_SIZE_V8,
	.other_enc_ctx	= MFC_OTHER_ENC_CTX_BUF_SIZE_V8,
};

struct s5p_mfc_buf_size buf_size_v8 = {
	.fw	= MAX_FW_SIZE_V8,
	.cpb	= MAX_CPB_SIZE_V8,
	.priv	= &mfc_buf_size_v8,
};

struct s5p_mfc_buf_align mfc_buf_align_v8 = {
	.base = 0,
};

static struct s5p_mfc_variant mfc_drvdata_v8 = {
	.version	= MFC_VERSION_V8,
	.version_bit	= MFC_V8_BIT,
	.port_num	= MFC_NUM_PORTS_V8,
	.buf_size	= &buf_size_v8,
	.buf_align	= &mfc_buf_align_v8,
	.def_fmt	= mfc_def_fmt_v6,
	.fw_name        = "s5p-mfc-v8.fw",
};

static struct platform_device_id mfc_driver_ids[] = {
	{
		.name = "s5p-mfc",
		.driver_data = (unsigned long)&mfc_drvdata_v5,
	}, {
		.name = "s5p-mfc-v5",
		.driver_data = (unsigned long)&mfc_drvdata_v5,
	}, {
		.name = "s5p-mfc-v6",
		.driver_data = (unsigned long)&mfc_drvdata_v6,
	}, {
		.name = "s5p-mfc-v7",
		.driver_data = (unsigned long)&mfc_drvdata_v7,
	}, {
		.name = "s5p-mfc-v8",
		.driver_data = (unsigned long)&mfc_drvdata_v8,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, mfc_driver_ids);

static const struct of_device_id exynos_mfc_match[] = {
	{
		.compatible = "samsung,mfc-v5",
		.data = &mfc_drvdata_v5,
	}, {
		.compatible = "samsung,mfc-v6",
		.data = &mfc_drvdata_v6,
	}, {
		.compatible = "samsung,mfc-v7",
		.data = &mfc_drvdata_v7,
	}, {
		.compatible = "samsung,mfc-v8",
		.data = &mfc_drvdata_v8,
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_mfc_match);

static void *mfc_get_drv_data(struct platform_device *pdev)
{
	struct s5p_mfc_variant *driver_data = NULL;

	if (pdev->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_node(of_match_ptr(exynos_mfc_match),
				pdev->dev.of_node);
		if (match)
			driver_data = (struct s5p_mfc_variant *)match->data;
	} else {
		driver_data = (struct s5p_mfc_variant *)
			platform_get_device_id(pdev)->driver_data;
	}
	return driver_data;
}

static struct platform_driver s5p_mfc_driver = {
	.probe		= s5p_mfc_probe,
	.remove		= s5p_mfc_remove,
	.id_table	= mfc_driver_ids,
	.driver	= {
		.name	= S5P_MFC_NAME,
		.owner	= THIS_MODULE,
		.pm	= &s5p_mfc_pm_ops,
		.of_match_table = exynos_mfc_match,
	},
};

module_platform_driver(s5p_mfc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kamil Debski <k.debski@samsung.com>");
MODULE_DESCRIPTION("Samsung S5P Multi Format Codec V4L2 driver");

