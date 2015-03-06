/*
 * drivers/media/platform/samsung/mfc5/s5p_mfc_intr.h
 *
 * Header file for Samsung MFC (Multi Function Codec - FIMV) driver
 * It contains waiting functions declarations.
 *
 * Kamil Debski, Copyright (C) 2011 Samsung Electronics
 * http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef S5P_MFC_INTR_H_
#define S5P_MFC_INTR_H_

#include "s5p_mfc_common.h"

/**
 * s5p_mfc_wait_for_done_ctx() - wait until context finishes remaining work
 * @ctx:	Context to wait for.
 *
 * This function waits until all runs possible on given context finish,
 * i.e. the context is no longer on the list of ready contexts.
 *
 * Must be called with dev->mfc_mutex held, because it relies on the fact
 * that no further actions, such as buffer queuing, will happen at the time
 * of waiting, so that it can end in finite time.
 *
 * Return:	0 if context finished all the work successfully, negative error
 *		if error happened and context was aborted.
 */
int s5p_mfc_wait_for_done_ctx(struct s5p_mfc_ctx *ctx);

/**
 * s5p_mfc_wait_for_done_dev() - wait until hardware finishes current run
 * @dev:	Device to wait for.
 *
 * This function waits for run currently executed on hardware to finish. It
 * must be used only by low level initialization code, at the time when
 * context scheduling is disabled, i.e. power sequencing and suspend/resume.
 *
 * Return:	0 if the run finished successfully, negative error otherwise.
 */
int s5p_mfc_wait_for_done_dev(struct s5p_mfc_dev *dev);

/**
 * s5p_mfc_wake_up() - signal completion of last run to waiting clients
 * @dev:	Device on which the run just completed.
 *
 * This function should be called whenever scheduled run stopped executing,
 * either as a result of hardware interrupt or a failure handled by the driver
 * itself. Its purpose is to wake up any potential processes waiting for the
 * hardware to complete running.
 */
void s5p_mfc_wake_up(struct s5p_mfc_dev *dev);

#endif /* S5P_MFC_INTR_H_ */
