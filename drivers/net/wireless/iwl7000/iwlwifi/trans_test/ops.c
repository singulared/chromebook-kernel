/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 * Copyright(c) 2015 Intel Deutschland GmbH
 * Copyright (C) 2018 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 * Copyright (C) 2018 Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>

#include "iwl-drv.h"
#include "iwl-prph.h"
#include "iwl-csr.h"
#include "tester_dev.h"
#ifdef CPTCFG_IWLWIFI_DEVICE_TESTMODE
#include "fw/testmode.h"
#endif

#define DRV_DESCRIPTION	"The new Intel(R) wireless AGN driver for Linux"
MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_AUTHOR(DRV_COPYRIGHT " " DRV_AUTHOR);
MODULE_LICENSE("GPL");

#define ERROR_START_OFFSET  (1 * sizeof(u32))
#define ERROR_ELEM_SIZE     (7 * sizeof(u32))

static struct {
	char *name;
	u8 num;
} advanced_lookup[] = {
	{ "NMI_INTERRUPT_WDG", 0x34 },
	{ "SYSASSERT", 0x35 },
	{ "UCODE_VERSION_MISMATCH", 0x37 },
	{ "BAD_COMMAND", 0x38 },
	{ "NMI_INTERRUPT_DATA_ACTION_PT", 0x3C },
	{ "FATAL_ERROR", 0x3D },
	{ "NMI_TRM_HW_ERR", 0x46 },
	{ "NMI_INTERRUPT_TRM", 0x4C },
	{ "NMI_INTERRUPT_BREAK_POINT", 0x54 },
	{ "NMI_INTERRUPT_WDG_RXF_FULL", 0x5C },
	{ "NMI_INTERRUPT_WDG_NO_RBD_RXF_FULL", 0x64 },
	{ "NMI_INTERRUPT_HOST", 0x66 },
	{ "NMI_INTERRUPT_ACTION_PT", 0x7C },
	{ "NMI_INTERRUPT_UNKNOWN", 0x84 },
	{ "NMI_INTERRUPT_INST_ACTION_PT", 0x86 },
	{ "ADVANCED_SYSASSERT", 0 },
};

/* Please keep this array *SORTED* by hex value.
 * Access is done through binary search.
 * A warning will be triggered on violation.
 */
static const struct iwl_hcmd_names iwl_trans_test_cmd_names[] = {
	HCMD_NAME(REPLY_ALIVE),
	HCMD_NAME(REPLY_ERROR),
	HCMD_NAME(REPLY_ECHO),
	HCMD_NAME(INIT_COMPLETE_NOTIF),
	HCMD_NAME(REPLY_TRANS_TEST_CMD),
	HCMD_NAME(REPLY_TRANS_TEST_TX),
	HCMD_NAME(REPLY_TRANS_TEST_RX),
	HCMD_NAME(REPLY_DEBUG_LOG_MSG),
};

static const struct iwl_hcmd_arr iwl_trans_test_cmd_groups[] = {
	[0x0] = HCMD_ARR(iwl_trans_test_cmd_names),
};

static const char *desc_lookup(u32 num)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(advanced_lookup) - 1; i++)
		if (advanced_lookup[i].num == num)
			return advanced_lookup[i].name;

	/* No entry matches 'num', so it is the last: ADVANCED_SYSASSERT */
	return advanced_lookup[i].name;
}

static void
iwl_trans_test_dump_nic_error_log(struct iwl_test_op_mode *test_op_mode)
{
	struct iwl_trans *trans = test_op_mode->trans;
	u32 base;
	struct iwl_error_event_table table;

	base = test_op_mode->error_event_table;
	if (test_op_mode->cur_ucode == IWL_UCODE_INIT) {
		if (!base)
			base = test_op_mode->fw->init_errlog_ptr;
	} else {
		if (!base)
			base = test_op_mode->fw->inst_errlog_ptr;
	}

	iwl_trans_read_mem_bytes(trans, base, &table, sizeof(table));

	if (ERROR_START_OFFSET <= table.valid * ERROR_ELEM_SIZE) {
		IWL_ERR(trans, "Start IWL Error Log Dump:\n");
		IWL_ERR(trans, "Status: 0x%08lX, count: %d\n",
			test_op_mode->status, table.valid);
	}

	IWL_ERR(test_op_mode, "0x%08X | %-28s\n", table.error_id,
		desc_lookup(table.error_id));
	IWL_ERR(test_op_mode, "0x%08X | uPc\n", table.pc);
	IWL_ERR(test_op_mode, "0x%08X | branchlink1\n", table.blink1);
	IWL_ERR(test_op_mode, "0x%08X | branchlink2\n", table.blink2);
	IWL_ERR(test_op_mode, "0x%08X | interruptlink1\n", table.ilink1);
	IWL_ERR(test_op_mode, "0x%08X | interruptlink2\n", table.ilink2);
	IWL_ERR(test_op_mode, "0x%08X | data1\n", table.data1);
	IWL_ERR(test_op_mode, "0x%08X | data2\n", table.data2);
	IWL_ERR(test_op_mode, "0x%08X | line\n", table.line);
	IWL_ERR(test_op_mode, "0x%08X | beacon time\n", table.bcon_time);
	IWL_ERR(test_op_mode, "0x%08X | tsf low\n", table.tsf_low);
	IWL_ERR(test_op_mode, "0x%08X | tsf hi\n", table.tsf_hi);
	IWL_ERR(test_op_mode, "0x%08X | time gp1\n", table.gp1);
	IWL_ERR(test_op_mode, "0x%08X | time gp2\n", table.gp2);
	IWL_ERR(test_op_mode, "0x%08X | time gp3\n", table.gp3);
	IWL_ERR(test_op_mode, "0x%08X | uCode version\n", table.ucode_ver);
	IWL_ERR(test_op_mode, "0x%08X | hw version\n", table.hw_ver);
	IWL_ERR(test_op_mode, "0x%08X | board version\n", table.brd_ver);
	IWL_ERR(test_op_mode, "0x%08X | hcmd\n", table.hcmd);
	IWL_ERR(test_op_mode, "0x%08X | isr0\n", table.isr0);
	IWL_ERR(test_op_mode, "0x%08X | isr1\n", table.isr1);
	IWL_ERR(test_op_mode, "0x%08X | isr2\n", table.isr2);
	IWL_ERR(test_op_mode, "0x%08X | isr3\n", table.isr3);
	IWL_ERR(test_op_mode, "0x%08X | isr4\n", table.isr4);
	IWL_ERR(test_op_mode, "0x%08X | isr_pref\n", table.isr_pref);
	IWL_ERR(test_op_mode, "0x%08X | wait_event\n", table.wait_event);
	IWL_ERR(test_op_mode, "0x%08X | l2p_control\n", table.l2p_control);
	IWL_ERR(test_op_mode, "0x%08X | l2p_duration\n", table.l2p_duration);
	IWL_ERR(test_op_mode, "0x%08X | l2p_mhvalid\n", table.l2p_mhvalid);
	IWL_ERR(test_op_mode, "0x%08X | l2p_addr_match\n",
		table.l2p_addr_match);
	IWL_ERR(test_op_mode, "0x%08X | lmpm_pmg_sel\n", table.lmpm_pmg_sel);
	IWL_ERR(test_op_mode, "0x%08X | timestamp\n", table.u_timestamp);
	IWL_ERR(test_op_mode, "0x%08X | flow_handler\n", table.flow_handler);
}

static void iwl_trans_test_fw_error(struct iwl_test_op_mode *test_op_mode,
				    bool ondemand)
{
	/* uCode is no longer loaded. */
	test_op_mode->ucode_loaded = false;

	/* Set the FW error flag -- cleared on iwl_down */
	set_bit(STATUS_FW_ERROR, &test_op_mode->status);

	iwl_abort_notification_waits(&test_op_mode->notif_wait);

	/* stop Rx and Tx processes */
	clear_bit(STATUS_RX_IN_PROGRESS, &test_op_mode->status);
	wake_up_interruptible(&test_op_mode->wait_rx_queue);
	clear_bit(STATUS_TX_IN_PROGRESS, &test_op_mode->status);
	wake_up_interruptible(&test_op_mode->wait_tx_queue);
}

static void iwl_trans_test_nic_error(struct iwl_op_mode *op_mode)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);

	iwl_trans_test_dump_nic_error_log(test_op_mode);
	iwl_trans_test_fw_error(test_op_mode, false);
}

static void iwl_trans_test_cmd_queue_full(struct iwl_op_mode *op_mode)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);

	if (test_and_set_bit(IWL_TRANS_TEST_CMD_QUEUE,
			     &test_op_mode->transport_queue_stop))
		IWL_DEBUG_TX_QUEUES(test_op_mode,
				    "Cmd Queue already stopped\n");
	else
		IWL_DEBUG_TX_QUEUES(test_op_mode, "Cmd queue stopped\n");

	iwl_trans_test_fw_error(test_op_mode, false);
}

static void iwl_trans_test_nic_config(struct iwl_op_mode *op_mode)
{
	struct iwl_test_op_mode *test_om = IWL_OP_MODE_GET_TEST(op_mode);
	u8 radio_cfg_type, radio_cfg_step, radio_cfg_dash;
	u32 reg_val = 0;

	radio_cfg_type = (test_om->fw->phy_config & FW_PHY_CFG_RADIO_TYPE) >>
			  FW_PHY_CFG_RADIO_TYPE_POS;
	radio_cfg_step = (test_om->fw->phy_config & FW_PHY_CFG_RADIO_STEP) >>
			  FW_PHY_CFG_RADIO_STEP_POS;
	radio_cfg_dash = (test_om->fw->phy_config & FW_PHY_CFG_RADIO_DASH) >>
			  FW_PHY_CFG_RADIO_DASH_POS;

	/* SKU control */
	reg_val |= CSR_HW_REV_STEP(test_om->trans->hw_rev) <<
				CSR_HW_IF_CONFIG_REG_POS_MAC_STEP;
	reg_val |= CSR_HW_REV_DASH(test_om->trans->hw_rev) <<
				CSR_HW_IF_CONFIG_REG_POS_MAC_DASH;

	/* radio configuration */
	reg_val |= radio_cfg_type << CSR_HW_IF_CONFIG_REG_POS_PHY_TYPE;
	reg_val |= radio_cfg_step << CSR_HW_IF_CONFIG_REG_POS_PHY_STEP;
	reg_val |= radio_cfg_dash << CSR_HW_IF_CONFIG_REG_POS_PHY_DASH;

	WARN_ON((radio_cfg_type << CSR_HW_IF_CONFIG_REG_POS_PHY_TYPE) &
		~CSR_HW_IF_CONFIG_REG_MSK_PHY_TYPE);

	/* silicon bits */
	reg_val |= CSR_HW_IF_CONFIG_REG_BIT_RADIO_SI;
	iwl_trans_set_bits_mask(test_om->trans,
				CSR_HW_IF_CONFIG_REG,
				CSR_HW_IF_CONFIG_REG_MSK_MAC_DASH |
				CSR_HW_IF_CONFIG_REG_MSK_MAC_STEP |
				CSR_HW_IF_CONFIG_REG_MSK_PHY_TYPE |
				CSR_HW_IF_CONFIG_REG_MSK_PHY_STEP |
				CSR_HW_IF_CONFIG_REG_MSK_PHY_DASH |
				CSR_HW_IF_CONFIG_REG_BIT_RADIO_SI |
				CSR_HW_IF_CONFIG_REG_BIT_MAC_SI,
				reg_val);

	IWL_DEBUG_INFO(test_om, "Radio type=0x%x-0x%x-0x%x\n", radio_cfg_type,
		       radio_cfg_step, radio_cfg_dash);

	/* W/A : NIC is stuck in a reset state after Early PCIe power off
	 * (PCIe power is lost before PERST# is asserted),
	 * causing ME FW to lose ownership and not being able to obtain it back.
	 */
	if (test_om->trans->cfg->device_family < IWL_DEVICE_FAMILY_8000)
		iwl_set_bits_mask_prph(test_om->trans,
				       APMG_PS_CTRL_REG,
				       APMG_PS_CTRL_EARLY_PWR_OFF_RESET_DIS,
				       ~APMG_PS_CTRL_EARLY_PWR_OFF_RESET_DIS);
}

/* Push test request to relevant kfifo and start pooling from it if empty */
int iwl_trans_test_push_to_kfifo(struct iwl_op_mode *op_mode, u8 test_num,
				 void *data)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	struct kfifo *fifo = &test_op_mode->test_fifos[test_num];
	u8 size = sizeof(struct iwl_test_request_info);


	if (!kfifo_avail(fifo)) {
		IWL_ERR(test_op_mode, "No room for requests in %s queue\n",
			get_device_cmd_string(test_op_mode, test_num));
		return -ENOSPC;
	}

	kfifo_in(fifo, data, size);

	/* if kfifo loop is not in progress, call it */
	if (kfifo_len(fifo) == size)
		schedule_work(&test_op_mode->test_workers[test_num]);

	return 0;
}

static void iwl_trans_test_process_fifo(struct iwl_test_op_mode *test_op_mode,
					u8 test_num)
{
	struct iwl_test_request_info req_info;
	int ret, error_occured = 0;

	while (kfifo_len(&test_op_mode->test_fifos[test_num]) > 0) {
		ret = kfifo_out(&test_op_mode->test_fifos[test_num], &req_info,
				sizeof(struct iwl_test_request_info));
		if (error_occured ||
		    test_bit(STATUS_STOP_TESTS, &test_op_mode->status))
			continue; /* empty kfifo but don't handle requests */

		if (ret != sizeof(struct iwl_test_request_info)) {
			IWL_ERR(test_op_mode,
				"%d) KFIFO error: Emptying %s KFIFO\n",
				req_info.cmd_cnt,
				get_device_cmd_string(test_op_mode, test_num));
			error_occured = 1;
		} else if (test_bit(STATUS_FW_ERROR, &test_op_mode->status)) {
			IWL_ERR(test_op_mode,
				"%d) FW Error occurred. Emptying %s KFIFO\n",
				req_info.cmd_cnt,
				get_device_cmd_string(test_op_mode, test_num));
			error_occured = 1;
		} else {
			test_op_mode->requests_handlers
					[test_num](test_op_mode, req_info);
		}
	}
}

void iwl_trans_test_fifo_worker(struct iwl_test_op_mode *test_op_mode,
				u8 test_num, u8 status_bit)
{
	do {
		if (test_and_set_bit(status_bit, &test_op_mode->status))
			return;

		iwl_trans_test_process_fifo(test_op_mode, test_num);
		clear_bit(status_bit, &test_op_mode->status);
	} while (kfifo_len(&test_op_mode->test_fifos[test_num]) > 0 &&
		!test_bit(status_bit, &test_op_mode->status));
		/* To prevent timing issues: if a new element was entered after
		 the function returned and before the status bit was cleared */
}

void iwl_trans_test_flush_requests(struct iwl_test_op_mode *test_op_mode)
{
	int i;
	for (i = 0; i < IWL_TRANS_TEST_NUM_TESTS_TYPE; i++) {
		flush_work(&test_op_mode->test_workers[i]);
		while (test_bit(STATUS_WORK_POS + i, &test_op_mode->status))
			schedule();
	}
}

static void iwl_trans_test_stop_sw_queue(struct iwl_op_mode *op_mode,
					 int queue)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);

	if (test_and_set_bit(queue, &test_op_mode->transport_queue_stop))
		IWL_DEBUG_TX_QUEUES(test_op_mode,
				    "Queue %d already stopped\n", queue);
	else
		IWL_DEBUG_TX_QUEUES(test_op_mode, "Queue %d stopped\n", queue);
}

static void iwl_trans_test_wake_sw_queue(struct iwl_op_mode *op_mode,
					 int queue)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);


	if (!test_and_clear_bit(queue, &test_op_mode->transport_queue_stop))
		IWL_DEBUG_TX_QUEUES(test_op_mode,
				    "Queue %d already awake\n", queue);
	else
		IWL_DEBUG_TX_QUEUES(test_op_mode, "Queue %d awake\n", queue);
}

static void iwl_trans_test_free_skb(struct iwl_op_mode *op_mode,
				    struct sk_buff *skb)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);

	iwl_trans_free_tx_cmd(test_op_mode->trans, *(void **)skb->cb);
	dev_kfree_skb_any(skb);
}

static bool iwl_trans_test_set_hw_rfkill_state(struct iwl_op_mode *op_mode,
					       bool state)
{
	/* no rfkill in this mode */
	return false;
}

/*****************************************************************************
 *
 * driver and module entry point
 *
 *****************************************************************************/
static int __init iwl_init(void)
{
	int ret = iwl_opmode_register("iwltest", &iwl_trans_test_ops);
	if (ret)
		pr_err("Unable to register op_mode: %d\n", ret);

	return ret;
}
module_init(iwl_init);

static void __exit iwl_exit(void)
{
	iwl_opmode_deregister("iwltest");
}
module_exit(iwl_exit);

/* xor of all bytes in data_buf */
u8 iwl_trans_test_compute_test_val(u8 *data_buf, size_t n)
{
	u8 val = 0;
	int i = 0;

	for (i = 0; i < n; i++)
		val ^= data_buf[i];
	return val;
}

/* generate random integer data into data_buf pointer, with requested size*/
static void iwl_trans_test_generate_random_data(struct iwl_test_op_mode
					 *test_op_mode,
					 char *data_buf, size_t size)
{
	int i, j;
	u32 val;

	for (i = 0; i < size; i += 4) {
		val = abs(prandom_u32());
		for (j = 0; j < 4 && i+j < size; j++)
			data_buf[i+j] = (u8)(abs(val >> (8 * j)));
	}
}

/**
 * generate data into data_buf pointer, with requested size, according
 * to saved pattern
 */
static void iwl_trans_test_generate_pattern_data(struct iwl_test_op_mode
						 *test_op_mode,
						 char *data_buf, size_t size)
{
	int i;
	char *patt = test_op_mode->data_pattern;
	u8 times = size / DATA_PATTERN_LENGTH;
	u8 rem = size % DATA_PATTERN_LENGTH;

	for (i = 0; i < times; i++)
		memcpy(data_buf + i * DATA_PATTERN_LENGTH, patt,
		       DATA_PATTERN_LENGTH);
	for (i = 0; i < rem; i++)
		data_buf[times * DATA_PATTERN_LENGTH + i] = patt[i];
}

/* Send host command */
int iwl_trans_test_send_hcmd(struct iwl_test_op_mode *test_op_mode,
			     struct iwl_host_cmd *cmd)
{
	if (test_bit(STATUS_FW_ERROR, &test_op_mode->status))
		return -EIO;

	/*
	 * Synchronous commands must hold
	 * the mutex, this ensures we don't try to send two
	 * (or more) synchronous commands at a time.
	 */
	if (!(cmd->flags & CMD_ASYNC))
		lockdep_assert_held(&test_op_mode->mutex);

	return iwl_trans_send_cmd(test_op_mode->trans, cmd);
}

/* for cmd where the response data is the sending status */
int iwl_trans_test_send_cmd_status(struct iwl_test_op_mode *test_op_mode,
					u8 id, void *data, u16 len)
{
	struct iwl_rx_packet *pkt;
	struct iwl_trans_test_hcmd_reply *resp;
	int ret, resp_len;

	struct iwl_host_cmd cmd = {
		.id = id,
		.len = {len, },
		.data = {data, },
		.flags = CMD_WANT_SKB,
	};

	ret = iwl_trans_test_send_hcmd(test_op_mode, &cmd);

	if (ret) {
		IWL_ERR(test_op_mode, "Failed to send status hcmd\n");
		return ret;
	}

	pkt = cmd.resp_pkt;

	resp_len = iwl_rx_packet_payload_len(pkt);

	if (resp_len != ALIGN(sizeof(struct iwl_trans_test_hcmd_reply), 4)) {
		ret = -EIO;
		goto out_free_resp;
	}

	resp = (struct iwl_trans_test_hcmd_reply *)pkt->data;
	ret = resp->status;

out_free_resp:
	iwl_free_resp(&cmd);
	return ret;
}

/**
 * Generate integer data of size "size" into data_buf,
 * according to generation method set by opmode.
 * Compute data test_val and store into test_val_ptr.
 */
void iwl_trans_test_generate_data(struct iwl_test_op_mode
					  *test_op_mode, char *data_buf,
					  size_t size, u8 *test_val_ptr)
{
	/* no need to generate data */
	if (size <= 0) {
		if (test_val_ptr)
			*test_val_ptr = 0;
		return;
	}

	test_op_mode->generate_data[test_op_mode->data_generation_method]
				(test_op_mode, data_buf, size);

	if (test_val_ptr)
		*test_val_ptr =
			iwl_trans_test_compute_test_val(data_buf, size);
}

u64 iwl_trans_test_get_timeout(struct iwl_test_op_mode *test_op_mode,
			       u32 times)
{
	u64 to = times * test_op_mode->ms_per_pkt *
		DIV_ROUND_UP(times, 1000);

	to = set_in_range(to, (u64)MIN_MS_PER_REQUEST,
			  (u64)test_op_mode->max_sec_per_request * 1000);

	return msecs_to_jiffies(to);
}

#ifdef CPTCFG_IWLWIFI_DEVICE_TESTMODE
static int iwl_trans_test_tm_send_hcmd(void *op_mode,
				       struct iwl_host_cmd *host_cmd)
{
	struct iwl_test_op_mode *test_op_mode =
		(struct iwl_test_op_mode *)op_mode;

	if (WARN_ON_ONCE(!op_mode))
		return -EINVAL;

	return iwl_trans_test_send_hcmd(test_op_mode, host_cmd);
}
#endif

static struct iwl_op_mode *
iwl_op_mode_trans_test_start(struct iwl_trans *trans,
			     const struct iwl_cfg *cfg,
			     const struct iwl_fw *fw,
			     struct dentry *dbgfs_dir)
{
	int err = 0;
	struct iwl_op_mode *op_mode;
	struct iwl_test_op_mode *test_op_mode;
	struct iwl_trans_config trans_cfg = {};
	int i, j = 0;
	static const u8 no_reclaim_cmds[] = {
		REPLY_TRANS_TEST_TX,
	};
	unsigned int test_op_mode_size = sizeof(struct iwl_op_mode) +
					sizeof(struct iwl_test_op_mode);

	/********************************
	 * 1. Allocating and configuring HW data
	/ ********************************/
	op_mode = kzalloc(test_op_mode_size, GFP_ATOMIC);
	if (!op_mode)
		return NULL;
	op_mode->ops = &iwl_trans_test_ops;

	test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	test_op_mode->dev = trans->dev;
	test_op_mode->trans = trans;
	test_op_mode->cfg = cfg;

	test_op_mode->fw = fw;

	/********************************
	 * 2. Configurations and transport init
	 ********************************/
	/*
	 * Populate the state variables that the transport layer needs
	 * to know about.
	 */
	trans_cfg.op_mode = op_mode;
	trans_cfg.no_reclaim_cmds = no_reclaim_cmds;
	trans_cfg.n_no_reclaim_cmds = ARRAY_SIZE(no_reclaim_cmds);

	switch (iwlwifi_mod_params.amsdu_size) {
	case IWL_AMSDU_DEF:
	case IWL_AMSDU_4K:
		trans_cfg.rx_buf_size = IWL_AMSDU_4K;
		break;
	case IWL_AMSDU_8K:
		trans_cfg.rx_buf_size = IWL_AMSDU_8K;
		break;
	case IWL_AMSDU_12K:
		trans_cfg.rx_buf_size = IWL_AMSDU_12K;
		break;
	default:
		pr_err("Unsupported amsdu_size: %d\n",
		       iwlwifi_mod_params.amsdu_size);
		trans_cfg.rx_buf_size = IWL_AMSDU_4K;
	}

	trans_cfg.cmd_fifo = IWL_TRANS_TEST_CMD_FIFO;
	trans_cfg.command_groups = iwl_trans_test_cmd_groups;
	trans_cfg.command_groups_size = ARRAY_SIZE(iwl_trans_test_cmd_groups);
	trans_cfg.cmd_queue = IWL_TRANS_TEST_CMD_QUEUE;

	/* first pointer is used by us */
	trans_cfg.cb_data_offs = sizeof(void *);

	/* Configure transport layer */
	iwl_trans_configure(test_op_mode->trans, &trans_cfg);
	trans->command_groups = trans_cfg.command_groups;
	trans->command_groups_size = trans_cfg.command_groups_size;

	trans->rx_mpdu_cmd = REPLY_RX_MPDU_CMD;
	trans->rx_mpdu_cmd_hdr_size = sizeof(struct iwl_rx_mpdu_res_start);

	/********************************
	 * 3. Setup services
	 ********************************/
	for (i = 0; i < IWL_TRANS_TEST_NUM_TESTS_TYPE; i++) {
		err = kfifo_alloc(&test_op_mode->test_fifos[i],
			  IWL_TRANS_TEST_PIPE_SIZE,
			  GFP_KERNEL);
		if (err) {
			IWL_ERR(test_op_mode, "Cannot allocate fifo\n");
			for (j = 0; j < i; j++)
				kfifo_free(&test_op_mode->test_fifos[j]);
			goto out;
		}
	}

	test_op_mode->ms_per_pkt = IWL_TRANS_TEST_MS_PER_PACKET;
	test_op_mode->max_sec_per_request = MAX_SEC_PER_REQUEST;

	init_waitqueue_head(&test_op_mode->wait_rx_queue);
	INIT_WORK(&test_op_mode->test_workers[TEST_RX],
		  iwl_trans_test_rx_worker);
	iwl_trans_test_reset_rx_task(test_op_mode);
	init_waitqueue_head(&test_op_mode->wait_tx_queue);
	iwl_trans_test_reset_tx_task(test_op_mode);
	INIT_WORK(&test_op_mode->test_workers[TEST_TX],
		  iwl_trans_test_tx_worker);
	test_op_mode->tx_req_num = 0;

	INIT_WORK(&test_op_mode->test_workers[TEST_HCMD],
		  iwl_trans_test_hcmd_worker);

	test_op_mode->status = 0;
	iwl_trans_test_setup_rx_handlers(test_op_mode);
	mutex_init(&test_op_mode->mutex);

	test_op_mode->data_pattern = DATA_PATTERN;

	test_op_mode->generate_data[GEN_PATTERN_DATA] =
		iwl_trans_test_generate_pattern_data;
	test_op_mode->generate_data[GEN_RANDOM_DATA] =
		iwl_trans_test_generate_random_data;
	test_op_mode->data_generation_method = GEN_RANDOM_DATA;

	/* setup requests handlers */
	test_op_mode->requests_handlers[TEST_HCMD] =
			iwl_trans_test_handle_hcmd_request;
	test_op_mode->requests_handlers[TEST_TX] =
			iwl_trans_test_handle_tx_request;
	test_op_mode->requests_handlers[TEST_RX] =
			iwl_trans_test_handle_rx_request;

#ifdef CPTCFG_IWLWIFI_DEVICE_TESTMODE
	iwl_tm_init(trans, test_op_mode->fw, &test_op_mode->mutex,
		    test_op_mode);
#endif

	/********************************
	 * 4. Read REV register and start hw
	 ********************************/
	IWL_INFO(test_op_mode, "Detected %s, REV=0x%X\n",
		 cfg->name, test_op_mode->trans->hw_rev);

	err = iwl_trans_start_hw(test_op_mode->trans);
	if (err)
		goto out;

	/********************************
	 * 5. Download init uCode
	 ********************************/
	mutex_lock(&test_op_mode->mutex);
	err = iwl_run_init_test_ucode(op_mode);
	mutex_unlock(&test_op_mode->mutex);

	if (err) {
		IWL_ERR(test_op_mode, "Failed to run init ucode: %d\n", err);
		goto out_stop_hw;
	}

	/********************************
	 * 6. Setup tet device and debugfs
	 ********************************/

	/* Tell FW to switch to transport tester mode */
	err = iwl_trans_test_activate_tx_mode(test_op_mode, 1);
	if (err) {
		IWL_ERR(test_op_mode, "Could not start tx test mode\n");
		goto out_stop_hw;
	} else {
		IWL_INFO(test_op_mode, "Transport Tester Tx mode activated\n");
	}

	err = iwl_trans_test_dbgfs_register(test_op_mode, dbgfs_dir);
	if (err) {
		IWL_ERR(test_op_mode, "Could not register Debugfs\n");
		goto out_dbgfs;
	}

	err = init_test_device(op_mode);
	if (err)
		goto out_close_device;

	return op_mode;

out_close_device:
	close_test_device(op_mode);
out_dbgfs:
out_stop_hw:
	iwl_trans_stop_device(test_op_mode->trans);
	iwl_trans_op_mode_leave(test_op_mode->trans);
out:
	kfree(op_mode);
	return NULL;
}

static void iwl_op_mode_trans_test_stop(struct iwl_op_mode *op_mode)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	int i;

	close_test_device(op_mode);

	IWL_INFO(test_op_mode, "Stop trans test: unloading\n");

	/*
	 * The tx/rx requests currently in progress will finish,
	 * and the rest will not be served.
	 */
	iwl_trans_test_flush_requests(test_op_mode);

	for (i = 0; i < IWL_TRANS_TEST_NUM_TESTS_TYPE; i++)
		kfifo_free(&test_op_mode->test_fifos[i]);

	iwl_trans_test_activate_tx_mode(test_op_mode, 0);

	test_op_mode->ucode_loaded = false;

	iwl_trans_stop_device(test_op_mode->trans);
	iwl_trans_op_mode_leave(test_op_mode->trans);

	mutex_destroy(&test_op_mode->mutex);
	kfree(op_mode);
}

const struct iwl_op_mode_ops iwl_trans_test_ops = {
	.start = iwl_op_mode_trans_test_start,
	.stop = iwl_op_mode_trans_test_stop,
	.rx = iwl_trans_test_rx_dispatch,
	.queue_full = iwl_trans_test_stop_sw_queue,
	.queue_not_full = iwl_trans_test_wake_sw_queue,
	.hw_rf_kill = iwl_trans_test_set_hw_rfkill_state,
	.free_skb = iwl_trans_test_free_skb,
	.nic_error = iwl_trans_test_nic_error,
	.cmd_queue_full = iwl_trans_test_cmd_queue_full,
	.nic_config = iwl_trans_test_nic_config,
#ifdef CPTCFG_IWLWIFI_DEVICE_TESTMODE
	.test_ops = {
		.send_hcmd = iwl_trans_test_tm_send_hcmd,
	},
#endif
};
