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
 * Copyright(c) 2018        Intel Corporation
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
 * Copyright(c) 2018        Intel Corporation
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
#include "iwl-trans-test.h"
#ifdef CPTCFG_IWLWIFI_DEVICE_TESTMODE
#include "iwl-tm-gnl.h"
#endif

/******************************************************************************
 *
 * RX handler implementations
 *
 ******************************************************************************/

/**
 * If the data_test_val check was correct - we will get a sucess status in
 * response packet data.
 */
static void iwl_trans_test_reply_data_echo(struct iwl_op_mode *op_mode,
					   struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_test_op_mode *test_op_mode  = IWL_OP_MODE_GET_TEST(op_mode);
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_trans_test_hcmd_reply *reply = (void *)pkt->data;

	u8 status = reply->status;
	if (status == DATA_TEST_VAL_OK)
		IWL_DEBUG_RX(test_op_mode, "Extended echo test passed\n");
	else
		IWL_ERR(test_op_mode,
			"Data echo test failed:"
			"first data chunk %s, second data chunk %s\n",
			status&DATA_TEST_ERR_FIRST ? "failed" : "succeed",
			status&DATA_TEST_ERR_SECOND ? "failed" : "succeed");
}

/* Rx handler for all REPLY_TRANS_TEST_CMD tests */
static void iwl_trans_test_reply_cmd(struct iwl_op_mode *op_mode,
				     struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_trans_test_hcmd_reply *reply = (void *)pkt->data;
	u32 resp_len = iwl_rx_packet_payload_len(pkt);

	/* Reply of wrong size */
	if (resp_len != ALIGN(sizeof(struct iwl_trans_test_hcmd_reply), 4))
		return;

	/* the only tester cmd that requires a handler */
	if (reply->type == REPLY_DATA_ECHO)
		iwl_trans_test_reply_data_echo(op_mode, rxb);
}

static void iwl_trans_test_rx_fw_error(struct iwl_op_mode *op_mode,
				       struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_test_op_mode *test = IWL_OP_MODE_GET_TEST(op_mode);
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_error_resp *err_resp = (void *)pkt->data;

	IWL_ERR(test, "FW Error notification: type 0x%08X cmd_id 0x%02X\n",
		le32_to_cpu(err_resp->error_type), err_resp->cmd_id);
	IWL_ERR(test, "FW Error notification: seq 0x%04X service 0x%08X\n",
		le16_to_cpu(err_resp->bad_cmd_seq_num),
		le32_to_cpu(err_resp->error_service));
	IWL_ERR(test, "FW Error notification: timestamp 0x%16llX\n",
		le64_to_cpu(err_resp->timestamp));
}

/**
 * Receive rx packet, test if its size and data fit expected ones, update
 * rx process accordingly
 */
static void iwl_trans_test_reply_rx(struct iwl_op_mode *op_mode,
				    struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_test_rx_pkt *rx_data = (void *)pkt->data;
	u32 resp_len = iwl_rx_packet_len(pkt);
	u16 expected_len = ALIGN(test_op_mode->rx_data_size, 4) +
		sizeof(struct iwl_cmd_header) + sizeof(struct iwl_test_rx_pkt);

	/* if out of rx process or received all packets in current one: exit */
	if (!test_bit(STATUS_RX_IN_PROGRESS, &test_op_mode->status) ||
	    test_op_mode->rx_remaining_packets == 0) {
		IWL_DEBUG_RX(test_op_mode, "Unexpected Rx packet received\n");
		goto out;
	}

	if (resp_len != expected_len) {
		WARN_ONCE(1,
			  "Received Rx pkt of wrong size.expected:%d, got:%d\n",
			  expected_len, resp_len);
		test_op_mode->rx_err_size_cnt++;
		goto out;
	}

	if (iwl_trans_test_check_received_rx_data
		(rx_data->data, test_op_mode->rx_data,
		 test_op_mode->rx_data_size)) {
			test_op_mode->rx_err_data_cnt++;
			goto out;
	} else {
		IWL_DEBUG_RX(test_op_mode, "Received correct Rx packet\n");
	}
out:
	if (!--test_op_mode->rx_remaining_packets) {
		clear_bit(STATUS_RX_IN_PROGRESS, &test_op_mode->status);
		wake_up_interruptible(&test_op_mode->wait_rx_queue);
	}
}

/* Compare the received rx_data with the expected data */
int iwl_trans_test_check_received_rx_data(u8 *rx_data, u8 *exp_data, u16 size)
{
	int i;

	/* If chunck size is bigger, memcmp should be used for comparison. */
	BUILD_BUG_ON(TRANS_TEST_RX_CHUNK_SIZE != 1);

	for (i = 0; i < size; i++)
		if (rx_data[i] != *exp_data)
			return -EINVAL;
	return 0;
}

void iwl_trans_test_reset_rx_task(struct iwl_test_op_mode *test_op_mode)
{
	memset(&test_op_mode->rx_data, 0, TRANS_TEST_RX_CHUNK_SIZE);
	test_op_mode->rx_err_size_cnt = 0;
	test_op_mode->rx_err_data_cnt = 0;
	test_op_mode->rx_remaining_packets = 0;
}

/* updates statistics and resets expected data when rx progress is over */
static void iwl_trans_test_close_rx_test(struct iwl_test_op_mode *test_op_mode,
					 u16 times, int tpt, u8 fast)
{
	test_op_mode->total_rx_expected += times;
	test_op_mode->total_rx_received += times -
		test_op_mode->rx_remaining_packets;
	test_op_mode->total_rx_size_errors += test_op_mode->rx_err_size_cnt;
	test_op_mode->total_rx_data_errors += test_op_mode->rx_err_data_cnt;
	if (tpt && fast) {
		test_op_mode->total_fast_rx_received += times;
		test_op_mode->average_rx_tpt +=
			((tpt - test_op_mode->average_rx_tpt) * times
			/ (long int)test_op_mode->total_fast_rx_received);
	}
	iwl_trans_test_reset_rx_task(test_op_mode);
}


/* As long as rx fifo is not empty - pull and handle rx commands info */
void iwl_trans_test_rx_worker(struct work_struct *work)
{
	struct iwl_test_op_mode *test_op_mode =
			container_of(work, struct iwl_test_op_mode,
				     test_workers[TEST_RX]);

	iwl_trans_test_fifo_worker(test_op_mode, TEST_RX, STATUS_RX_WORK);
}

/* send rx request hcmd to fw and wait for RX pakets to arrive */
void iwl_trans_test_handle_rx_request(struct iwl_test_op_mode *test_op_mode,
				     struct iwl_test_request_info req_info)
{
	int ret = 0;
	u16 rx_timeout;
	int tpt = 0;
	struct iwl_send_rx_hcmd_data rx_cmd_data = {
		.hcmd_api.type = REPLY_SEND_RX,
		.times = cpu_to_le32(req_info.times),
		.size = cpu_to_le16(req_info.size)
	};

	if (test_and_set_bit(STATUS_RX_IN_PROGRESS, &test_op_mode->status)) {
		IWL_ERR(test_op_mode,
			"%d) Rx already in progress.\n", req_info.cmd_cnt);
		return;
	}

	IWL_INFO(test_op_mode,
		 "%d) Start serving %s Rx request for %d packets of size %d\n",
		 req_info.cmd_cnt, req_info.flags ? "Fast" : "",
		 req_info.times, req_info.size);

	if (req_info.flags) { /* in sync mode */
		rx_cmd_data.interval = RX_SYNC_INTERVAL;
		rx_cmd_data.rx_per_session =
			(req_info.size < RX_SYNC_MODE_SIZE_LIMIT) ?
			cpu_to_le32(RX_SYNC_SMALL_NUM_PKT) :
			cpu_to_le32(RX_SYNC_LARGE_NUM_PKT);
	} else { /* in async mode */
		rx_cmd_data.interval = RX_ASYNC_INTERVAL;
		rx_cmd_data.rx_per_session =
			cpu_to_le32(min(6000 / req_info.size, 300));
	}

	iwl_trans_test_generate_data(test_op_mode, rx_cmd_data.data,
				     TRANS_TEST_RX_CHUNK_SIZE,
				     &test_op_mode->data_rx_test_val);

	/* save info for comparison on rx reply handler */
	test_op_mode->rx_data_size = req_info.size;
	memcpy(&test_op_mode->rx_data, rx_cmd_data.data,
	       TRANS_TEST_RX_CHUNK_SIZE);
	test_op_mode->rx_remaining_packets = req_info.times;

	rx_timeout = iwl_trans_test_get_timeout(test_op_mode, req_info.times);

	/* send rx request hcmd to fw */
	mutex_lock(&test_op_mode->mutex);
	ret = iwl_trans_test_send_cmd_status
		(test_op_mode, REPLY_TRANS_TEST_CMD, &rx_cmd_data,
		 sizeof(struct iwl_send_rx_hcmd_data));
	mutex_unlock(&test_op_mode->mutex);

	if (ret) {
		IWL_INFO(test_op_mode, "Failed to send rx request\n");
		clear_bit(STATUS_RX_IN_PROGRESS, &test_op_mode->status);
		goto out;
	}

	/* wait for packets to arrive */
	ret = wait_event_interruptible_timeout
		(test_op_mode->wait_rx_queue,
		!test_bit(STATUS_RX_IN_PROGRESS, &test_op_mode->status),
		rx_timeout);

	/* if timeout or failed to send */
	if (ret <= 0) {
		/* if STATUS_RX_IN_PROG is off, the rx was complete
		 * by the time we got here, ignore timeout */
		if (test_and_clear_bit(STATUS_RX_IN_PROGRESS,
				       &test_op_mode->status)){
			IWL_ERR(test_op_mode,
				"\n%d) Rx test timed out:\n Received %d packets"
				" of size %d out of %d in %d ms,\n"
				"including %d size errors, %d data errors\n"
				"*******************************\n",
				req_info.cmd_cnt, req_info.times -
				test_op_mode->rx_remaining_packets,
				req_info.size,
				req_info.times, jiffies_to_msecs(rx_timeout),
				test_op_mode->rx_err_size_cnt,
				test_op_mode->rx_err_data_cnt);
		}
	} else {
		rx_timeout = jiffies_to_msecs(rx_timeout - ret);
		tpt = (req_info.times * req_info.size)
					/ max_t(u16, rx_timeout, 1);
		IWL_INFO(test_op_mode,
			 "\n%d) %s Rx test finished within %d msecs:\n"
			 "Received all %d packets of size %d, %d errors.\n"
			 "Troughput: %d b/msec.\n"
			 "*******************************\n",
			 req_info.cmd_cnt, req_info.flags ? "Fast" : "",
			 rx_timeout, req_info.times, req_info.size,
			 test_op_mode->rx_err_data_cnt +
			 test_op_mode->rx_err_size_cnt, tpt);
	}
out:
	iwl_trans_test_close_rx_test(test_op_mode, req_info.times, tpt,
				     req_info.flags);
}

/**
 * iwl_setup_rx_handlers - Initialize Rx handler callbacks
 *
 * Setup the RX handlers for each of the reply types sent from the uCode
 * to the host.
 */
void iwl_trans_test_setup_rx_handlers(struct iwl_test_op_mode *test_op_mode)
{
	void (**handlers)(struct iwl_op_mode *op_mode,
			  struct iwl_rx_cmd_buffer *rxb);

	handlers = test_op_mode->rx_handlers;
	handlers[REPLY_ERROR] = iwl_trans_test_rx_fw_error;
	handlers[REPLY_TRANS_TEST_CMD] = iwl_trans_test_reply_cmd;
	handlers[REPLY_TRANS_TEST_RX] = iwl_trans_test_reply_rx;
	handlers[REPLY_TRANS_TEST_TX] = iwl_trans_test_rx_reply_tx;

	/* set up notification wait support */
	iwl_notification_wait_init(&test_op_mode->notif_wait);
}

void iwl_trans_test_rx_dispatch(struct iwl_op_mode *op_mode,
				struct napi_struct *napi,
				struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);

	 /* Do the notification wait before RX handlers so
	 * even if the RX handler consumes the RXB we have
	 * access to it in the notification wait entry.
	 */
	iwl_notification_wait_notify(&test_op_mode->notif_wait, pkt);

#ifdef CPTCFG_IWLWIFI_DEVICE_TESTMODE
	/*
	 * RX data may be forwarded to userspace in case the user
	 * requested to monitor the rx w/o affecting the regular flow.
	 * In this case the iwl_test object will handle forwarding the rx
	 * data to user space.
	 */
	iwl_tm_gnl_send_rx(test_op_mode->trans, rxb);
#endif

	/* Based on type of command response or notification,
	 *   handle those that need handling via function in
	 *   rx_handlers table.  See iwl_setup_rx_handlers() */
	if (test_op_mode->rx_handlers[pkt->hdr.cmd]) {
		test_op_mode->rx_handlers_stats[pkt->hdr.cmd]++;

		test_op_mode->rx_handlers[pkt->hdr.cmd](op_mode, rxb);
	}
}
