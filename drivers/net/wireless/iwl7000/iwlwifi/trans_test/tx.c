/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
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

static int iwl_reclaim(struct iwl_test_op_mode *test_op_mode, int sta_id,
		       int tid, int txq_id, int ssn, struct sk_buff_head *skbs)
{
	iwl_trans_reclaim(test_op_mode->trans, txq_id, ssn, skbs);
	return 0;
}

/*
 * Will only work for transport types whos txqs are of size at most 0xff-1.
 * This holds for all current types: pcie, sdio.
 * Does not support aggragations.
 */
void iwl_trans_test_rx_reply_tx(struct iwl_op_mode *op_mode,
				struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	u16 sequence = le16_to_cpu(pkt->hdr.sequence);
	int txq_id = SEQ_TO_QUEUE(sequence);
	u8 *data = (void *)pkt->data;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)data;
	u8 hdr_len = ALIGN(ieee80211_hdrlen(hdr->frame_control), 4);
	/* This will only work for single TX at a time. For aggragation the fw
	 * will have to send the ssn at the end of the TX response */
	u16 ssn = SEQ_TO_INDEX(sequence)+1;
	u16 req_num = 0;
	struct iwl_device_cmd *dev_cmd;
	u32 frame_len;
	struct sk_buff_head skbs;
	struct sk_buff *skb;

	__skb_queue_head_init(&skbs);

	/* we can free until ssn % q.n_bd not inclusive */
	WARN_ON_ONCE(iwl_reclaim(test_op_mode, 0, 0, txq_id, ssn, &skbs));

	/* process frames */
	while (!skb_queue_empty(&skbs)) {
		skb = __skb_dequeue(&skbs);

		dev_cmd = *(void **)skb->cb;
		req_num = ((struct iwl_tx_cmd *)dev_cmd->payload)->req_num;
		iwl_trans_free_tx_cmd(test_op_mode->trans, dev_cmd);

		kfree_skb(skb);
	}

	/*
	 * If we are in Tx task - Check tx_reply status and updat statistics
	 * else, Tx task was timed out and no new one started.
	 * in this case error msg was already sent, ignore the data.
	 */
	if ((!test_bit(STATUS_TX_IN_PROGRESS,
		       &test_op_mode->status))) {
		IWL_DEBUG_RX(test_op_mode, "Received unexpected Tx reply\n");
		return; /* proceed as usual, but don't count to statistics */
	}

	/* if this reply belongs to a packet from a former request - ignore */
	if (req_num != test_op_mode->tx_req_num)
		return;

	/* check data - need to know exactly where data started*/
	test_op_mode->curr_tx_stat.n_tx_received_replies++;
	IWL_DEBUG_RX(test_op_mode, "in reply TX %u\n",
		     test_op_mode->curr_tx_stat.n_tx_received_replies);

	frame_len = iwl_rx_packet_len(pkt);
	/* check if we received the correct frame size */
	if (frame_len != (test_op_mode->tx_curr_size +
			  sizeof(struct iwl_cmd_header) + hdr_len)) {
		test_op_mode->curr_tx_stat.n_tx_received_wrong_size++;
		IWL_INFO(test_op_mode, "Received TX reply of wrong size %d\n",
			 frame_len);
	} else {
		u8 test_val = iwl_trans_test_compute_test_val
				(data + hdr_len, test_op_mode->tx_curr_size);

		if (test_val == test_op_mode->data_tx_test_val) {
			test_op_mode->curr_tx_stat.n_tx_correct_replies++;
		} else {
			test_op_mode->curr_tx_stat.n_tx_received_data_err++;
			IWL_INFO(test_op_mode,
				 "TX CRC ERR! test_val = %d, stored val = %d\n",
				 test_val, test_op_mode->data_tx_test_val);
		}
	}

	/* If this was the last frame expected - wake up waiting queue */
	if (test_op_mode->tx_finished_sending &&
	    (test_op_mode->curr_tx_stat.n_tx_sent_succesfuly ==
	    test_op_mode->curr_tx_stat.n_tx_received_replies)){
		clear_bit(STATUS_TX_IN_PROGRESS,
			  &test_op_mode->status);
		wake_up(&test_op_mode->wait_tx_queue);
	}
}

/***************************
 * Tx test helpers
 **************************/
void iwl_trans_test_reset_tx_task(struct iwl_test_op_mode *test_op_mode)
{
	memset(&test_op_mode->curr_tx_stat, 0, sizeof(struct tx_stat));
	test_op_mode->tx_finished_sending = 0;
	test_op_mode->tx_curr_size = 0;
	test_op_mode->tx_req_num++;
}

/* At the end of a tx task - add current task stats to the total tx stats */
static void iwl_trans_test_update_stats(struct iwl_test_op_mode *test_op_mode,
					int throughput)
{
	struct tx_stat *curr = &test_op_mode->curr_tx_stat;
	struct tx_stat *total = &test_op_mode->total_tx_stat;
	total->n_tx_sent += curr->n_tx_sent;
	total->n_tx_sent_succesfuly += curr->n_tx_sent_succesfuly;
	total->n_tx_received_replies += curr->n_tx_received_replies;
	total->n_tx_correct_replies += curr->n_tx_correct_replies;
	total->n_tx_received_data_err += curr->n_tx_received_data_err;
	total->n_tx_received_wrong_size += curr->n_tx_received_wrong_size;
	if (throughput > 0) {/* then n_tx_received_replies must be > 0 */
		total->n_tx_recieved_for_tpt += curr->n_tx_received_replies;
		total->average_tpt += ((throughput - total->average_tpt) *
		    (int)curr->n_tx_received_replies /
		    (long int)total->n_tx_recieved_for_tpt);
	}
}

int iwl_trans_test_activate_tx_mode(struct iwl_test_op_mode *test_op_mode,
				    u8 active)
{
	int err;
	u16 len = sizeof(struct iwl_debug_cmd_api);
	struct iwl_debug_cmd_api tx_data = {
		.type = ACTIVATE_TX,
		.enable = active,
	};

	mutex_lock(&test_op_mode->mutex);
	err = iwl_trans_test_send_cmd_status(test_op_mode, REPLY_TRANS_TEST_CMD,
					     &tx_data, len);
	mutex_unlock(&test_op_mode->mutex);
	return err;
}

/* create tx request  */
void iwl_trans_test_handle_tx_request(struct iwl_test_op_mode *test_op_mode,
				     struct iwl_test_request_info req_info)
{
	int i, ret = 0;
	u32 times = req_info.times;
	u16 size = req_info.size;
	u8 q_id = req_info.flags;
	char *data = NULL;
	unsigned long total_timeout;
	unsigned long time_before;
	u64 tx_timeout = 0;
	int tpt = 0;
	u16 byte_cnt = (size + sizeof(struct iwl_tx_cmd) + ieee80211_hdrlen(0));

	if (q_id == IWL_TRANS_TEST_CMD_QUEUE ||
	    (q_id != IWL_TRANS_TEST_CHOOSE_RANDOM_QUEUE &&
	     q_id >= test_op_mode->cfg->base_params->num_of_queues)) {
		IWL_ERR(test_op_mode,
			"%d) Could not handle tx request: Invalid Queue %d\n",
			req_info.cmd_cnt, q_id);
		return;
	}

	if (test_and_set_bit(STATUS_TX_IN_PROGRESS, &test_op_mode->status)) {
		IWL_ERR(test_op_mode, "%d) Tx already in progress\n",
			req_info.cmd_cnt);
		return;
	}

	IWL_INFO(test_op_mode,
		 "%d) Start serving Tx request for %u packets of size %d, "
		 "queue %d\n", req_info.cmd_cnt, times, size, q_id);

	iwl_trans_test_reset_tx_task(test_op_mode);
	test_op_mode->tx_curr_size = ALIGN(size, 4);

	if (size > 0) {
		data = kmalloc(size, GFP_KERNEL);
		iwl_trans_test_generate_data
			(test_op_mode, data, size,
			&test_op_mode->data_tx_test_val);
	}

	time_before = jiffies;
	for (i = 0; i < times; i++) {
		test_op_mode->curr_tx_stat.n_tx_sent++;

		ret = iwl_trans_test_send_txc(test_op_mode, size, data, q_id);
		IWL_DEBUG_RX(test_op_mode, "Sent tx %d, returned %d\n", i, ret);

		if (ret) {
			/* If one packet failed - continue to send the others
			 unless it's due to a fw error */
			IWL_ERR(test_op_mode,
				"Failed to send %d'th tx packet of size %d\n",
				i, size);
			if (test_bit(STATUS_FW_ERROR, &test_op_mode->status))
				goto fw_err;
		} else {
			test_op_mode->curr_tx_stat.n_tx_sent_succesfuly++;
		}
	}

	test_op_mode->tx_finished_sending = true;

	/* Total time to wait for replies before declaring timeout */
	total_timeout = iwl_trans_test_get_timeout(test_op_mode, times);
	/* Substract the time that already elapsed send tx sending started. */
	if (time_is_after_jiffies(time_before + total_timeout))
		tx_timeout = total_timeout - (jiffies - time_before);

	/*
	 * wait for all requested replies to arive, or for timeout.
	 * if already finished - tx_in_prog is cleared, no waiting will occure.
	 * If the time passed already - will not wait.
	 */
	ret = wait_event_interruptible_timeout(test_op_mode->wait_tx_queue,
					       !test_bit(STATUS_TX_IN_PROGRESS,
							 &test_op_mode->status),
							 tx_timeout);

	if (ret <= 0) {
		tx_timeout = jiffies_to_msecs(total_timeout);
		/* Replies to arrive in the next tx session will be ignored */
		IWL_ERR(test_op_mode,
			"\n%d) Tx task could not complete within %llu "
			"msecs:\n%u packets of size %d sent\n"
			"%u packets sent succsesfully through queue %d\n"
			"%u replies received\n%u correct replies received\n"
			"*******************************\n",
			req_info.cmd_cnt, tx_timeout, times, size,
			test_op_mode->curr_tx_stat.n_tx_sent_succesfuly, q_id,
			test_op_mode->curr_tx_stat.n_tx_received_replies,
			test_op_mode->curr_tx_stat.n_tx_correct_replies);
	} else {
		tx_timeout = jiffies_to_msecs(total_timeout - ret);
		tpt = times * byte_cnt / max_t(u16, tx_timeout, 1);

		IWL_INFO(test_op_mode,
			 "\n%d) Tx task complete within %llu msecs:\n%u"
			 " packets of size %d sent through queue %d\n"
			 "%u packets sent succsesfully\n%u replies received\n"
			 "%u correct replies received.\n"
			 "Throughput = %d b/msec.\n"
			 "*******************************\n",
			 req_info.cmd_cnt,
			 tx_timeout, times, size, q_id,
			 test_op_mode->curr_tx_stat.n_tx_sent_succesfuly,
			 test_op_mode->curr_tx_stat.n_tx_received_replies,
			 test_op_mode->curr_tx_stat.n_tx_correct_replies,
			 tpt);
	}
	iwl_trans_test_update_stats(test_op_mode, tpt);

fw_err:
	kfree(data);
	clear_bit(STATUS_TX_IN_PROGRESS, &test_op_mode->status);
}

/* As long as TX fifo is not empty - pull and handle tx commands info */
void iwl_trans_test_tx_worker(struct work_struct *work)
{
	struct iwl_test_op_mode *test_op_mode =
			container_of(work, struct iwl_test_op_mode,
				     test_workers[TEST_TX]);

	iwl_trans_test_fifo_worker(test_op_mode, TEST_TX, STATUS_TX_WORK);
}

/* choose randonely among active queues */
static int iwl_trans_test_choose_queue(struct iwl_test_op_mode *test_op_mode)
{
	return test_op_mode->active_queues[prandom_u32()
				%test_op_mode->n_active_queues];
}

/*
 * driver sets seq_number according to tx-cmd counter - all
 * are diveded by 16.
 */
static __le16
iwl_trans_test_tx_generate_next_seq(struct iwl_test_op_mode *test_op_mode)
{
	test_op_mode->tx_seq = increase_cyclic(test_op_mode->tx_seq,
						IWL_TRANS_TEST_TX_SEQ_STEP,
						IWL_TRANS_TEST_MAX_SEQ);
	return cpu_to_le16(test_op_mode->tx_seq);
}

/* Send tx command through requested queue */
int iwl_trans_test_send_txc(struct iwl_test_op_mode *test_op_mode,
			    size_t payload_size, void *data, u8 txq_id)
{
	int ret;
	struct iwl_device_cmd *dev_cmd;
	struct iwl_tx_cmd *tx_cmd;
	struct sk_buff *skb;
	struct ieee80211_hdr hdr;
	u8 hdr_len;
	char *skb_ptr;

	hdr.frame_control = 0;
	hdr.duration_id = 0;
	hdr.seq_ctrl = iwl_trans_test_tx_generate_next_seq(test_op_mode);

	/* use the hdr size according to hdr->fc */
	hdr_len = ieee80211_hdrlen(hdr.frame_control);

	skb = alloc_skb(payload_size+hdr_len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	/* Copy hdr and than payload into skb command */
	skb_ptr = skb_put(skb, hdr_len);
	memcpy(skb_ptr, &hdr, hdr_len);

	/* if payload is needed - generate payload and copy
	to the skb, just after the hdr */
	if (payload_size > 0) {
		memcpy(skb_put(skb, payload_size), data,
		       payload_size);
	}

	dev_cmd = iwl_trans_alloc_tx_cmd(test_op_mode->trans);

	if (unlikely(!dev_cmd))
		return -ENOMEM;

	memset(dev_cmd, 0, sizeof(*dev_cmd));
	dev_cmd->hdr.cmd = REPLY_TRANS_TEST_TX;

	tx_cmd = (struct iwl_tx_cmd *)dev_cmd->payload;

	/* store dev_cmd in skb->cb's first pointer */
	*((void **)skb->cb) = dev_cmd;

	/* Total # bytes to be transmitted */
	tx_cmd->len = cpu_to_le16(skb->len);
	tx_cmd->req_num = test_op_mode->tx_req_num;
	memcpy(tx_cmd->hdr, &hdr, hdr_len);

	if (txq_id == IWL_TRANS_TEST_CHOOSE_RANDOM_QUEUE)
		txq_id = iwl_trans_test_choose_queue(test_op_mode);

	do {
		schedule();
	} while (test_bit(txq_id, &test_op_mode->transport_queue_stop) &&
		!test_bit(STATUS_FW_ERROR, &test_op_mode->status));

	/*iwl_trans_tx requires:
	1. skb which contains mac hdr and payload
	2. device_cmd that contains dev_cmd_hdr and payload, which is the
	tx_cmd itself. The fileds of tx_cmd are set inspde the transport.
	The transport creates 2 TFDS:
	1) THE dev_cmd_hdr, tx_cmd and MAC hdr.
	2) the payload. */
	ret = iwl_trans_tx(test_op_mode->trans,
			   skb,
			   dev_cmd,
			   txq_id);

	if (ret)
		iwl_trans_free_tx_cmd(test_op_mode->trans, dev_cmd);

	return ret;
}

/*****************
 * hcmd test
 *****************/
void iwl_trans_test_handle_hcmd_request(struct iwl_test_op_mode *test_op_mode,
				       struct iwl_test_request_info req_info)
{
	u32 times = req_info.times;
	u16 size1 = max_t(u16, req_info.size, DATA_ECHO_HDR_LEN + 1);
	u16 size2 = req_info.size_chunk_two;
	u8 flags = req_info.flags;
	u32 i, ret;
	unsigned long time_before = jiffies;
	struct iwl_data_echo_data_first *data = NULL;
	struct iwl_data_echo_data *data2 = NULL;
	u8 first_df = 0, second_df = 0;
	struct iwl_host_cmd cmd = {
		.id = REPLY_TRANS_TEST_CMD,
	};

	if (!(flags & SEND_DATA_ECHO)) {
		for (i = 0; i < times; i++) {
			ret = iwl_trans_test_send_echo_hcmd(test_op_mode, 0);
			if (ret) {
				IWL_ERR(test_op_mode,
					"%d) Sending echo command %u failed!\n",
					req_info.cmd_cnt, i);
				return;
			} else {
				IWL_DEBUG_RX(test_op_mode,
					     "Sent echo cmd %u\n", i);
			}
		}

		IWL_INFO(test_op_mode,
			 "%d) Sending %u echos complete within %u msecs\n",
			 req_info.cmd_cnt, times,
			 jiffies_to_msecs(jiffies - time_before));

		return;
	}

	if (flags & FIRST_DATA_NOCOPY)
		first_df |= IWL_HCMD_DFL_NOCOPY;
	if (flags & FIRST_DATA_DUP)
		first_df |= IWL_HCMD_DFL_DUP;
	if (size2) {
		if (flags & SECOND_DATA_NOCOPY)
			second_df |= IWL_HCMD_DFL_NOCOPY;
		if (flags & SECOND_DATA_DUP)
			second_df |= IWL_HCMD_DFL_DUP;
	}

	data = kmalloc(size1, GFP_KERNEL);
	if (!data)
		return;
	data->hcmd_api.type = REPLY_DATA_ECHO;

	/*
	 * length sent to fw include only payload and test_val of each chunk.
	 * length of the first data chunk from drivers point of view includes
	 * the entire iwl_data_echo_data_first struct (8 more bytes).
	 */
	data->length[0] = cpu_to_le16(size1 - DATA_ECHO_HDR_LEN);
	data->length[1] = cpu_to_le16(size2);
	iwl_test_fill_data_echo_data(test_op_mode, &(data->echo_data),
				     size1 - DATA_ECHO_HDR_LEN);
	if (size2) {
		data2 = kmalloc(size2, GFP_KERNEL);
		if (!data2)
			goto free_buf;
		iwl_test_fill_data_echo_data(test_op_mode, data2, size2);
	}

	cmd.len[0] = size1;
	cmd.len[1] = size2;
	cmd.data[0] = data;
	cmd.data[1] = data2;
	cmd.dataflags[0] = first_df;
	cmd.dataflags[1] = second_df;

	IWL_INFO(test_op_mode,
		 "%d) Sending %u data ECHO hcmd: size1 = %d, size2 = %d\n",
		 req_info.cmd_cnt, times, size1, size2);
	if (flags & DATAFLAGS_MSK)
		IWL_INFO(test_op_mode,
			 "nocopy1 = %s, dup1 = %s, nocopy2 = %s dup2 = %s\n",
			 first_df & IWL_HCMD_DFL_NOCOPY ? "true" : "false",
			 first_df & IWL_HCMD_DFL_DUP ? "true" : "false",
			 second_df & IWL_HCMD_DFL_NOCOPY ? "true" : "false",
			 second_df & IWL_HCMD_DFL_DUP ? "true" : "false");

	for (i = 0; i < times; i++) {
		ret = iwl_trans_test_send_hcmd_test(test_op_mode, &cmd);
		if (ret) {
			IWL_ERR(test_op_mode, "%d) Sending cmd %u failed\n",
				req_info.cmd_cnt, i);
			goto free_buf;
		} else {
			IWL_DEBUG_RX(test_op_mode,
				     "Sending data echo command"
				     "%u succeed\n", i);
		}
	}

	IWL_INFO(test_op_mode,
		 "\n%d) Sending %u data_echos complete within %u msecs"
		 "\n*******************************\n",
		 req_info.cmd_cnt, times,
		 jiffies_to_msecs(jiffies - time_before));

free_buf:
	kfree(data);
	kfree(data2);
}

/* As long as rx fifo is not empty - pull and handle rx commands info */
void iwl_trans_test_hcmd_worker(struct work_struct *work)
{
	struct iwl_test_op_mode *test_op_mode =
			container_of(work, struct iwl_test_op_mode,
				     test_workers[TEST_HCMD]);

	iwl_trans_test_fifo_worker(test_op_mode, TEST_HCMD, STATUS_HCMD_WORK);
}

/****************************
 * Hcmd test helpers
 ****************************/
static bool iwl_test_hcmd_should_fail(struct iwl_host_cmd *cmd)
{
	if ((cmd->dataflags[0]&IWL_HCMD_DFL_NOCOPY &&
	     cmd->dataflags[0]&IWL_HCMD_DFL_DUP) ||
	   (cmd->dataflags[1]&IWL_HCMD_DFL_NOCOPY &&
	    cmd->dataflags[1]&IWL_HCMD_DFL_DUP) ||
	    /* Nocopy chunk followed by normal copied one */
	   (cmd->dataflags[0]&IWL_HCMD_DFL_NOCOPY && cmd->len[1] > 0 &&
	    !(cmd->dataflags[1]&(IWL_HCMD_DFL_NOCOPY | IWL_HCMD_DFL_DUP))) ||
	    /* A copied chunk larger than 320B */
	   ((!(cmd->dataflags[0]&(IWL_HCMD_DFL_NOCOPY | IWL_HCMD_DFL_DUP))) &&
	    cmd->len[0] > DEF_CMD_PAYLOAD_SIZE) ||
	   ((!(cmd->dataflags[1]&(IWL_HCMD_DFL_NOCOPY | IWL_HCMD_DFL_DUP))) &&
	    cmd->len[1] > DEF_CMD_PAYLOAD_SIZE) ||
	    /* A chunk larger than 4k */
	   (cmd->len[0] > 4096 || cmd->len[1] > 4096))
		return true;

	return false;
}

void iwl_test_fill_data_echo_data(struct iwl_test_op_mode *test_op_mode,
					struct iwl_data_echo_data *echo_data,
					size_t len)
{
	/* Len-1 bytes for data itself, 1 byte for the computed test_val */
	iwl_trans_test_generate_data(test_op_mode,
				     echo_data->data,
				     len - 1, &echo_data->data_test_val);
}

/* for regular echo commands */
int iwl_trans_test_send_echo_hcmd(struct iwl_test_op_mode *test_op_mode,
				  u32 flags)
{
	struct iwl_host_cmd cmd = {
		.id = REPLY_ECHO,
		.len = { 0, },
		.data = { NULL, },
		.flags = flags,
	};

	return iwl_trans_test_send_hcmd_test(test_op_mode, &cmd);
}

/* send host command and perform required testing */
int iwl_trans_test_send_hcmd_test(struct iwl_test_op_mode *test_op_mode,
				  struct iwl_host_cmd *cmd)
{
	int ret;

	if (!(cmd->flags & CMD_ASYNC))
		mutex_lock(&test_op_mode->mutex);

	ret = iwl_trans_test_send_hcmd(test_op_mode, cmd);

	if (iwl_test_hcmd_should_fail(cmd)) {
		IWL_INFO(test_op_mode, "HCMD Test should fail, ret=%d\n", ret);
		goto out_fail;
	}

	/*if should succeed but failed, return fail*/
	if (ret)
		goto out;

	if (!(cmd->flags & CMD_ASYNC)) {
		/* aloud other commands to be executed */
		mutex_unlock(&test_op_mode->mutex);

		if (cmd->flags & CMD_WANT_SKB) {
			ret |= iwl_test_cmd_check_echo_resp(test_op_mode, cmd);
			iwl_free_resp(cmd);
		}
	}

	return ret;
out:
	IWL_ERR(test_op_mode, "Sending host command failed\n");
out_fail:
	if (!(cmd->flags & CMD_ASYNC))
		mutex_unlock(&test_op_mode->mutex);
	if (cmd->resp_pkt)
		iwl_free_resp(cmd);
	return ret;
}

/**
 * checks if the response packet contains correct cmd id. The sucees status was
 * examined while processing the rx packet, here we only want to make sure
 * CMD_WANT_HCMD returned the response packet as needed.
 */
int iwl_test_cmd_check_echo_resp(struct iwl_test_op_mode *test_op_mode,
				 struct iwl_host_cmd *cmd)
{
	struct iwl_rx_packet *resp = cmd->resp_pkt;
	struct iwl_cmd_header hdr;

	if (!resp) {
		return -EINVAL;
	}
	hdr = resp->hdr;

	switch (hdr.cmd) {
	case(REPLY_ECHO):
	case(REPLY_TRANS_TEST_CMD):
		return 0;
	default:
		return -EINVAL;
	}
}

