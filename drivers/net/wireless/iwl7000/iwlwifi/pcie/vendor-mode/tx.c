/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2018 Intel Corporation
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
 * BSD LICENSE
 *
 * Copyright(c) 2018 Intel Corporation
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
#include <linux/pm_runtime.h>
#include <net/tso.h>

#include "iwl-debug.h"
#include "iwl-csr.h"
#include "iwl-io.h"
#include "../internal.h"
#include "fw/api/tx.h"
#include "trans.h"

/**
 * DOC: Vendor mode TX operation
 *
 * The Tx queues reside in host DRAM, and is comprised of a circular buffer
 * of buffer descriptors (TDs), each of which points to *one* data
 * buffer. The driver updates adding a new buffer to the queue by moving the hw
 * "write" pointer (i.e. "doorbell"). The device acknowledges reading by moving
 * the "read" pointer" (i.e. "txq tail").
 * As mentioned above, and different than in legacy mode, devices in vendor
 * mode work with single data buffer descriptors, so the sent memory must be
 * contiguous in the DRAM.
 *
 * Sending and receiving buffer from the device is done in two different
 * processes - tx buffers are replied by a completion, an interrupt with
 * some meta data created by the RFH, but no rb.
 */

static struct iwl_tx_transfer_desc *
iwl_pcie_ven_mode_build_tr(struct iwl_trans *trans,
			   struct iwl_txq *txq,
			   struct iwl_device_cmd *dev_cmd,
			   struct sk_buff *skb,
			   struct iwl_cmd_meta *out_meta)
{
	int idx = iwl_pcie_get_cmd_index(txq, txq->write_ptr);
	struct iwl_tx_transfer_desc *tr = iwl_pcie_get_tfd(trans, txq, idx);
	dma_addr_t buf_addr;

	memset(tr, 0, sizeof(*tr));

	/* Add cmd to skb data */
	memcpy(skb_push(skb, sizeof(struct iwl_tx_cmd_gen3)),
	       &dev_cmd->payload, sizeof(struct iwl_tx_cmd_gen3));

	/* Add dev_cmd to skb data */
	memcpy(skb_push(skb, sizeof(struct iwl_cmd_header)),
	       &dev_cmd->hdr, sizeof(struct iwl_cmd_header));

	/* Verify the skb is not fragmented */
	if (WARN(skb_is_nonlinear(skb),
		 "skb data is fragmented, and should be linear\n"))
		goto out_err;

	buf_addr = dma_map_single(trans->dev, skb->data, skb->len,
				  DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(trans->dev, buf_addr)))
		goto out_err;

	dma_sync_single_for_device(trans->dev, buf_addr,
				   skb->len, DMA_BIDIRECTIONAL);

	/* Fill the actual tr */
	tr->type_n_size = cpu_to_le32((1 & IWL_TX_TD_TYPE) |
				      ((skb->len << 8) & IWL_TX_TD_SIZE));
	put_unaligned_le64(buf_addr, &tr->addr);

	trace_iwlwifi_dev_tx(trans->dev, skb, tr, sizeof(*tr), &skb->data,
			     sizeof(struct iwl_tx_cmd_gen3) +
			     sizeof(struct iwl_cmd_header),
			     sizeof(struct iwl_tx_cmd_gen3) +
			     sizeof(struct iwl_cmd_header));
	trace_iwlwifi_dev_tx_data(trans->dev, skb,
				  sizeof(struct iwl_tx_cmd_gen3) +
				  sizeof(struct iwl_cmd_header));

	return tr;

out_err:
	iwl_pcie_ven_mode_tr_unmap(trans, txq, idx);
	return NULL;
}

static int iwl_pcie_ven_mode_enqueue_hcmd(struct iwl_trans *trans,
					  struct iwl_host_cmd *cmd)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct iwl_txq *txq = trans_pcie->txq[trans_pcie->cmd_queue];
	struct iwl_device_cmd *out_cmd;
	struct iwl_cmd_meta *out_meta;
	unsigned long flags;
	dma_addr_t phys_addr;
	int i, cmd_pos, idx;
	u16 copy_size, cmd_size;
	u8 group_id = iwl_cmd_groupid(cmd->id);
	const u8 *cmddata[IWL_MAX_CMD_TBS_PER_TFD];
	u16 cmdlen[IWL_MAX_CMD_TBS_PER_TFD];
	struct iwl_tx_transfer_desc *tr;

	copy_size = sizeof(struct iwl_cmd_header_wide);
	cmd_size = sizeof(struct iwl_cmd_header_wide);

	/*
	 * Go over the hcmd chunks and find the overall size
	 * that should be copied.
	 */
	for (i = 0; i < IWL_MAX_CMD_TBS_PER_TFD; i++) {
		cmddata[i] = cmd->data[i];
		cmdlen[i] = cmd->len[i];

		if (!cmd->len[i])
			continue;

		/* need at least IWL_FIRST_TB_SIZE copied */
		if (copy_size < IWL_FIRST_TB_SIZE) {
			int copy = IWL_FIRST_TB_SIZE - copy_size;

			if (copy > cmdlen[i])
				copy = cmdlen[i];
			cmdlen[i] -= copy;
			cmddata[i] += copy;
			copy_size += copy;
		}

		copy_size += cmdlen[i];
		cmd_size += cmd->len[i];
	}

	/*
	 * If any of the command structures end up being larger than the
	 * TFD_MAX_PAYLOAD_SIZE and they aren't dynamically allocated into
	 * separate TFDs, then we will need to increase the size of the buffers
	 */
	if (WARN(copy_size > TFD_MAX_PAYLOAD_SIZE,
		 "Command %s (%#x) is too large (%d bytes)\n",
		 iwl_get_cmd_string(trans, cmd->id), cmd->id, copy_size)) {
		idx = -EINVAL;
		goto free_dup_buf;
	}

	spin_lock_bh(&txq->lock);

	idx = iwl_pcie_get_cmd_index(txq, txq->write_ptr);
	tr = iwl_pcie_get_tfd(trans, txq, txq->write_ptr);
	memset(tr, 0, sizeof(*tr));

	if (iwl_queue_space(trans, txq) < ((cmd->flags & CMD_ASYNC) ? 2 : 1)) {
		spin_unlock_bh(&txq->lock);

		IWL_ERR(trans, "No space in command queue\n");
		iwl_op_mode_cmd_queue_full(trans->op_mode);
		idx = -ENOSPC;
		goto free_dup_buf;
	}

	out_cmd = txq->entries[idx].cmd;
	out_meta = &txq->entries[idx].meta;

	/* re-initialize to NULL */
	memset(out_meta, 0, sizeof(*out_meta));
	if (cmd->flags & CMD_WANT_SKB)
		out_meta->source = cmd;

	/* set up the header */
	out_cmd->hdr_wide.cmd = iwl_cmd_opcode(cmd->id);
	out_cmd->hdr_wide.group_id = group_id;
	out_cmd->hdr_wide.version = iwl_cmd_version(cmd->id);
	out_cmd->hdr_wide.length =
		cpu_to_le16(cmd_size - sizeof(struct iwl_cmd_header_wide));
	out_cmd->hdr_wide.reserved = 0;
	out_cmd->hdr_wide.sequence =
		cpu_to_le16(QUEUE_TO_SEQ(trans_pcie->cmd_queue) |
					 INDEX_TO_SEQ(txq->write_ptr));

	if (!(cmd->flags & CMD_ASYNC)) {
		trans_pcie->sync_cmd_data.cmd_id = out_cmd->hdr_wide.cmd;
		trans_pcie->sync_cmd_data.group_id = out_cmd->hdr_wide.group_id;
	}

	cmd_pos = sizeof(struct iwl_cmd_header_wide);

	/* and copy the data that needs to be copied */
	for (i = 0; i < IWL_MAX_CMD_TBS_PER_TFD; i++) {
		int copy;

		if (!cmd->len[i])
			continue;

		copy = cmd->len[i];
		memcpy((u8 *)out_cmd + cmd_pos, cmd->data[i], copy);
		cmd_pos += copy;
	}

	IWL_DEBUG_HC(trans,
		     "Sending command %s (%.2x.%.2x), seq: 0x%04X, %d bytes at %d[%d]:%d\n",
		     iwl_get_cmd_string(trans, cmd->id), group_id,
		     out_cmd->hdr_wide.cmd,
		     le16_to_cpu(out_cmd->hdr_wide.sequence),
		     cmd_size, txq->write_ptr, idx, trans_pcie->cmd_queue);

	/* Create TR */
	phys_addr = dma_map_single(trans->dev, (u8 *)&out_cmd->hdr_wide,
				   copy_size, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(trans->dev, phys_addr)) {
		idx = -ENOMEM;
		goto out;
	}

	dma_sync_single_for_device(trans->dev, phys_addr,
				   copy_size, DMA_BIDIRECTIONAL);

	/* Fill TR */
	tr->type_n_size = cpu_to_le32((1 & IWL_TX_TD_TYPE) |
				      ((copy_size << 8) & IWL_TX_TD_SIZE));
	put_unaligned_le64(phys_addr, &tr->addr);

	out_meta->flags = cmd->flags;
	if (WARN_ON_ONCE(txq->entries[idx].free_buf))
		kzfree(txq->entries[idx].free_buf);

	trace_iwlwifi_dev_hcmd(trans->dev, cmd, cmd_size, &out_cmd->hdr_wide);

	/* start timer if queue currently empty */
	if (txq->read_ptr == txq->write_ptr && txq->wd_timeout)
		mod_timer(&txq->stuck_timer, jiffies + txq->wd_timeout);

	spin_lock_irqsave(&trans_pcie->reg_lock, flags);
	if (!(cmd->flags & CMD_SEND_IN_IDLE) &&
	    !trans_pcie->ref_cmd_in_flight) {
		trans_pcie->ref_cmd_in_flight = true;
		IWL_DEBUG_RPM(trans, "set ref_cmd_in_flight - ref\n");
		iwl_trans_ref(trans);
	}
	/* Increment and update queue's write index */
	txq->write_ptr = iwl_queue_inc_wrap(trans, txq->write_ptr);
	iwl_pcie_gen2_txq_inc_wr_ptr(trans, txq);
	spin_unlock_irqrestore(&trans_pcie->reg_lock, flags);

out:
	spin_unlock_bh(&txq->lock);
free_dup_buf:
	return idx;
}

#define HOST_COMPLETE_TIMEOUT	(2 * HZ * CPTCFG_IWL_TIMEOUT_FACTOR)

static int iwl_pcie_ven_mode_send_hcmd_sync(struct iwl_trans *trans,
					    struct iwl_host_cmd *cmd)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	const char *cmd_str = iwl_get_cmd_string(trans, cmd->id);
	struct iwl_txq *txq = trans_pcie->txq[trans_pcie->cmd_queue];
	int cmd_idx;
	int ret;

	IWL_DEBUG_INFO(trans, "Attempting to send sync command %s\n", cmd_str);

	if (WARN(test_and_set_bit(STATUS_SYNC_HCMD_ACTIVE,
				  &trans->status),
		 "Command %s: a command is already active!\n", cmd_str))
		return -EIO;

	if ((cmd->flags & CMD_WANT_SKB) &&
	    (WARN(test_and_set_bit(STATUS_HCMD_RESP, &trans->status),
		 "Command %s: Didn't get a response!\n", cmd_str)))
		return -EIO;

	IWL_DEBUG_INFO(trans, "Setting HCMD_ACTIVE for command %s\n", cmd_str);

	if (pm_runtime_suspended(&trans_pcie->pci_dev->dev)) {
		ret = wait_event_timeout(trans_pcie->d0i3_waitq,
				 pm_runtime_active(&trans_pcie->pci_dev->dev),
				 msecs_to_jiffies(IWL_TRANS_IDLE_TIMEOUT));
		if (!ret) {
			IWL_ERR(trans, "Timeout exiting D0i3 before hcmd\n");
			return -ETIMEDOUT;
		}
	}

	cmd_idx = iwl_pcie_ven_mode_enqueue_hcmd(trans, cmd);
	if (cmd_idx < 0) {
		ret = cmd_idx;
		clear_bit(STATUS_SYNC_HCMD_ACTIVE, &trans->status);
		clear_bit(STATUS_HCMD_RESP, &trans->status);
		IWL_ERR(trans, "Error sending %s: enqueue_hcmd failed: %d\n",
			cmd_str, ret);
		return ret;
	}

	ret = wait_event_timeout(trans_pcie->wait_command_queue,
				 !test_bit(STATUS_SYNC_HCMD_ACTIVE,
					   &trans->status),
				 HOST_COMPLETE_TIMEOUT);
	if (!ret) {
		IWL_ERR(trans, "Error sending %s: time out after %dms.\n",
			cmd_str, jiffies_to_msecs(HOST_COMPLETE_TIMEOUT));

		IWL_ERR(trans, "Current CMD queue read_ptr %d write_ptr %d\n",
			txq->read_ptr, txq->write_ptr);

		clear_bit(STATUS_SYNC_HCMD_ACTIVE, &trans->status);
		clear_bit(STATUS_HCMD_RESP, &trans->status);
		IWL_DEBUG_INFO(trans, "Clearing HCMD_ACTIVE for command %s\n",
			       cmd_str);
		ret = -ETIMEDOUT;

		iwl_force_nmi(trans);
		iwl_trans_fw_error(trans);

		goto cancel;
	}

	if (test_bit(STATUS_FW_ERROR, &trans->status)) {
		IWL_ERR(trans, "FW error in SYNC CMD %s\n", cmd_str);
		dump_stack();
		ret = -EIO;
		goto cancel;
	}

	if (!(cmd->flags & CMD_SEND_IN_RFKILL) &&
	    test_bit(STATUS_RFKILL_OPMODE, &trans->status)) {
		IWL_DEBUG_RF_KILL(trans, "RFKILL in SYNC CMD... no rsp\n");
		ret = -ERFKILL;
		goto cancel;
	}

	if (cmd->flags & CMD_WANT_SKB) {
		ret = wait_event_timeout(trans_pcie->wait_command_response,
					 !test_bit(STATUS_HCMD_RESP,
						   &trans->status),
					 HOST_COMPLETE_TIMEOUT);
		if (!ret) {
			ret = -ETIMEDOUT;
			IWL_ERR(trans,
				"Didn't get notification on SYNC CMD %s\n",
				cmd_str);
			iwl_force_nmi(trans);
			goto cancel;
		}
	}

	if (cmd->flags & CMD_WANT_SKB) {
		cmd->resp_pkt = trans_pcie->sync_cmd_data.resp_pkt;
		cmd->_rx_page_addr = trans_pcie->sync_cmd_data._rx_page_addr;
		cmd->_rx_page_order = trans_pcie->sync_cmd_data._rx_page_order;
		trans_pcie->sync_cmd_data.cmd_id = -1;
	}

	if ((cmd->flags & CMD_WANT_SKB) && !cmd->resp_pkt) {
		IWL_ERR(trans, "Error: Response NULL in '%s'\n", cmd_str);
		ret = -EIO;
		goto cancel;
	}

	return 0;

cancel:
	if (cmd->flags & CMD_WANT_SKB) {
		/*
		 * Cancel the CMD_WANT_SKB flag for the cmd in the
		 * TX cmd queue. Otherwise in case the cmd comes
		 * in later, it will possibly set an invalid
		 * address (cmd->meta.source).
		 */
		txq->entries[cmd_idx].meta.flags &= ~CMD_WANT_SKB;
	}

	if (cmd->resp_pkt) {
		iwl_free_resp(cmd);
		cmd->resp_pkt = NULL;
	}

	return ret;
}

int iwl_trans_pcie_ven_mode_send_hcmd(struct iwl_trans *trans,
				      struct iwl_host_cmd *cmd)
{
	if (!(cmd->flags & CMD_SEND_IN_RFKILL) &&
	    test_bit(STATUS_RFKILL_OPMODE, &trans->status)) {
		IWL_DEBUG_RF_KILL(trans, "Dropping CMD 0x%x: RF KILL\n",
				  cmd->id);
		return -ERFKILL;
	}

	if (cmd->flags & CMD_ASYNC) {
		int ret;

		/* An asynchronous command can not expect an SKB to be set. */
		if (WARN_ON(cmd->flags & CMD_WANT_SKB))
			return -EINVAL;

		ret = iwl_pcie_ven_mode_enqueue_hcmd(trans, cmd);
		if (ret < 0) {
			IWL_ERR(trans,
				"Error sending %s: enqueue_hcmd failed: %d\n",
				iwl_get_cmd_string(trans, cmd->id), ret);
			return ret;
		}
		return 0;
	}

	return iwl_pcie_ven_mode_send_hcmd_sync(trans, cmd);
}

void iwl_pcie_ven_mode_tr_unmap(struct iwl_trans *trans,
				struct iwl_txq *txq, int index)
{
	struct iwl_tx_transfer_desc *tr =
		(void *)iwl_pcie_get_tfd(trans, txq, index);

	dma_unmap_single(trans->dev,
			 (dma_addr_t)(le64_to_cpu(tr->addr)),
			 ((le32_to_cpu(tr->type_n_size) & IWL_TX_TD_SIZE) >> 8),
			 DMA_TO_DEVICE);
}

/*
 * iwl_pcie_hcmd_complete - Pull unused buffers off the queue and reclaim them
 */
void iwl_pcie_ven_mode_cmd_complete(struct iwl_trans *trans,
				    struct iwl_tx_completion_desc *tcd)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	int txq_id = tcd->tr_id;
	int index = tcd->tag;
	struct iwl_txq *txq = trans_pcie->txq[txq_id];
	struct iwl_cmd_meta *meta;
	struct iwl_device_cmd *cmd;
	int cmd_index;
	u8 group_id;
	u32 cmd_id;

	/*
	 * If a Tx command is being handled and it isn't in the actual
	 * command queue then there a command routing bug has been introduced
	 * in the queue management code.
	 */
	if (WARN(txq_id != IWL_MSIX_VEN_MODE_IRQ_MCR &&
		 txq_id != IWL_MSIX_VEN_MODE_IRQ_HCMD,
		 "wrong command queue %d, readp=%d writep=%d\n",
		 txq_id, txq->read_ptr,
		 txq->write_ptr)) {
		return;
	}

	cmd_index = iwl_pcie_get_cmd_index(txq, index);
	cmd = txq->entries[cmd_index].cmd;
	meta = &txq->entries[cmd_index].meta;
	group_id = cmd->hdr.group_id;
	cmd_id = iwl_cmd_id(cmd->hdr.cmd, group_id, 0);

	spin_lock_bh(&txq->lock);

	iwl_pcie_ven_mode_tr_unmap(trans, txq, index);

	iwl_pcie_cmdq_reclaim(trans, txq_id, cmd_index);

	if (!(meta->flags & CMD_ASYNC)) {
		if (!test_bit(STATUS_SYNC_HCMD_ACTIVE, &trans->status)) {
			IWL_WARN(trans,
				 "HCMD_ACTIVE already clear for command %s\n",
				 iwl_get_cmd_string(trans, cmd_id));
		}
		clear_bit(STATUS_SYNC_HCMD_ACTIVE, &trans->status);

		if (meta->flags & CMD_WANT_SKB &&
		    !test_bit(STATUS_HCMD_RESP, &trans->status)) {
			IWL_WARN(trans,
				 "HCMD_RESP already clear for command %s\n",
				 iwl_get_cmd_string(trans, cmd_id));
		}

		IWL_DEBUG_INFO(trans, "Clearing HCMD_ACTIVE for command %s\n",
			       iwl_get_cmd_string(trans, cmd_id));
		wake_up(&trans_pcie->wait_command_queue);
	}

	meta->flags = 0;

	spin_unlock_bh(&txq->lock);
}

void iwl_pcie_ven_mode_hcmd_handle_resp(struct iwl_trans *trans,
					struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	u8 cmd_id = pkt->hdr.cmd, group_id = pkt->hdr.group_id;

	/* Input error checking is done when commands are added to queue. */
	if (cmd_id == trans_pcie->sync_cmd_data.cmd_id &&
	    group_id == trans_pcie->sync_cmd_data.group_id) {
		struct page *p = rxb_steal_page(rxb);

		trans_pcie->sync_cmd_data.resp_pkt = pkt;
		trans_pcie->sync_cmd_data._rx_page_addr =
			(unsigned long)page_address(p);
		trans_pcie->sync_cmd_data._rx_page_order =
			trans_pcie->rx_page_order;

		clear_bit(STATUS_HCMD_RESP, &trans->status);

		wake_up(&trans_pcie->wait_command_response);
	}
}

int iwl_trans_pcie_ven_mode_tx(struct iwl_trans *trans, struct sk_buff *skb,
			       struct iwl_device_cmd *dev_cmd, int txq_id)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct iwl_tx_cmd_gen3 *tx_cmd = (void *)dev_cmd->payload;
	struct iwl_cmd_meta *out_meta;
	struct iwl_txq *txq = trans_pcie->txq[txq_id];
	int idx;
	void *tr;

	if (WARN_ONCE(!test_bit(txq_id, trans_pcie->queue_used),
		      "TX on unused queue %d\n", txq_id))
		return -EINVAL;

	if (skb_is_nonlinear(skb) &&
	    __skb_linearize(skb))
		return -ENOMEM;

	spin_lock(&txq->lock);

	if (iwl_queue_space(trans, txq) < txq->high_mark) {
		iwl_stop_queue(trans, txq);

		/* don't put the packet on the ring, if there is no room */
		if (unlikely(iwl_queue_space(trans, txq) < 3)) {
			struct iwl_device_cmd **dev_cmd_ptr;

			dev_cmd_ptr = (void *)((u8 *)skb->cb +
					       trans_pcie->dev_cmd_offs);

			*dev_cmd_ptr = dev_cmd;
			__skb_queue_tail(&txq->overflow_q, skb);
			spin_unlock(&txq->lock);
			return 0;
		}
	}

	idx = iwl_pcie_get_cmd_index(txq, txq->write_ptr);

	/* Set up driver data for this TFD */
	txq->entries[idx].skb = skb;
	txq->entries[idx].cmd = dev_cmd;

	dev_cmd->hdr.sequence =
		cpu_to_le16((u16)(QUEUE_TO_SEQ(txq_id) |
			    INDEX_TO_SEQ(idx)));

	/* Set up first empty entry in queue's array of Tx/cmd buffers */
	out_meta = &txq->entries[idx].meta;
	out_meta->flags = 0;

	tr = iwl_pcie_ven_mode_build_tr(trans, txq, dev_cmd, skb, out_meta);
	if (!tr) {
		spin_unlock(&txq->lock);
		return -1;
	}

	/*
	 * Set up entry for this TFD in Tx byte-count array
	 * Remove 2 bytes of the padding we added for THC
	 * as they shouldn't be included in the tx cmd length.
	 */
	iwl_pcie_gen2_update_byte_tbl(trans_pcie, txq,
				      le16_to_cpu(tx_cmd->len - 2), 1);

	/* start timer if queue currently empty */
	if (txq->read_ptr == txq->write_ptr) {
		if (txq->wd_timeout)
			mod_timer(&txq->stuck_timer, jiffies + txq->wd_timeout);
		IWL_DEBUG_RPM(trans, "Q: %d first tx - take ref\n", txq->id);
		iwl_trans_ref(trans);
	}

	/* Tell device the write index *just past* this latest filled TFD */
	txq->write_ptr = iwl_queue_inc_wrap(trans, txq->write_ptr);
	iwl_pcie_gen2_txq_inc_wr_ptr(trans, txq);
	/*
	 * At this point the frame is "transmitted" successfully
	 * and we will get a TX status notification eventually.
	 */
	spin_unlock(&txq->lock);
	return 0;
}

int iwl_pcie_enqueue_ven_mode_cmd(struct iwl_trans *trans,
				  struct iwl_ven_mode_cmd *cmd)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct iwl_txq *txq = trans_pcie->txq[0];
	unsigned long flags;
	dma_addr_t phys_addr;
	struct iwl_cmd_meta *out_meta;
	int idx = iwl_pcie_get_cmd_index(txq, txq->write_ptr);
	struct iwl_tx_transfer_desc *tr =
		iwl_pcie_get_tfd(trans, txq, txq->write_ptr);
	void *data = kmalloc(cmd->len, GFP_KERNEL);
	int ret = 0;

	memset(tr, 0, sizeof(*tr));

	spin_lock_bh(&txq->lock);

	if (iwl_queue_space(trans, txq) < 2) {
		spin_unlock_bh(&txq->lock);

		IWL_ERR(trans, "No space in command queue\n");
		iwl_op_mode_cmd_queue_full(trans->op_mode);
		ret = -ENOSPC;
		goto out;
	}

	out_meta = &txq->entries[idx].meta;

	/* re-initialize to NULL */
	memset(out_meta, 0, sizeof(*out_meta));

	IWL_DEBUG_HC(trans,
		     "Sending vendor mode command type %d, %d bytes at %d[%d]:%d\n",
		     cmd->type, cmd->len, txq->write_ptr, idx, 0);

	/* Create TR */
	memcpy(data, cmd->data, cmd->len);
	phys_addr = dma_map_single(trans->dev, data,
				   cmd->len, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(trans->dev, phys_addr)) {
		ret = -ENOMEM;
		goto out;
	}

	dma_sync_single_for_device(trans->dev, phys_addr,
				   cmd->len, DMA_BIDIRECTIONAL);

	/* Fill TR */
	tr->type_n_size = cpu_to_le32((1 & IWL_TX_TD_TYPE) |
				      ((cmd->len << 8) & IWL_TX_TD_SIZE));
	put_unaligned_le64(phys_addr, &tr->addr);
	out_meta->flags = cmd->flags;

	/* TODO - Add tracepoint for ipc cmds */

	/* start timer if queue currently empty */
	if (txq->read_ptr == txq->write_ptr && txq->wd_timeout)
		mod_timer(&txq->stuck_timer, jiffies + txq->wd_timeout);

	spin_lock_irqsave(&trans_pcie->reg_lock, flags);
	if (!(cmd->flags & CMD_SEND_IN_IDLE) &&
	    !trans_pcie->ref_cmd_in_flight) {
		trans_pcie->ref_cmd_in_flight = true;
		IWL_DEBUG_RPM(trans, "set ref_cmd_in_flight - ref\n");
		iwl_trans_ref(trans);
	}

	/* Increment and update queue's write index */
	txq->write_ptr = iwl_queue_inc_wrap(trans, txq->write_ptr);
	iwl_pcie_gen2_txq_inc_wr_ptr(trans, txq);
	spin_unlock_irqrestore(&trans_pcie->reg_lock, flags);

out:

	spin_unlock_bh(&txq->lock);
	return ret;
}

#define VEN_MODE_COMPLETE_TIMEOUT	(2 * HZ * CPTCFG_IWL_TIMEOUT_FACTOR)
int iwl_pcie_send_ven_mode_cmd(struct iwl_trans *trans,
			       struct iwl_ven_mode_cmd *cmd)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct iwl_txq *txq = trans_pcie->txq[trans_pcie->mtr];
	int ret;

	if (unlikely(!(cmd->flags & CMD_SEND_IN_RFKILL) &&
		     test_bit(STATUS_RFKILL_OPMODE, &trans->status)))
		return -ERFKILL;

	if (unlikely(test_bit(STATUS_FW_ERROR, &trans->status)))
		return -EIO;

	if (unlikely(trans->state != IWL_TRANS_FW_ALIVE)) {
		IWL_ERR(trans, "%s bad state = %d\n", __func__, trans->state);
		return -EIO;
	}

	if (WARN_ONCE((cmd->flags & CMD_ASYNC), "Vendor cmds should be SYNC"))
		cmd->flags &= ~CMD_ASYNC;

	if (WARN(test_and_set_bit(STATUS_SYNC_HCMD_ACTIVE,
				  &trans->status),
		 "Command %d: a command is already active!\n", cmd->type))
		return -EIO;

	ret = iwl_pcie_enqueue_ven_mode_cmd(trans, cmd);
	if (ret) {
		clear_bit(STATUS_SYNC_HCMD_ACTIVE, &trans->status);
		IWL_ERR(trans, "Error sending %d: enqueue_hcmd failed: %d\n",
			cmd->type, ret);
		return ret;
	}

	ret = wait_event_timeout(trans_pcie->wait_command_queue,
				 !test_bit(STATUS_SYNC_HCMD_ACTIVE,
					   &trans->status),
				 VEN_MODE_COMPLETE_TIMEOUT);

	if (!ret) {
		IWL_ERR(trans, "Error sending %d: time out after %dms.\n",
			cmd->type,
			jiffies_to_msecs(VEN_MODE_COMPLETE_TIMEOUT));

		IWL_ERR(trans, "Current CMD queue read_ptr %d write_ptr %d\n",
			txq->read_ptr, txq->write_ptr);

		clear_bit(STATUS_SYNC_HCMD_ACTIVE, &trans->status);
		clear_bit(STATUS_HCMD_RESP, &trans->status);
		IWL_DEBUG_INFO(trans, "Clearing HCMD_ACTIVE for command %d\n",
			       cmd->type);

		iwl_force_nmi(trans);
		iwl_trans_fw_error(trans);

		return -ETIMEDOUT;
	}

	if (test_bit(STATUS_FW_ERROR, &trans->status)) {
		IWL_ERR(trans, "FW error in SYNC CMD %d\n", cmd->type);
		dump_stack();
		return -EIO;
	}

	return 0;
}

int iwl_pcie_ven_mode_set_txq(struct iwl_trans *trans, dma_addr_t txq_dma,
			      dma_addr_t txq_bc_dma)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct iwl_tx_cr *tx_cr;
	struct iwl_ven_mode_tr_open cmd_txq_tr;
	struct iwl_ven_mode_cr_open cmd_txq_cr;
	struct iwl_ven_mode_cmd txq_tr = {
		.type = 1,
		.len = sizeof(cmd_txq_tr),
		.data = &cmd_txq_tr,
		.flags = CMD_SEND_IN_RFKILL,
	};
	struct iwl_ven_mode_cmd txq_cr = {
		.type = 2,
		.len = sizeof(cmd_txq_cr),
		.data = &cmd_txq_cr,
		.flags = CMD_SEND_IN_RFKILL,
	};
	int ret;

	if (trans_pcie->last_tr_opened >= IWL_NUM_OF_TRANSFER_RINGS)
		return -1;

	/* Open tr and cr for txq queue */
	memset(&cmd_txq_tr, 0, sizeof(cmd_txq_tr));
	cmd_txq_tr.type = 1;
	cmd_txq_tr.ring_id = cpu_to_le16(trans_pcie->last_tr_opened);
	cmd_txq_tr.ind_arr_vec = cpu_to_le16(cmd_txq_tr.ring_id);
	cmd_txq_tr.ring_addr_dma =
		cpu_to_le64(txq_dma);
	cmd_txq_tr.bc_table_ring_addr =
		cpu_to_le64(txq_bc_dma);
	cmd_txq_tr.entries = cpu_to_le16(TFD_TX_CMD_SLOTS);
	cmd_txq_tr.completion_id = cpu_to_le16(IWL_MSIX_VEN_MODE_IRQ_TX_DATA);
	cmd_txq_tr.doorbell_vec = cpu_to_le16(IWL_MSIX_VEN_MODE_IRQ_TX_DATA);
	/* byte count entry size is 2 bytes */
	cmd_txq_tr.flags = cpu_to_le16((2 << 12) &
				       IWL_TR_OPEN_ENTRY_WID_BUF_SIZE);

	ret = iwl_pcie_send_ven_mode_cmd(trans, &txq_tr);
	if (ret)
		return ret;

	if (!iwl_pcie_ven_mode_tx_cr_alloc(trans,
					   IWL_MSIX_VEN_MODE_IRQ_TX_DATA)) {
		trans_pcie->tx_cr[IWL_MSIX_VEN_MODE_IRQ_TX_DATA]->id =
			IWL_MSIX_VEN_MODE_IRQ_TX_DATA;
		trans_pcie->tx_cr[IWL_MSIX_VEN_MODE_IRQ_TX_DATA]->read = 0;
		trans_pcie->tx_cr[IWL_MSIX_VEN_MODE_IRQ_TX_DATA]->write = 0;
	} else {
		goto done;
	}

	memset(&cmd_txq_cr, 0, sizeof(cmd_txq_cr));
	cmd_txq_cr.type = 2;
	cmd_txq_cr.optional_footer_size = 16;
	cmd_txq_cr.cr_id = cpu_to_le16(IWL_MSIX_VEN_MODE_IRQ_TX_DATA);
	cmd_txq_cr.ind_arr_vec = cpu_to_le16(cmd_txq_cr.cr_id);
	tx_cr = trans_pcie->tx_cr[cmd_txq_cr.cr_id];
	cmd_txq_cr.ring_addr_dma = cpu_to_le64(tx_cr->tcd_dma);
	cmd_txq_cr.entries = cpu_to_le16(RX_QUEUE_SIZE);
	cmd_txq_cr.cg_id = cpu_to_le16(-1);
	cmd_txq_cr.msix_vec = cpu_to_le16(cmd_txq_cr.cr_id);

	ret = iwl_pcie_send_ven_mode_cmd(trans, &txq_cr);
	if (ret)
		return ret;

done:
	trans_pcie->last_tr_opened++;

	return le16_to_cpu(cmd_txq_tr.ring_id);
}

int iwl_trans_pcie_ven_mode_dyn_txq_alloc(struct iwl_trans *trans,
					  __le16 flags, u8 sta_id, u8 tid,
					  int cmd_id, int size,
					  unsigned int timeout)
{
	struct iwl_txq *txq = NULL;
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct iwl_vendor_mode_conf_tr_cmd cmd = {
		.flags = 0,
		.sta_id = sta_id,
		.tid = tid,
	};
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(VENDOR_MODE_CONFIG_TR, VENDOR_MODE_GROUP, 0),
		.len = { sizeof(cmd) },
		.data = { &cmd, },
	};
	int ret;

	ret = iwl_trans_pcie_dyn_txq_alloc_dma(trans, &txq, size, timeout);
	if (ret)
		return ret;

	ret = iwl_pcie_ven_mode_set_txq(trans, txq->dma_addr, txq->bc_tbl.dma);
	if (ret <= 0)
		goto error;

	cmd.tr_id = cpu_to_le16(ret);

	ret = iwl_trans_send_cmd(trans, &hcmd);
	if (ret)
		goto error;

	if (cmd.tr_id >= ARRAY_SIZE(trans_pcie->txq)) {
		WARN_ONCE(1, "queue index %d unsupported", cmd.tr_id);
		ret = -EIO;
		goto error;
	}

	if (test_and_set_bit(cmd.tr_id, trans_pcie->queue_used)) {
		WARN_ONCE(1, "queue %d already used", cmd.tr_id);
		ret = -EIO;
		goto error;
	}

	txq->id = cmd.tr_id;
	trans_pcie->txq[cmd.tr_id] = txq;
	txq->read_ptr = 0;
	txq->write_ptr = 0;

	/* Place first TFD at index corresponding to start sequence number */
	iwl_write_direct32(trans, HBUS_TARG_WRPTR,
			   (txq->write_ptr) | (cmd.tr_id << 16));
	IWL_DEBUG_TX_QUEUES(trans, "Activate queue %d\n", cmd.tr_id);

	return cmd.tr_id;

error:
	iwl_pcie_gen2_txq_free_memory(trans, txq);
	return ret;
}
