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
#include "trans.h"

void iwl_pcie_ven_mode_map_rx_causes(struct iwl_trans *trans)
{
	u32 val, idx;

	val = BIT(MSIX_FH_INT_CAUSES_Q(0));
	for (idx = 1; idx < trans->num_rx_queues; idx++) {
		iwl_write8(trans, CSR_MSIX_RX_IVAR(idx),
			   MSIX_FH_INT_CAUSES_Q(idx));
		val |= BIT(MSIX_FH_INT_CAUSES_Q(idx));
	}
	/* Unmask interrupt */
	iwl_write32(trans, CSR_MSIX_FH_INT_MASK_AD, ~val);

	/* Cause 0 is used for MCR and other default causes */
	iwl_write8(trans, CSR_MSIX_RX_IVAR(0), MSIX_FH_INT_CAUSES_Q(0) |
		   MSIX_NON_AUTO_CLEAR_CAUSE);
}

int iwl_pcie_ven_mode_set_interrupt_capa(struct pci_dev *pdev,
					 struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	int num_irqs, i;

	for (i = 0; i < IWL_MAX_RX_HW_QUEUES; i++)
		trans_pcie->msix_entries[i].entry = i;

	num_irqs = pci_enable_msix_range(pdev, trans_pcie->msix_entries,
					 MSIX_MIN_INTERRUPT_VECTORS_VEN_MODE,
					 IWL_MAX_RX_HW_QUEUES);
	if (num_irqs < 0) {
		IWL_DEBUG_INFO(trans,
			       "Failed to enable msi-x mode (irq num %d).\n",
			       num_irqs);
		return -EINVAL;
	}
	trans_pcie->def_irq = 0;

	IWL_DEBUG_INFO(trans,
		       "MSI-X enabled. %d interrupt vectors were allocated\n",
		       num_irqs);

	trans_pcie->trans->num_rx_queues = num_irqs;
	trans_pcie->alloc_vecs = num_irqs;
	trans_pcie->msix_enabled = true;
	return 0;
}

int iwl_pcie_ven_mode_init_msix_handler(struct pci_dev *pdev,
					struct iwl_trans_pcie *trans_pcie)
{
	int i;

	for (i = 0; i < trans_pcie->alloc_vecs; i++) {
		int ret = 0;
		struct msix_entry *msix_entry;
		irq_handler_t handler = NULL;
		const char *qname = queue_name(&pdev->dev, trans_pcie, i);

		if (!qname)
			return -ENOMEM;

		msix_entry = &trans_pcie->msix_entries[i];
		switch (i) {
		case IWL_MSIX_VEN_MODE_IRQ_MCR:
			handler = iwl_pcie_irq_msix_handler;
			break;
		case IWL_MSIX_VEN_MODE_IRQ_HCMD:
		case IWL_MSIX_VEN_MODE_IRQ_TX_DATA:
			handler = iwl_pcie_irq_tx_completion_msix_handler;
			break;
		case IWL_MSIX_VEN_MODE_IRQ_LOG:
		case IWL_MSIX_VEN_MODE_IRQ_NOTIF:
		case IWL_MSIX_VEN_MODE_IRQ_RX_DATA:
		default:
			handler = iwl_pcie_irq_rx_msix_handler;
			break;
		}

		if (!handler)
			return -EINVAL;

		ret = devm_request_threaded_irq(&pdev->dev,
						msix_entry->vector,
						iwl_pcie_msix_isr,
						handler,
						IRQF_SHARED,
						qname,
						msix_entry);

		if (ret) {
			IWL_ERR(trans_pcie->trans,
				"Error allocating IRQ %d\n", i);

			return ret;
		}
	}

	return 0;
}

int iwl_pcie_ven_mode_tx_cr_alloc(struct iwl_trans *trans, int cr_id)
{
	struct device *dev = trans->dev;
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct iwl_tx_completion_desc *tx_cd;
	struct iwl_tx_cr *tx_cr;

	if (trans_pcie->tx_cr[cr_id])
		return -EINVAL;

	tx_cr = kzalloc(sizeof(*tx_cr), GFP_KERNEL);
	if (!tx_cr)
		return -ENOMEM;
	trans_pcie->tx_cr[cr_id] = tx_cr;

	spin_lock_init(&tx_cr->lock);

	tx_cr->queue_size = RX_QUEUE_SIZE;
	tx_cr->tcd = dma_zalloc_coherent(dev,
					 sizeof(*tx_cd) *
					 tx_cr->queue_size,
					 &tx_cr->tcd_dma,
					 GFP_KERNEL);
	if (!tx_cr->tcd) {
		kfree(tx_cr);
		return -ENOMEM;
	}

	return 0;
}

static int iwl_pcie_ven_mode_txq_init(struct iwl_trans *trans, int txq_id)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	int ret;

	ret = iwl_pcie_gen2_tx_init(trans, txq_id, TFD_TX_CMD_SLOTS);
	if (ret)
		goto error;

	/* Allocate the completion ring for this TR */
	if (!trans_pcie->tx_cr[txq_id]) {
		ret = iwl_pcie_ven_mode_tx_cr_alloc(trans, txq_id);
		if (ret)
			return ret;
	}
	trans_pcie->tx_cr[txq_id]->id = txq_id;
	trans_pcie->tx_cr[txq_id]->read = 0;
	trans_pcie->tx_cr[txq_id]->write = 0;

	return 0;

error:
	iwl_pcie_gen2_tx_free(trans);
	return ret;
}

static int iwl_pcie_ven_mode_rx_init(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	int ret, i, rxq_count = 0;

	ret = _iwl_pcie_rx_init(trans);
	if (ret)
		return ret;

	/**
	 *  Vendor mode defines special id's for the rx queues.
	 *  Set them here.
	 */
	for (i = 0; i < trans->num_rx_queues; i++) {
		struct iwl_rxq *rxq = &trans_pcie->rxq[i];

		if (i == 0) {
			rxq->id = IWL_MSIX_VEN_MODE_IRQ_NOTIF;
		} else {
			rxq->id = IWL_MSIX_VEN_MODE_IRQ_RX_DATA +
				rxq_count;
			rxq_count++;
		}
	}

	return 0;
}

static void iwl_pcie_ven_mode_free_head_tail(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);

	if (trans_pcie->cr_head)
		dma_free_coherent(trans->dev,
				  sizeof(__le16) * 32,
				  trans_pcie->cr_head,
				  trans_pcie->cr_head_dma_addr);
	trans_pcie->cr_head = NULL;

	if (trans_pcie->cr_tail)
		dma_free_coherent(trans->dev,
				  sizeof(__le16) * 32,
				  trans_pcie->cr_tail,
				  trans_pcie->cr_tail_dma_addr);
	trans_pcie->cr_tail = NULL;

	if (trans_pcie->tr_tail)
		dma_free_coherent(trans->dev,
				  sizeof(__le16) * 534,
				  trans_pcie->tr_tail,
				  trans_pcie->tr_tail_dma_addr);
	trans_pcie->tr_tail = NULL;
}

static int iwl_pcie_ven_mode_init_head_tail(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct device *dev = trans->dev;

	/* Allocate the cr head - pointer to receive buffer */
	trans_pcie->cr_head =
		dma_zalloc_coherent(dev, IWL_VEN_MODE_CR_NUM * sizeof(__le16),
				    &trans_pcie->cr_head_dma_addr,
				    GFP_KERNEL);
	if (!trans_pcie->cr_head)
		goto err;

	/* Allocate the cr tail - pointer to read receive buffer */
	trans_pcie->cr_tail =
		dma_zalloc_coherent(dev, IWL_VEN_MODE_CR_NUM * sizeof(__le16),
				    &trans_pcie->cr_tail_dma_addr,
				    GFP_KERNEL);
	if (!trans_pcie->cr_tail)
		goto err;

	/* Allocate the cr tail - pointer to read receive buffer */
	trans_pcie->tr_tail =
		dma_zalloc_coherent(dev,
				    IWL_NUM_OF_TRANSFER_RINGS * sizeof(__le16),
				    &trans_pcie->tr_tail_dma_addr,
				    GFP_KERNEL);
	if (!trans_pcie->tr_tail)
		goto err;

	return 0;

err:
	iwl_pcie_ven_mode_free_head_tail(trans);

	return -ENOMEM;
}

static int iwl_pcie_ven_mode_nic_init(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	int i;

	spin_lock(&trans_pcie->irq_lock);
	iwl_pcie_gen2_apm_init(trans);
	spin_unlock(&trans_pcie->irq_lock);

	trans_pcie->mtr = 0;
	trans_pcie->last_tr_opened = IWL_MSIX_VEN_MODE_IRQ_NOTIF;

	iwl_op_mode_nic_config(trans->op_mode);

	/* Allocate or reset MTR and cmd queue */
	for (i = 0; i <= trans_pcie->cmd_queue; i++)
		if (iwl_pcie_ven_mode_txq_init(trans, i))
			return -ENOMEM;

	/* Allocate the command RX queue, or reset if it is already allocated */
	if (iwl_pcie_ven_mode_rx_init(trans))
		return -ENOMEM;

	/* alloc the rings tail and head lists */
	if (iwl_pcie_ven_mode_init_head_tail(trans))
		return -ENOMEM;

	/* enable shadow regs in HW */
	iwl_set_bit(trans, CSR_MAC_SHADOW_REG_CTRL, 0x800FFFFF);
	IWL_DEBUG_INFO(trans, "Enabling shadow registers in device\n");

	return 0;
}

int iwl_trans_pcie_ven_mode_start_fw(struct iwl_trans *trans,
				     const struct fw_img *fw,
				     bool run_in_rfkill)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	bool hw_rfkill;
	int ret;

	trans_pcie->ven_mode_init_mode = IWL_VEN_MODE_STARTING_IML;

	/* This may fail if AMT took ownership of the device */
	if (iwl_pcie_prepare_card_hw(trans)) {
		IWL_WARN(trans, "Exit HW not ready\n");
		ret = -EIO;
		goto out;
	}

	iwl_enable_rfkill_int(trans);

	iwl_write32(trans, CSR_INT, 0xFFFFFFFF);

	/*
	 * We enabled the RF-Kill interrupt and the handler may very
	 * well be running. Disable the interrupts to make sure no other
	 * interrupt can be fired.
	 */
	iwl_disable_interrupts(trans);

	/* Make sure it finished running */
	iwl_pcie_synchronize_irqs(trans);

	mutex_lock(&trans_pcie->mutex);

	/* If platform's RF_KILL switch is NOT set to KILL */
	hw_rfkill = iwl_pcie_check_hw_rf_kill(trans);
	if (hw_rfkill && !run_in_rfkill) {
		ret = -ERFKILL;
		goto out;
	}

	/* Someone called stop_device, don't try to start_fw */
	if (trans_pcie->is_down) {
		IWL_WARN(trans,
			 "Can't start_fw since the HW hasn't been started\n");
		ret = -EIO;
		goto out;
	}

	/* make sure rfkill handshake bits are cleared */
	iwl_write32(trans, CSR_UCODE_DRV_GP1_CLR, CSR_UCODE_SW_BIT_RFKILL);
	iwl_write32(trans, CSR_UCODE_DRV_GP1_CLR,
		    CSR_UCODE_DRV_GP1_BIT_CMD_BLOCKED);

	/* clear (again), then enable host interrupts */
	iwl_write32(trans, CSR_INT, 0xFFFFFFFF);

	ret = iwl_pcie_ven_mode_nic_init(trans);
	if (ret) {
		IWL_ERR(trans, "Unable to init nic\n");
		goto out;
	}

	ret = iwl_pcie_ctxt_info_gen3_init(trans, fw);
	if (ret)
		goto out;

	/* re-check RF-Kill state since we may have missed the interrupt */
	hw_rfkill = iwl_pcie_check_hw_rf_kill(trans);
	if (hw_rfkill && !run_in_rfkill)
		ret = -ERFKILL;

	ret = wait_event_timeout(trans_pcie->ven_mode_init_state,
				 (trans_pcie->ven_mode_init_mode ==
				  IWL_VEN_MODE_DOWNLOAD_FW),
				 CPTCFG_IWL_TIMEOUT_FACTOR * HZ);
	if (!ret) {
		ret = !ret;
		goto out;
	}

	ret = 0;

out:
	mutex_unlock(&trans_pcie->mutex);
	return ret;
}

static int iwl_pcie_ven_mode_set_cmd_queue(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct iwl_ven_mode_tr_open cmd_hcmd_tr = {
		.type = IWL_VEN_MODE_OPEN_TR,
		.ring_id = cpu_to_le16(trans_pcie->cmd_queue),
		.ind_arr_vec = cpu_to_le16(trans_pcie->cmd_queue),
		.ring_addr_dma =
			cpu_to_le64(trans_pcie->txq[trans_pcie->cmd_queue]->dma_addr),
		.bc_table_ring_addr =
			cpu_to_le64(trans_pcie->txq[trans_pcie->cmd_queue]->bc_tbl.dma),
		.entries = cpu_to_le16(TFD_TX_CMD_SLOTS),
		.completion_id = cpu_to_le16(trans_pcie->cmd_queue),
		.doorbell_vec = cpu_to_le16(trans_pcie->cmd_queue),
		.flags =
			cpu_to_le16((2 << 12) & IWL_TR_OPEN_ENTRY_WID_BUF_SIZE)
	}, cmd_notif_tr = {
		.type = IWL_VEN_MODE_OPEN_TR,
		.ring_id = cpu_to_le16(512),
		.ind_arr_vec = cmd_notif_tr.ring_id,
		.ring_addr_dma = cpu_to_le64(trans_pcie->rxq->bd_dma),
		.entries = cpu_to_le16(trans_pcie->rxq->queue_size),
		.completion_id = 0,
		.doorbell_vec = cmd_notif_tr.ring_id,
		.flags = cpu_to_le16(IWL_TR_OPEN_OUT_ORD_COMP | IWL_TR_OPEN_COMP_TYPE)
	};
	struct iwl_ven_mode_cr_open cmd_hcmd_cr = {
		.type = IWL_VEN_MODE_OPEN_CR,
		.optional_footer_size = 16,
		.cr_id = cpu_to_le16(IWL_MSIX_VEN_MODE_IRQ_HCMD),
		.ind_arr_vec = cpu_to_le16(IWL_MSIX_VEN_MODE_IRQ_HCMD),
		.ring_addr_dma =
			cpu_to_le64(trans_pcie->tx_cr[IWL_MSIX_VEN_MODE_IRQ_HCMD]->tcd_dma),
		.entries = cpu_to_le16(RX_QUEUE_SIZE),
		.cg_id = cpu_to_le16(-1),
		.msix_vec = cpu_to_le16(IWL_MSIX_VEN_MODE_IRQ_HCMD)
	}, cmd_notif_cr = {
		.type = IWL_VEN_MODE_OPEN_CR,
		.optional_footer_size = 16,
		.cr_id = cpu_to_le16(IWL_MSIX_VEN_MODE_IRQ_NOTIF),
		.ind_arr_vec = cpu_to_le16(cmd_notif_cr.cr_id),
		.ring_addr_dma = cpu_to_le64(trans_pcie->rxq->used_bd_dma),
		.entries = cpu_to_le16(trans_pcie->rxq->queue_size),
		.cg_id = cpu_to_le16(0),
		.msix_vec = cpu_to_le16(cmd_notif_cr.cr_id)
	};
	struct iwl_ven_mode_cmd cmd_tr = {
		.type = IWL_VEN_MODE_OPEN_TR,
		.len = sizeof(cmd_hcmd_tr),
		.data = &cmd_hcmd_tr,
		.flags = CMD_SEND_IN_RFKILL,
	};
	struct iwl_ven_mode_cmd notif_tr = {
		.type = IWL_VEN_MODE_OPEN_TR,
		.len = sizeof(cmd_notif_tr),
		.data = &cmd_notif_tr,
		.flags = CMD_SEND_IN_RFKILL,
	};
	struct iwl_ven_mode_cmd hcmd_cr = {
		.type = IWL_VEN_MODE_OPEN_CR,
		.len = sizeof(cmd_hcmd_cr),
		.data = &cmd_hcmd_cr,
		.flags = CMD_SEND_IN_RFKILL,
	};
	struct iwl_ven_mode_cmd notif_cr = {
		.type = IWL_VEN_MODE_OPEN_CR,
		.len = sizeof(cmd_notif_cr),
		.data = &cmd_notif_cr,
		.flags = CMD_SEND_IN_RFKILL,
	};
	int ret;

	ret = iwl_pcie_send_ven_mode_cmd(trans, &cmd_tr);
	if (ret)
		return ret;

	ret = iwl_pcie_send_ven_mode_cmd(trans, &hcmd_cr);
	if (ret)
		return ret;

	ret = iwl_pcie_send_ven_mode_cmd(trans, &notif_tr);
	if (ret)
		return ret;

	ret = iwl_pcie_send_ven_mode_cmd(trans, &notif_cr);
	if (ret)
		return ret;

	return 0;
}

static int iwl_pcie_download_fw_chunk(struct iwl_trans *trans,
				      struct iwl_self_init_dram *dram,
				      void *curr_chunk, int chunk_size,
				      int chunk_num, int section_type)
{
	int ret;

	while (chunk_size > 0) {
		struct iwl_pcie_ven_mode_fw_dl *fw_dl_cmd;
		unsigned int copy_size = (chunk_size > BIT(11)) ?
			BIT(11) : chunk_size;
		struct iwl_host_cmd hcmd = {
			.id = WIDE_ID(SYSTEM_GROUP, DOWNLOAD_FW_CHUNK),
		};
		int cmd_size = copy_size + sizeof(*fw_dl_cmd);

		cmd_size = ALIGN(cmd_size, 4);
		hcmd.len[0] = cmd_size;

		fw_dl_cmd = kzalloc(cmd_size, GFP_KERNEL);

		if (!fw_dl_cmd) {
			ret = -ENOMEM;
			return ret;
		}

		hcmd.data[0] = fw_dl_cmd;

		fw_dl_cmd->section_type = section_type;
		fw_dl_cmd->section_num = chunk_num;
		fw_dl_cmd->chunk_size = cpu_to_le16(copy_size);

		memcpy(fw_dl_cmd->data, curr_chunk, copy_size);

		ret = iwl_trans_send_cmd(trans, &hcmd);
		if (ret)
			return ret;

		kfree(fw_dl_cmd);

		curr_chunk += copy_size;
		chunk_size -= copy_size;
	}

	return 0;
}

#define FW_ALIVE_TIMEOUT	(HZ * CPTCFG_IWL_TIMEOUT_FACTOR)
int iwl_trans_pcie_ven_mode_send_fw(struct iwl_trans *trans,
				    const struct fw_img *fw)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct iwl_pcie_ven_mode_trig_fw_dl trig_dl = {};
	struct iwl_self_init_dram *dram = &trans_pcie->init_dram;
	struct iwl_host_cmd trig_cmd = {
		.id = WIDE_ID(SYSTEM_GROUP, BEGIN_FW_DOWNLOAD),
		.data = { &trig_dl, },
		.len = { sizeof(trig_dl), },
		.flags = CMD_ASYNC,
	};
	int i, ret, err, lmac_cnt, umac_cnt;

	ret = iwl_pcie_ven_mode_set_cmd_queue(trans);
	if (ret)
		return ret;

	/* trigger fw download */
	ret = iwl_trans_send_cmd(trans, &trig_cmd);
	if (ret)
		return ret;

	lmac_cnt = iwl_pcie_get_num_sections(fw, 0);
	/* add 1 due to separator */
	umac_cnt = iwl_pcie_get_num_sections(fw, lmac_cnt + 1);

	/* send umac sections */
	for (i = 0; i < umac_cnt; i++) {
		void *curr_chunk = dram->fw[i + lmac_cnt].block;
		int chunk_size = dram->fw[i + lmac_cnt].size;

		ret = iwl_pcie_download_fw_chunk(trans, dram, curr_chunk,
						 chunk_size, i,
						 IWL_IMAGE_UMAC);
		if (ret)
			return ret;
	}

	ret = iwl_trans_send_cmd(trans, &trig_cmd);
	if (ret)
		return ret;

	/* send lmac sections */
	for (i = 0; i < lmac_cnt; i++) {
		void *curr_chunk = dram->fw[i].block;
		int chunk_size = dram->fw[i].size;

		ret = iwl_pcie_download_fw_chunk(trans, dram, curr_chunk,
						 chunk_size, i,
						 IWL_IMAGE_LMAC);
		if (ret)
			return ret;
	}

	err = wait_event_timeout(trans_pcie->ven_mode_init_state,
				 (trans_pcie->ven_mode_init_mode ==
				  IWL_VEN_MODE_ALIVE),
				 FW_ALIVE_TIMEOUT);
	if (!err) {
		IWL_ERR(trans, "Vendor mode alive timeout, init state is %d\n",
			trans_pcie->ven_mode_init_mode);
		return !err;
	}

	return ret;
}
