/******************************************************************************
 *
 * Copyright(c) 2018 Intel Corporation
 *
 * Portions of this file are derived from the ipw3945 project, as well
 * as portions of the ieee80211 subsystem header files.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 *****************************************************************************/
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/gfp.h>

#include "iwl-prph.h"
#include "iwl-io.h"
#include "../internal.h"
#include "iwl-op-mode.h"
#include "iwl-context-info-gen3.h"
#include "fw/api/tx.h"
#include "trans.h"

struct iwl_tx_cr_id {
	int irq;
	int tx_cr_slot;
};

static struct iwl_trans_pcie *iwl_pcie_get_trans_pcie(struct msix_entry *entry)
{
	u8 queue = entry->entry;
	struct msix_entry *entries = entry - queue;

	return container_of(entries, struct iwl_trans_pcie, msix_entries[0]);
}

static u32 iwl_msix_get_entry(struct iwl_trans_pcie *trans_pcie, int irq)
{
	int i;

	for (i = 0; i < trans_pcie->alloc_vecs; i++) {
		struct msix_entry *msix_entry;

		msix_entry = &trans_pcie->msix_entries[i];
		if (msix_entry->vector == irq)
			return msix_entry->entry;
	}

	return -1;
}

/*
 * iwl_pcie_tx_completion_handle - responses on mtr, hcmd and tx
 */
void iwl_pcie_tx_completion_handle(struct iwl_trans *trans, int irq)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	u32 r, i = 0, tx_cr_slot = iwl_msix_get_entry(trans_pcie, irq);
	struct iwl_tx_cr *tx_cr =
		trans_pcie->tx_cr[tx_cr_slot];
	__le16 *cr_head = &trans_pcie->cr_head[tx_cr_slot];
	__le16 *cr_tail = &trans_pcie->cr_tail[tx_cr_slot];

	spin_lock(&tx_cr->lock);

	/*
	 * uCode's read index (stored in shared DRAM) indicates the last Rx
	 * buffer that the driver may process (last buffer filled by ucode).
	 */
	r = le16_to_cpu(ACCESS_ONCE(*cr_head)) & 0x0FFF;
	i = tx_cr->read;

	/* Interrupt, but nothing sent from uCode */
	if (i == r)
		IWL_DEBUG_RX(trans, "Q %d: HW = SW = %d\n", tx_cr->id, r);

	while (i != r) {
		struct iwl_tx_completion_desc *tcd = (void *)&tx_cr->tcd[i];

		IWL_DEBUG_RX(trans, "Q %d: HW = %d, SW = %d\n", tx_cr->id,
			     r, i);

		if (tx_cr_slot == IWL_MSIX_VEN_MODE_IRQ_TX_DATA)
			iwl_op_mode_rx_ven_mode_cr(trans->op_mode, tcd,
						   tx_cr_slot);
		else if (tx_cr_slot == IWL_MSIX_VEN_MODE_IRQ_MCR ||
			 tx_cr_slot == IWL_MSIX_VEN_MODE_IRQ_HCMD)
			iwl_pcie_ven_mode_cmd_complete(trans, tcd);

		i = (i + 1) & (tx_cr->queue_size - 1);
	}

	/* Backtrack one entry */
	tx_cr->read = i;
	*cr_tail = r;
	spin_unlock(&tx_cr->lock);
}

/*
 * iwl_pcie_tx_completion_msix_handle -
 * entry function for receiving responses on tx
 */
irqreturn_t iwl_pcie_irq_tx_completion_msix_handler(int irq, void *dev_id)
{
	struct msix_entry *entry = dev_id;
	struct iwl_trans_pcie *trans_pcie = iwl_pcie_get_trans_pcie(entry);
	struct iwl_trans *trans = trans_pcie->trans;

	trace_iwlwifi_dev_irq_msix(trans->dev, entry, false, 0, 0);

	if (WARN_ON(entry->entry >= trans->num_rx_queues))
		return IRQ_NONE;

	lock_map_acquire(&trans->sync_cmd_lockdep_map);

	local_bh_disable();
	iwl_pcie_tx_completion_handle(trans, irq);
	local_bh_enable();

	iwl_pcie_clear_irq(trans, entry);

	lock_map_release(&trans->sync_cmd_lockdep_map);

	return IRQ_HANDLED;
}
