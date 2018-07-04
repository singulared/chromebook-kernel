/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
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
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
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

#ifndef __iwl_pcie_vendor_mode__
#define __iwl_pcie_vendor_mode__

#include <linux/pm_runtime.h>
#include <net/tso.h>

#include "iwl-debug.h"
#include "iwl-csr.h"
#include "iwl-io.h"
#include "../internal.h"
#include "fw/api/tx.h"
#include "fw/api/commands.h"
#include "iwl-context-info-gen3.h"

#define CSR_VEN_MODE_STATE_UNINIT	0x0
#define CSR_VEN_MODE_STATE_INIT		0x1
#define CSR_VEN_MODE_STATE_RUNNING	0x2
#define CSR_VEN_MODE_STATE_ERROR	0x4

#define CSR_VEN_MODE_BOOT_ROM		0x00
#define CSR_VEN_MODE_BOOT_SBS		0x01
#define CSR_VEN_MODE_BOOT_OS		0x02
#define CSR_VEN_MODE_BOOT_ABORT		0x03
#define CSR_VEN_MODE_BOOT_ALIVE		0x12

#define CSR_VEN_MODE_BOOT_STAGE		0x108
#define CSR_VEN_MODE_CONTROL		0x10c
#define CSR_VEN_MODE_STATUS		0x110

#define IWL_VEN_MODE_NUM_TBS		1

#define VENDOR_MODE_GROUP      0x11

/**
 * enum iwl_vendor_mode_cmds - Supported vendor mode commands
 */
enum iwl_vendor_mode_cmds {
	/* Commands */
	/**
	 * @VENDOR_MODE_CONFIG_TR:
	 * Perform a scan using configuration
	 * defined in &struct iwl_fmac_scan_cmd.
	 */
	VENDOR_MODE_CONFIG_TR = 0x0,
};

/**
 * enum iwl_config_tr_flags - configuring tr flags
 * @IWL_CONFIG_TR_HDR_CVR_DISABLE: disable 802.3 header conversion
 * @IWL_CONFIG_TR_AMPDU_OF_AMSDU: ampdu of amsdu's
 * @IWL_CONFIG_TR_AGG_DIS: disable aggregation
 */
enum iwl_config_tr_flags {
	IWL_CONFIG_TR_HDR_CVR_DISABLE		= BIT(0),
	IWL_CONFIG_TR_AMPDU_OF_AMSDU		= BIT(1),
	IWL_CONFIG_TR_AGG_DIS			= BIT(2),
};

/**
 * struct iwl_vendor_mode_conf_tr_cmd - match tr to ra-tid
 * @tr_id: tr number
 * @sta_id: station id
 * @tid: queue tid
 * @flags: see &enum iwl_config_tr_flags
 * @packet_life_time: time till dropping by FW, in usec
 */
struct iwl_vendor_mode_conf_tr_cmd {
	__le16 tr_id;
	u8 sta_id;
	u8 tid;
	__le32 flags;
	__le32 packet_life_time;
} __packed /* WIFI_TX_TR_CONFIG_API_S_VER_1 */;

/**
 * enum iwl_context_info_mr_flags - message rings flags
 * @IWL_MR_OUT_OF_ORDER: Indicates whether out of order completions
 *	allowed for the MTR
 * @IWL_MR_IN_PLACE_COMP: Indicates whether in place completion is used
 */
enum iwl_context_info_mr_flags {
	IWL_MR_OUT_OF_ORDER = BIT(0),
	IWL_MR_IN_PLACE_COMP = BIT(1),
};

/**
 * struct iwl_tx_cr - tx completion ring
 * @id: queue index
 * @tcd: pointer to the list of tx completion descriptors
 * @tcd_dma: dma address to pass to the ucode
 * @cr_tail: driver's pointer to the completion ring tail buffer
 * @cr_tail_dma: physical address of the buffer for the completion ring tail
 * @read: Shared index to newest available Rx buffer
 * @write: Shared index to oldest written Rx packet
 * @queue_size: the size of the tx completion ring
 * @rb_stts: driver's pointer to receive buffer status
 * @rb_stts_dma: bus address of receive buffer status
 * @lock: lock for the tx_cr
 */
struct iwl_tx_cr {
	int id;
	struct iwl_tx_completion_desc *tcd;
	dma_addr_t tcd_dma;
	__le16 *cr_tail;
	dma_addr_t cr_tail_dma;
	u32 read;
	u32 write;
	u32 queue_size;
	__le16 *rb_stts;
	dma_addr_t rb_stts_dma;
	spinlock_t lock; /* protects completion process */
};

#define MSIX_MIN_INTERRUPT_VECTORS_VEN_MODE	6
#define IWL_TX_TD_TYPE				0xff
#define IWL_TX_TD_SIZE				0xffffff00

/**
 * struct iwl_tx_transfer_desc - tx transfer descriptor
 * @type_n_size: buffer type (bit 0: external buff valid,
 *	bit 1: optional footer valid, bit 2-7: reserved)
 *	size - buffer size (stored in bits 8-31)
 * @addr: address of the tx packet
 * @reserved: reserved
 */
struct iwl_tx_transfer_desc {
	__le32 type_n_size;
	__le64 addr;
	__le32 reserved;
} __packed;

/**
 * enum iwl_ven_mode_ring_cmd_types
 * @IWL_VEN_MODE_OPEN_TR: open transfer ring command type
 * @IWL_VEN_MODE_OPEN_CR: open completion ring command type
 */
enum iwl_ven_mode_ring_cmd_types {
	IWL_VEN_MODE_OPEN_TR = 1,
	IWL_VEN_MODE_OPEN_CR = 2,
};

/**
 * struct iwl_ven_mode_cmd - vendor mode cmd to the ucode
 *
 * @type: vendor mode command type
 * @reserved: reserved
 * @flags: can be CMD_*, &enum CMD_MODE
 * @len: length of the data
 * @data: array of chunks that composes the data of the host command
 */
struct iwl_ven_mode_cmd {
	u8 type;
	u8 reserved[5];
	u32 flags;
	u16 len;
	void *data;
};

#define IWL_TR_OPEN_ENTRY_WID_BUF_SIZE	0x3800
#define IWL_CD_STTS_ICE_SPEC_MASK	0xf
#define IWL_CD_STTS_WIFI_SPEC_MASK	0xf

/**
 * enum iwl_tr_open_flags
 * @IWL_TR_OPEN_OPT_COMP: Indicates if optimized completion is enabled
 * @IWL_TR_OPEN_REL: Indicates if TR is reliable
 * @IWL_TR_OPEN_OUT_ORD_COMP: whether out of order completions allowed for
 *	the TR
 * @IWL_TR_OPEN_IN_PLACE_COMP: whether in place completion is used for the TR
 * @IWL_TR_OPEN_VIRT_TR: Indicates whether this is a Virtual TR
 * @IWL_TR_OPEN_SYNC: Indicates whether TR is associated with
 *	synchronized UL DL
 * @IWL_TR_OPEN_COMP_TYPE: Indicates whether Completion ID is associated
 *	with a ring or group
 * @IWL_TR_OPEN_ENTRY_CH_BUF_SIZE: Indicates whether Buffer Size Ring Entry
 * shall indicate chained TDs via most significant bit
 */
enum iwl_tr_open_flags {
	IWL_TR_OPEN_OPT_COMP		= BIT(3),
	IWL_TR_OPEN_REL			= BIT(4),
	IWL_TR_OPEN_OUT_ORD_COMP	= BIT(5),
	IWL_TR_OPEN_IN_PLACE_COMP	= BIT(6),
	IWL_TR_OPEN_VIRT_TR		= BIT(7),
	IWL_TR_OPEN_SYNC		= BIT(8),
	IWL_TR_OPEN_COMP_TYPE		= BIT(9),
	IWL_TR_OPEN_ENTRY_CH_BUF_SIZE	= BIT(10),
};

/**
 * struct iwl_ven_mode_tr_open - opening a new tr
 * @type: message type
 * @optional_header_size: Size of the Optional Header in the TD associated
 * @optional_footer_size: Size of the Optional Footer in the TD associated
 * @reserved1: reserved
 * @ring_id: Ring ID this message is for
 * @ind_arr_vec: Index Array Vector to use for the Ring
 * @ring_addr_dma: 64-bit address of ring in host memory
 * @bc_table_ring_addr: 64-bit address of buffer size ring in host memory
 * @entries: This field gives the number of TDs in the ring
 * @completion_id: The ID of the completion ring or completion group which
 *	corresponds to the TR
 * @doorbell_vec: The Doorbell Vector associated with the TR
 * @flags: tr open flags depicted in &enum iwl_tr_open_flags
 * @priority: the peripheral the priority with which data shall be
 *	dequeued from the TR
 * @msix_vec: The MSI which shall be generated by the peripheral after
 *	updating TR entry
 * @msix_mod_del: After updating TR entry, the maximum time, in µsec,
 *	after which an MSI shall be generated
 * @msix_mod_bytes: After updating TR entry, the maximum number of bytes
 *	corresponding to the TD, after which an MSI shall be generated by
 *	the peripheral
 * @accum_del: After receiving DL data, the maximum time, in µsec,
 *	after which the peripheral shall transfer the data over the
 *	link and complete the buffer
 * @accum_bytes: The maximum number of bytes of DL data accumulated after which
 *	the peripheral shall transfer the data over the link and
 *	complete the buffer
 * @reserved2: reserved
 * @data: Optional field, describing data specific to MAC
 */
struct iwl_ven_mode_tr_open {
	u8 type;
	u8 optional_header_size;
	u8 optional_footer_size;
	u8 reserved1;
	__le16 ring_id;
	__le16 ind_arr_vec;
	__le64 ring_addr_dma;
	__le64 bc_table_ring_addr;
	__le16 entries;
	__le16 completion_id;
	__le16 doorbell_vec;
	__le16 flags;
	__le32 priority;
	__le16 msix_vec;
	__le16 msix_mod_del;
	__le32 msix_mod_bytes;
	__le16 accum_del;
	__le32 accum_bytes;
	__le16 reserved2;
	u8 data[0];
} __packed;

/**
 * struct iwl_ven_mode_cr_open - opening a new cr
 * @type: message type
 * @optional_header_size: Size of the Optional Header in the CD
 *	associated with the CR
 * @optional_footer_size: Size of the Optional Footer in the CD
 *	associated with the CR
 * @reserved1: reserved
 * @cr_id: CR ID this message is for
 * @ind_arr_vec: Index Array Vector to use for the Ring
 * @ring_addr_dma: 64-bit address of Ring in host memory
 * @entries: This field gives the number of CDs in the Ring
 * @cg_id: Completion group this ring is associated with
 * @doorbell_vec: The Doorbell Vector associated with the CR
 * @flags: CR flags
 * @priority: the peripheral the priority of the CR
 * @msix_vec: The MSI which shall be generated by the
 *	peripheral for the CD queued to this CR
 * @msix_mod_del: After queueing a CD, the maximum time, in
 *	µsec, after which an MSI shall be generated by the peripheral
 * @msix_mod_bytes: The maximum number of bytes completed after which
 *	an MSI shall be generated by the peripheral
 * @accum_del: After receiving DL data, the maximum time, in µsec,
 *	after which the peripheral shall transfer the data over the
 *	link and complete the buffer
 * @accum_bytes: The maximum number of bytes of DL data accumulated after
 *	which the peripheral shall transfer the data over the link and
 *	complete the buffer
 * @reserved2: reserved
 * @data: Optional field, describing data specific to MAC
 */
struct iwl_ven_mode_cr_open {
	u8 type;
	u8 optional_header_size;
	u8 optional_footer_size;
	u8 reserved1;
	__le16 cr_id;
	__le16 ind_arr_vec;
	__le64 ring_addr_dma;
	__le16 entries;
	__le16 cg_id;
	__le16 doorbell_vec;
	__le16 flags;
	__le32 priority;
	__le16 msix_vec;
	__le16 msix_mod_del;
	__le32 msix_mod_bytes;
	__le16 accum_del;
	__le32 accum_bytes;
	__le16 reserved2;
	u8 data[0];
} __packed;

/**
 * enum iwl_image_load_type - downloaded image type
 * @IWL_IMAGE_UMAC: umac image
 * @IWL_IMAGE_LMAC: lmac image, copied by the fw for cdb
 */
enum iwl_load_image_type {
	IWL_IMAGE_UMAC			= 0,
	IWL_IMAGE_LMAC			= 1,
};

/**
 * struct iwl_pcie_ven_mode_fw_dl - download fw chunk
 * @section_type: part of enum iwl_image_load_type
 * @section_num: descriptor number, consecutive
 * @chunk_size: the size of the sent chunk
 * @data: the chunk itself
 */
struct iwl_pcie_ven_mode_fw_dl {
	u8 section_type;
	u8 section_num;
	__le16 chunk_size;
	__le32 reserved;
	u8 data[0];
} __packed; /* FW_IMAGE_BLOCK_INFO_VER_1 */

/**
 * struct iwl_pcie_ven_mode_trig_fw_dl - trigger fw download
 * @flags: download flags
 */
struct iwl_pcie_ven_mode_trig_fw_dl {
	__le32 flags;
} __packed; /* FW_IMAGE_DOWNLOAD_TRIGGER_VER_1 */

void iwl_pcie_ven_mode_map_rx_causes(struct iwl_trans *trans);
int iwl_pcie_ven_mode_set_interrupt_capa(struct pci_dev *pdev,
					 struct iwl_trans *trans);
int iwl_pcie_ven_mode_init_msix_handler(struct pci_dev *pdev,
					struct iwl_trans_pcie *trans_pcie);
int iwl_pcie_ven_mode_tx_cr_alloc(struct iwl_trans *trans, int cr_id);
int iwl_trans_pcie_ven_mode_start_fw(struct iwl_trans *trans,
				     const struct fw_img *fw,
				     bool run_in_rfkill);
int iwl_trans_pcie_ven_mode_send_hcmd(struct iwl_trans *trans,
				      struct iwl_host_cmd *cmd);
irqreturn_t iwl_pcie_irq_tx_completion_msix_handler(int irq, void *dev_id);
void iwl_pcie_ven_mode_cmd_complete(struct iwl_trans *trans,
				    struct iwl_tx_completion_desc *tcd);
void iwl_pcie_ven_mode_hcmd_handle_resp(struct iwl_trans *trans,
					struct iwl_rx_cmd_buffer *rxb);
int iwl_pcie_send_ven_mode_cmd(struct iwl_trans *trans,
			       struct iwl_ven_mode_cmd *cmd);
int iwl_trans_pcie_ven_mode_send_fw(struct iwl_trans *trans,
				    const struct fw_img *fw);
void iwl_pcie_tx_completion_handle(struct iwl_trans *trans, int irq);
int iwl_trans_pcie_ven_mode_tx(struct iwl_trans *trans, struct sk_buff *skb,
			       struct iwl_device_cmd *dev_cmd, int txq_id);
int iwl_trans_pcie_ven_mode_dyn_txq_alloc(struct iwl_trans *trans,
					  __le16 flags, u8 sta_id, u8 tid,
					  int cmd_id, int size,
					  unsigned int timeout);
int iwl_pcie_ven_mode_set_txq(struct iwl_trans *trans, dma_addr_t txq_dma,
			      dma_addr_t txq_bc_dma);
void iwl_pcie_ven_mode_tr_unmap(struct iwl_trans *trans,
				struct iwl_txq *txq, int index);

/**
 * enum iwl_msix_ven_mode_irqs - irqs mapping for mode2
 * @IWL_MSIX_VEN_MODE_IRQ_MCR: MCR irq, completion on MTR
 * @IWL_MSIX_VEN_MODE_IRQ_HCMD: completion on host commands
 * @IWL_MSIX_VEN_MODE_IRQ_NOTIF: interrupts for FW notifications
 * @IWL_MSIX_VEN_MODE_IRQ_LOG: interrupts for FW logs
 * @IWL_MSIX_VEN_MODE_IRQ_TX_DATA: completion on TX
 * @IWL_MSIX_VEN_MODE_IRQ_RX_DATA: interrupts for RX
 */
enum iwl_msix_ven_mode_irqs {
	IWL_MSIX_VEN_MODE_IRQ_MCR = 0,
	IWL_MSIX_VEN_MODE_IRQ_HCMD = 1,
	IWL_MSIX_VEN_MODE_IRQ_NOTIF = 2,
	IWL_MSIX_VEN_MODE_IRQ_LOG = 3,
	IWL_MSIX_VEN_MODE_IRQ_TX_DATA = 4,
	IWL_MSIX_VEN_MODE_IRQ_RX_DATA = 5,
};

#endif  /* __iwl_pcie_vendor_mode__ */
