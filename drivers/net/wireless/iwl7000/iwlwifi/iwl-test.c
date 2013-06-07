/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2010 - 2013 Intel Corporation. All rights reserved.
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
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2010 - 2013 Intel Corporation. All rights reserved.
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

#include <linux/export.h>
#include <net/netlink.h>

#include "iwl-drv.h"
#include "iwl-io.h"
#include "iwl-fh.h"
#include "iwl-prph.h"
#include "iwl-trans.h"
#include "iwl-test.h"
#include "iwl-csr.h"
#include "iwl-testmode.h"
#include "iwl-op-mode.h"

/*
 * Periphery registers absolute lower bound. This is used in order to
 * differentiate registery access through HBUS_TARG_PRPH_* and
 * HBUS_TARG_MEM_* accesses.
 */
#define IWL_ABS_PRPH_START (0xA00000)

/*
 * The TLVs used in the gnl message policy between the kernel module and
 * user space application. iwl_testmode_gnl_msg_policy is to be carried
 * through the NL80211_CMD_TESTMODE channel regulated by nl80211.
 * See iwl-testmode.h
 */
static
struct nla_policy iwl_testmode_gnl_msg_policy[IWL_TM_ATTR_MAX] = {
	[IWL_TM_ATTR_COMMAND] = { .type = NLA_U32, },

	[IWL_TM_ATTR_UCODE_CMD_ID] = { .type = NLA_U8, },
	[IWL_TM_ATTR_UCODE_CMD_DATA] = { .type = NLA_UNSPEC, },

	[IWL_TM_ATTR_REG_OFFSET] = { .type = NLA_U32, },
	[IWL_TM_ATTR_REG_VALUE8] = { .type = NLA_U8, },
	[IWL_TM_ATTR_REG_VALUE32] = { .type = NLA_U32, },

	[IWL_TM_ATTR_SYNC_RSP] = { .type = NLA_UNSPEC, },
	[IWL_TM_ATTR_UCODE_RX_PKT] = { .type = NLA_UNSPEC, },

	[IWL_TM_ATTR_EEPROM] = { .type = NLA_UNSPEC, },

	[IWL_TM_ATTR_TRACE_ADDR] = { .type = NLA_UNSPEC, },
	[IWL_TM_ATTR_TRACE_DUMP] = { .type = NLA_UNSPEC, },
	[IWL_TM_ATTR_TRACE_SIZE] = { .type = NLA_U32, },

	[IWL_TM_ATTR_FIXRATE] = { .type = NLA_U32, },

	[IWL_TM_ATTR_UCODE_OWNER] = { .type = NLA_U8, },

	[IWL_TM_ATTR_MEM_ADDR] = { .type = NLA_U32, },
	[IWL_TM_ATTR_BUFFER_SIZE] = { .type = NLA_U32, },
	[IWL_TM_ATTR_BUFFER_DUMP] = { .type = NLA_UNSPEC, },

	[IWL_TM_ATTR_FW_VERSION] = { .type = NLA_U32, },
	[IWL_TM_ATTR_DEVICE_ID] = { .type = NLA_U32, },
	[IWL_TM_ATTR_FW_TYPE] = { .type = NLA_U32, },
	[IWL_TM_ATTR_FW_INST_SIZE] = { .type = NLA_U32, },
	[IWL_TM_ATTR_FW_DATA_SIZE] = { .type = NLA_U32, },

	[IWL_TM_ATTR_ENABLE_NOTIFICATION] = {.type = NLA_FLAG, },
};

static inline void iwl_test_trace_clear(struct iwl_test *tst)
{
	memset(&tst->trace, 0, sizeof(struct iwl_test_trace));
}

static void iwl_test_trace_stop(struct iwl_test *tst)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	if (!tst->trace.enabled)
		return;

	if (tst->trace.cpu_addr && tst->trace.dma_addr)
		dma_free_coherent(op_mode->trans->dev,
				  tst->trace.size,
				  tst->trace.cpu_addr,
				  tst->trace.dma_addr);

	iwl_test_trace_clear(tst);
}

static inline void iwl_test_mem_clear(struct iwl_test *tst)
{
	memset(&tst->mem, 0, sizeof(struct iwl_test_mem));
}

static inline void iwl_test_mem_stop(struct iwl_test *tst)
{
	if (!tst->mem.in_read)
		return;

	iwl_test_mem_clear(tst);
}

/*
 * Initializes the test object
 * During the lifetime of the test object it is assumed that the transport is
 * started. The test object should be stopped before the transport is stopped.
 */
void iwl_test_init(struct iwl_op_mode *op_mode)
{
	iwl_test_trace_clear(&op_mode->tst);
	iwl_test_mem_clear(&op_mode->tst);
}

/*
 * Stop the test object
 */
void iwl_test_free(struct iwl_op_mode *op_mode)
{
	iwl_test_mem_stop(&op_mode->tst);
	iwl_test_trace_stop(&op_mode->tst);
}

static inline int iwl_test_send_cmd(struct iwl_test *tst,
				    struct iwl_host_cmd *cmd)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	return op_mode->ops->test_ops.send_cmd(op_mode, cmd);
}

static inline bool iwl_test_valid_hw_addr(struct iwl_test *tst, u32 addr)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	return op_mode->ops->test_ops.valid_hw_addr(addr);
}

static inline u32 iwl_test_fw_ver(struct iwl_test *tst)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	return op_mode->ops->test_ops.get_fw_ver(op_mode);
}

static inline struct sk_buff*
iwl_test_alloc_reply(struct iwl_test *tst, int len)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	return op_mode->ops->test_ops.alloc_reply(op_mode, len);
}

static inline int iwl_test_reply(struct iwl_test *tst, struct sk_buff *skb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	return op_mode->ops->test_ops.reply(op_mode, skb);
}

static inline struct sk_buff*
iwl_test_alloc_event(struct iwl_test *tst, int len)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	return op_mode->ops->test_ops.alloc_event(op_mode, len);
}

static inline void
iwl_test_event(struct iwl_test *tst, struct sk_buff *skb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	return op_mode->ops->test_ops.event(op_mode, skb);
}

/*
 * This function handles the user application commands to the fw. The fw
 * commands are sent in a synchronuous manner. In case that the user requested
 * to get commands response, it is send to the user.
 */
static int iwl_test_fw_cmd(struct iwl_test *tst, struct nlattr **tb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	struct iwl_host_cmd cmd;
	struct iwl_rx_packet *pkt;
	struct sk_buff *skb;
	void *reply_buf;
	u32 reply_len;
	int ret;
	bool cmd_want_skb;

	memset(&cmd, 0, sizeof(struct iwl_host_cmd));

	if (!tb[IWL_TM_ATTR_UCODE_CMD_ID] ||
	    !tb[IWL_TM_ATTR_UCODE_CMD_DATA]) {
		IWL_ERR(op_mode->trans,
			"Missing fw command mandatory fields\n");
		return -ENOMSG;
	}

	cmd.flags = CMD_ON_DEMAND | CMD_SYNC;
	cmd_want_skb = nla_get_flag(tb[IWL_TM_ATTR_UCODE_CMD_SKB]);
	if (cmd_want_skb)
		cmd.flags |= CMD_WANT_SKB;

	cmd.id = nla_get_u8(tb[IWL_TM_ATTR_UCODE_CMD_ID]);
	cmd.data[0] = nla_data(tb[IWL_TM_ATTR_UCODE_CMD_DATA]);
	cmd.len[0] = nla_len(tb[IWL_TM_ATTR_UCODE_CMD_DATA]);
	cmd.dataflags[0] = IWL_HCMD_DFL_NOCOPY;
	IWL_DEBUG_INFO(op_mode->trans, "test fw cmd=0x%x, flags 0x%x, len %d\n",
		       cmd.id, cmd.flags, cmd.len[0]);

	ret = iwl_test_send_cmd(tst, &cmd);
	if (ret) {
		IWL_ERR(op_mode->trans, "Failed to send hcmd\n");
		return ret;
	}
	if (!cmd_want_skb)
		return ret;

	/* Handling return of SKB to the user */
	pkt = cmd.resp_pkt;
	if (!pkt) {
		IWL_ERR(op_mode->trans,
			"HCMD received a null response packet\n");
		return ret;
	}

	reply_len = le32_to_cpu(pkt->len_n_flags) & FH_RSCSR_FRAME_SIZE_MSK;
	skb = iwl_test_alloc_reply(tst, reply_len + 20);
	reply_buf = kmemdup(&pkt->hdr, reply_len, GFP_KERNEL);
	if (!skb || !reply_buf) {
		kfree_skb(skb);
		kfree(reply_buf);
		return -ENOMEM;
	}

	/* The reply is in a page, that we cannot send to user space. */
	iwl_free_resp(&cmd);

	if (nla_put_u32(skb, IWL_TM_ATTR_COMMAND,
			IWL_TM_CMD_DEV2APP_UCODE_RX_PKT) ||
	    nla_put(skb, IWL_TM_ATTR_UCODE_RX_PKT, reply_len, reply_buf))
		goto nla_put_failure;
	return iwl_test_reply(tst, skb);

nla_put_failure:
	IWL_DEBUG_INFO(op_mode->trans, "Failed creating NL attributes\n");
	kfree(reply_buf);
	kfree_skb(skb);
	return -ENOMSG;
}

/*
 * Handles the user application commands for register access.
 */
static int iwl_test_reg(struct iwl_test *tst, struct nlattr **tb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	u32 ofs, val32, cmd;
	u8 val8;
	struct sk_buff *skb;
	int status = 0;
	struct iwl_trans *trans = op_mode->trans;

	if (!tb[IWL_TM_ATTR_REG_OFFSET]) {
		IWL_ERR(trans, "Missing reg offset\n");
		return -ENOMSG;
	}

	ofs = nla_get_u32(tb[IWL_TM_ATTR_REG_OFFSET]);
	IWL_DEBUG_INFO(trans, "test reg access cmd offset=0x%x\n", ofs);

	cmd = nla_get_u32(tb[IWL_TM_ATTR_COMMAND]);

	/*
	 * Allow access only to FH/CSR/HBUS in direct mode.
	 * Since we don't have the upper bounds for the CSR and HBUS segments,
	 * we will use only the upper bound of FH for sanity check.
	 */
	if (ofs >= FH_MEM_UPPER_BOUND) {
		IWL_ERR(trans, "offset out of segment (0x0 - 0x%x)\n",
			FH_MEM_UPPER_BOUND);
		return -EINVAL;
	}

	switch (cmd) {
	case IWL_TM_CMD_APP2DEV_DIRECT_REG_READ32:
		val32 = iwl_read_direct32(trans, ofs);
		IWL_DEBUG_INFO(trans, "32 value to read 0x%x\n", val32);

		skb = iwl_test_alloc_reply(tst, 20);
		if (!skb) {
			IWL_ERR(trans, "Memory allocation fail\n");
			return -ENOMEM;
		}
		if (nla_put_u32(skb, IWL_TM_ATTR_REG_VALUE32, val32))
			goto nla_put_failure;
		status = iwl_test_reply(tst, skb);
		if (status < 0)
			IWL_ERR(trans, "Error sending msg : %d\n", status);
		break;

	case IWL_TM_CMD_APP2DEV_DIRECT_REG_WRITE32:
		if (!tb[IWL_TM_ATTR_REG_VALUE32]) {
			IWL_ERR(trans, "Missing value to write\n");
			return -ENOMSG;
		} else {
			val32 = nla_get_u32(tb[IWL_TM_ATTR_REG_VALUE32]);
			IWL_DEBUG_INFO(trans, "32b write val=0x%x\n", val32);
			iwl_write_direct32(trans, ofs, val32);
		}
		break;

	case IWL_TM_CMD_APP2DEV_DIRECT_REG_WRITE8:
		if (!tb[IWL_TM_ATTR_REG_VALUE8]) {
			IWL_ERR(trans, "Missing value to write\n");
			return -ENOMSG;
		} else {
			val8 = nla_get_u8(tb[IWL_TM_ATTR_REG_VALUE8]);
			IWL_DEBUG_INFO(trans, "8b write val=0x%x\n", val8);
			iwl_write8(trans, ofs, val8);
		}
		break;

	default:
		IWL_ERR(trans, "Unknown test register cmd ID\n");
		return -ENOMSG;
	}

	return status;

nla_put_failure:
	kfree_skb(skb);
	return -EMSGSIZE;
}

/*
 * Handles the request to start FW tracing. Allocates of the trace buffer
 * and sends a reply to user space with the address of the allocated buffer.
 */
static int iwl_test_trace_begin(struct iwl_test *tst, struct nlattr **tb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	struct sk_buff *skb;
	int status = 0;

	if (tst->trace.enabled)
		return -EBUSY;

	if (!tb[IWL_TM_ATTR_TRACE_SIZE])
		tst->trace.size = TRACE_BUFF_SIZE_DEF;
	else
		tst->trace.size =
			nla_get_u32(tb[IWL_TM_ATTR_TRACE_SIZE]);

	if (!tst->trace.size)
		return -EINVAL;

	if (tst->trace.size < TRACE_BUFF_SIZE_MIN ||
	    tst->trace.size > TRACE_BUFF_SIZE_MAX)
		return -EINVAL;

	tst->trace.cpu_addr = dma_alloc_coherent(op_mode->trans->dev,
						 tst->trace.size,
						 &tst->trace.dma_addr,
						 GFP_KERNEL);
	if (!tst->trace.cpu_addr)
		return -ENOMEM;

	tst->trace.enabled = true;

	memset(tst->trace.cpu_addr, 0x03B, tst->trace.size);

	skb = iwl_test_alloc_reply(tst, sizeof(tst->trace.dma_addr) + 20);
	if (!skb) {
		IWL_ERR(op_mode->trans, "Memory allocation fail\n");
		iwl_test_trace_stop(tst);
		return -ENOMEM;
	}

	if (nla_put(skb, IWL_TM_ATTR_TRACE_ADDR,
		    sizeof(tst->trace.dma_addr),
		    (u64 *)&tst->trace.dma_addr))
		goto nla_put_failure;

	status = iwl_test_reply(tst, skb);
	if (status < 0)
		IWL_ERR(op_mode->trans, "Error sending msg : %d\n", status);

	tst->trace.nchunks = DIV_ROUND_UP(tst->trace.size,
					  DUMP_CHUNK_SIZE);

	return status;

nla_put_failure:
	kfree_skb(skb);
	if (nla_get_u32(tb[IWL_TM_ATTR_COMMAND]) ==
	    IWL_TM_CMD_APP2DEV_BEGIN_TRACE)
		iwl_test_trace_stop(tst);
	return -EMSGSIZE;
}

/*
 * Handles indirect read from the periphery or the SRAM. The read is performed
 * to a temporary buffer. The user space application should later issue a dump
 */
static int iwl_test_indirect_read(struct iwl_test *tst, u32 addr, u32 size)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	struct iwl_trans *trans = op_mode->trans;
	unsigned long flags;
	int i;

	if (size & 0x3)
		return -EINVAL;

	tst->mem.size = size;
	tst->mem.addr = kmalloc(tst->mem.size, GFP_KERNEL);
	if (tst->mem.addr == NULL)
		return -ENOMEM;

	/* Hard-coded periphery absolute address */
	if (IWL_ABS_PRPH_START <= addr &&
	    addr < IWL_ABS_PRPH_START + PRPH_END) {
			if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
				return -EIO;
			}
			iwl_write32(trans, HBUS_TARG_PRPH_RADDR,
				    addr | (3 << 24));
			for (i = 0; i < size; i += 4)
				*(u32 *)(tst->mem.addr + i) =
					iwl_read32(trans, HBUS_TARG_PRPH_RDAT);
			iwl_trans_release_nic_access(trans, &flags);
	} else { /* target memory (SRAM) */
		iwl_trans_read_mem(trans, addr, tst->mem.addr,
				   tst->mem.size / 4);
	}

	tst->mem.nchunks =
		DIV_ROUND_UP(tst->mem.size, DUMP_CHUNK_SIZE);
	tst->mem.in_read = true;
	return 0;

}

/*
 * Handles indirect write to the periphery or SRAM. The  is performed to a
 * temporary buffer.
 */
static int iwl_test_indirect_write(struct iwl_test *tst, u32 addr,
	u32 size, unsigned char *buf)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	struct iwl_trans *trans = op_mode->trans;
	u32 val, i;
	unsigned long flags;

	if (IWL_ABS_PRPH_START <= addr &&
	    addr < IWL_ABS_PRPH_START + PRPH_END) {
		/* Periphery writes can be 1-3 bytes long, or DWORDs */
		if (size < 4) {
			memcpy(&val, buf, size);
			if (!iwl_trans_grab_nic_access(trans, false, &flags))
					return -EIO;
			iwl_write32(trans, HBUS_TARG_PRPH_WADDR,
				    (addr & 0x0000FFFF) |
				    ((size - 1) << 24));
			iwl_write32(trans, HBUS_TARG_PRPH_WDAT, val);
			iwl_trans_release_nic_access(trans, &flags);
		} else {
			if (size % 4)
				return -EINVAL;
			for (i = 0; i < size; i += 4)
				iwl_write_prph(trans, addr+i,
					       *(u32 *)(buf+i));
		}
	} else if (iwl_test_valid_hw_addr(tst, addr)) {
		iwl_trans_write_mem(trans, addr, buf, size / 4);
	} else {
		return -EINVAL;
	}
	return 0;
}

/*
 * Handles the user application commands for indirect read/write
 * to/from the periphery or the SRAM.
 */
static int iwl_test_indirect_mem(struct iwl_test *tst, struct nlattr **tb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	u32 addr, size, cmd;
	unsigned char *buf;

	/* Both read and write should be blocked, for atomicity */
	if (tst->mem.in_read)
		return -EBUSY;

	cmd = nla_get_u32(tb[IWL_TM_ATTR_COMMAND]);
	if (!tb[IWL_TM_ATTR_MEM_ADDR]) {
		IWL_ERR(op_mode->trans,
			"Error finding memory offset address\n");
		return -ENOMSG;
	}
	addr = nla_get_u32(tb[IWL_TM_ATTR_MEM_ADDR]);
	if (!tb[IWL_TM_ATTR_BUFFER_SIZE]) {
		IWL_ERR(op_mode->trans,
			"Error finding size for memory reading\n");
		return -ENOMSG;
	}
	size = nla_get_u32(tb[IWL_TM_ATTR_BUFFER_SIZE]);

	if (cmd == IWL_TM_CMD_APP2DEV_INDIRECT_BUFFER_READ) {
		return iwl_test_indirect_read(tst, addr,  size);
	} else {
		if (!tb[IWL_TM_ATTR_BUFFER_DUMP])
			return -EINVAL;
		buf = (unsigned char *)nla_data(tb[IWL_TM_ATTR_BUFFER_DUMP]);
		return iwl_test_indirect_write(tst, addr, size, buf);
	}
}

/*
 * Enable notifications to user space
 */
static int iwl_test_notifications(struct iwl_test *tst,
				  struct nlattr **tb)
{
	tst->notify = nla_get_flag(tb[IWL_TM_ATTR_ENABLE_NOTIFICATION]);
	return 0;
}

/*
 * Handles the request to get the device id
 */
static int iwl_test_get_dev_id(struct iwl_test *tst, struct nlattr **tb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	u32 devid = op_mode->trans->hw_id;
	struct sk_buff *skb;
	int status;

	IWL_DEBUG_INFO(op_mode->trans, "hw version: 0x%x\n", devid);

	skb = iwl_test_alloc_reply(tst, 20);
	if (!skb) {
		IWL_ERR(op_mode->trans, "Memory allocation fail\n");
		return -ENOMEM;
	}

	if (nla_put_u32(skb, IWL_TM_ATTR_DEVICE_ID, devid))
		goto nla_put_failure;
	status = iwl_test_reply(tst, skb);
	if (status < 0)
		IWL_ERR(op_mode->trans, "Error sending msg : %d\n", status);

	return 0;

nla_put_failure:
	kfree_skb(skb);
	return -EMSGSIZE;
}

/*
 * Handles the request to get the FW version
 */
static int iwl_test_get_fw_ver(struct iwl_test *tst, struct nlattr **tb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	struct sk_buff *skb;
	int status;
	u32 ver = iwl_test_fw_ver(tst);

	IWL_DEBUG_INFO(op_mode->trans, "uCode version raw: 0x%x\n", ver);

	skb = iwl_test_alloc_reply(tst, 20);
	if (!skb) {
		IWL_ERR(op_mode->trans, "Memory allocation fail\n");
		return -ENOMEM;
	}

	if (nla_put_u32(skb, IWL_TM_ATTR_FW_VERSION, ver))
		goto nla_put_failure;

	status = iwl_test_reply(tst, skb);
	if (status < 0)
		IWL_ERR(op_mode->trans, "Error sending msg : %d\n", status);

	return 0;

nla_put_failure:
	kfree_skb(skb);
	return -EMSGSIZE;
}

/*
 * Parse the netlink message and validate that the IWL_TM_ATTR_CMD exists
 */
int iwl_test_parse(struct iwl_test *tst, struct nlattr **tb,
		   void *data, int len)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	int result;

	result = nla_parse(tb, IWL_TM_ATTR_MAX - 1, data, len,
			iwl_testmode_gnl_msg_policy);
	if (result) {
		IWL_ERR(op_mode->trans, "Fail parse gnl msg: %d\n", result);
		return result;
	}

	/* IWL_TM_ATTR_COMMAND is absolutely mandatory */
	if (!tb[IWL_TM_ATTR_COMMAND]) {
		IWL_ERR(op_mode->trans, "Missing testmode command type\n");
		return -ENOMSG;
	}
	return 0;
}
IWL_EXPORT_SYMBOL(iwl_test_parse);

/*
 * Handle test commands.
 * Returns 1 for unknown commands (not handled by the test object); negative
 * value in case of error.
 */
int iwl_test_handle_cmd(struct iwl_test *tst, struct nlattr **tb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	int result;

	switch (nla_get_u32(tb[IWL_TM_ATTR_COMMAND])) {
	case IWL_TM_CMD_APP2DEV_UCODE:
		IWL_DEBUG_INFO(op_mode->trans, "test cmd to uCode\n");
		result = iwl_test_fw_cmd(tst, tb);
		break;

	case IWL_TM_CMD_APP2DEV_DIRECT_REG_READ32:
	case IWL_TM_CMD_APP2DEV_DIRECT_REG_WRITE32:
	case IWL_TM_CMD_APP2DEV_DIRECT_REG_WRITE8:
		IWL_DEBUG_INFO(op_mode->trans, "test cmd to register\n");
		result = iwl_test_reg(tst, tb);
		break;

	case IWL_TM_CMD_APP2DEV_BEGIN_TRACE:
		IWL_DEBUG_INFO(op_mode->trans,
			       "test uCode trace cmd to driver\n");
		result = iwl_test_trace_begin(tst, tb);
		break;

	case IWL_TM_CMD_APP2DEV_END_TRACE:
		iwl_test_trace_stop(tst);
		result = 0;
		break;

	case IWL_TM_CMD_APP2DEV_INDIRECT_BUFFER_READ:
	case IWL_TM_CMD_APP2DEV_INDIRECT_BUFFER_WRITE:
		IWL_DEBUG_INFO(op_mode->trans, "test indirect memory cmd\n");
		result = iwl_test_indirect_mem(tst, tb);
		break;

	case IWL_TM_CMD_APP2DEV_NOTIFICATIONS:
		IWL_DEBUG_INFO(op_mode->trans, "test notifications cmd\n");
		result = iwl_test_notifications(tst, tb);
		break;

	case IWL_TM_CMD_APP2DEV_GET_FW_VERSION:
		IWL_DEBUG_INFO(op_mode->trans, "test get FW ver cmd\n");
		result = iwl_test_get_fw_ver(tst, tb);
		break;

	case IWL_TM_CMD_APP2DEV_GET_DEVICE_ID:
		IWL_DEBUG_INFO(op_mode->trans, "test Get device ID cmd\n");
		result = iwl_test_get_dev_id(tst, tb);
		break;

	default:
		IWL_DEBUG_INFO(op_mode->trans, "Unknown test command\n");
		result = 1;
		break;
	}
	return result;
}
IWL_EXPORT_SYMBOL(iwl_test_handle_cmd);

static int iwl_test_trace_dump(struct iwl_test *tst, struct sk_buff *skb,
			       struct netlink_callback *cb)
{
	int idx, length;

	if (!tst->trace.enabled || !tst->trace.cpu_addr)
		return -EFAULT;

	idx = cb->args[4];
	if (idx >= tst->trace.nchunks)
		return -ENOENT;

	length = DUMP_CHUNK_SIZE;
	if (((idx + 1) == tst->trace.nchunks) &&
	    (tst->trace.size % DUMP_CHUNK_SIZE))
		length = tst->trace.size %
			DUMP_CHUNK_SIZE;

	if (nla_put(skb, IWL_TM_ATTR_TRACE_DUMP, length,
		    tst->trace.cpu_addr + (DUMP_CHUNK_SIZE * idx)))
		goto nla_put_failure;

	cb->args[4] = ++idx;
	return 0;

 nla_put_failure:
	return -ENOBUFS;
}

static int iwl_test_buffer_dump(struct iwl_test *tst, struct sk_buff *skb,
				struct netlink_callback *cb)
{
	int idx, length;

	if (!tst->mem.in_read)
		return -EFAULT;

	idx = cb->args[4];
	if (idx >= tst->mem.nchunks) {
		iwl_test_mem_stop(tst);
		return -ENOENT;
	}

	length = DUMP_CHUNK_SIZE;
	if (((idx + 1) == tst->mem.nchunks) &&
	    (tst->mem.size % DUMP_CHUNK_SIZE))
		length = tst->mem.size % DUMP_CHUNK_SIZE;

	if (nla_put(skb, IWL_TM_ATTR_BUFFER_DUMP, length,
		    tst->mem.addr + (DUMP_CHUNK_SIZE * idx)))
		goto nla_put_failure;

	cb->args[4] = ++idx;
	return 0;

 nla_put_failure:
	return -ENOBUFS;
}

/*
 * Handle dump commands.
 * Returns 1 for unknown commands (not handled by the test object); negative
 * value in case of error.
 */
int iwl_test_dump(struct iwl_test *tst, u32 cmd, struct sk_buff *skb,
		  struct netlink_callback *cb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	int result;

	switch (cmd) {
	case IWL_TM_CMD_APP2DEV_READ_TRACE:
		IWL_DEBUG_INFO(op_mode->trans, "uCode trace cmd\n");
		result = iwl_test_trace_dump(tst, skb, cb);
		break;

	case IWL_TM_CMD_APP2DEV_INDIRECT_BUFFER_DUMP:
		IWL_DEBUG_INFO(op_mode->trans, "testmode sram dump cmd\n");
		result = iwl_test_buffer_dump(tst, skb, cb);
		break;

	default:
		result = 1;
		break;
	}
	return result;
}
IWL_EXPORT_SYMBOL(iwl_test_dump);

/*
 * Multicast a spontaneous messages from the device to the user space.
 */
static void iwl_test_send_rx(struct iwl_test *tst,
			     struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_op_mode *op_mode =
		container_of(tst, struct iwl_op_mode, tst);

	struct sk_buff *skb;
	struct iwl_rx_packet *data;
	int length;

	data = rxb_addr(rxb);
	length = le32_to_cpu(data->len_n_flags) & FH_RSCSR_FRAME_SIZE_MSK;

	/* the length doesn't include len_n_flags field, so add it manually */
	length += sizeof(__le32);

	skb = iwl_test_alloc_event(tst, length + 20);
	if (skb == NULL) {
		IWL_ERR(op_mode->trans, "Out of memory for message to user\n");
		return;
	}

	if (nla_put_u32(skb, IWL_TM_ATTR_COMMAND,
			IWL_TM_CMD_DEV2APP_UCODE_RX_PKT) ||
	    nla_put(skb, IWL_TM_ATTR_UCODE_RX_PKT, length, data))
		goto nla_put_failure;

	iwl_test_event(tst, skb);
	return;

nla_put_failure:
	kfree_skb(skb);
	IWL_ERR(op_mode->trans, "Ouch, overran buffer, check allocation!\n");
}

/*
 * Called whenever a Rx frames is recevied from the device. If notifications to
 * the user space are requested, sends the frames to the user.
 */
void iwl_test_rx(struct iwl_test *tst, struct iwl_rx_cmd_buffer *rxb)
{
	if (tst->notify)
		iwl_test_send_rx(tst, rxb);
}
IWL_EXPORT_SYMBOL(iwl_test_rx);
