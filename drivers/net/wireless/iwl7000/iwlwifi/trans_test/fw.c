/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
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
#include "fw/testmode.h"
#endif

enum iwl_test_tx_fifo {
	IWL_TEST_TX_FIFO_BK = 0,
	IWL_TEST_TX_FIFO_BE,
	IWL_TEST_TX_FIFO_VI,
	IWL_TEST_TX_FIFO_VO,
	IWL_TEST_TX_FIFO_MCAST = 5,
};

static const u8 iwl_test_ac_to_tx_fifo[] = {
	IWL_TEST_TX_FIFO_BK,
	IWL_TEST_TX_FIFO_BE,
	IWL_TEST_TX_FIFO_VI,
	IWL_TEST_TX_FIFO_VO,
};

#define	TEST_UCODE_ALIVE_TIMEOUT	HZ

#define UCODE_VALID_OK			cpu_to_le32(0x1)

struct iwl_test_alive_data {
	bool valid;
	u32 scd_base_addr;
};

static bool iwl_alive_fn(struct iwl_notif_wait_data *notif_wait,
			 struct iwl_rx_packet *pkt,
			 void *data)
{
	struct iwl_test_op_mode *test_op_mode =
		container_of(notif_wait, struct iwl_test_op_mode, notif_wait);
	struct iwl_test_alive_data *alive_data = data;
	struct iwl_alive_resp *palive;

	palive = (void *)pkt->data;

	test_op_mode->error_event_table =
		le32_to_cpu(palive->error_event_table_ptr);
	test_op_mode->log_event_table =
		le32_to_cpu(palive->log_event_table_ptr);
	alive_data->scd_base_addr = le32_to_cpu(palive->scd_base_ptr);

	alive_data->valid = le16_to_cpu(palive->status) == IWL_ALIVE_STATUS_OK;
#ifdef CPTCFG_IWLWIFI_DEVICE_TESTMODE
	iwl_tm_set_fw_ver(test_op_mode->trans, palive->ucode_major,
			  palive->ucode_minor);
#endif
	IWL_DEBUG_FW(test_op_mode,
		     "Alive ucode status 0x%04x revision 0x%01X 0x%01X flags 0x%01X\n",
		     le16_to_cpu(palive->status), palive->ver_type,
		     palive->ver_subtype, palive->flags);
	return true;
}

static int iwl_test_load_ucode_wait_alive(struct iwl_op_mode *op_mode,
					 enum iwl_ucode_type ucode_type)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	struct iwl_notification_wait alive_wait;
	struct iwl_test_alive_data alive_data;
	const struct fw_img *fw;
	int ret;
	enum iwl_ucode_type old_type = test_op_mode->cur_ucode;
	static const u16 alive_cmd[] = { REPLY_ALIVE };
	const u8 *queue_to_txf;
	u8 i, j = 0;

	test_op_mode->cur_ucode = ucode_type;
	fw = iwl_get_ucode_image(test_op_mode->fw, ucode_type);

	test_op_mode->ucode_loaded = false;

	if (!fw)
		return -EINVAL;
	iwl_init_notification_wait(&test_op_mode->notif_wait, &alive_wait,
				   alive_cmd, ARRAY_SIZE(alive_cmd),
				   iwl_alive_fn, &alive_data);

	ret = iwl_trans_start_fw(test_op_mode->trans, fw, true);
	if (ret) {
		test_op_mode->cur_ucode = old_type;
		iwl_remove_notification(&test_op_mode->notif_wait, &alive_wait);
		return ret;
	}

	/*
	 * Some things may run in the background now, but we
	 * just wait for the ALIVE notification here.
	 */
	ret = iwl_wait_notification(&test_op_mode->notif_wait, &alive_wait,
				    TEST_UCODE_ALIVE_TIMEOUT);

	if (ret) {
		test_op_mode->cur_ucode = old_type;
		return ret;
	}

	if (!alive_data.valid) {
		IWL_ERR(test_op_mode, "Loaded ucode is not valid!\n");
		test_op_mode->cur_ucode = old_type;
		return -EIO;
	}
	IWL_INFO(test_op_mode, "Alive notification received!\n");

	iwl_trans_fw_alive(test_op_mode->trans, alive_data.scd_base_addr);

	queue_to_txf = iwl_test_ac_to_tx_fifo;

	for (i = 0; i < test_op_mode->cfg->base_params->num_of_queues; i++) {
		if (i != IWL_TRANS_TEST_CMD_QUEUE) {
			test_op_mode->active_queues[j++] = i;
			test_op_mode->n_active_queues++;
			iwl_trans_ac_txq_enable(test_op_mode->trans, i,
						queue_to_txf[i % 4], 0);
		}
	}
	test_op_mode->ucode_loaded = true;

	test_op_mode->transport_queue_stop = 0;

	return 0;
}

int iwl_run_init_test_ucode(struct iwl_op_mode *op_mode)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	int ret = 0;

	if (test_op_mode->init_ucode_run)
		return 0;

	lockdep_assert_held(&test_op_mode->mutex);

	/* Will also start the device */
	ret = iwl_test_load_ucode_wait_alive(op_mode, IWL_UCODE_INIT);

	if (!ret)
		test_op_mode->init_ucode_run = true;

	return ret;
}
