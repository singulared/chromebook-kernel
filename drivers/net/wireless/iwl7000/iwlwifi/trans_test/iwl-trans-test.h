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
#ifndef __IWL_TRANS_TEST_H__
#define __IWL_TRANS_TEST_H__

#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/cdev.h>

#include "fw/notif-wait.h"
#include "iwl-trans.h"
#include "fw-api.h"
#include "iwl-io.h"
#include "iwl-op-mode.h"

extern const struct iwl_op_mode_ops iwl_trans_test_ops;

/* Test command types */
enum cmd_type {
	TEST_HCMD,
	TEST_TX,
	TEST_RX,
	TEST_ACCESS,
	TEST_PRINT_STATISTICS,
	TEST_CHANGE_SETTINGS,
	TEST_SYNC,
	TEST_TYPE_NUM,
};

/**
 * Inside driver information : request parameters.
 * Commin for the 3 test types: hcmd, tx, rx.
 * times: times to perform action
 * size: data size of each pkt to be sent.
 * size_chunk_two: for data_echo only, if command contains two chunks.
 * flags: queue number for tx, sync mode for rx, flags for data_echo.
 */
struct iwl_test_request_info {
	u32 times;
	u16 size;
	u16 size_chunk_two;
	u8 flags;
	u16 cmd_cnt;
} __packed;

struct tx_stat {
	u32 n_tx_sent;
	u32 n_tx_sent_succesfuly;
	u32 n_tx_received_replies;
	u32 n_tx_correct_replies;
	u32 n_tx_received_data_err;
	u32 n_tx_received_wrong_size;
	u32 n_tx_recieved_for_tpt;
	int average_tpt; /* b/msec */
};

/* Data generation methods */
#define DATA_PATTERN "a"
#define DATA_PATTERN_LENGTH 1
#define GEN_PATTERN_DATA 0
#define GEN_RANDOM_DATA 1
#define NUM_DATA_GEN_METHODS 2

/**
 *Rx test parameters
 */
/* Intervals between different sending rx sessions - in 10usec.*/
#define RX_SYNC_INTERVAL 10
#define RX_ASYNC_INTERVAL 100
/* Number of rx packets to send in each session */
#define RX_SYNC_MODE_SIZE_LIMIT 1500
#define RX_SYNC_SMALL_NUM_PKT 1000
#define RX_SYNC_LARGE_NUM_PKT 200

/* Opmode status */
#define STATUS_FW_ERROR	0
#define STATUS_RX_IN_PROGRESS	1
#define STATUS_TX_IN_PROGRESS	2
#define STATUS_STOP_TESTS	3
#define STATUS_RX_WORK		4
#define STATUS_TX_WORK		5
#define STATUS_HCMD_WORK	6
#define STATUS_WORK_POS STATUS_RX_WORK

/* Test flags - Hcmd - extended echo*/
#define SEND_DATA_ECHO		BIT(0)
#define FIRST_DATA_NOCOPY	BIT(1)
#define FIRST_DATA_DUP		BIT(2)
#define SECOND_DATA_NOCOPY	BIT(3)
#define SECOND_DATA_DUP	BIT(4)
#define DATAFLAGS_MSK		0X1e

/* Data path constant */
#define IWL_TRANS_TEST_TX_SEQ_STEP 0X10
#define IWL_TRANS_TEST_MAX_SEQ 160000
#define IWL_TRANS_TEST_MS_PER_PACKET 4
#define IWL_TRANS_TEST_PIPE_CAPCITY 100
#define IWL_TRANS_TEST_PIPE_SIZE \
	(sizeof(struct iwl_test_request_info) * IWL_TRANS_TEST_PIPE_CAPCITY)
#define IWL_TRANS_TEST_CHOOSE_RANDOM_QUEUE 33

#define DATA_ECHO_HDR_LEN 8
#define MIN_MS_PER_REQUEST 30
#define MAX_SEC_PER_REQUEST 180
#define REPLY_MAX 0xff
#define IWL_TRANS_TEST_NUM_TESTS_TYPE 3
#define MAX_TESTER_INPUT_LENGTH 100

struct iwl_test_op_mode {
	/* for logger access */
	struct device *dev;

	struct iwl_trans *trans;
	bool init_ucode_run;

	const struct iwl_cfg *cfg;
	const struct iwl_fw *fw;

	unsigned long status;

	/* For synchronous commands sending */
	struct mutex mutex;
	/* Transport queues state */
	u8 n_active_queues;
	unsigned long transport_queue_stop;
	u8 active_queues[IWL_MAX_HW_QUEUES];

	/* Data path structures */
	u8 ms_per_pkt;
	u16 max_sec_per_request;

	struct kfifo test_fifos[IWL_TRANS_TEST_NUM_TESTS_TYPE];
	struct work_struct test_workers[IWL_TRANS_TEST_NUM_TESTS_TYPE];

	/* protect kfifos */
	void (*requests_handlers[IWL_TRANS_TEST_NUM_TESTS_TYPE])
			(struct iwl_test_op_mode *test_op_mode,
			 struct iwl_test_request_info req_info);

	/* tx test */
	struct tx_stat curr_tx_stat;
	struct tx_stat total_tx_stat;

	wait_queue_head_t wait_tx_queue;
	bool tx_finished_sending;
	u8 data_tx_test_val;
	u16 tx_curr_size;
	u16 tx_seq;

	/* State the serial number of the current tx request */
	u16 tx_req_num;

	/* rx test */
	/* current rx progress */
	wait_queue_head_t wait_rx_queue;

	/* Current expected data */
	u8 rx_data[TRANS_TEST_RX_CHUNK_SIZE];
	u16 rx_data_size;
	u8 data_rx_test_val;

	/* Current rx process stats */
	u16 rx_remaining_packets;
	u16 rx_err_size_cnt;
	u16 rx_err_data_cnt;

	/* total rx stat */
	u32 total_rx_expected;
	u32 total_rx_received;
	u32 total_rx_size_errors;
	u32 total_rx_data_errors;
	u32 total_fast_rx_received;
	int average_rx_tpt;

	/*tester device*/
	struct cdev test_cdev;
	int major;
	bool is_dev_open;
	struct class *test_class;
	const char *const *device_cmd_strings;
	char msg[MAX_TESTER_INPUT_LENGTH];

	/* protect tester device */
	struct mutex dev_lock;
	dev_t dev_no;

	/* Perform the actual tests */
	void (*test_handlers[TEST_TYPE_NUM])
			(struct iwl_op_mode *op_mode, char *cmd, u16 cmd_cnt);

	/* device_pointers: pointers to ucode event tables */
	u32 error_event_table;
	u32 log_event_table;

	/* indicator of loaded ucode image */
	enum iwl_ucode_type cur_ucode;
	struct iwl_notif_wait_data notif_wait;
	bool ucode_loaded;

	void (*rx_handlers[REPLY_MAX])(struct iwl_op_mode *op_mode,
				       struct iwl_rx_cmd_buffer *rxb);

	/*counters */
	u32 rx_handlers_stats[REPLY_MAX];

	/* data pattern for comparing */
	char *data_pattern;

	void (*generate_data[NUM_DATA_GEN_METHODS])
	       (struct iwl_test_op_mode *test_op_mode,
	       char *data_buf, size_t size);

	u8 data_generation_method;

#ifdef CPTCFG_IWLWIFI_DEBUGFS
	struct dentry *debugfs_dir;
	u32 dbgfs_sram_offset, dbgfs_sram_len;
#endif
};

/* Extract TEST_OP_MODE from op_mode and */
#define IWL_OP_MODE_GET_TEST(_iwl_op_mode)		\
	((struct iwl_test_op_mode *)(_iwl_op_mode)->op_mode_specific)

/* Utils */
#define increase_cyclic(x, n, m) ((x + n) % m)
#define set_in_range(x, a, b) (min(b, max(x, a)))

/* uCode load Method */
int iwl_run_init_test_ucode(struct iwl_op_mode *op_mode);

/* trans_test debugfs */
#ifdef CPTCFG_IWLWIFI_DEBUGFS
int iwl_trans_test_dbgfs_register(struct iwl_test_op_mode *test_op_mode,
					struct dentry *dbgfs_dir);
#endif

static inline const char *get_device_cmd_string(struct iwl_test_op_mode *test,
						 u8 test_num)
{
	if (test_num >= TEST_TYPE_NUM)
		return "INVALID DEVICE CMD";
	return test->device_cmd_strings[test_num];
}

/* Rx handlers functions */
void iwl_trans_test_rx_reply_tx(struct iwl_op_mode *op_mode,
				struct iwl_rx_cmd_buffer *rxb);
void iwl_trans_test_rx_dispatch(struct iwl_op_mode *op_mode,
				struct napi_struct *napi,
				struct iwl_rx_cmd_buffer *rxb);
void iwl_trans_test_setup_rx_handlers(struct iwl_test_op_mode *test_op_mode);

/* Rx test functions */
void iwl_trans_test_handle_rx_request(struct iwl_test_op_mode *test_op_mode,
				      struct iwl_test_request_info rx_info);
int iwl_trans_test_check_received_rx_data(u8 *rx_data, u8 *exp_data,
					  u16 exp_len);
void iwl_trans_test_rx_worker(struct work_struct *work);
void iwl_trans_test_reset_rx_task(struct iwl_test_op_mode *test_op_mode);

/* data generation*/
void iwl_trans_test_generate_data(struct iwl_test_op_mode
					  *test_op_mode, char *data_buf,
					  size_t size, u8 *test_val_ptr);

/* Tx test functions */
int iwl_trans_test_activate_tx_mode(struct iwl_test_op_mode *test_op_mode,
				     u8 active);
int iwl_trans_test_send_txc(struct iwl_test_op_mode *test_op_mode,
			size_t payload_size, void *data, u8 txq_id);
void iwl_trans_test_handle_tx_request(struct iwl_test_op_mode *test_op_mode,
				      struct iwl_test_request_info tx_info);
void iwl_trans_test_tx_worker(struct work_struct *work);
void iwl_trans_test_reset_tx_task(struct iwl_test_op_mode *test_op_mode);

/* Hcmd test functions */
int iwl_trans_test_send_echo_hcmd(struct iwl_test_op_mode *test_op_mode,
				  u32 flags);
int iwl_test_cmd_check_echo_resp(struct iwl_test_op_mode *test_op_mode,
				 struct iwl_host_cmd *cmd);
void iwl_trans_test_hcmd_worker(struct work_struct *work);
void iwl_trans_test_handle_hcmd_request(struct iwl_test_op_mode *test_op_mode,
					struct iwl_test_request_info hcmd_info);
void iwl_test_fill_data_echo_data(struct iwl_test_op_mode *test_op_mode,
				  struct iwl_data_echo_data *echo_data,
				  size_t len);
int iwl_trans_test_send_hcmd(struct iwl_test_op_mode *test_op_mode,
				struct iwl_host_cmd *cmd);
int iwl_trans_test_send_hcmd_test(struct iwl_test_op_mode *test_op_mode,
					struct iwl_host_cmd *cmd);

/* General datapath functions */
u8 iwl_trans_test_compute_test_val(u8 *data_buf, size_t n);
int iwl_trans_test_send_cmd_status(struct iwl_test_op_mode *test_op_mode,
				   u8 id, void *data, u16 len);
int iwl_trans_test_push_to_kfifo(struct iwl_op_mode *op_mode,
					u8 test_num, void *data);
void iwl_trans_test_fifo_worker(struct iwl_test_op_mode *test_op_mode,
				u8 test_num, u8 status_bit);
u64 iwl_trans_test_get_timeout(struct iwl_test_op_mode *test_op_mode,
			       u32 times);
void iwl_trans_test_flush_requests(struct iwl_test_op_mode *test_op_mode);

#endif /* __IWL_TRANS_TEST_H__ */
