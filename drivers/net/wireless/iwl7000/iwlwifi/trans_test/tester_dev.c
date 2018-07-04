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
#include "tester_dev.h"

#define DEV_NAME "test_dev"

static const struct file_operations fops;
static void setup_test_handlers(struct iwl_op_mode *op_mode);

static const char *const trans_test_device_cmd_strings[TEST_TYPE_NUM] = {
	[TEST_HCMD] = "HCMD",
	[TEST_TX] = "TX",
	[TEST_RX] = "RX",
	[TEST_ACCESS] = "ACCESS",
	[TEST_PRINT_STATISTICS] = "PRINT_STATISTICS",
	[TEST_CHANGE_SETTINGS] = "CHANGE_SETTINGS",
	[TEST_SYNC] = "SYNC",
};

int init_test_device(struct iwl_op_mode *op_mode)
{
	int ret;
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);

	ret = alloc_chrdev_region(&test_op_mode->dev_no, 0, 1, DEV_NAME);
	if (ret) {
		IWL_ERR(test_op_mode, "Cannot register char device\n");
		return ret;
	}
	test_op_mode->major = MAJOR(test_op_mode->dev_no);
	test_op_mode->dev_no = MKDEV(test_op_mode->major, 0);
	test_op_mode->test_class = class_create(THIS_MODULE, DEV_NAME);
	cdev_init(&test_op_mode->test_cdev, &fops);
	test_op_mode->test_cdev.owner = THIS_MODULE;
	test_op_mode->is_dev_open = false;
	ret = cdev_add(&test_op_mode->test_cdev, test_op_mode->dev_no, 1);
	if (ret) {
		IWL_ERR(test_op_mode, "Cannot register char device\n");
		return ret;
	}

	device_create(test_op_mode->test_class, NULL, test_op_mode->dev_no,
		      NULL, DEV_NAME);

	setup_test_handlers(op_mode);
	test_op_mode->device_cmd_strings = trans_test_device_cmd_strings;
	mutex_init(&test_op_mode->dev_lock);

	return 0;
}

void close_test_device(struct iwl_op_mode *op_mode)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	mutex_destroy(&test_op_mode->dev_lock);

	device_destroy(test_op_mode->test_class, test_op_mode->dev_no);
	cdev_del(&test_op_mode->test_cdev);
	class_destroy(test_op_mode->test_class);
	unregister_chrdev_region(test_op_mode->dev_no, 1);
}

/**
 *read input string of size len starting from a offset in cmd and return the
 * integer representation in 'base' base.
 */
static int msg_get_val(char *cmd, int a, int length, int base)
{
	int res;
	char char_val[MAX_TESTER_INPUT_LENGTH];

	memcpy(char_val, cmd+a, length);
	char_val[length] = '\0';
	WARN_ON(kstrtoint(char_val, base, &res));
	return res;
}

static int device_open(struct inode *inode, struct file *filp)
{
	struct cdev *i_cdev = inode->i_cdev;
	struct iwl_test_op_mode *test_op_mode = container_of
			(i_cdev, struct iwl_test_op_mode, test_cdev);

	filp->private_data = test_op_mode->trans->op_mode;

	if (test_op_mode->is_dev_open) {
		IWL_ERR(test_op_mode, "Device already open\n");
		return -EBUSY;
	}

	test_op_mode->is_dev_open = true;

	return 0;
}

static int device_release(struct inode *inode, struct file *filp)
{
	struct iwl_op_mode *op_mode = filp->private_data;
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);

	test_op_mode->is_dev_open = false;
	return 0;
}

static ssize_t device_read(struct file *filp, char __user *buff, size_t legth,
		   loff_t *offset)
{
	return 0;
}

static ssize_t device_write(struct file *filp, const char __user *buff,
			    size_t length, loff_t *offset)
{
	struct iwl_op_mode *op_mode = filp->private_data;
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	int type;
	char cmd[MAX_TESTER_INPUT_LENGTH + 1];
	static u16 cmd_cnt;
	unsigned long count = min_t(unsigned long, MAX_TESTER_INPUT_LENGTH,
				    length);

	if (test_bit(STATUS_FW_ERROR, &test_op_mode->status)) {
		IWL_ERR(test_op_mode, "FW error occurred\n");
		IWL_ERR(test_op_mode, "Reload the driver an try again.\n");
		return -EINVAL;
	}

	mutex_lock(&test_op_mode->dev_lock);
	cmd_cnt++;
	if (copy_from_user(cmd, buff, count)) {
		mutex_unlock(&test_op_mode->dev_lock);
		return -EINVAL;
	}

	cmd[count] = '\0';
	type = msg_get_val(cmd, 0, 1, 10);
	mutex_unlock(&test_op_mode->dev_lock);

	if (type < 0 || type >= TEST_TYPE_NUM)
		return -EINVAL;

	test_op_mode->test_handlers[type](op_mode, cmd, cmd_cnt);
	return count;
}

static void trans_test_regular_access(struct iwl_test_op_mode *test_op_mode,
				     u32 addr, u32 val, u16 cmd_cnt)
{
	u32 read = 0;

	if (!addr) {
		val = prandom_u32();
		addr = DEFAULT_TA_ADDR;
	}

	iwl_write32(test_op_mode->trans, addr, val);
	read = iwl_read32(test_op_mode->trans, addr);

	if (val == read)
		IWL_INFO(test_op_mode,
			 "%d) Regular TA test passed, val=0x%X,address=0x%X\n",
			 cmd_cnt, read, addr);
	else
		IWL_ERR(test_op_mode,
			"%d) Regular TA test failed:wrote:0x%X, "
			"read:0x%X address= 0x%X\n",
			cmd_cnt, val, read, addr);
}


static void trans_test_direct_access(struct iwl_test_op_mode *test_op_mode,
				     u32 addr, u32 val, u16 cmd_cnt)
{
	u32 read = 0;

	if (!addr) {
		val = prandom_u32();
		addr = DEFAULT_TA_ADDR;
	}

	iwl_write_direct32(test_op_mode->trans, addr, val);
	read = iwl_read_direct32(test_op_mode->trans, addr);

	if (val == read)
		IWL_INFO(test_op_mode,
			 "%d) Direct TA test passed, val=0x%X, address=0x%X\n",
			 cmd_cnt, read, addr);
	else
		IWL_ERR(test_op_mode,
			"%d) Direct TA test failed:wrote:0x%X "
			"read:0x%X address= 0x%X\n",
			cmd_cnt, val, read, addr);
}

static void trans_test_prph_access(struct iwl_test_op_mode *test_op_mode,
				   u32 addr, u32 val, u16 cmd_cnt)
{
	u32 read = 0;
	if (!addr) {
		val = prandom_u32();
		addr = DEFAULT_PRPH_ADDR;
	}

	iwl_write_prph(test_op_mode->trans, addr, val);
	read = iwl_read_prph(test_op_mode->trans, addr);

	if (val == read)
		IWL_INFO(test_op_mode,
			 "%d) prph TA test passed, val=0x%X, address=0x%X\n",
			 cmd_cnt, read, addr);
	else
		IWL_ERR(test_op_mode,
			"%d) prph TA test failed:wrote:0x%X "
			"read:0x%X address= 0x%X\n",
			cmd_cnt, val, read, addr);
}

static void trans_test_mem_access(struct iwl_test_op_mode *test_op_mode,
				  u32 dwords, u16 cmd_cnt)
{
	u32 offset = 0;
	u32 address;
	u8 byte_size;
	u8 test_val, read_test_val;
	u32 *val;

	/* Avoid dividing by 0 */
	u32 tmp  = sizeof(struct iwl_error_event_table)/sizeof(u32) - dwords;
	if (tmp)
		offset = prandom_u32() % tmp;

	address = test_op_mode->error_event_table + offset;
	byte_size = dwords * sizeof(u32);
	test_val = 0;
	read_test_val = 0;
	val = kmalloc(dwords * sizeof(u32), GFP_KERNEL);
	iwl_trans_test_generate_data(test_op_mode, (u8 *)val,
				     byte_size, &test_val);

	iwl_trans_write_mem(test_op_mode->trans, address, val, dwords);
	memset(val, 0, dwords * sizeof(u32));
	iwl_trans_read_mem(test_op_mode->trans, address, val, dwords);

	read_test_val = iwl_trans_test_compute_test_val((u8 *)val, byte_size);

	if (test_val == read_test_val)
		IWL_INFO(test_op_mode,
			 "%d) TA mem: read %d dwords correctly from offset %d\n",
			 cmd_cnt, dwords, offset);
	else
		IWL_ERR(test_op_mode, "%d) TA Mem access failed\n", cmd_cnt);

	kfree(val);
}

static void trans_test_grab_access(struct iwl_test_op_mode *test_op_mode,
				  int times, u16 cmd_cnt)
{
	int i;
	unsigned long flags;
	u32 val = prandom_u32(), address = DEFAULT_TA_ADDR;

	if (!iwl_trans_grab_nic_access(test_op_mode->trans, &flags)) {
		IWL_ERR(test_op_mode,
			"%d) TA nic access test Failed\n", cmd_cnt);
		return;
	}

	for (i = 0; i < times; i++) {
		iwl_write32(test_op_mode->trans, address, val);
		if (iwl_read32(test_op_mode->trans, address) != val) {
			IWL_ERR(test_op_mode,
				"%d) TA nic access test Failed\n", cmd_cnt);
			iwl_trans_release_nic_access(test_op_mode->trans,
						     &flags);
			return;
		}
	}

	iwl_trans_release_nic_access(test_op_mode->trans, &flags);
	IWL_INFO(test_op_mode, "%d) TA nic access test complete\n", cmd_cnt);
}

static void trans_test_access(struct iwl_op_mode *op_mode, char *cmd,
			      u16 cmd_cnt)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	int access_type = msg_get_val(cmd, TA_TYPE, 1, 10);
	int times;
	u32 address, val;

	switch (access_type) {
	case TA_GRAB_NIC_ACCESS:
		times = msg_get_val(cmd, TA_TIMES, 4, 10);
		trans_test_grab_access(test_op_mode, times, cmd_cnt);
		break;
	case TA_MEM:
		times = msg_get_val(cmd, TA_TIMES, 2, 10);
		trans_test_mem_access(test_op_mode, times, cmd_cnt);
		break;
	case TA_REGULAR:
		address = msg_get_val(cmd, TA_ADDR_START, TA_ADDR_LEN, 16);
		val = msg_get_val(cmd, TA_VAL_START, TA_ADDR_LEN, 16);
		trans_test_regular_access(test_op_mode, address, val, cmd_cnt);
		break;
	case TA_DIRECT:
		address = msg_get_val(cmd, TA_ADDR_START, TA_ADDR_LEN, 16);
		val = msg_get_val(cmd, TA_VAL_START, TA_ADDR_LEN, 16);
		trans_test_direct_access(test_op_mode, address, val, cmd_cnt);
		break;
	case TA_PRPH:
		address = msg_get_val(cmd, TA_ADDR_START, TA_ADDR_LEN, 16);
		val = msg_get_val(cmd, TA_VAL_START, TA_ADDR_LEN, 16);
		trans_test_prph_access(test_op_mode, address, val, cmd_cnt);
		break;
	default:
		IWL_ERR(test_op_mode, "%d) Wrong TA test valuen\n", cmd_cnt);
	}
}

/* parse and send cmd request */
static void trans_test_send_cmd(struct iwl_op_mode *op_mode, char *cmd,
				u16 cmd_cnt)
{
	int ret = 0;
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	struct iwl_test_request_info hcmd_info = {
		.times = msg_get_val(cmd, TIMES_POS, TIMES_LEN, 10),
		.size = msg_get_val(cmd, SIZE1_POS, SIZE_LEN, 10),
		.size_chunk_two = msg_get_val(cmd, SIZE2_POS, SIZE_LEN, 10),
		.flags = msg_get_val(cmd, FLAGS_POS, FLAGS_LEN, 2),
		.cmd_cnt = cmd_cnt
	};

	ret = iwl_trans_test_push_to_kfifo(op_mode, TEST_HCMD, &hcmd_info);

	if (ret)
		IWL_ERR(test_op_mode,
			"%d) failed to process Hcmd request\n", cmd_cnt);
}

/* parse and send tx request */
static void trans_test_tx(struct iwl_op_mode *op_mode, char *cmd,
			  u16 cmd_cnt)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	int ret = 0;

	struct iwl_test_request_info tx_cmd_info = {
		.times = msg_get_val(cmd, TIMES_POS, TIMES_LEN, 10),
		.size = msg_get_val(cmd, SIZE1_POS, SIZE_LEN, 10),
		.flags = msg_get_val(cmd, Q_NUM, 2, 10),
		.cmd_cnt = cmd_cnt
	};

	ret = iwl_trans_test_push_to_kfifo(op_mode, TEST_TX, &tx_cmd_info);

	if (ret)
		IWL_ERR(test_op_mode,
			"%d) Tx: failed to process Tx request\n", cmd_cnt);
}

/* Prints saved tx or rx statistic: about all requests that had been finished */
static void trans_test_print_statistics(struct iwl_op_mode *op_mode, char *cmd,
				 u16 cmd_cnt)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	int type = msg_get_val(cmd, SET_TYPE_POS, 1, 10);
	struct __maybe_unused tx_stat stat;

	switch (type) {
	case TEST_TX:
		stat = test_op_mode->total_tx_stat;
		IWL_INFO(test_op_mode,
			 "\n******************\n"
			 "%d) Tx_statistic:\nTotal sent: %u\n"
			 "Sent successfully: %u\n"
			 "Replies Received: %u\nCorrect replies Received: %u\n"
			 "data test err received:%u\n"
			 "bad params err received:%u\n"
			 "Average TPT: %d b/msec."
			 "\n******************\n", cmd_cnt, stat.n_tx_sent,
			stat.n_tx_sent_succesfuly, stat.n_tx_received_replies,
			stat.n_tx_correct_replies,
			stat.n_tx_received_data_err,
			stat.n_tx_received_wrong_size,
			stat.average_tpt);
		break;
	case TEST_RX:
		IWL_INFO(test_op_mode,
			 "\n******************\n"
			 "%d) Rx_statistic:\nTotal expected:%d\n"
			 "received: %d\nReceived with size errors: %d\n"
			 "Received with data errors: %d\nAverage TPT of"
			 " successful fast tests: %d b/msec\n"
			 "******************\n", cmd_cnt,
			test_op_mode->total_rx_expected,
			test_op_mode->total_rx_received,
			test_op_mode->total_rx_size_errors,
			test_op_mode->total_rx_data_errors,
			test_op_mode->average_rx_tpt);
		break;

	default:
		break;
	}
}

/* parse and send cmd request */
static void trans_test_rx(struct iwl_op_mode *op_mode, char *cmd, u16 cmd_cnt)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	int ret;
	struct iwl_test_request_info rx_cmd_info = {
		.size = msg_get_val(cmd, SIZE1_POS, SIZE_LEN, 10),
		.times = msg_get_val(cmd, TIMES_POS, TIMES_LEN, 10),
		.flags = msg_get_val(cmd, SYNC_MODE_POS, 1, 10),
		.cmd_cnt = cmd_cnt
	};

	ret = iwl_trans_test_push_to_kfifo(op_mode, TEST_RX, &rx_cmd_info);

	if (ret)
		IWL_ERR(test_op_mode,
			"%d) Rx: failed to process Rx request\n", cmd_cnt);
}

/* Switch to a different data generation method */
static void trans_test_settings(struct iwl_op_mode *op_mode, char *cmd,
				u16 cmd_cnt)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	int set_type = msg_get_val(cmd, SET_TYPE_POS, 1, 10);
	u8 param;
	switch (set_type) {
	case SET_DATA_GEN:
		param = msg_get_val(cmd, SET_VAL_POS, SET_VAL_LEN, 10);
		IWL_INFO(test_op_mode,
			 "%d) Setting data generation method to %s\n", cmd_cnt,
			 param == GEN_PATTERN_DATA ? "pattern" : "random");
		test_op_mode->data_generation_method = (u8)param;
		break;
	case SET_TOTAL_TIMEOUT:
		param = msg_get_val(cmd, SET_VAL_POS, SET_VAL_LEN, 10);
		IWL_INFO(test_op_mode,
			 "%d) Setting max seconds per request to %d\n",
			 cmd_cnt, param);
		test_op_mode->max_sec_per_request = param;
		break;
	case SET_MS_PER_PKT:
		param = msg_get_val(cmd, SET_VAL_POS, SET_VAL_LEN, 10);
		IWL_INFO(test_op_mode,
			 "%d) Setting ms per packet to %d\n", cmd_cnt, param);
		test_op_mode->ms_per_pkt = param;
	default:
		break; /* ignore */
	}
}

/**
 * Wait for all former requests to be served before continuing.
 * If bit(1) is set - don't perform any tests that are not already in progress.
 */
static void trans_test_sync(struct iwl_op_mode *op_mode, char *cmd, u16 cmd_cnt)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	int close_now = msg_get_val(cmd, 1, 1, 10);

	if (close_now) {
		set_bit(STATUS_STOP_TESTS, &test_op_mode->status);
		IWL_INFO(test_op_mode, "%d) Cancel all unstarted tests\n",
			 cmd_cnt);
	} else {
		IWL_INFO(test_op_mode,
			 "%d) Waiting for all current tests to finish\n",
			 cmd_cnt);
	}
	iwl_trans_test_flush_requests(test_op_mode);

	IWL_INFO(test_op_mode,
		 "%d) All tests finished. Can get new requests\n", cmd_cnt);

	clear_bit(STATUS_STOP_TESTS, &test_op_mode->status);
}

static void setup_test_handlers(struct iwl_op_mode *op_mode)
{
	struct iwl_test_op_mode *test_op_mode = IWL_OP_MODE_GET_TEST(op_mode);
	void (**handlers)(struct iwl_op_mode *op_mode, char *cmd, u16 cmd_cnt);

	handlers = test_op_mode->test_handlers;
	handlers[TEST_HCMD] = trans_test_send_cmd;
	handlers[TEST_TX] = trans_test_tx;
	handlers[TEST_RX] = trans_test_rx;
	handlers[TEST_ACCESS] = trans_test_access;
	handlers[TEST_PRINT_STATISTICS] = trans_test_print_statistics;
	handlers[TEST_CHANGE_SETTINGS] = trans_test_settings;
	handlers[TEST_SYNC] = trans_test_sync;
}

static const struct file_operations fops = {
	.read = device_read,
	.write = device_write,
	.open = device_open,
	.release = device_release,
};
