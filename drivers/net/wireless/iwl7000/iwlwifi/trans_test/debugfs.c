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
 * in the file called COPYING.
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

static ssize_t iwl_dbgfs_sram_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct iwl_test_op_mode *test_op_mode = file->private_data;
	const struct fw_img *img;
	int ofs, len, pos = 0;
	size_t bufsz, ret;
	char *buf;
	u8 *ptr;

	if (!test_op_mode->ucode_loaded)
		return -EINVAL;

	/* default is to dump the entire data segment */
	if (!test_op_mode->dbgfs_sram_offset && !test_op_mode->dbgfs_sram_len) {
		img = &test_op_mode->fw->img[test_op_mode->cur_ucode];
		ofs = img->sec[IWL_UCODE_SECTION_DATA].offset;
		len = img->sec[IWL_UCODE_SECTION_DATA].len;
	} else {
		ofs = test_op_mode->dbgfs_sram_offset;
		len = test_op_mode->dbgfs_sram_len;
	}

	bufsz = len * 4 + 256;
	buf = kzalloc(bufsz, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ptr = kzalloc(len, GFP_KERNEL);
	if (!ptr) {
		kfree(buf);
		return -ENOMEM;
	}

	pos += scnprintf(buf + pos, bufsz - pos, "sram_len: 0x%x\n", len);
	pos += scnprintf(buf + pos, bufsz - pos, "sram_offset: 0x%x\n", ofs);

	iwl_trans_read_mem_bytes(test_op_mode->trans, ofs, ptr, len);
	for (ofs = 0; ofs < len; ofs += 16) {
		pos += scnprintf(buf + pos, bufsz - pos, "0x%.4x ", ofs);
		hex_dump_to_buffer(ptr + ofs, 16, 16, 1, buf + pos,
				   bufsz - pos, false);
		pos += strlen(buf + pos);
		if (bufsz - pos > 0)
			buf[pos++] = '\n';
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, pos);

	kfree(buf);
	kfree(ptr);

	return ret;
}

static ssize_t iwl_dbgfs_sram_write(struct file *file,
				    const char __user *user_buf, size_t count,
				    loff_t *ppos)
{
	struct iwl_test_op_mode *test_op_mode = file->private_data;
	char buf[64];
	int buf_size;
	u32 offset, len;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) -  1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (sscanf(buf, "%x,%x", &offset, &len) == 2) {
		if ((offset & 0x3) || (len & 0x3))
			return -EINVAL;
		test_op_mode->dbgfs_sram_offset = offset;
		test_op_mode->dbgfs_sram_len = len;
	} else {
		test_op_mode->dbgfs_sram_offset = 0;
		test_op_mode->dbgfs_sram_len = 0;
	}

	return count;
}

#define TRANS_TEST_DEBUGFS_READ_WRITE_FILE_OPS(name)			\
static const struct file_operations iwl_dbgfs_##name##_ops = {		\
	.write = iwl_dbgfs_##name##_write,				\
	.read = iwl_dbgfs_##name##_read,				\
	.open = simple_open,						\
	.llseek = generic_file_llseek,					\
};

#define TRANS_TEST_DEBUGFS_ADD_FILE(name, parent, mode)			\
	do {								\
		if (!debugfs_create_file(#name, mode, parent, test_op_mode,\
					 &iwl_dbgfs_##name##_ops))	\
			goto err;					\
	} while (0)

/* Device wide debugfs entries */
TRANS_TEST_DEBUGFS_READ_WRITE_FILE_OPS(sram);

/*
 * In the degenarated transport tester mode the only use of the debugfs is for
 * sram read/write and dbgm registring.
 */
int iwl_trans_test_dbgfs_register(struct iwl_test_op_mode *test_op_mode,
				  struct dentry *dbgfs_dir)
{
	test_op_mode->debugfs_dir = dbgfs_dir;

	TRANS_TEST_DEBUGFS_ADD_FILE(sram, test_op_mode->debugfs_dir,
				    S_IWUSR | S_IRUSR);

	return 0;
err:
	IWL_ERR(test_op_mode, "Can't create test Debugfs directory\n");
	return -ENOMEM;
}

