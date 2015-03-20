/*
 *  ChromeOS platform support code. Glue layer between higher level functions
 *  and per-platform firmware interfaces.
 *
 *  Copyright (C) 2010 The Chromium OS Authors
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/chromeos_platform.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/tpm.h>
#include <linux/types.h>

#include "chromeos.h"

static struct chromeos_vbc *chromeos_vbc_ptr;
static struct mutex vbc_mutex;

static int vbc_read(u8 *buf, int buf_size);
static int vbc_write_byte(unsigned offset, u8 value);
static int vbc_get_kernel_flags(u32 *flags);
static int vbc_set_kernel_flags(u32 flags, u32 mask);
static int vbc_get_or_set_kernel_flags(bool get, u32 *pflags,
				       u32 flags, u32 mask);

/* the following defines are copied from
 * vboot_reference:firmware/lib/vboot_nvstorage.c
 * and other vboot_reference sources
 */
#define RECOVERY_OFFSET	      2
#define VBNV_RECOVERY_RW_INVALID_OS  0x43
#define KERN_NV_TPM_ATTACK_FLAG      0x00000020
#define KERNEL_FIELD_OFFSET	  11

int chromeos_set_need_recovery(void)
{
	if (!chromeos_legacy_set_need_recovery())
		return 0;

	return vbc_write_byte(RECOVERY_OFFSET, VBNV_RECOVERY_RW_INVALID_OS);
}

static int chromeos_get_tpm_attack_flag(int *value)
{
	int flags;
	int status = vbc_get_kernel_flags(&flags);
	if (status < 0)
		return status;
	*value = !!(flags & KERN_NV_TPM_ATTACK_FLAG);
	return 0;
}

/* Sets the "TPM attacked" bit (not necessarily attacked, but the TPM might
 * think so).  User level needs to clear this bit after reading it.
 */
static int chromeos_set_tpm_attack_flag(int value)
{
	if (value)
		return vbc_set_kernel_flags(KERN_NV_TPM_ATTACK_FLAG,
					    KERN_NV_TPM_ATTACK_FLAG);
	else
		return vbc_set_kernel_flags(0, KERN_NV_TPM_ATTACK_FLAG);
}

/*
 * Lifted from vboot_reference:firmware/lib/vboot_nvstorage.c and formatted.
 *
 * Return CRC-8 of the data, using x^8 + x^2 + x + 1 polynomial. A table-based
 * algorithm would be faster, but for only 15 bytes isn't worth the code size.
 */
static u8 crc8(const u8 *data, int len)
{
	unsigned crc = 0;
	int i, j;

	for (j = len; j; j--, data++) {
		crc ^= (*data << 8);
		for (i = 8; i; i--) {
			if (crc & 0x8000)
				crc ^= (0x1070 << 3);
			crc <<= 1;
		}
	}
	return (u8)(crc >> 8);
}

static int vbc_get_kernel_flags(u32 *flags)
{
	return vbc_get_or_set_kernel_flags(true  /* get */,
					   flags,
					   0     /* unused for get */,
					   0     /* unused for get */);
}


static int vbc_set_kernel_flags(u32 flags, u32 mask)
{
	return vbc_get_or_set_kernel_flags(false  /* set */,
					   NULL   /* unused for set */,
					   flags,
					   mask);
}


static int vbc_get_or_set_kernel_flags(bool get, u32 *pflags,
				       u32 flags, u32 mask)
{
	u8 buf[MAX_VBOOT_CONTEXT_BUFFER_SIZE];
	int old_flags, new_flags;
	ssize_t size;
	int retval = 0;

	if (!chromeos_vbc_ptr)
		return -ENOSYS;

	mutex_lock(&vbc_mutex);

	size = vbc_read(buf, sizeof(buf));
	if (size < 0) {
		retval = size;
		goto end;
	}
	if (size == 0 || size < KERNEL_FIELD_OFFSET + 4 || size > sizeof(buf)) {
		retval = -EIO;
		goto end;
	}

	old_flags =
		(buf[KERNEL_FIELD_OFFSET + 0] << 0) |
		(buf[KERNEL_FIELD_OFFSET + 1] << 8) |
		(buf[KERNEL_FIELD_OFFSET + 2] << 16) |
		(buf[KERNEL_FIELD_OFFSET + 3] << 24);

	if (get) {
		*pflags = old_flags;
		retval = 0;
		goto end;
	}

	new_flags = (old_flags & ~mask) | (flags & mask);
	if (new_flags == old_flags) {
		retval = 0;
		goto end;
	}

	buf[KERNEL_FIELD_OFFSET + 0] = (u8) (new_flags >> 0);
	buf[KERNEL_FIELD_OFFSET + 1] = (u8) (new_flags >> 8);
	buf[KERNEL_FIELD_OFFSET + 2] = (u8) (new_flags >> 16);
	buf[KERNEL_FIELD_OFFSET + 3] = (u8) (new_flags >> 24);
	buf[size - 1] = crc8(buf, size - 1);

	retval = chromeos_vbc_ptr->write(buf, size);
end:
	mutex_unlock(&vbc_mutex);
	return retval;
}

static int vbc_write_byte(unsigned offset, u8 value)
{
	u8 buf[MAX_VBOOT_CONTEXT_BUFFER_SIZE];
	ssize_t size;
	int retval = 0;

	if (!chromeos_vbc_ptr)
		return -ENOSYS;

	mutex_lock(&vbc_mutex);
	size = vbc_read(buf, sizeof(buf));
	if (size == 0) {
		retval = -EIO;
		goto end;
	}
	if (size < 0) {
		retval = size;
		goto end;
	}

	if (offset >= (size - 1)) {
		retval = -EINVAL;
		goto end;
	}

	if (buf[offset] == value)
		goto end;

	buf[offset] = value;
	buf[size - 1] = crc8(buf, size - 1);

	retval = chromeos_vbc_ptr->write(buf, size);
end:
	mutex_unlock(&vbc_mutex);
	return retval;
}

/*
 * Read vboot context and verify it.  If everything checks out, return number
 * of bytes in the vboot context buffer, or an error code.
 */
static int vbc_read(u8 *buf, int buf_size)
{
	ssize_t size;

	if (!chromeos_vbc_ptr)
		return -ENOSYS;

	size = chromeos_vbc_ptr->read(buf, buf_size);
	if (size == 0 || size > buf_size)
		return -EIO;
	if (size < 0)
		return size;

	if (buf[size - 1] != crc8(buf, size - 1)) {
		pr_err("%s: vboot context contents corrupted\n", __func__);
		return -EIO;
	}
	return size;
}

int chromeos_vbc_register(struct chromeos_vbc *chromeos_vbc)
{
	int retval;
	if (chromeos_vbc_ptr)
		return -EINVAL;
	chromeos_vbc_ptr = chromeos_vbc;
	mutex_init(&vbc_mutex);
	/* Register the "attack flag" callbacks, used when the TPM is reset in
	 * the middle of a command.
	 */
	retval = tpm_register_cpend_flag_cbs(chromeos_get_tpm_attack_flag,
					     chromeos_set_tpm_attack_flag);
	if (retval) {
		pr_err("chromeos: tpm attack callbacks: %d\n", retval);
		return retval;
	}
	return 0;
}
