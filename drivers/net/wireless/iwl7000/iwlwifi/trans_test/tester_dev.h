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
#ifndef __TESTER_DEV__
#define __TESTER_DEV__

#include "iwl-trans-test.h"
#include "iwl-prph.h"
#include "iwl-fh.h"

/*
 * commands parameters' offsets
 */
/* Settings */
#define SET_TYPE_POS 1
#define SET_VAL_POS 2
#define SET_VAL_LEN 4
/* Datapath */
#define TIMES_POS 1
#define TIMES_LEN 7
#define SIZE1_POS 8
#define SIZE_LEN 4
#define SYNC_MODE_POS 12
#define SIZE2_POS 12
#define FLAGS_POS 16
#define FLAGS_LEN 5
#define Q_NUM 12
/* Target access */
#define TA_TYPE 1
#define TA_TIMES 2
#define TA_ADDR_START 2
#define TA_ADDR_LEN 8
#define TA_VAL_START 10

/*
 * commands parameters
 */
/* Settings */
#define SET_DATA_GEN 0
#define SET_TOTAL_TIMEOUT 1
#define SET_MS_PER_PKT 2
/* Target access options */
#define TA_REGULAR 0
#define TA_DIRECT 1
#define TA_PRPH 2
#define TA_MEM 3
#define TA_GRAB_NIC_ACCESS 4
#define DEFAULT_PRPH_ADDR SCD_AGGR_SEL
#define DEFAULT_TA_ADDR FH_TFDIB_CTRL1_REG(0xA);

/* device functions */
int init_test_device(struct iwl_op_mode *op_mode);
void close_test_device(struct iwl_op_mode *op_mode);

#endif /* __TESTER_DEV__ */
