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
#ifndef __IWL_FW_API_H__

/*
 * 16.0 uCode and above commands
 */
enum {
	INIT_COMPLETE_NOTIF = 0x4,

	/* NVM */
	NVM_ACCESS_CMD = 0x88,
	SET_CALIB_DEFAULT_CMD = 0x8e,

	/* Phy */
	PHY_CONFIGURATION_CMD = 0x8a,
	CALIB_RES_NOTIF_PHY_DB = 0x8b,

	/* PHY DB*/
	PHY_DB_CMD = 0x6c, /* TEMP API - The actual is 0x8c */
};

/* queue and FIFO numbers by usage */
enum {
	IWL_TRANS_TEST_CMD_QUEUE = 9,
	IWL_TRANS_TEST_NUM_QUEUES = 20,
	IWL_TRANS_TEST_CMD_FIFO = 7
};

enum {
	REPLY_ALIVE = 0x1,
	REPLY_ERROR = 0x2,
	REPLY_ECHO = 0x3,	/* test command */

	DBG_CFG = 0X9,

	/* RXON and QOS commands */
	REPLY_RXON = 0x10,
	REPLY_RXON_ASSOC = 0x11,
	REPLY_QOS_PARAM = 0x13,
	REPLY_RXON_TIMING = 0x14,

	/* Multi-Station support */
	REPLY_ADD_STA = 0x18,
	REPLY_REMOVE_STA = 0x19,
	REPLY_REMOVE_ALL_STA = 0x1a,	/* not used */
	REPLY_TXFIFO_FLUSH = 0x1e,

	/* Security */
	REPLY_WEPKEY = 0x20,

	/* RX, TX, LEDs */
	REPLY_TX = 0x1c,
	REPLY_LEDS_CMD = 0x48,
	REPLY_TX_LINK_QUALITY_CMD = 0x4e,

	/* WiMAX coexistence */
	COEX_PRIORITY_TABLE_CMD = 0x5a,
	COEX_MEDIUM_NOTIFICATION = 0x5b,
	COEX_EVENT_CMD = 0x5c,

	/* Calibration */
	TEMPERATURE_NOTIFICATION = 0x62,
	CALIBRATION_CFG_CMD = 0x65,
	CALIBRATION_RES_NOTIFICATION = 0x66,
	CALIBRATION_COMPLETE_NOTIFICATION = 0x67,

	/* Scan offload */
	REPLY_SCAN_OFFLOAD_CONFIG_CMD = 0x68,
	REPLY_SCAN_OFFLOAD_REQUEST_CMD = 0x69,
	REPLY_SCAN_OFFLOAD_ABORT_CMD = 0x6A,
	REPLY_SCAN_OFFLOAD_COMPLETE = 0x6D,
	REPLY_SCAN_OFFLOAD_UPDATE_PROFILES_CMD = 0x6E,

	/* 802.11h related */
	REPLY_QUIET_CMD = 0x71,	/* not used */
	REPLY_CHANNEL_SWITCH = 0x72,
	CHANNEL_SWITCH_NOTIFICATION = 0x73,
	REPLY_SPECTRUM_MEASUREMENT_CMD = 0x74,
	SPECTRUM_MEASURE_NOTIFICATION = 0x75,

	/* Power Management */
	POWER_TABLE_CMD = 0x77,
	PM_SLEEP_NOTIFICATION = 0x7A,
	PM_DEBUG_STATISTIC_NOTIFIC = 0x7B,

	/* Scan commands and notifications */
	REPLY_SCAN_CMD = 0x80,
	REPLY_SCAN_ABORT_CMD = 0x81,
	SCAN_START_NOTIFICATION = 0x82,
	SCAN_COMPLETE_NOTIFICATION = 0x84,

	/* IBSS/AP commands */
	BEACON_NOTIFICATION = 0x90,
	REPLY_TX_BEACON = 0x91,
	WHO_IS_AWAKE_NOTIFICATION = 0x94,	/* not used */

	/* Miscellaneous commands */
	REPLY_TX_POWER_DBM_CMD = 0x95,
	QUIET_NOTIFICATION = 0x96,		/* not used */
	REPLY_TX_PWR_TABLE_CMD = 0x97,
	REPLY_TX_POWER_DBM_CMD_V1 = 0x98,	/* old version of API */
	TX_ANT_CONFIGURATION_CMD = 0x98,
	MEASURE_ABORT_NOTIFICATION = 0x99,	/* not used */

	/* Bluetooth device coexistence config command */
	REPLY_BT_CONFIG = 0x9b,

	/* Statistics */
	REPLY_STATISTICS_CMD = 0x9c,
	STATISTICS_NOTIFICATION = 0x9d,

	/* RF-KILL commands and notifications */
	REPLY_CARD_STATE_CMD = 0xa0,
	CARD_STATE_NOTIFICATION = 0xa1,

	/* Missed beacons notification */
	MISSED_BEACONS_NOTIFICATION = 0xa2,

	REPLY_CT_KILL_CONFIG_CMD = 0xa4,
	SENSITIVITY_CMD = 0xa8,
	REPLY_PHY_CALIBRATION_CMD = 0xb0,

	REPLY_RX_PHY_CMD = 0xc0,
	REPLY_RX_MPDU_CMD = 0xc1,
	REPLY_COMPRESSED_BA = 0xc5,

	/* BT Coex */
	REPLY_BT_COEX_PRIO_TABLE = 0xcc,
	REPLY_BT_COEX_PROT_ENV = 0xcd,
	REPLY_BT_COEX_PROFILE_NOTIF = 0xce,

	/* PAN commands */
	REPLY_WIPAN_PARAMS = 0xb2,
	REPLY_WIPAN_RXON = 0xb3,	/* use REPLY_RXON structure */
	REPLY_WIPAN_RXON_TIMING = 0xb4,	/* use REPLY_RXON_TIMING structure */
	REPLY_WIPAN_RXON_ASSOC = 0xb6,	/* use REPLY_RXON_ASSOC structure */
	REPLY_WIPAN_QOS_PARAM = 0xb7,	/* use REPLY_QOS_PARAM structure */
	REPLY_WIPAN_WEPKEY = 0xb8,	/* use REPLY_WEPKEY structure */
	REPLY_WIPAN_P2P_CHANNEL_SWITCH = 0xb9,
	REPLY_WIPAN_NOA_NOTIFICATION = 0xbc,
	REPLY_WIPAN_DEACTIVATION_COMPLETE = 0xbd,

	REPLY_SF_CONFIG = 0xd1,
	REPLY_BEACON_FILTERING_CMD = 0xd2,

	REPLY_WOWLAN_PATTERNS = 0xe0,
	REPLY_WOWLAN_WAKEUP_FILTER = 0xe1,
	REPLY_WOWLAN_TSC_RSC_PARAMS = 0xe2,
	REPLY_WOWLAN_TKIP_PARAMS = 0xe3,
	REPLY_WOWLAN_KEK_KCK_MATERIAL = 0xe4,
	REPLY_WOWLAN_GET_STATUS = 0xe5,
	REPLY_D3_CONFIG = 0xd3,

	REPLY_DEBUG_LOG_MSG = 0xf7,

	/* Transport Tester commands */
	REPLY_TRANS_TEST_CMD = 0xfa,
	REPLY_TRANS_TEST_TX = 0xfb,
	REPLY_TRANS_TEST_RX = 0xfc,

	REPLY_MAX = 0xff
};

/***********************************
 * (0xf1 - 0xf3)
 * Trans test commands API
 *
 ***********************************/
/*
 * Size in bytes of Rx expected data sent to ucode in rx_hcmd.
 * If requested test size is bigger - ucode
 * should duplicate the given data chunk as many times as needed
 */
#define TRANS_TEST_RX_CHUNK_SIZE 1

/* status of data_echo reply: states which data chunks
 * failed in test val comparision to original data generated by driver*/
#define DATA_TEST_VAL_OK 0
#define DATA_TEST_ERR_FIRST BIT(0)
#define DATA_TEST_ERR_SECOND BIT(1)

/* For REPLY_TRANS_TEST_CMD: specific test id */
enum trans_test_cmd_type {
	ACTIVATE_TX = 0x0,
	REPLY_DATA_ECHO,
	REPLY_SEND_RX
};

/**
 * Common Data header for all REPLY_TRANS_TEST_CMD commands
 * @type: trans_test_cmd_type.
 * @enable: for TX only: enable/disable tx test mode
 */
struct iwl_debug_cmd_api {
	u8 type;
	u8 enable;
	__le16 reserved;
} __packed; /* TRNS_TEST_CMD_API_S_VER1 */

/**
 * Payload of a single data chunk for ehco check:
 * @data_test_val: Xor on data bytes.
 * @data: payload for echo command.
 */
struct iwl_data_echo_data {
	u8 data_test_val;
	u8 data[0];
} __packed;

/**
 * First data chunk of a data_echo command.
 * @length: Length of each data chunk
 */
struct iwl_data_echo_data_first {
	struct iwl_debug_cmd_api hcmd_api;
	__le16 length[IWL_MAX_CMD_TBS_PER_TFD];
	struct iwl_data_echo_data echo_data;
} __packed;

/**
 *Telling ucode number and content of rx packets to send back to driver
 * @times: Total number of packets to generate.
 * @rx_per_session: maximun #pkts to generate in a session.
 * @size: size in bytes of each packet.
 * @interval:time in 10usecs to wait between rx sessions.
 * @data: data pattern of the payload.
 */
struct iwl_send_rx_hcmd_data {
	struct iwl_debug_cmd_api hcmd_api;
	__le32 times;
	__le32 rx_per_session;
	__le16 size;
	u8 interval;
	u8 data[TRANS_TEST_RX_CHUNK_SIZE];
} __packed; /* TRNS_TEST_RX_API_S_VER1 */

/**
 * reply for REPLY_TRANS_TEST_CMD.
 * @type: specific test number.
 * @status: success status.
 */
struct iwl_trans_test_hcmd_reply {
	u8 type;
	u8 status;
	__le16 reserved;
} __packed; /* TRNS_TEST_HCMD_RESP_API_VER1 */

/**
 * data of trans test  ucode generated rx packet.
 * @test number - specific rx test, currently only 0
 */
struct iwl_test_rx_pkt {
	u8 test_num;
	u8 reserved[3];
	u8 data[0];
} __packed; /* TRNS_TEST_RX_RSP_API_S_VER1 */

/*
 * REPLY_TRANS_TEST_TX
 * @len: byte count of tx data (including mac hdr).
 * the other fields are not used by ucode, and are defined here for
 * compatibility with transport API.
 * @req_num: for internal driver use only, keep track of the request
 * from which this tx_cmd is sent.
 */
struct iwl_tx_cmd {
	__le16 len;
	u8 sta_id;
	u8 sec_ctl;
	__le32 tx_flags;
	struct {
		u8 try_cnt;
		u8 btkill_cnt;
		__le16 reserved;
	} scratch; /* DRAM_SCRATCH_API_U_VER_1 */
	__le32 dram_lsb_ptr;
	u8 dram_msb_ptr;
	u16 req_num;
	u8 reserved[37];
	u8 payload[0];
	struct ieee80211_hdr hdr[0];
} __packed; /* TRNS_TEST_TX_API_S_VER1 */

struct iwl_rx_mpdu_res_start {
	__le16 byte_count;
	__le16 reserved;
} __packed;

#define IWL_TID_NON_QOS IWL_MAX_TID_COUNT

/******************************************************************************
 * (0a)
 * Alive and Error Commands & Responses:
 *
 *****************************************************************************/
/*
 * Note: This structure is read from the device with IO accesses,
 * and the reading already does the endian conversion. As it is
 * read with u32-sized accesses, any members with a different size
 * need to be ordered correctly though!
 */
struct iwl_error_event_table {
	u32 valid;		/* (nonzero) valid, (0) log is empty */
	u32 error_id;		/* type of error */
	u32 pc;			/* program counter */
	u32 blink1;		/* branch link */
	u32 blink2;		/* branch link */
	u32 ilink1;		/* interrupt link */
	u32 ilink2;		/* interrupt link */
	u32 data1;		/* error-specific data */
	u32 data2;		/* error-specific data */
	u32 line;		/* source code line of error */
	u32 bcon_time;		/* beacon timer */
	u32 tsf_low;		/* network timestamp function timer */
	u32 tsf_hi;		/* network timestamp function timer */
	u32 gp1;		/* GP1 timer register */
	u32 gp2;		/* GP2 timer register */
	u32 gp3;		/* GP3 timer register */
	u32 ucode_ver;		/* uCode version */
	u32 hw_ver;		/* HW Silicon version */
	u32 brd_ver;		/* HW board version */
	u32 log_pc;		/* log program counter */
	u32 frame_ptr;		/* frame pointer */
	u32 stack_ptr;		/* stack pointer */
	u32 hcmd;		/* last host command header */
	u32 isr0;		/* isr status register LMPM_NIC_ISR0:
				 * rxtx_flag */
	u32 isr1;		/* isr status register LMPM_NIC_ISR1:
				 * host_flag */
	u32 isr2;		/* isr status register LMPM_NIC_ISR2:
				 * enc_flag */
	u32 isr3;		/* isr status register LMPM_NIC_ISR3:
				 * time_flag */
	u32 isr4;		/* isr status register LMPM_NIC_ISR4:
				 * wico interrupt */
	u32 isr_pref;		/* isr status register LMPM_NIC_PREF_STAT */
	u32 wait_event;		/* wait event() caller address */
	u32 l2p_control;	/* L2pControlField */
	u32 l2p_duration;	/* L2pDurationField */
	u32 l2p_mhvalid;	/* L2pMhValidBits */
	u32 l2p_addr_match;	/* L2pAddrMatchStat */
	u32 lmpm_pmg_sel;	/* indicate which clocks are turned on
				 * (LMPM_PMG_SEL) */
	u32 u_timestamp;	/* indicate when the date and time of the
				 * compilation */
	u32 flow_handler;	/* FH read/write pointers, RX credit */
} __packed;

/*
 * Only INIT is used in this mode.
 */
enum iwl_firmware_type {
	UCODE_TYPE_PROTOCOL		= 0x1,
	UCODE_TYPE_OPERATIONAL	= 0x2,
	UCODE_TYPE_CALIB		= 0x3,
	UCODE_TYPE_HW_TEST		= UCODE_TYPE_PROTOCOL,
	UCODE_TYPE_WOWLAN		= 0x5,
	UCODE_TYPE_REDUCED		= 0x6,
	UCODE_TYPE_FPGA		= 0x7,
	UCODE_TYPE_UNITTEST		= 0x8,
	UCODE_TYPE_INIT		= 0x9,
	UCODE_TYPE_INIT_HW_TEST	= UCODE_TYPE_INIT,
};

#define IWL_ALIVE_STATUS_ERR 0xDEAD
#define IWL_ALIVE_STATUS_OK 0xCAFE

struct iwl_alive_resp {
	__le16 status;
	__le16 flags;
	u8 ucode_minor;
	u8 ucode_major;
	__le16 id;
	u8 api_minor;
	u8 api_major;
	u8 ver_subtype;
	u8 ver_type;
	u8 mac;
	u8 opt;
	__le16 reserved2;
	__le32 timestamp;
	__le32 error_event_table_ptr;	/* SRAM address for error log */
	__le32 log_event_table_ptr;	/* SRAM address for event log */
	__le32 cpu_register_ptr;
	__le32 dbgm_config_ptr;
	__le32 alive_counter_ptr;
	__le32 scd_base_ptr;
} __packed;

/**
 * struct iwl_error_resp - FW error indication
 * ( REPLY_ERROR = 0x2 )
 * @error_type: one of FW_ERR_*
 * @cmd_id: the command ID for which the error occured
 * @bad_cmd_seq_num: sequence number of the erroneous command
 * @error_service: which service created the error, applicable only if
 *	error_type = 2, otherwise 0
 * @timestamp: TSF in usecs.
 */
struct iwl_error_resp {
	__le32 error_type;
	u8 cmd_id;
	u8 reserved1;
	__le16 bad_cmd_seq_num;
	__le32 error_service;
	__le64 timestamp;
} __packed;

#endif /* __IWL_FW_API_H__ */
