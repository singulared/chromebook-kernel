/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2016 - 2017 Intel Deutschland GmbH
 * Copyright(c) 2018 Intel Corporation
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
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2016 - 2017 Intel Deutschland GmbH
 * Copyright(c) 2018 Intel Corporation
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

#ifndef __iwl_fw_api_fmac_h__
#define __iwl_fw_api_fmac_h__

#define FMAC_GROUP		0x10

/**
 * enum iwl_fmac_cmds - Supported FMAC commands and notifications
 */
enum iwl_fmac_cmds {
	/* Commands */
	/**
	 * @FMAC_SCAN:
	 * Perform a scan using configuration defined in
	 * &struct iwl_fmac_scan_cmd.
	 * The scan flow is asynchronous and upon completion a
	 * %FMAC_SCAN_COMPLETE notification is sent by fmac using
	 * &struct iwl_fmac_scan_complete_notif.
	 */
	FMAC_SCAN = 0x0,

	/**
	 * @FMAC_SCAN_ABORT:
	 * Stop an ongoing scan. The command is defined in
	 * &struct iwl_fmac_scan_abort_cmd.
	 */
	FMAC_SCAN_ABORT = 0x1,

	/**
	 * @FMAC_ADD_VIF:
	 * Add a virtual interface. The interface configuration is
	 * defined in &struct iwl_fmac_add_vif_cmd.
	 */
	FMAC_ADD_VIF = 0x2,

	/**
	 * @FMAC_DEL_VIF:
	 * Delete a virtual interface. The command is defined in
	 * &struct iwl_fmac_del_vif_cmd.
	 */
	FMAC_DEL_VIF = 0x3,

	/**
	 * @FMAC_CONNECT:
	 * As a station interface, connect to a network, using the configuration
	 * defined in &struct iwl_fmac_connect_cmd. The connect flow is
	 * asynchronous and upon completion a %FMAC_CONNECT_RESULT notification
	 * is sent by FMAC using &struct iwl_fmac_connect_result.
	 */
	FMAC_CONNECT = 0x4,

	/**
	 * @FMAC_DISCONNECT:
	 * As station interface, disconnect. The command is defined in
	 * &struct iwl_fmac_disconnect_cmd.
	 */
	FMAC_DISCONNECT = 0x5,

	/**
	 * @FMAC_SAR: TODO
	 */
	FMAC_SAR = 0x6,

	/**
	 * @FMAC_NVM:
	 * Apply the global NVM configuration using configuration defined in
	 * &struct iwl_fmac_nvm_cmd.
	 */
	FMAC_NVM = 0x7,

#ifdef CPTCFG_IWLFMAC_9000_SUPPORT
	/**
	 * @FMAC_REQ_QUEUE:
	 * Request a new transmit queue, using the configuration in
	 * &struct iwl_fmac_req_queue. Only used with 9000-series devices.
	 */
	FMAC_REQ_QUEUE = 0x8,

	/**
	 * @FMAC_REL_QUEUE:
	 * Release a queue allocated for <RA, TID>, using the configuration in
	 * &struct iwl_fmac_rel_queue. Only used with 9000-series devices.
	 */
	FMAC_REL_QUEUE = 0x9,
#endif

#ifdef CPTCFG_IWLFMAC_9000_SUPPORT
	/**
	 * @FMAC_SCD_QUEUE_CFG:
	 * Configure a transmit queue, as defined in
	 * &struct iwl_fmac_scd_txq_cfg_cmd.
	 * Only used with 9000-series devices.
	 */
	FMAC_SCD_QUEUE_CFG = 0xb,
#endif

	/**
	 * @FMAC_CONFIG:
	 * Configure global or interface specific settings as defined
	 * in &struct iwl_fmac_config_cmd.
	 */
	FMAC_CONFIG = 0xc,

	/**
	 * @FMAC_START_AP:
	 * For an AP interface, start AP functionality based on the
	 * configuration defined in &struct iwl_fmac_start_ap_cmd. The flow is
	 * asynchronous and upon completion a %FMAC_AP_STARTED notification
	 * is sent by FMAC using &struct iwl_fmac_ap_started.
	 */
	FMAC_START_AP = 0xd,

	/**
	 * @FMAC_STOP_AP:
	 * For an AP interface, stop AP functionality. The command is
	 * defined in &struct iwl_fmac_stop_ap_cmd.
	 */
	FMAC_STOP_AP = 0xe,

	/**
	 * @FMAC_REG_CFG: TODO
	 */
	FMAC_REG_CFG = 0xf,

	/**
	 * @FMAC_START_NAN:
	 * Start NAN operation using configuration defined in
	 * &struct iwl_fmac_cfg_nan_cmd. After this command NAN functions
	 * can be added.
	 */
	FMAC_START_NAN = 0x10,

	/**
	 * @FMAC_STOP_NAN:
	 * Stop NAN operation.
	 */
	FMAC_STOP_NAN = 0x11,

	/**
	 * @FMAC_ADD_NAN_FUNC:
	 * Add a NAN function
	 */
	FMAC_ADD_NAN_FUNC = 0x12,

	/**
	 * @FMAC_DEL_NAN_FUNC:
	 * Remove a NAN function.
	 */
	FMAC_DEL_NAN_FUNC = 0x13,

	/**
	 * @FMAC_SET_KEY:
	 * Set the key after a successful IEEE802.1X authentication.
	 * The available key types are defined in &iwl_fmac_key_type.
	 */
	FMAC_SET_KEY = 0x14,

	/**
	 * @FMAC_ACK_STA_REMOVED:
	 * Acknowledge that station removal was processed and the driver has
	 * stopped using the station ID; uses the notification
	 * &struct iwl_fmac_sta_removed as the command struct.
	 */
	FMAC_ACK_STA_REMOVED = 0x15,

	/**
	 * @FMAC_TEST_FIPS:
	 * Test security algorithms implemented in FMAC
	 */
	FMAC_TEST_FIPS = 0x16,

	/**
	 * @FMAC_START_FTM_RESPONDER:
	 * Start FTM responder. Only valid for an AP interface.
	 */
	FMAC_START_FTM_RESPONDER = 0x17,

	/**
	 * @FMAC_NEIGHBOR_REQUEST:
	 * send neighbor report request to the AP that we are currently
	 * connected to.
	 */
	FMAC_NEIGHBOR_REQUEST = 0x19,

	/**
	 * @FMAC_FTM_REQUEST:
	 * Request a new FTM measurement, defined in
	 * &struct iwl_fmac_ftm_request_cmd
	 */
	FMAC_FTM_REQUEST = 0x1a,

	/**
	 * @FMAC_FTM_ABORT:
	 * Abort a previously requested FTM measurement, defined in
	 * &struct iwl_fmac_ftm_abort_cmd
	 */
	FMAC_FTM_ABORT = 0x1b,

	/**
	 * @FMAC_JOIN_IBSS:
	 * Join an IBSS network, defined in &struct iwl_fmac_join_ibss_cmd
	 */
	FMAC_JOIN_IBSS = 0x1c,

	/**
	 * @FMAC_LEAVE_IBSS:
	 * Leave an IBSS network, defined in &struct iwl_fmac_leave_ibss_cmd
	 */
	FMAC_LEAVE_IBSS = 0x1d,

	/**
	 * @FMAC_IBSS_STATE_CLEARED_ACK:
	 * Acknowledge that the driver cleared ibss state
	 * uses the notification
	 * &struct iwl_fmac_merge_ibss as the command struct.
	 */
	FMAC_IBSS_STATE_CLEARED_ACK = 0x1e,

	/**
	 * @FMAC_MIC_FAILURE:
	 * Inform FMAC about TKIP MMIC failures, FMAC will run countermeasures.
	 * &struct iwl_fmac_mic_failure as the command struct.
	 */
	FMAC_MIC_FAILURE = 0x1f,

	/**
	 * @FMAC_SET_MONITOR_CHAN:
	 * Set channel of monitor interface.
	 * &struct iwl_fmac_set_monitor_chan_cmd as the command struct.
	 */
	FMAC_SET_MONITOR_CHAN = 0x20,

	/**
	 * @FMAC_NAN_NDP:
	 * NAN Data Path command. Uses &struct iwl_fmac_nan_ndp_cmd as command
	 * struct.
	 */
	FMAC_NAN_NDP = 0x21,

	/**
	 * @FMAC_HOST_BASED_AP:
	 * Manage (start / modify / stop) a host based AP.
	 * &struct iwl_fmac_host_ap_cmd as the command struct.
	 */
	FMAC_HOST_BASED_AP = 0x22,

	/* Notifications */

	/**
	 * @FMAC_NAN_CLUSTER_NOTIFY:
	 * NAN Cluster change notification. Uses &struct
	 * iwl_fmac_nan_cluster_notif as notification struct.
	 */
	FMAC_NAN_CLUSTER_NOTIFY = 0xec,

	/**
	 * @FMAC_NAN_NDP_NOTIFY:
	 * NAN Data Path notification. Uses &struct iwl_fmac_nan_ndp_cmd as
	 * notification struct.
	 */
	FMAC_NAN_NDP_NOTIFY = 0xed,

	/**
	 * @FMAC_IBSS_MERGING:
	 * Notify that IBSS started to merge with another IBSS
	 * The host needs to leave the current IBSS, and once done indicate that
	 * is done using FMAC_IBSS_STATE_CLEARED_ACK.
	 * defined in &struct iwl_fmac_merge_ibss
	 */
	FMAC_IBSS_MERGING = 0xee,

	/**
	 * @FMAC_IBSS_JOINED:
	 * Notify that vif has joined an IBSS, defined in
	 * &struct iwl_fmac_ibss_joined
	 */
	FMAC_IBSS_JOINED = 0xef,

#ifdef CPTCFG_IWLFMAC_9000_SUPPORT
	/**
	 * @FMAC_SEND_FRAME:
	 * Notification about a frame that should be sent by the host
	 * on FMAC's behalf as defined in &struct iwl_fmac_send_frame_notif
	 * Only used with 9000-series devices.
	 */
	FMAC_SEND_FRAME = 0xf0,
#endif

	/**
	 * @FMAC_FTM_RESPONSE:
	 * FTM measurement response notification, defined in
	 * &struct iwl_fmac_ftm_response_notify
	 */
	FMAC_FTM_RESPONSE = 0xf1,

	/**
	 * @FMAC_NEIGHBOR_REPORT:
	 * Notification about a neighbor report. This notification is used to
	 * notify the host about neighboring APs as reported by the currently
	 * connected AP.
	 */
	FMAC_NEIGHBOR_REPORT = 0xf2,

	/**
	 * @FMAC_EAPOL:
	 * Notification about a received EAPOL frame. This notification is
	 * used to notify the host about EAPOL frames required for IEEE802.1X
	 * authentication. Other EAPOL frames are not passed to the host.
	 */
	FMAC_EAPOL = 0xf3,

	/**
	 * @FMAC_NAN_DISCOVERY_EVENT: TODO
	 */
	FMAC_NAN_DISCOVERY_EVENT = 0xf4,

	/**
	 * @FMAC_NAN_DISCOVERY_TERMINATE:
	 * For NAN interface. NAN function terminated notification with
	 * more info given in &struct iwl_fmac_nan_de_term.
	 */
	FMAC_NAN_DISCOVERY_TERMINATE = 0xf5,

	/**
	 * @FMAC_REG_UPDATE: TODO
	 */
	FMAC_REG_UPDATE = 0xf6,

	/**
	 * @FMAC_TRIGGER_NOTIF: TODO
	 */
	FMAC_TRIGGER_NOTIF = 0xf7,

	/**
	 * @FMAC_AP_STARTED:
	 * For AP interface, AP started notification carrying
	 * additional information in &struct iwl_fmac_ap_started.
	 */
	FMAC_AP_STARTED = 0xf8,

	/**
	 * @FMAC_STA_ADDED:
	 * Station added notification, where the new station
	 * configuration is defined in &struct iwl_fmac_sta_added.
	 */
	FMAC_STA_ADDED = 0xf9,

	/**
	 * @FMAC_STA_REMOVED:
	 * Station removed notification, where the removed station
	 * identity etc. are defined in &struct iwl_fmac_sta_removed.
	 */
	FMAC_STA_REMOVED = 0xfa,

	/**
	 * @FMAC_KEYS_UPDATE:
	 * Notification about new keys, where the new key configuration is
	 * given in &struct iwl_fmac_keys_update_notif.
	 */
	FMAC_KEYS_UPDATE = 0xfb,

	/**
	 * @FMAC_DISCONNECTED:
	 * For station interface, disconnection from a network notification,
	 * with additional information given in &struct iwl_fmac_disconnect_cmd.
	 */
	FMAC_DISCONNECTED = 0xfc,

	/**
	 * @FMAC_DEBUG:
	 * Debug information notification with additional information given
	 * in &struct iwl_fmac_debug_notif.
	 */
	FMAC_DEBUG = 0xfd,

	/**
	 * @FMAC_CONNECT_RESULT:
	 * Connect request result notification, with the
	 * connection information given in &struct iwl_fmac_connect_result.
	 */
	FMAC_CONNECT_RESULT = 0xfe,

	/**
	 * @FMAC_SCAN_COMPLETE:
	 * Scan completed notification, with additional information
	 * in &struct iwl_fmac_scan_complete_notif.
	 */
	FMAC_SCAN_COMPLETE = 0xff,
};

#define IWL_FMAC_MAX_SSIDS	20
#define IWL_FMAC_MAX_CHANS	50

#ifdef CPTCFG_IWLFMAC_9000_SUPPORT
/*
 * Value used, in 9000-series API, when no queue is
 * assigned/present.
 */
#define IWL_FMAC_NO_QUEUE	0xff
#endif

/**
 * struct iwl_fmac_scan_cmd - MLME scan command
 * @vif_id: vif_id returned by &FMAC_ADD_VIF command
 * @random_mac: use randomized mac address
 * @n_ssids: number of ssids in &ssids.
 * @n_freqs: number of freqs in &freqs
 * @flags: currently unused.
 * @rates_24: currently unused.
 * @rates_52: currently unused.
 * @ssids: SSIDs to scan for (active scan only)
 * @ssids_lengths: lengths of the SSIDs in &ssids
 * @freqs: freqs in MHz. If none are specified all the supported frequencies are
 *	scanned.
 * @bssid: BSSID to scan for (most commonly, the wildcard BSSID).
 * @ie_len: length of IEs in octets.
 * @ie: optional IEs added to probe request.
 *
 * Request a scan operation on &freqs, probing for the networks
 * specified by &ssids. The scan execution is done in an asynchronous manner,
 * and the completion of the flow is indicated via %FMAC_SCAN_COMPLETE
 * notification.
 */
struct iwl_fmac_scan_cmd {
	u8 vif_id;
	u8 random_mac;
	u8 n_ssids;
	u8 n_freqs;
	__le32 flags;
	__le16 rates_24;
	__le16 rates_52;
	u8 ssids[IWL_FMAC_MAX_SSIDS][IEEE80211_MAX_SSID_LEN];
	u8 ssids_lengths[IWL_FMAC_MAX_SSIDS];
	__le16 freqs[IWL_FMAC_MAX_CHANS];
	u8 bssid[ETH_ALEN];
	__le16 ie_len;
#ifndef _MSC_VER
	u8 ie[0];
#endif
	/* pad to a multiple of 4 bytes */
} __packed;

/**
 * struct iwl_fmac_scan_abort_cmd - MLME scan abort command
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp
 * @reserved: for alignment.
 *
 * Request to abort an ongoing scan operation initiated by %FMAC_SCAN command.
 */
struct iwl_fmac_scan_abort_cmd {
	u8 vif_id;
	u8 reserved[3];
} __packed;

/**
 * enum iwl_fmac_vif_type - Interface types supported by fmac
 * @IWL_FMAC_IFTYPE_IBSS: IBSS interface. Not supported yet.
 * @IWL_FMAC_IFTYPE_MGD: Managed interface.
 * @IWL_FMAC_IFTYPE_AP: Access Point interface.
 * @IWL_FMAC_IFTYPE_P2P_CLIENT: P2P Client interface. Not supported yet.
 * @IWL_FMAC_IFTYPE_P2P_GO: P2P Group Owner interface. Not supported yet.
 * @IWL_FMAC_IFTYPE_P2P_DEVICE: P2P Device interface. Not supported yet.
 * @IWL_FMAC_IFTYPE_NAN_DEVICE: NAN Device interface
 * @IWL_FMAC_IFTYPE_MONITOR: Sniffer Device interface.
 * @IWL_FMAC_IFTYPE_HOST_BASED_AP: Access Point interface, but handled by the
 *      host. All management frames will be forwarded to the host. There can be
 *      at most one such vif in the system.
 * @IWL_FMAC_IFTYPE_NAN_DATA: NAN Data interface.
 * @IWL_FMAC_IFTYPE_ANY: catch-all interface type for config command.
 */
enum iwl_fmac_vif_type {
	IWL_FMAC_IFTYPE_IBSS = 0,
	IWL_FMAC_IFTYPE_MGD,
	IWL_FMAC_IFTYPE_AP,
	IWL_FMAC_IFTYPE_P2P_CLIENT,
	IWL_FMAC_IFTYPE_P2P_GO,
	IWL_FMAC_IFTYPE_P2P_DEVICE,
	IWL_FMAC_IFTYPE_NAN_DEVICE,
	IWL_FMAC_IFTYPE_MONITOR,
	IWL_FMAC_IFTYPE_HOST_BASED_AP,
	IWL_FMAC_IFTYPE_NAN_DATA,
	IWL_FMAC_IFTYPE_ANY = 0xff,
};

#define IWL_FMAC_STATION_COUNT	16

enum iwl_fmac_tx_fifo {
	IWL_FMAC_TX_FIFO_BK = 0,
	IWL_FMAC_TX_FIFO_BE,
	IWL_FMAC_TX_FIFO_VI,
	IWL_FMAC_TX_FIFO_VO,
	IWL_FMAC_TX_FIFO_MCAST = 5,
	IWL_FMAC_TX_FIFO_CMD = 7,
};

static const u8 iwl_fmac_tid_to_tx_fifo[] = {
	IWL_FMAC_TX_FIFO_BE,
	IWL_FMAC_TX_FIFO_BK,
	IWL_FMAC_TX_FIFO_BK,
	IWL_FMAC_TX_FIFO_BE,
	IWL_FMAC_TX_FIFO_VI,
	IWL_FMAC_TX_FIFO_VI,
	IWL_FMAC_TX_FIFO_VO,
	IWL_FMAC_TX_FIFO_VO,
	IWL_FMAC_TX_FIFO_VO /* MGMT is mapped to VO */
};

static const u8 iwl_fmac_11ax_tid_to_tx_fifo[] = {
	IWL_GEN2_TRIG_TX_FIFO_BE,
	IWL_GEN2_TRIG_TX_FIFO_BK,
	IWL_GEN2_TRIG_TX_FIFO_BK,
	IWL_GEN2_TRIG_TX_FIFO_BE,
	IWL_GEN2_TRIG_TX_FIFO_VI,
	IWL_GEN2_TRIG_TX_FIFO_VI,
	IWL_GEN2_TRIG_TX_FIFO_VO,
	IWL_GEN2_TRIG_TX_FIFO_VO,
	IWL_GEN2_TRIG_TX_FIFO_VO /* MGMT is mapped to VO */
};

/**
 * struct iwl_fmac_add_vif_cmd - Add a new virtual interface.
 * @addr: the mac address that should be assigned to the interface.
 * @type: the requested interface type as specified in %iwl_fmac_vif_type.
 * @reserved: for alignment.
 *
 * Add a new virtual interface. The flow is a synchronous one, and upon
 * completion, the operation result is conveyed using &iwl_fmac_add_vif_resp.
 */
struct iwl_fmac_add_vif_cmd {
	u8 addr[ETH_ALEN];
	u8 type;
	u8 reserved;
} __packed;

/**
 * enum iwl_fw_add_vif_resp_status - Status of %FMAC_ADD_VIF command.
 * @IWL_ADD_VIF_SUCCESS: Success to add a new interface.
 * @IWL_ADD_VIF_FAILURE: Failure to add a new interface.
 */
enum iwl_fw_add_vif_resp_status {
	IWL_ADD_VIF_SUCCESS = 0,
	IWL_ADD_VIF_FAILURE,
};

/**
 * struct iwl_fmac_del_vif_cmd - Delete a virtual interface.
 * @id: the interface id, as returned in &iwl_fmac_add_vif_resp in case of a
 * @reserved: for alignment.
 * successful %FMAC_ADD_VIF command.
 */
struct iwl_fmac_del_vif_cmd {
	u8 id;
	u8 reserved[3];
} __packed;

/**
 * enum iwl_fmac_nan_add_func_resp_status - Add function status
 *
 * @IWL_FMAC_NAN_DE_FUNC_STATUS_SUCCESSFUL: function added successfully.
 * @IWL_FMAC_NAN_DE_FUNC_STATUS_INSUFFICIENT_ENTRIES: exceed number of supported
 *     services.
 * @IWL_FMAC_NAN_DE_FUNC_STATUS_INSUFFICIENT_MEMORY: not enough memory.
 * @IWL_FMAC_NAN_DE_FUNC_STATUS_INVALID_INSTANCE: invalid instance.
 * @IWL_FMAC_NAN_DE_FUNC_STATUS_UNSPECIFIED: unspecified reason.
 */
enum iwl_fmac_nan_add_func_resp_status {
	IWL_FMAC_NAN_DE_FUNC_STATUS_SUCCESSFUL,
	IWL_FMAC_NAN_DE_FUNC_STATUS_INSUFFICIENT_ENTRIES,
	IWL_FMAC_NAN_DE_FUNC_STATUS_INSUFFICIENT_MEMORY,
	IWL_FMAC_NAN_DE_FUNC_STATUS_INVALID_INSTANCE,
	IWL_FMAC_NAN_DE_FUNC_STATUS_UNSPECIFIED,
};

enum srf_type {
	SRF_BF_TYPE	    = BIT(0),
	SRF_INCLUDE	    = BIT(1),
	SRF_BLOOM_FILTER_IDX    = BIT(2) | BIT(3),
};

/**
 * struct iwl_fmac_add_vif_resp - response for a %FMAC_ADD_VIF command.
 * @status: see &iwl_fw_add_vif_resp_status.
 * @id: on successful operation, would hold the new interface identifier.
 * @reserved: for alignment.
 */
struct iwl_fmac_add_vif_resp {
	u8 status;
	u8 id;
	__le16 reserved;
} __packed;

/**
 * enum iwl_fmac_connection_flags - connection flags
 * @IWL_FMAC_FREQ_IN_USE: use only the specified frequency.
 * @IWL_FMAC_FREQ_HINT: use as an hint to optimize connection time.
 * @IWL_FMAC_CONNECT_FLAGS_BSSID_WHITELIST: If this is set, the BSSIDs list is
 *	a whitelist, i.e. a list of the acceptable BSSIDs for connection.
 *	Otherwise, the BSSIDs list is a blacklist specifying disallowed BSSIDs.
 */
enum iwl_fmac_connection_flags {
	IWL_FMAC_FREQ_IN_USE			= BIT(0),
	IWL_FMAC_FREQ_HINT			= BIT(1),
	IWL_FMAC_CONNECT_FLAGS_BSSID_WHITELIST	= BIT(2),
};

/*
 * Supported cipher suites (both pairwise and group):
 * @IWL_FMAC_CIPHER_NONE:
 * @IWL_FMAC_CIPHER_WEP40:
 * @IWL_FMAC_CIPHER_WEP104:
 * @IWL_FMAC_CIPHER_TKIP:
 * @IWL_FMAC_CIPHER_CCMP:
 * @IWL_FMAC_CIPHER_GCMP:
 * @IWL_FMAC_CIPHER_GCMP_256:
 * @IWL_FMAC_CIPHER_CCMP_256:
 */
#define IWL_FMAC_CIPHER_NONE BIT(0)
#define IWL_FMAC_CIPHER_WEP40 BIT(1)
#define IWL_FMAC_CIPHER_WEP104 BIT(2)
#define IWL_FMAC_CIPHER_TKIP BIT(3)
#define IWL_FMAC_CIPHER_CCMP BIT(4)
#define IWL_FMAC_CIPHER_GCMP BIT(6)
#define IWL_FMAC_CIPHER_GCMP_256 BIT(8)
#define IWL_FMAC_CIPHER_CCMP_256 BIT(9)
#define IWL_FMAC_SUPPORTED_CIPHERS	(IWL_FMAC_CIPHER_NONE	| \
					 IWL_FMAC_CIPHER_WEP40	| \
					 IWL_FMAC_CIPHER_WEP104 | \
					 IWL_FMAC_CIPHER_TKIP	| \
					 IWL_FMAC_CIPHER_CCMP	| \
					 IWL_FMAC_CIPHER_GCMP	| \
					 IWL_FMAC_CIPHER_GCMP_256 | \
					 IWL_FMAC_CIPHER_CCMP_256)

/**
 * Supported key management suites:
 * @IWL_FMAC_KEY_MGMT_IEEE8021X:
 * @IWL_FMAC_KEY_MGMT_PSK:
 * @IWL_FMAC_KEY_MGMT_IEEE8021X_SHA256:
 * @IWL_FMAC_KEY_MGMT_PSK_SHA256:
 * @IWL_FMAC_KEY_MGMT_IEEE8021X_SUITE_B:
 * @IWL_FMAC_KEY_MGMT_IEEE8021X_SUITE_B_192:
 */
#define IWL_FMAC_KEY_MGMT_IEEE8021X	BIT(0)
#define IWL_FMAC_KEY_MGMT_PSK		BIT(1)
#define IWL_FMAC_KEY_MGMT_FT_IEEE8021X		BIT(5)
#define IWL_FMAC_KEY_MGMT_FT_PSK		BIT(6)
#define IWL_FMAC_KEY_MGMT_IEEE8021X_SHA256	BIT(7)
#define IWL_FMAC_KEY_MGMT_PSK_SHA256	BIT(8)
#define IWL_FMAC_KEY_MGMT_IEEE8021X_SUITE_B	BIT(16)
#define IWL_FMAC_KEY_MGMT_IEEE8021X_SUITE_B_192	BIT(17)
#define IWL_FMAC_SUPPORTED_KEY_MGMT	(IWL_FMAC_KEY_MGMT_PSK	| \
					 IWL_FMAC_KEY_MGMT_PSK_SHA256 | \
					 IWL_FMAC_KEY_MGMT_FT_IEEE8021X | \
					 IWL_FMAC_KEY_MGMT_FT_PSK | \
					 IWL_FMAC_KEY_MGMT_IEEE8021X | \
					 IWL_FMAC_KEY_MGMT_IEEE8021X_SHA256 | \
					 IWL_FMAC_KEY_MGMT_IEEE8021X_SUITE_B | \
					 IWL_FMAC_KEY_MGMT_IEEE8021X_SUITE_B_192)

/**
 * Supported security protocols:
 * @IWL_FMAC_PROTO_WPA:
 * @IWL_FMAC_PROTO_RSN:
 */
#define IWL_FMAC_PROTO_WPA BIT(0)
#define IWL_FMAC_PROTO_RSN BIT(1)
#define IWL_FMAC_SUPPORTED_PROTO	(IWL_FMAC_PROTO_WPA | \
					 IWL_FMAC_PROTO_RSN)

/**
 * enum iwl_fmac_mfp_mode: Supported Management Frame Protection modes.
 * @IWL_FMAC_MFP_NO: management frame protection not used
 * @IWL_FMAC_MFP_OPTIONAL: management frame protection is optional
 * @IWL_FMAC_MFP_REQUIRED: management frame protection is required
 */
enum iwl_fmac_mfp_mode {
	IWL_FMAC_MFP_NO,
	IWL_FMAC_MFP_OPTIONAL,
	IWL_FMAC_MFP_REQUIRED,
};

#define IWL_NUM_WEP_KEYS 4
#define IWL_MAX_WEP_KEY_LEN 13

/**
 * struct iwl_fmac_crypto - Security configuration.
 * @cipher_group: the allowed group cipher suite as specified in
 *	%IWL_FMAC_CIPHER_\*.
 * @ciphers_pairwise: the allowed pairwise cipher suites as specified in
 *	%IWL_FMAC_CIPHER_\*
 * @key_mgmt: the supported key management suites as specified in
 *	%IWL_FMAC_KEY_MGMT_\*. If set to NONE only wep section of the union
 *	below will be accessed. If PSK is set the key and proto will be read
 *	from wpa section.
 * @mfp: the Management Frame Protection configuration. The allowed
 *	configurations are specified in %iwl_fmac_mfp_mode. Only supported
 *	for station mode for now. This option is not supported on 9000 devices.
 * @reserved: reserved
 * @psk: the pre-shared key used with key management suites
 *	%IWL_FMAC_KEY_MGMT_PSK and %IWL_FMAC_KEY_MGMT_PSK_SHA256.
 * @proto: the allowed protocol as specified in %IWL_FMAC_PROTO_\*.
 * @key: WEP keys data.
 * @key_len: WEP key length (can vary between 5 or 13)
 * @def_key: default wep key, the other keys aren't used. The default key
 *	is also used for shared WEP authentication.
 * @reserved: for future use and alignment.
 * @u: union of the various types of key material
 */
struct iwl_fmac_crypto {
	__le32 cipher_group;
	__le32 ciphers_pairwise;
	__le32 key_mgmt;
	u8 mfp;
	u8 reserved[3];
	union {
		struct {
			u8 psk[32];
			__le32 proto;
		} __packed wpa;
		struct {
			u8 key[IWL_NUM_WEP_KEYS][IWL_MAX_WEP_KEY_LEN];
			u8 key_len[IWL_NUM_WEP_KEYS];
			u8 def_key;
			u8 reserved1[3];
		} __packed wep;
	} u;
} __packed;

#define IWL_FMAC_MAX_BSSIDS	10

/**
 * struct iwl_fmac_connect_cmd - connect to a network.
 * @vif_id: the virtual interface identifier as returned in
 *	&iwl_fmac_add_vif_resp.
 * @max_retries: number of retries before notifying connection failure.
 * @center_freq: optional frequency that can be used to limit the connection
 *	only for BSSs on the specified frequency.
 * @flags: see &enum iwl_fmac_connection_flags.
 * @bssid: optional parameter to limit the connection only to a BSS
 *	with the specified BSSID.
 * @reserved1: for alignment.
 * @ssid_len: the length of %ssid.
 * @ssid: the SSID of the network to connect to.
 * @crypto: the connection security configuration as specified in
 *	%iwl_fmac_crypto.
 * @reserved2: for alignment.
 * @n_bssids: number of BSSIDs in the @bssids array.
 * @bssids: array of @n_bssids. Depending on the @flags field, this is either
 *	a blacklist (i.e. specifies disallowed BSSIDs, and all other BSSIDs are
 *	allowed) or a whitelist (i.e. speficies a list of acceptable BSSIDs, and
 *	all other BSSIDs are disallowed). If this array is empty, all BSSIDs are
 *	allowed.
 *
 * A connect request to the network specified in %ssid. The command is allowed
 * iff the interface specified in %vif_id is currently idle (i.e., not connected
 * or trying to connect). The flow is an asynchronous one, and upon completion,
 * the operation result is conveyed by %FMAC_CONNECT_RESULT.
 */
struct iwl_fmac_connect_cmd {
	u8 vif_id;
	u8 max_retries;
	__le16 center_freq;
	__le32 flags;
	u8 bssid[ETH_ALEN];
	u8 reserved1;
	u8 ssid_len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];

	struct iwl_fmac_crypto crypto;
	u8 reserved2[3];
	u8 n_bssids;
	u8 bssids[IWL_FMAC_MAX_BSSIDS * ETH_ALEN];
} __packed;

#define UMAC_DEFAULT_KEYS 4
#define IWL_FMAC_MAX_PN_LEN	16

/**
 * struct iwl_fmac_key - Meta data for an fmac key entry.
 * @valid: 1 if the key is valid for use; Otherwise 0.
 * @keyidx: a SW key identifier.
 * @hw_keyidx: a HW key identifier.
 * @rx_pn_len: the number of valid octets in &rx_pn.
 * @rx_pn: the Rx packet number in the order needed for PN comparison for
 *	&cipher.
 * @cipher: the cipher suite associated with the key (one of
 *	%IWL_FMAC_CIPHER_\*).
 */
struct iwl_fmac_key {
	u8 valid;
	u8 keyidx;
	u8 hw_keyidx;
	u8 rx_pn_len;
	u8 rx_pn[IWL_FMAC_MAX_PN_LEN];
	__le32 cipher;
} __packed;

/**
 * struct iwl_fmac_keys - Describing a set of keys.
 * @ptk: an array of pairwise transient keys as specified in %iwl_fmac_key.
 * @gtk: an array of group transient keys as specified in %iwl_fmac_key.
 * @wep_tx_keyidx: default WEP TX key index
 * @reserved: for alignment.
 */
struct iwl_fmac_keys {
	struct iwl_fmac_key ptk[UMAC_DEFAULT_KEYS];
	struct iwl_fmac_key gtk[UMAC_DEFAULT_KEYS];
	u8 wep_tx_keyidx;
	u8 reserved[3];
} __packed;

/**
 * struct iwl_fmac_connect_result - connect result notification.
 */
struct iwl_fmac_connect_result {
	/**
	 * @vif_id:
	 * the interface identifier returned in &iwl_fmac_add_vif_resp
	 */
	u8 vif_id;

	/**
	 * @sta_id:
	 * on successful connection, holds a station entry index associated
	 * with AP the station interface associated with.
	 */
	u8 sta_id;

	/**
	 * @center_freq:
	 * on successful connection, the center frequency of the BSS.
	 */
	__le16 center_freq;

	/**
	 * @status:
	 * status code as defined in IEEE 802.11-2016 Table 9-46
	 * ("Status codes").
	 */
	__le16 status;

	/**
	 * @bssid:
	 * on successful connection, the bssid of the BSS.
	 */
	u8 bssid[ETH_ALEN];

	/**
	 * @signal:
	 * on successful connection, the signal in dBm of the BSS.
	 */
	__le32 signal;

	/**
	 * @capability:
	 * on successful connection, the BSS capabilities as reported in
	 * the beacon/probe response.
	 */
	__le16 capability;

	/**
	 * @beacon_int:
	 * on successful connection, the beacon interval of the BSS.
	 */
	__le16 beacon_int;

	/**
	 * @tsf: TODO
	 */
	__le64 tsf;

	/**
	 * @presp_ielen:
	 * the length of the probe response ies.
	 */
	__le32 presp_ielen;

	/**
	 * @beacon_ielen:
	 * the length of the beacon ies.
	 */
	__le32 beacon_ielen;

	/**
	 * @assoc_req_ie_len:
	 * the length of the association request ies.
	 */
	__le32 assoc_req_ie_len;

	/**
	 * @assoc_resp_ie_len:
	 * the length of the association response ies.
	 */
	__le32 assoc_resp_ie_len;

	/**
	 * @qos:
	 * 1 iff the BSS supports WMM.
	 */
	u8 qos;

	/**
	 * @bk_acm:
	 * 1 iff %qos and the BK AC requires admission control.
	 */
	u8 bk_acm;

	/**
	 * @be_acm:
	 * 1 iff %qos and the BE AC requires admission control.
	 */
	u8 be_acm;

	/**
	 * @vi_acm:
	 * 1 iff %qos and the VI AC requires admission control.
	 */
	u8 vi_acm;

	/**
	 * @vo_acm:
	 * 1 iff %qos and the VO AC requires admission control.
	 */
	u8 vo_acm;

	/**
	 * @not_found:
	 * 1 iff no BSS was found suitable for connection.
	 */
	u8 not_found;

	/**
	 * @authorized: TODO
	 */
	u8 authorized;

	/**
	 * @reserved:
	 * reserved
	 */
	u8 reserved;

	/**
	 * @keys:
	 * On successful connection to a secure network that does not require
	 * 802.1x authentication and key derivation, holds the security keys as
	 * defined in &iwl_fmac_keys.
	 */
	struct iwl_fmac_keys keys;

	/**
	 * @ie_data:
	 * the probe response ies (&presp_ielen), followed by the beacon ies
	 * (&beacon_ielen), followed by the association request ies
	 * (&assoc_req_ie_len) followed by the association response ies
	 * (&assoc_resp_ie_len).
	 */
#ifndef _MSC_VER
	u8 ie_data[0];
#endif
} __packed;

/**
 * struct iwl_fmac_disconnect_cmd - disconnect from a network.
 * @vif_id: the virtual interface identifier as returned in
 *	&iwl_fmac_add_vif_resp
 * @locally_generated: 1 if the disconnection was locally generated; Otherwise
 *	0.
 * @reason: reason code for disconnection, if available
 *
 * Can be used both as a command to fmac requesting it to disconnect, and can
 * also be used as a notification sent from fmac to indicate that a previous
 * connection is no longer valid.
 */
struct iwl_fmac_disconnect_cmd {
	u8 vif_id;
	u8 locally_generated;
	__le16 reason;
} __packed;

/**
 * enum iwl_fmac_dbg_type - support debug notification types.
 * @IWL_FMAC_DBG_INT_CMD: Debug notification describing an internal command
 *	from fmac.
 * @IWL_FMAC_DBG_INT_RESP: Debug notification describing an internal command
 *	response to fmac.
 * @IWL_FMAC_DBG_INT_NOTIF: Debug notification describing an asynchronous
 *	notification received by fmac.
 * @IWL_FMAC_DBG_INT_TX: Debug notification describing a frame being
 *	transmitter by fmac.
 */
enum iwl_fmac_dbg_type {
	IWL_FMAC_DBG_INT_CMD,
	IWL_FMAC_DBG_INT_RESP,
	IWL_FMAC_DBG_INT_NOTIF,
	IWL_FMAC_DBG_INT_TX,
};

/**
 * struct iwl_fmac_debug_notif - Notification containing debug data.
 * @type: See %iwl_fmac_dbg_type.
 * @reserved: for alignment.
 * @data: type dependent data.
 *
 * Sent asynchronously from fmac, to notify about fmac interaction with other
 * components.
 */
struct iwl_fmac_debug_notif {
	u8 type;
	u8 reserved[3];
#ifndef _MSC_VER
	u8 data[0];
#endif
} __packed;

/**
 * struct iwl_fmac_keys_update_notif - Notification about update keys.
 * @vif_id: the virtual interface identifier as returned in
 *	&iwl_fmac_add_vif_resp.
 * @sta_id: holds a station entry index associated with the station for which
 *	the keys were updated.
 * @reserved: for alignment.
 * @keys: see &iwl_fmac_keys.
 *
 * The notification is sent from fmac to indicate that new keys were derived for
 * the given station.
 */
struct iwl_fmac_keys_update_notif {
	u8 vif_id;
	u8 sta_id;
	u8 reserved[2];

	struct iwl_fmac_keys keys;
} __packed;

/**
 * struct iwl_fmac_scan_complete_notif - Scan complete notification
 * @aborted: 1 if the scan was aborted; Otherwise 0.
 * @reserved: for alignment.
 *
 * Used to notify about the completion of a scan request originated by calling
 * %FMAC_SCAN.
 */
struct iwl_fmac_scan_complete_notif {
	u8 aborted;
	u8 reserved[3];
} __packed;

/**
 * enum iwl_fmac_nvm_sku_cap - Supported capabilities.
 * @NVM_SKU_CAP_BAND_24GHZ_ENABLED: Operation on 2.4 GHz enabled.
 * @NVM_SKU_CAP_BAND_52GHZ_ENABLED: Operation on 5.2 GHz enabled.
 * @NVM_SKU_CAP_11N_ENABLED: 802.11n support enabled.
 * @NVM_SKU_CAP_11AC_ENABLED: 80211.11ac support enabled.
 * @NVM_SKU_CAP_AMT_ENABLED: AMT enabled.
 * @NVM_SKU_CAP_IPAN_ENABLED: P2P enabled.
 * @NVM_SKU_CAP_MIMO_DISABLED: MIMO is disabled.
 */
enum iwl_fmac_nvm_sku_cap {
	NVM_SKU_CAP_BAND_24GHZ_ENABLED = 0x1,
	NVM_SKU_CAP_BAND_52GHZ_ENABLED = 0x2,
	NVM_SKU_CAP_11N_ENABLED = 0x4,
	NVM_SKU_CAP_11AC_ENABLED = 0x8,
	NVM_SKU_CAP_AMT_ENABLED = 0x10,
	NVM_SKU_CAP_IPAN_ENABLED = 0x20,
	NVM_SKU_CAP_MIMO_DISABLED = 0x40,
};

/**
 * enum iwl_fmac_nvm_ht_cap - Supported HT capabilities.
 * @NVM_HT_CAP_LDPC_CODING: LDPC enabled.
 * @NVM_HT_CAP_SUP_WIDTH_20_40: 40 MHz is supported.
 * @NVM_HT_CAP_SM_PS: SMPS enabled.
 * @NVM_HT_CAP_GRN_FLD: Green field supported.
 * @NVM_HT_CAP_SGI_20: Short guard interval in 20MHz enabled.
 * @NVM_HT_CAP_SGI_40: Short guard interval in 40MHz enabled.
 * @NVM_HT_CAP_TX_STBC: Transmit STBC enabled.
 * @NVM_HT_CAP_RX_STBC: Received STBC enabled
 * @NVM_HT_CAP_DELAY_BA: Delayed block acknowledgment enabled.
 * @NVM_HT_CAP_MAX_AMSDU: large A-MSDU size is supported
 * @NVM_HT_CAP_DSSSCCK40: DSSS-CCK40 is supported
 * @NVM_HT_CAP_RESERVED: (reserved)
 * @NVM_HT_CAP_40MHZ_INTOLERANT: device is 40 MHz intolerant
 * @NVM_HT_CAP_LSIG_TXOP_PROT: L-SIG TXOP protection is supported
 *
 * See 9.4.2.56.2 ("HT Capability Information field") in P802.11Revmc_D5.0.
 */
enum iwl_fmac_nvm_ht_cap {
	NVM_HT_CAP_LDPC_CODING = 0x0001,
	NVM_HT_CAP_SUP_WIDTH_20_40 = 0x0002,
	NVM_HT_CAP_SM_PS = 0x000C,
	NVM_HT_CAP_GRN_FLD = 0x0010,
	NVM_HT_CAP_SGI_20 = 0x0020,
	NVM_HT_CAP_SGI_40 = 0x0040,
	NVM_HT_CAP_TX_STBC = 0x0080,
	NVM_HT_CAP_RX_STBC = 0x0300,
	NVM_HT_CAP_DELAY_BA	= 0x0400,
	NVM_HT_CAP_MAX_AMSDU = 0x0800,
	NVM_HT_CAP_DSSSCCK40 = 0x1000,
	NVM_HT_CAP_RESERVED	= 0x2000,
	NVM_HT_CAP_40MHZ_INTOLERANT	= 0x4000,
	NVM_HT_CAP_LSIG_TXOP_PROT = 0x8000,
};

/**
 * enum iwl_fmac_nvm_vht_cap - Supported VHT capabilities.
 * @NVM_VHT_CAP_MAX_MPDU_LENGTH_3895: max MPDU (A-MSDU) length 3895 bytes
 * @NVM_VHT_CAP_MAX_MPDU_LENGTH_7991: max MPDU (A-MSDU) length 7991 bytes
 * @NVM_VHT_CAP_MAX_MPDU_LENGTH_11454: max MPDU (A-MSDU) length 11454 bytes
 * @NVM_VHT_CAP_MAX_MPDU_MASK: Mask of supported MPDU lengths.
 * @NVM_VHT_CAP_SUPP_CHAN_WIDTH_160MHZ: Operation in 160MHz channels supported.
 * @NVM_VHT_CAP_SUPP_CHAN_WIDTH_160_80PLUS80MHZ: Operation in 80MHz + 80MHz
 *	channels supported.
 * @NVM_VHT_CAP_SUPP_CHAN_WIDTH_MASK: Supported channel widths.
 * @NVM_VHT_CAP_RXLDPC: LDPC is supported on RX
 * @NVM_VHT_CAP_SHORT_GI_80: short guard interval supported in 80 MHz
 * @NVM_VHT_CAP_SHORT_GI_160: short guard interval supported in 160 MHz
 * @NVM_VHT_CAP_TXSTBC: TX STBC supported
 * @NVM_VHT_CAP_RXSTBC_1: RX STBC support: 1 chain
 * @NVM_VHT_CAP_RXSTBC_2: RX STBC support: 2 chains
 * @NVM_VHT_CAP_RXSTBC_3: RX STBC support: 3 chains
 * @NVM_VHT_CAP_RXSTBC_4: RX STBC support: 4 chains
 * @NVM_VHT_CAP_RXSTBC_MASK: RX STBC mask
 * @NVM_VHT_CAP_SU_BEAMFORMER_CAPABLE: Single user Beamformer supported.
 * @NVM_VHT_CAP_SU_BEAMFORMEE_CAPABLE: Single user Beamformee supported.
 * @NVM_VHT_CAP_BEAMFORMEE_STS_MASK: beamformee STS mask
 * @NVM_VHT_CAP_SOUNDING_DIMENSIONS_MASK: sounding dimensions mask
 * @NVM_VHT_CAP_MU_BEAMFORMER_CAPABLE: Multi user Beamformer supported.
 * @NVM_VHT_CAP_MU_BEAMFORMEE_CAPABLE: Multi user Beacmformee supported.
 * @NVM_VHT_CAP_VHT_TXOP_PS: VHT TXOP PS supported
 * @NVM_VHT_CAP_HTC_VHT: HTC supported
 * @NVM_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK: A-MPDU length exponent mask
 * @NVM_VHT_CAP_VHT_LINK_ADAPTATION_VHT_UNSOL_MFB:
 *	VHT link adaptation: unsolicited MFB supported
 * @NVM_VHT_CAP_VHT_LINK_ADAPTATION_VHT_MRQ_MFB:
 *	VHT link adaptation: MRQ MFB is supported
 * @NVM_VHT_CAP_RX_ANTENNA_PATTERN: RX antenna pattern
 * @NVM_VHT_CAP_TX_ANTENNA_PATTERN: TX antenna pattern
 *
 * See 9.4.2.158.2 ("VHT Capabilities Information field") in P802.11Revmc_D5.0.
 */
enum iwl_fmac_nvm_vht_cap {
	NVM_VHT_CAP_MAX_MPDU_LENGTH_3895 = 0x00000000,
	NVM_VHT_CAP_MAX_MPDU_LENGTH_7991 = 0x00000001,
	NVM_VHT_CAP_MAX_MPDU_LENGTH_11454 = 0x00000002,
	NVM_VHT_CAP_MAX_MPDU_MASK = 0x00000003,
	NVM_VHT_CAP_SUPP_CHAN_WIDTH_160MHZ = 0x00000004,
	NVM_VHT_CAP_SUPP_CHAN_WIDTH_160_80PLUS80MHZ = 0x00000008,
	NVM_VHT_CAP_SUPP_CHAN_WIDTH_MASK = 0x0000000C,
	NVM_VHT_CAP_RXLDPC = 0x00000010,
	NVM_VHT_CAP_SHORT_GI_80 = 0x00000020,
	NVM_VHT_CAP_SHORT_GI_160 = 0x00000040,
	NVM_VHT_CAP_TXSTBC = 0x00000080,
	NVM_VHT_CAP_RXSTBC_1 = 0x00000100,
	NVM_VHT_CAP_RXSTBC_2 = 0x00000200,
	NVM_VHT_CAP_RXSTBC_3 = 0x00000300,
	NVM_VHT_CAP_RXSTBC_4 = 0x00000400,
	NVM_VHT_CAP_RXSTBC_MASK = 0x00000700,
	NVM_VHT_CAP_SU_BEAMFORMER_CAPABLE = 0x00000800,
	NVM_VHT_CAP_SU_BEAMFORMEE_CAPABLE = 0x00001000,
	NVM_VHT_CAP_BEAMFORMEE_STS_MASK = 0x0000e000,
	NVM_VHT_CAP_SOUNDING_DIMENSIONS_MASK = 0x00070000,
	NVM_VHT_CAP_MU_BEAMFORMER_CAPABLE = 0x00080000,
	NVM_VHT_CAP_MU_BEAMFORMEE_CAPABLE = 0x00100000,
	NVM_VHT_CAP_VHT_TXOP_PS = 0x00200000,
	NVM_VHT_CAP_HTC_VHT = 0x00400000,
	NVM_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK = 0x03800000,
	NVM_VHT_CAP_VHT_LINK_ADAPTATION_VHT_UNSOL_MFB = 0x08000000,
	NVM_VHT_CAP_VHT_LINK_ADAPTATION_VHT_MRQ_MFB = 0x0c000000,
	NVM_VHT_CAP_RX_ANTENNA_PATTERN = 0x10000000,
	NVM_VHT_CAP_TX_ANTENNA_PATTERN = 0x20000000,
};

#define NVM_HT_MCS_MASK_LEN		10

/**
 * struct iwl_fmac_nvm_mcs_info - Supported HT MCSes
 * @rx_mask: RX mask (like in 802.11)
 * @rx_highest: RX highest (like in 802.11)
 * @tx_params: TX parameters (like in 802.11)
 * @reserved: for alignment.
 *
 * See 9.4.2.56.4 ("Supported MCS Set field") in P802.11Revmc_D5.0.
 */
struct iwl_fmac_nvm_mcs_info {
	u8 rx_mask[NVM_HT_MCS_MASK_LEN];
	__le16 rx_highest;
	u8 tx_params;
	u8 reserved[3];
} __packed;

/**
 * struct iwl_fmac_nvm_vht_mcs_info - Supported VHT MCSes
 * @rx_mcs_map: RX MCS map (like in 802.11)
 * @rx_highest: RX highest (like in 802.11)
 * @tx_mcs_map: TX MCS map (like in 802.11)
 * @tx_highest: TX highest (like in 802.11)
 *
 * See 9.4.2.158.3 ("Supported VHT-MCS and NSS Set field") in P802.11Revmc_D5.0.
 */
struct iwl_fmac_nvm_vht_mcs_info {
	__le16 rx_mcs_map;
	__le16 rx_highest;
	__le16 tx_mcs_map;
	__le16 tx_highest;
} __packed;

/**
 * enum iwl_fmac_nvm_bands - Supported bands.
 * @NVM_BAND_24GHZ: Operation on 2.4GHz.
 * @NVM_BAND_52GHZ: Operation on 5.2GHz.
 * @NVM_NUM_BANDS: number of defined/possible bands
 */
enum iwl_fmac_nvm_bands {
	NVM_BAND_24GHZ,
	NVM_BAND_52GHZ,
	NVM_NUM_BANDS
};

/**
 * struct iwl_fmac_nvm_ht - supported HT capabilities.
 * @ht_supported: 1 if HT is supported; Otherwise 0.
 * @reserved: for alignment.
 * @cap: See &iwl_fmac_nvm_ht_cap.
 * @ampdu_factor: A-MPDU factor (like in 802.11)
 * @ampdu_density: A-MPDU density (like in 802.11)
 * @mcs: See &iwl_fmac_nvm_mcs_info.
 */
struct iwl_fmac_nvm_ht {
	u8 ht_supported;
	u8 reserved[3];
	__le16 cap;
	u8 ampdu_factor;
	u8 ampdu_density;
	struct iwl_fmac_nvm_mcs_info mcs;
} __packed;

/**
 * struct iwl_fmac_nvm_vht - supported VHT capabilities
 * @vht_supported: 1 if VHT is supported; Otherwise 0.
 * @reserved: for alignment.
 * @cap: See %iwl_fmac_nvm_vht_cap
 * @vht_mcs: See %iwl_fmac_nvm_vht_mcs_info
 */
struct iwl_fmac_nvm_vht {
	u8 vht_supported;
	u8 reserved[3];
	__le32 cap;
	struct iwl_fmac_nvm_vht_mcs_info vht_mcs;
} __packed;

/**
 * struct iwl_fmac_nvm_cmd - NVM configuration command.
 * @sku_cap: See &enum iwl_fmac_nvm_sku_cap
 * @n_addr: number of supported addresses.
 * @hw_addr: hw base address.
 * @valid_ant: Valid antenna configuration.
 * @reserved: for alignment.
 * @ht: HT configuration for each band. See &iwl_fmac_nvm_ht.
 * @vht: VHT configuration for each band. See &iwl_fmac_nvm_vht.
 *
 * The command is sent once in the lifetime of fmac, as part of the
 * initialization flow, to configure the runtime capabilities and supported
 * features of fmac.
 */
struct iwl_fmac_nvm_cmd {
	u8 sku_cap;
	u8 n_addr;
	u8 hw_addr[ETH_ALEN];
#define NVM_CMD_TX_ANT(_x) ((_x) & 0xf)
#define NVM_CMD_RX_ANT(_x) (((_x) & 0xf0) >> 4)
	u8 valid_ant;
	u8 reserved[3];
	struct iwl_fmac_nvm_ht ht[NVM_NUM_BANDS];
	struct iwl_fmac_nvm_vht vht[NVM_NUM_BANDS];
} __packed;

#ifdef CPTCFG_IWLFMAC_9000_SUPPORT
/**
 * struct iwl_fmac_req_queue - Request Transmit queue.
 * @vif_id: the vif_id of the STA to use.
 * @sta_id: sta_id to add a queue to.
 * @tid: the TID of the traffic for the requested queue.
 * @reserved: for alignment.
 *
 * The command is used to request a transmit queue for the given
 * <station, TID>.
 *
 * Note that this is only used with 9000-series devices.
 */
struct iwl_fmac_req_queue {
	u8 vif_id;
	u8 sta_id;
	u8 tid;
	u8 reserved;
} __packed;

/**
 * struct iwl_fmac_req_queue_response - Response to a transmit queue allocation
 * request.
 * @queue: the queue allocated for the request. 0xff means failure.
 * @reserved: for alignment.
 *
 * Note that this is only used with 9000-series devices.
 */
struct iwl_fmac_req_queue_response {
	u8 queue;
	u8 reserved[3];
} __packed;

/**
 * struct iwl_fmac_rel_queue - Request to release a transmit queue.
 * @vif_id: the vif_id of the STA to use.
 * @sta_id: sta_id to release the TID from.
 * @tid: the TID of the traffic to remove
 * @reserved: for alignment.
 *
 * The command is used to request to release the transmit queue allocation for
 * the <sta, TID>
 *
 * Note that this is only used with 9000-series devices.
 */
struct iwl_fmac_rel_queue {
	u8 vif_id;
	u8 sta_id;
	u8 tid;
	u8 reserved;
} __packed;

/**
 * struct iwl_fmac_rel_queue_response - Response to a transmit queue release
 * request.
 * @free_queue: 1 if the queue should be freed, 0 otherwise.
 * @reserved: for alignment.
 *
 * Note that this is only used with 9000-series devices.
 */
struct iwl_fmac_rel_queue_response {
	u8 free_queue;
	u8 reserved[3];
} __packed;
#endif

/**
 * struct iwl_fmac_rs_fixed_cmd - set fixed rate for transmit.
 * @sta_id: station to set the rate.
 * @vif_id: the vif_id of the STA to use.
 * @reduced_txp: set power reduction.
 * @reserved: for alignment.
 * @hw_rate: the fixed value for the rate in LMAC format.
 *
 * The command is used to request to disable the transmit rate scaling
 * algorithm, and instead use the given fixed rate.
 */
struct iwl_fmac_rs_fixed_cmd {
	u8 sta_id;
	u8 vif_id;
	u8 reduced_txp;
	u8 reserved;
	__le32 hw_rate;
} __packed;

#ifdef CPTCFG_IWLFMAC_9000_SUPPORT
/**
 * struct iwl_fmac_scd_txq_cfg_cmd - FMAC txq hw scheduler config command.
 * @vif_id: the vif_id of the STA to use.
 * @reserved1: for alignment.
 * @token: token (unused)
 * @sta_id: sta_id to use.
 * @tid: the TID of the traffic for the requested queue.
 * @scd_queue: scheduler queue to configure.
 * @enable: 1 queue enable, 0 queue disable.
 * @aggregate: 1 aggregated queue, 0 otherwise.
 * @tx_fifo: &enum iwl_fmac_tx_fifo.
 * @window: BA window size.
 * @ssn: SSN for the BA agreement.
 * @reserved2: for alignment.
 *
 * Note that this is only used with 9000-series devices.
 */
struct iwl_fmac_scd_txq_cfg_cmd {
	u8 vif_id;
	u8 reserved1[3];
	u8 token;
	u8 sta_id;
	u8 tid;
	u8 scd_queue;
	u8 enable;
	u8 aggregate;
	u8 tx_fifo;
	u8 window;
	__le16 ssn;
	__le16 reserved2;
} __packed;
#endif

/**
 * enum iwl_fmac_sync_source - Source of the Rx multi queue synchronization
 * request
 * @IWL_FMAC_SYNC_SRC_DRIVER: the request originated in the driver.
 * @IWL_FMAC_SYNC_SRC_FMAC: the request originated in fmac.
 */
enum iwl_fmac_sync_source {
	IWL_FMAC_SYNC_SRC_DRIVER,
	IWL_FMAC_SYNC_SRC_FMAC,
};

/**
 * enum iwl_fmac_sync_type - Type of the Rx multi queue synchronization request.
 * @IWL_FMAC_SYNC_TYPE_DELBA: request due to Rx delba.
 */
enum iwl_fmac_sync_type {
	IWL_FMAC_SYNC_TYPE_DELBA,
};

/**
 * struct iwl_rxq_sync_payload - shared sync notification payload.
 * @src: see &enum iwl_fmac_sync_source.
 * @type: see &enum iwl_fmac_sync_type for FMAC-sourced messages.
 * @reserved: for alignment.
 * @payload: payload for the message.
 *
 * This is the sync message payload, sometimes generated by the
 * FMAC firmware and possibly for use by the driver.
 */
struct iwl_rxq_sync_payload {
	u8 src;
	u8 type;
	u8 reserved[2];
#ifndef _MSC_VER
	u8 payload[0];
#endif
} __packed;

/**
 * struct iwl_rx_sync_delba - shared sync notification for delba.
 * @hdr: see &iwl_req_sync_payload.
 * @sta_id: the corresponding station identifier.
 * @ba_id: the bloack ack identifier.
 * @reserved: for alignment.
 */
struct iwl_rx_sync_delba {
	struct iwl_rxq_sync_payload hdr;
	u8 sta_id;
	u8 ba_id;
	u8 reserved[2];
} __packed;

/**
 * enum fmac_ps_mode - Enumerates the support power schemes for the device.
 * @FMAC_PS_MODE_CAM: No power save.
 * @FMAC_PS_MODE_BALANCED: Balanced power save.
 * @FMAC_PS_MODE_LP: Low power save mode.
 */
enum fmac_ps_mode {
	FMAC_PS_MODE_CAM = 1,
	FMAC_PS_MODE_BALANCED,
	FMAC_PS_MODE_LP,
};

/**
 * enum fmac_bt_cfg_mode - Enumerates the support the BT Coex modes.
 * @FMAC_BT_CFG_NW: N-wire.
 * @FMAC_BT_CFG_DISABLE: BT Coex disabled.
 * @FMAC_BT_CFG_BT: BT always gets the antenna.
 * @FMAC_BT_CFG_WIFI: WIFI always gets the antenna.
 */
enum fmac_bt_cfg_mode {
	FMAC_BT_CFG_NW = 0,
	FMAC_BT_CFG_DISABLE,
	FMAC_BT_CFG_BT,
	FMAC_BT_CFG_WIFI,
};

/**
 * enum fmac_uapsd_enable_mode - Enumerates the bits for U-APSD enablement.
 * @FMAC_UAPSD_ENABLE_BSS: U-APSD is enabled for BSS role.
 * @FMAC_UAPSD_ENABLE_P2P_CLIENT: U-APSD is enabled for P2P Client role.
 */
enum fmac_uapsd_enable_mode {
	FMAC_UAPSD_ENABLE_BSS = BIT(0),
	FMAC_UAPSD_ENABLE_P2P_CLIENT = BIT(1),
};

/**
 * enum umac_scan_type - defines the possible scan types
 * @IWL_SCAN_TYPE_NOT_SET: the scan type is undefined
 * @IWL_SCAN_TYPE_UNASSOC: scan type to be used when unassociated
 * @IWL_SCAN_TYPE_WILD: agressive scan that can be used when the
 *	latency requirement and the throughput are not high.
 * @IWL_SCAN_TYPE_MILD: gentle scan that can be used when there is
 *	some throughput without low latency requirements.
 * @IWL_SCAN_TYPE_FRAGMENTED: fragmented scan types where small blocks
 *	of scan are performed separately in order to prevent latency
 *	and throughput disruptions.
 * @IWL_SCAN_TYPE_MAX: highest index of scan.
 */
enum umac_scan_type {
	IWL_SCAN_TYPE_NOT_SET,
	IWL_SCAN_TYPE_UNASSOC,
	IWL_SCAN_TYPE_WILD,
	IWL_SCAN_TYPE_MILD,
	IWL_SCAN_TYPE_FRAGMENTED,
	IWL_SCAN_TYPE_MAX,
};

#define IWL_FMAC_POWER_LEVEL_UNSET 0xff

/**
 * enum fmac_sad_mode: choose the single antenna diversity mode (SAD)
 * @FMAC_SAD_ENABLED: toogle the enablement of SAD
 * @FMAC_SAD_NIC_DEFAULT: use NIC default value
 * @FMAC_SAD_ANT_A: choose antenna A by default
 * @FMAC_SAD_ANT_B: choose antenna B by default
 */
enum fmac_sad_mode {
	FMAC_SAD_ENABLED	= BIT(0),
	FMAC_SAD_NIC_DEFAULT	= 0 << 1,
	FMAC_SAD_ANT_A		= 1 << 1,
	FMAC_SAD_ANT_B		= 2 << 1,
};

/**
 * enum iwl_fmac_config_id - configuration id.
 * @IWL_FMAC_STATIC_CONFIG_U32_START: first static global config that fit a
 *	u32. A static config is a config that can't be modified after
 *	@IWL_FMAC_STATIC_CONFIG_COMPLETE has been sent.
 * @IWL_FMAC_STATIC_CONFIG_POWER_SCHEME: see &enum fmac_ps_mode.
 * @IWL_FMAC_STATIC_CONFIG_COEX_MODE: see &enum fmac_bt_cfg_mode.
 * @IWL_FMAC_STATIC_CONFIG_COEX_SYNC2SCO: boolean.
 * @IWL_FMAC_STATIC_CONFIG_COEX_PLCR: boolean.
 * @IWL_FMAC_STATIC_CONFIG_COEX_MPLUT: boolean.
 * @IWL_FMAC_STATIC_CONFIG_DEPRECATED_1: Not in use
 * @IWL_FMAC_STATIC_CONFIG_DEPRECATED_2: Not in use
 * @IWL_FMAC_STATIC_CONFIG_UAPSD_ENABLED: bitmap for U-APSD enablement. Check
 *	&enum fmac_uapsd_enable_mode. Default is 0.
 * @IWL_FMAC_STATIC_CONFIG_LTR_MODE: PCIe link training mode
 * @IWL_FMAC_STATIC_CONFIG_SINGLE_ANT_DIVERSITY_CONF: see &enum fmac_sad_mode.
 * @IWL_FMAC_STATIC_CONFIG_U32_MAX: highest index of static global
 *	configuration.
 * @IWL_FMAC_STATIC_CONFIG_U32_NUM: number of static global configs that fit
 *	a u32. A static config is a config that can't be modified after
 *	@IWL_FMAC_STATIC_CONFIG_COMPLETE has been sent.
 *
 * @IWL_FMAC_CONFIG_U32_START: first u32 config that is global but can be
 *	changed on the fly.
 * @IWL_FMAC_CONFIG_INTERNAL_CMD_TO_HOST: forward internal commands to
 *	host for debug.
 * @IWL_FMAC_CONFIG_RS_STAT_THOLD: threshold for sending RS statistics
 *	notifications from LMAC.
 * @IWL_FMAC_CONFIG_SCAN_TYPE: force scan type, regardless of internal
 *	policies, according to &enum umac_scan_type.
 * @IWL_FMAC_CONFIG_U32_MAX:highest index of configs that fit a u32 and that
 *	can be changed on the fly.
 * @IWL_FMAC_CONFIG_U32_NUM: number of configs that fit	a u32 and that can
 *	be changed on the fly.
 *
 * @IWL_FMAC_CONFIG_START: first config that doesn't fit a u32. Those
 *	configurations may or may not be modified on the fly. Depending
 *	on the configuration. The firmware will not enforce any policy.
 * @IWL_FMAC_CONFIG_DEBUG_LEVEL: debug level of the FMAC component in the
 *	firmware. Since it can't be stored in the same place as other
 *	CONFIG_U32 confs, it is in this section.
 * @IWL_FMAC_CONFIG_TRIGGER: trigger configuration
 * @IWL_FMAC_CONFIG_MAX: highest index of configs that don't fit a u32.
 * @IWL_FMAC_CONFIG_NUM: number of configs that don't git a u32.
 *
 * @IWL_FMAC_CONFIG_VIF_START: first per-vif configuration
 * @IWL_FMAC_CONFIG_VIF_POWER_DISABLED: power save disablement
 * @IWL_FMAC_CONFIG_VIF_TXPOWER_USER: user-configured txpower in dbm,
 *	or IWL_FMAC_UNSET_POWER_LEVEL if unset
 * @IWL_FMAC_CONFIG_VIF_LOW_LATENCY: user-configured low latency mode.
 * @IWL_FMAC_CONFIG_VIF_RESPONDER_FLAGS: user-configured responder flags.
 *	see &iwl_tof_responder_cfg_flags.
 * @IWL_FMAC_CONFIG_VIF_CMD_FIELDS: user-configured cmd valid fields.
 *	see iwl_tof_responder_cmd_valid_field.
 * @IWL_FMAC_CONFIG_VIF_FTM_RESP_ASAP: user-configured FTM responder asap.
 * @IWL_FMAC_CONFIG_VIF_MAX: highest index of per-vif config
 * @IWL_FMAC_CONFIG_VIF_NUM: number of per-vif configs
 *
 * @IWL_FMAC_STATIC_CONFIG_COMPLETE: indicates that all the static
 *	configuration has been applied. Must be sent once in the firmware's
 *	life. No data should be attached to this configuration.
 */
enum iwl_fmac_config_id {
	IWL_FMAC_STATIC_CONFIG_U32_START = 0x0,
	IWL_FMAC_STATIC_CONFIG_POWER_SCHEME = IWL_FMAC_STATIC_CONFIG_U32_START,
	IWL_FMAC_STATIC_CONFIG_COEX_MODE,
	IWL_FMAC_STATIC_CONFIG_COEX_SYNC2SCO,
	IWL_FMAC_STATIC_CONFIG_COEX_PLCR,
	IWL_FMAC_STATIC_CONFIG_COEX_MPLUT,
	IWL_FMAC_STATIC_CONFIG_DEPRECATED_1,
	IWL_FMAC_STATIC_CONFIG_DEPRECATED_2,
	IWL_FMAC_STATIC_CONFIG_UAPSD_ENABLED,
	IWL_FMAC_STATIC_CONFIG_LTR_MODE,
	IWL_FMAC_STATIC_CONFIG_SINGLE_ANT_DIVERSITY_CONF,
	IWL_FMAC_STATIC_CONFIG_U32_MAX,
	IWL_FMAC_STATIC_CONFIG_U32_NUM = IWL_FMAC_STATIC_CONFIG_U32_MAX -
		IWL_FMAC_STATIC_CONFIG_U32_START,

	IWL_FMAC_CONFIG_U32_START = 0x100,
	IWL_FMAC_CONFIG_INTERNAL_CMD_TO_HOST = IWL_FMAC_CONFIG_U32_START,
	IWL_FMAC_CONFIG_RS_STAT_THOLD,
	IWL_FMAC_CONFIG_SCAN_TYPE,
	IWL_FMAC_CONFIG_U32_MAX,
	IWL_FMAC_CONFIG_U32_NUM = IWL_FMAC_CONFIG_U32_MAX -
		IWL_FMAC_CONFIG_U32_START,

	IWL_FMAC_CONFIG_START = 0x200,
	IWL_FMAC_CONFIG_DEBUG_LEVEL = IWL_FMAC_CONFIG_START,
	IWL_FMAC_CONFIG_TRIGGER,
	IWL_FMAC_CONFIG_MAX,
	IWL_FMAC_CONFIG_NUM = IWL_FMAC_CONFIG_MAX - IWL_FMAC_CONFIG_START,

	IWL_FMAC_CONFIG_VIF_START = 0x300,
	IWL_FMAC_CONFIG_VIF_POWER_DISABLED = IWL_FMAC_CONFIG_VIF_START,
	IWL_FMAC_CONFIG_VIF_TXPOWER_USER,
	IWL_FMAC_CONFIG_VIF_LOW_LATENCY,
	IWL_FMAC_CONFIG_VIF_RESPONDER_FLAGS,
	IWL_FMAC_CONFIG_VIF_CMD_FIELDS,
	IWL_FMAC_CONFIG_VIF_FTM_RESP_ASAP,
	IWL_FMAC_CONFIG_VIF_MAX,
	IWL_FMAC_CONFIG_VIF_NUM =
		IWL_FMAC_CONFIG_VIF_MAX - IWL_FMAC_CONFIG_VIF_START,

	IWL_FMAC_STATIC_CONFIG_COMPLETE = 0xffff,
};

#define IWL_FMAC_VIF_ID_GLOBAL 0xff

/**
 * struct iwl_fmac_config_cmd - configuration command.
 * @vif_id: vif_id or IWL_FMAC_VIF_ID_GLOBAL for global configuration.
 * @reserved: for alignment.
 * @config_id: see &enum iwl_fmac_config_id.
 * @len: the length of the configuration in bytes (must be a multiple of 4).
 * @data: the data of the configuration.
 */
struct iwl_fmac_config_cmd {
	u8 vif_id;
	u8 reserved[3];
	__le16 config_id;
	__le16 len;
#ifndef _MSC_VER
	u8 data[0];
#endif
} __packed;

/**
 * enum iwl_fmac_chan_width - channel widths.
 * @IWL_CHAN_WIDTH_20_NOHT: 20MHz without HT.
 * @IWL_CHAN_WIDTH_20: 20MHz with HT.
 * @IWL_CHAN_WIDTH_40: 40MHz.
 * @IWL_CHAN_WIDTH_80: 80MHz.
 * @IWL_CHAN_WIDTH_160: 160MHz (including 80MHz + 80MHz).
 * @IWL_NUM_CHAN_WIDTH: number of supported channel width values
 */
enum iwl_fmac_chan_width {
	IWL_CHAN_WIDTH_20_NOHT,
	IWL_CHAN_WIDTH_20,
	IWL_CHAN_WIDTH_40,
	IWL_CHAN_WIDTH_80,
	IWL_CHAN_WIDTH_160,
	IWL_NUM_CHAN_WIDTH
};

#define IWL_FMAC_NUM_CHAIN_LIMITS	2
#define IWL_FMAC_NUM_SUB_BANDS		5

struct iwl_fmac_sar_restrictions {
	__le16 per_chain_restriction[IWL_FMAC_NUM_CHAIN_LIMITS][IWL_FMAC_NUM_SUB_BANDS];
} __packed;

/**
 * enum iwl_fmac_hidden_ssid - types of hidden ssid
 * @IWL_FMAC_HIDDEN_SSID_NONE: not hidden
 * @IWL_FMAC_HIDDEN_SSID_ZERO_LEN: use zero length in the SSID IE.
 * @IWL_FMAC_HIDDEN_SSID_ZERO_BYTES: use real length, but zero the SSID bytes
 */
enum iwl_fmac_hidden_ssid {
	IWL_FMAC_HIDDEN_SSID_NONE = 0,
	IWL_FMAC_HIDDEN_SSID_ZERO_LEN = 1,
	IWL_FMAC_HIDDEN_SSID_ZERO_BYTES = 2,
};

/**
 * struct iwl_fmac_chandef - channel definition.
 * @control_freq: control frequency.
 * @reserved: for alignment.
 * @center_freq1: center frequency for the channel.
 * @bandwidth: see &iwl_fmac_chan_width.
 * @reserved2: for alignment.
 */
struct iwl_fmac_chandef {
	__le16 control_freq;
	__le16 center_freq1;
	__le16 reserved;
	u8 bandwidth;
	u8 reserved2;
} __packed;

/**
 * struct iwl_fmac_start_ap_cmd - Request to start AP functionality.
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp.
 * @dtim_period: the DTIM beacon in units of &beacon_int.
 * @hidden_ssid: see &iwl_fmac_hidden_ssid.
 * @ssid_len: the length of &ssid.
 * @ssid:the SSID to be used for the AP.
 * @beacon_int: the beacon interval in TU.
 * @inactivity_timeout: the max inactivity for clients, before they are removed
 *	from the BSS (given in seconds).
 * @chandef: see &iwl_fmac_chandef.
 * @crypto: the connection security configuration as specified in
 *	&iwl_fmac_crypto.
 *
 * The command is used to request to start AP functionality. The flow to start
 * the AP is an asynchronous flow, and upon completion a %FMAC_AP_STARTED event
 * will be sent from fmac.
 */
struct iwl_fmac_start_ap_cmd {
	u8 vif_id;
	u8 dtim_period;
	u8 hidden_ssid;
	u8 ssid_len;
	u8 ssid[32];
	__le32 beacon_int;
	__le32 inactivity_timeout;

	struct iwl_fmac_chandef chandef;

	struct iwl_fmac_crypto crypto;
} __packed;

/**
 * enum iwl_fmac_start_ap_resp_status - Status in &iwl_fmac_start_ap_resp.
 * @IWL_FMAC_START_AP_SUCCESS: Success to start AP.
 * @IWL_FMAC_START_AP_INVALID_PARAMS: Fail to start AP due to invalid params.
 * @IWL_FMAC_START_AP_UNSUPPORTED_PARAMS: Fail to start AP due to unsupported
 * params.
 */
enum iwl_fmac_start_ap_resp_status {
	IWL_FMAC_START_AP_SUCCESS = 0,
	IWL_FMAC_START_AP_INVALID_PARAMS,
	IWL_FMAC_START_AP_UNSUPPORTED_PARAMS
};

/**
 * enum iwl_fmac_ap_started_status - Status in &iwl_fmac_start_ap_resp.
 * @IWL_FMAC_AP_STARTED_STATUS_SUCCESS: Success to start AP.
 * @IWL_FMAC_AP_STARTED_STATUS_FAILURE: Fail to start AP.
 */
enum iwl_fmac_ap_started_status {
	IWL_FMAC_AP_STARTED_STATUS_SUCCESS = 0,
	IWL_FMAC_AP_STARTED_STATUS_FAILURE,
};

/**
 * enum iwl_fmac_action - for struct iwl_fmac_host_ap_cmd's action
 * @IWL_FMAC_START: to start the host based AP
 * @IWL_FMAC_STOP: to stop the host based AP
 * @IWL_FMAC_MODIFY: the modify the host based AP
 */
enum iwl_fmac_action {
	IWL_FMAC_START	= 0,
	IWL_FMAC_STOP	= 1,
	IWL_FMAC_MODIFY	= 2,
};

/**
 * enum iwl_fmac_host_ap_changed - describe what field is valid
 * @IWL_FMAC_CTS_PROT_CHANGED: use_cts_prot is valid
 * @IWL_FMAC_SHORT_PREAMBLE_CHANGED: use_short_preamble is valid
 * @IWL_FMAC_SHORT_SLOT_CHANGED: use_short_slot is valid
 * @IWL_FMAC_BASIC_RATES_CHANGED: basic_rates_bitmap is valid
 * @IWL_FMAC_HT_OPMODE_CHANGED: ht_opmode is valid
 * @IWL_FMAC_AC_PARAMS_CHANGED_BK: ac_params for BK is valid
 * @IWL_FMAC_AC_PARAMS_CHANGED_BE: ac_params for BE is valid
 * @IWL_FMAC_AC_PARAMS_CHANGED_VI: ac_params for VI is valid
 * @IWL_FMAC_AC_PARAMS_CHANGED_VO: ac_params for VO is valid
 */
enum iwl_fmac_host_ap_changed {
	IWL_FMAC_CTS_PROT_CHANGED	= BIT(0),
	IWL_FMAC_SHORT_PREAMBLE_CHANGED	= BIT(1),
	IWL_FMAC_SHORT_SLOT_CHANGED	= BIT(2),
	IWL_FMAC_BASIC_RATES_CHANGED	= BIT(3),
	IWL_FMAC_HT_OPMODE_CHANGED	= BIT(4),
	IWL_FMAC_AC_PARAMS_CHANGED_BK	= BIT(5),
	IWL_FMAC_AC_PARAMS_CHANGED_BE	= BIT(6),
	IWL_FMAC_AC_PARAMS_CHANGED_VI	= BIT(7),
	IWL_FMAC_AC_PARAMS_CHANGED_VO	= BIT(8),
};

/**
 * struct iwl_fmac_ac_params - describes the AC params
 * @txop: maximum burst time
 * @cw_min: minimum contention window
 * @cw_max: maximum contention window
 * @aifs: Arbitration interframe space
 * @reserved: for alignment
 */
struct iwl_fmac_ac_params {
	__le16 txop;
	__le16 cw_min;
	__le16 cw_max;
	u8 aifs;
	u8 reserved;
} __packed;

/**
 * struct iwl_fmac_host_ap_cmd - manage a host based AP vif
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp.
 *	The vif's type must be %@IWL_FMAC_IFTYPE_HOST_BASED_AP.
 * @action: see &enum iwl_fmac_action. Note: not all fields are relevant for
 *	all the actions.
 * @dtim_period: the DTIM beacon in units of &beacon_int. Ignored in any action
 *	that is not %IWL_FMAC_START.
 * @use_cts_prot: Whether to use CTS protection
 * @use_short_preamble: Whether the use of short preambles is allowed
 * @use_short_slot: Whether the use of short slot time is allowed
 * @basic_rates_bitmap: bitmap of basic rates:
 *	bit  0:  1Mbps bit  1: 2Mbps  bit 2:  5Mbps bit 3: 11Mbps bit 4:  6Mbps
 *	bit  5:  9Mbps bit  6: 12Mbps bit 7: 18Mbps bit 8: 24Mbps bit 9: 36Mbps
 *	bit 10: 48Mbps bit 11: 54Mbps
 *	Can left to 0 if no update is required.
 * @ht_opmode: HT Operation mode
 * @beacon_int: the beacon interval in TU. Ignored in any &action that is not
 *	%IWL_FMAC_START.
 * @inactivity_timeout: the max inactivity for clients, before they are removed
 *	from the BSS (given in seconds). Ignored in any &action that is not
 *	%IWL_FMAC_START.
 * @chandef: see &iwl_fmac_chandef. Ignored in any &action that is not
 *	%IWL_FMAC_START.
 * @changed: indicates what field changed. See &enum iwl_fmac_host_ap_changed.
 * @ac_params: the AC parameters. The order of the AC in the array is:
 *	0: BK, 1: BE, 2: VI, 3: VO
 *
 * The command is used to manage (start / modify / stop) host based AP
 * functionality.
 * The flow to manage the host based AP is a synchronous flow as opposed to
 * the regular AP mode. The response of this command is &struct
 * iwl_fmac_host_ap_resp. All the management and EAPOL frames will be handled
 * in the host.
 */
struct iwl_fmac_host_ap_cmd {
	u8 vif_id;
	u8 action;
	u8 dtim_period;
	u8 use_cts_prot;
	u8 use_short_preamble;
	u8 use_short_slot;
	__le16 basic_rates_bitmap;
	__le16 ht_opmode;
	__le16 beacon_int;
	__le32 inactivity_timeout;
	struct iwl_fmac_chandef chandef;
	struct iwl_fmac_ac_params ac_params[4];
	__le32 changed;
} __packed;

/**
 * struct iwl_fmac_host_ap_resp - Response of the %FMAC_HOST_BASED_AP
 */
struct iwl_fmac_host_ap_resp {
	/**
	 * @vif_id:
	 * the interface identifier returned in &iwl_fmac_add_vif_resp.
	 */
	u8 vif_id;

	/**
	 * @mcast_sta_id:
	 * the identifier allocation for the used for broadcast and  multicast
	 * transmissions. Relevant only if the %action was %IWL_FMAC_START.
	 */
	u8 mcast_sta_id;

	/**
	 * @bcast_sta_id:
	 * the identifier allocation for the used for broadcast management
	 * frames. Relevant only if the %action was %IWL_FMAC_START.
	 */
	u8 bcast_sta_id;

#ifdef CPTCFG_IWLFMAC_9000_SUPPORT
	/**
	 * @mcast_queue:
	 * queue allocation for broadcast and multicast transmissions.
	 * Only valid for 9000-series devices, otherwise reserved.
	 * Relevant only if the %action was %IWL_FMAC_START.
	 */
	u8 mcast_queue;

	/**
	 * @bcast_queue:
	 * queue allocation for broadcast management frames.
	 * Only valid for 9000-series devices, otherwise reserved.
	 * Relevant only if the %action was %IWL_FMAC_START.
	 */
	u8 bcast_queue;

	/**
	 * @reserved:
	 * for alignment.
	 */
	u8 reserved[3];
#else
	/**
	 * @reserved: reserved
	 */
	u8 reserved[5];
#endif

	/**
	 * @status:
	 * status defined in &iwl_fmac_ap_started_status.
	 */
	__le32 status;
} __packed;

/**
 * enum iwl_fmac_ibss_joined_status - Status in &iwl_fmac_ibss_joined.
 * @IWL_FMAC_IBSS_JOINED_STATUS_SUCCESS: Success to join ibss.
 * @IWL_FMAC_IBSS_JOINED_STATUS_FAILURE: Fail to join ibss.
 */
enum iwl_fmac_ibss_joined_status {
	IWL_FMAC_IBSS_JOINED_STATUS_SUCCESS = 0,
	IWL_FMAC_IBSS_JOINED_STATUS_FAILURE,
};

/**
 * struct iwl_fmac_start_ap_resp - Response to %FMAC_START_AP command.
 * @status: status defined in &iwl_fmac_start_ap_resp_status.
 */
struct iwl_fmac_start_ap_resp {
	__le32 status;
} __packed;

/**
 * struct iwl_fmac_stop_ap_cmd - Stop AP functionality.
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp
 * @reserved: for alignment.
 */
struct iwl_fmac_stop_ap_cmd {
	u8 vif_id;
	u8 reserved[3];
} __packed;

/**
 * struct iwl_fmac_sta_added - Notify about a newly associated station.
 */
struct iwl_fmac_sta_added {
	/**
	 * @vif_id:
	 * the interface identifier returned in &iwl_fmac_add_vif_resp.
	 */
	u8 vif_id;

	/**
	 * @sta_id:
	 * holds a station entry index associated the newly associated station
	 */
	u8 sta_id;

	/**
	 * @addr:
	 * MAC address of the new station.
	 */
	u8 addr[ETH_ALEN];

	/**
	 * @qos:
	 * 1 is qos is supported; otherwise 0.
	 */
	u8 qos;

	/**
	 * @reserved:
	 * reserved
	 */
	u8 reserved[3];

	/**
	 * @keys:
	 * station PTKs (for a secured network)
	 */
	struct iwl_fmac_keys keys;
} __packed;

/**
 * struct iwl_fmac_sta_removed - Notify about a removed station.
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp.
 * @sta_id: holds a station entry index associated the removed station.
 * @reserved: reserved
 */
struct iwl_fmac_sta_removed {
	u8 vif_id;
	u8 sta_id;
	u8 reserved[2];
} __packed;

/**
 * struct iwl_fmac_ap_started - Notify that AP functionality has started.
 */
struct iwl_fmac_ap_started {
	/**
	 * @vif_id:
	 * the interface identifier returned in &iwl_fmac_add_vif_resp.
	 */
	u8 vif_id;

	/**
	 * @mcast_sta_id:
	 * the identifier allocation for the used for broadcast and
	 * multicast transmissions.
	 */
	u8 mcast_sta_id;

#ifdef CPTCFG_IWLFMAC_9000_SUPPORT
	/**
	 * @mcast_queue:
	 * queue allocation for broadcast and multicast transmissions.
	 * Only valid for 9000-series devices, otherwise reserved.
	 */
	u8 mcast_queue;

	/**
	 * @reserved:
	 * for alignment.
	 */
	u8 reserved;
#else
	/**
	 * @reserved: reserved
	 */
	__le16 reserved;
#endif

	/**
	 * @keys:
	 * GTKs (for a secured AP)
	 */
	struct iwl_fmac_keys keys; /* gtk keys will be sent here */

	/**
	 * @chandef:
	 * chandef the AP started on.
	 */
	struct iwl_fmac_chandef chandef;

	/**
	 * @status:
	 * status defined in &iwl_fmac_ap_started_status.
	 */
	__le32 status;
} __packed;

/**
 * enum iwl_fmac_dbg_trigger - triggers available
 */
enum iwl_fmac_dbg_trigger {
	/**
	 * @IWL_FMAC_DBG_TRIGGER_INVALID:
	 * (reserved)
	 */
	IWL_FMAC_DBG_TRIGGER_INVALID = 0,

	/**
	 * @IWL_FMAC_DBG_TRIGGER_MISSED_BEACONS:
	 * trigger on missed beacons
	 */
	IWL_FMAC_DBG_TRIGGER_MISSED_BEACONS = 3,

	/**
	 * @IWL_FMAC_DBG_TRIGGER_MAX:
	 * maximum number of triggers supported
	 */
	IWL_FMAC_DBG_TRIGGER_MAX /* must be last */
};

/**
 * struct iwl_fmac_trigger_cmd
 * @len: length of %data
 * @id: &enum iwl_fmac_dbg_trigger
 * @vif_type: %iwl_fmac_vif_type
 * @data: trigger-dependent data
 */
struct iwl_fmac_trigger_cmd {
	__le32 len;
	__le32 id;
	__le32 vif_type;
#ifndef _MSC_VER
	u8 data[0];
#endif
} __packed;

#define MAX_TRIGGER_STR 64
/**
 * struct iwl_fmac_trigger_notif - notification with invoked trigger info
 * @id: &enum iwl_fmac_dbg_trigger
 * @data: string that describes what happened
 */
struct iwl_fmac_trigger_notif {
	__le32 id;
	u8 data[MAX_TRIGGER_STR];
} __packed;

enum iwl_fmac_mcc_source {
	IWL_FMAC_MCC_SOURCE_OLD_FW = 0,
	IWL_FMAC_MCC_SOURCE_ME = 1,
	IWL_FMAC_MCC_SOURCE_BIOS = 2,
	IWL_FMAC_MCC_SOURCE_3G_LTE_HOST = 3,
	IWL_FMAC_MCC_SOURCE_3G_LTE_DEVICE = 4,
	IWL_FMAC_MCC_SOURCE_WIFI = 5,
	IWL_FMAC_MCC_SOURCE_RESERVED = 6,
	IWL_FMAC_MCC_SOURCE_DEFAULT = 7,
	IWL_FMAC_MCC_SOURCE_UNINITIALIZED = 8,
	IWL_FMAC_MCC_SOURCE_MCC_API = 9,
	IWL_FMAC_MCC_SOURCE_GET_CURRENT = 0x10,
	IWL_FMAC_MCC_SOURCE_GETTING_MCC_TEST_MODE = 0x11,
};

/**
 * struct iwl_fmac_reg_cmd - send regulatory data to FW
 * @mcc: country code or "ZZ" for default
 * @source_id: &enum iwl_fmac_mcc_source
 * @reserved: reserved
 */
struct iwl_fmac_reg_cmd {
	__le16 mcc;
	u8 source_id;
	u8 reserved;
} __packed;

/**
 * struct iwl_fmac_reg_resp - response to %FMAC_REG_CFG, %FMAC_REG_UPDATE notif
 * @mcc: the current MCC
 * @source_id: the MCC source, see &enum iwl_fmac_mcc_source
 * @n_channels: number of channels in @channels
 * @channels: channel control data map, 32bits for each channel. Only the first
 *	16bits are used.
 * @reserved: reserved
 *
 * Contains the new channel control profile map and the current MCC (mobile
 * country code). The new MCC may be different than what was requested in
 * FMAC_REG_CFG, if this is a cmd response.
 */
struct iwl_fmac_reg_resp {
	__le16 mcc;
	u8 source_id;
	u8 reserved[1];
	__le32 n_channels;
#ifndef _MSC_VER
	__le32 channels[0];
#endif
} __packed;

/**
 * struct iwl_fw_dbg_trigger_missed_bcon - configures trigger for missed
 *	beacons
 * @stop_consec_missed_bcon: stop recording if threshold is crossed.
 *	stop recording means to collect the current dump data.
 * @stop_consec_missed_bcon_since_rx: stop recording if threshold is crossed.
 * @reserved: reserved
 */
struct iwl_fmac_dbg_trigger_missed_bcon {
	__le32 stop_consec_missed_bcon;
	__le32 stop_consec_missed_bcon_since_rx;
	u8 reserved[24];
} __packed;

/**
 * enum iwl_fmac_nan_dual_band_conf - NAN dual band configuration
 *
 * Defines the NAN dual band mode of operation
 * @IWL_FMAC_NAN_BAND_DEFAULT: device default mode
 * @IWL_FMAC_NAN_BAND_SINGLE: 2.4GHz only mode
 * @IWL_FMAC_NAN_BAND_DUAL: 2.4GHz and 5GHz mode
  */
enum iwl_fmac_nan_dual_band_conf {
	IWL_FMAC_NAN_BAND_DEFAULT,
	IWL_FMAC_NAN_BAND_SINGLE,
	IWL_FMAC_NAN_BAND_DUAL,
};

/* TODO: read it from tlv */
#define NAN_MAX_SUPPORTED_DE_ENTRIES 10

/**
 * struct iwl_fmac_cfg_nan_cmd - configure NAN parameters
 *
 * @vif_addr: NAN vif address
 * @master_pref: master preference value
 * @dual: dual band operation mode
 */
struct iwl_fmac_cfg_nan_cmd {
	u8 vif_addr[6];
	u8 master_pref;
	u8 dual;
} __packed;

/**
 * enum iwl_fmac_nan_func_type - NAN function type
 *
 * @IWL_FMAC_NAN_DE_FUNC_PUBLISH: Publish function.
 * @IWL_FMAC_NAN_DE_FUNC_SUBSCRIBE: Subscribe function.
 * @IWL_FMAC_NAN_DE_FUNC_FOLLOW_UP: Follow up function.
 * @IWL_FMAC_NAN_DE_FUNC_NOT_VALID: Invalid.
 */
enum iwl_fmac_nan_func_type {
	IWL_FMAC_NAN_DE_FUNC_PUBLISH = 0,
	IWL_FMAC_NAN_DE_FUNC_SUBSCRIBE = 1,
	IWL_FMAC_NAN_DE_FUNC_FOLLOW_UP = 2,

	/* keep last */
	IWL_FMAC_NAN_DE_FUNC_NOT_VALID,
};

/**
 * enum iwl_fmac_nan_func_flags- NAN DE function flags
 *
 * @IWL_FMAC_NAN_DE_FUNC_FLAG_UNSOLICITED_OR_ACTIVE: unsolicited publish or
 *     active subscribe
 * @IWL_FMAC_NAN_DE_FUNC_FLAG_UNICAST: unicast
 * @IWL_FMAC_NAN_DE_FUNC_FLAG_CLOSE_RANGE: function limited for devices in close
 *     range.
 */
enum iwl_fmac_nan_func_flags {
	IWL_FMAC_NAN_DE_FUNC_FLAG_UNSOLICITED_OR_ACTIVE = BIT(0),
	IWL_FMAC_NAN_DE_FUNC_FLAG_UNICAST = BIT(1),
	IWL_FMAC_NAN_DE_FUNC_FLAG_CLOSE_RANGE = BIT(2),
};

/**
 * struct iwl_fmac_add_set_nan_func_cmd - add/set NAN function command
 *
 * @instance_id: instance id of the DE function
 * @type: &enum iwl_fmac_nan_func_type
 * @service_id: service id
 * @flags: a combination of &enum iwl_fmac_nan_func_flags
 * @flw_up_id: Follow up id. Valid only if function type is follow up.
 * @flw_up_req_id: Follow up requestor id. Valid only if function type is
 *	follow up.
 * @ttl: ttl in DW's or 0 for infinite
 * @serv_info_len: length of service specific information
 * @srf_len: length of the srf
 * @rx_filter_len: length of rx filter
 * @tx_filter_len: length of tx filter
 * @data_len: length of data
 * @flw_up_dest: Follow up destination address. Valid only if function type
 *	is follow up.
 * @data: dw aligned fields - service_info, srf, rxFilter, txFilter
 * @reserved: reserved
 */
struct iwl_fmac_add_set_nan_func_cmd {
	u8 instance_id;
	u8 type;
	u8 service_id[6];
	__le16 flags;
	u8 flw_up_id;
	u8 flw_up_req_id;
	__le32 ttl;
	u8 serv_info_len;
	u8 srf_len;
	u8 rx_filter_len;
	u8 tx_filter_len;
	u8 data_len;
	u8 flw_up_dest[ETH_ALEN];
	u8 reserved;
#ifndef _MSC_VER
	u8 data[0];
#endif
} __packed;

/**
 * struct iwl_fmac_del_nan_func_cmd - remove NAN function command
 *
 * @instance_id: instance id of the DE function to remove
 * @reserved: reserved
 */
struct iwl_fmac_del_nan_func_cmd {
	u8 instance_id;
	u8 reserved[3];
} __packed;

/**
 * struct iwl_fmac_nan_add_func_res - Add NAN function response
 *
 * @instance_id: assigned instance_id (if added)
 * @status: status of the command (NAN_DE_FUNC_STATUS_*)
 * @free_entries: how many more functions can be defined
 * @free_buffers: how many more attribute buffers can be configured
 */
struct iwl_fmac_nan_add_func_res {
	u8 instance_id;
	u8 status;
	u8 free_entries;
	u8 free_buffers;
} __packed;

/**
 * enum iwl_fmac_nan_de_term_reason - NAN function termination reason
 *
 * @IWL_FMAC_NAN_DE_TERM_FAILURE: error case.
 * @IWL_FMAC_NAN_DE_TERM_TTL_REACHED: requested TTL reached.
 * @IWL_FMAC_NAN_DE_TERM_DRIVER_REQ: requested by the driver
 */
enum iwl_fmac_nan_de_term_reason {
	IWL_FMAC_NAN_DE_TERM_FAILURE = 0,
	IWL_FMAC_NAN_DE_TERM_TTL_REACHED,
	IWL_FMAC_NAN_DE_TERM_DRIVER_REQ,
};

/**
 * struct iwl_fmac_nan_de_term_event - NAN function terminated event
 *
 * @type: type of terminated function (&enum iwl_fw_nan_func_type)
 * @instance_id: instance id
 * @reason: termination reason (&enum iwl_fmac_nan_de_term_reason)
 * @reserved1: reserved
 */
struct iwl_fmac_nan_de_term_event {
	u8 type;
	u8 instance_id;
	u8 reason;
	u8 reserved1;
} __packed;

/**
 * struct iwl_fmac_nan_disc_evt_notification - NAN discovery event
 *
 * @peer_mac_addr: peer address
 * @type: Type of matching function
 * @instance_id: local matching instance id
 * @peer_instance: peer matching instance id
 * @service_info_len: service specific information length
 * @attrs_len: additional peer attributes length
 * @buf: service specific information followed by attributes
 * @reserved1: reserved
 */
struct iwl_fmac_nan_disc_evt_notification {
	u8 peer_mac_addr[6];
	__le16 reserved1;
	u8 type;
	u8 instance_id;
	u8 peer_instance;
	u8 service_info_len;
	__le32 attrs_len;
#ifndef _MSC_VER
	u8 buf[0];
#endif
} __packed;

/**
 * struct iwl_fmac_rx_eapol_notif - EAPOL RX notification
 * @addr: frame source address
 * @len: frame length in bytes
 * @data: frame body
 *
 * This message is used to pass 802.1X EAPOL frames to the host.
 * The host is expected to send the response EAPOL via the TX path,
 * and if 802.1X authentication fails the host should disconnect with
 * disconnect reason 23 (IEEE 802.1x authentication failed).
 */
struct iwl_fmac_rx_eapol_notif {
	u8 addr[ETH_ALEN];
	__le16 len;
#ifndef _MSC_VER
	u8 data[0];
#endif
} __packed;

#ifdef CPTCFG_IWLFMAC_9000_SUPPORT
/**
 * struct iwl_fmac_send_frame_notif - Ask the host to send a frame
 * @vif_id: the interface identifier
 * @reserved: reserved
 * @len: frame length in bytes
 * @dst_addr: the destination MAC address
 * @src_addr: the source MAC address
 * @proto: the protocol (only EAP for now)
 * @data: frame body starts with a valid 802.11 MAC header
 *
 * This message is used to instruct the host to send a frame. This is
 * used to use the host's PN pool and avoid racing between the host and
 * FMAC.
 */
struct iwl_fmac_send_frame_notif {
	u8 vif_id;
	u8 reserved;
	__le16 len;
	u8 dst_addr[ETH_ALEN];
	u8 src_addr[ETH_ALEN];
	__be16 proto;
#ifndef _MSC_VER
	u8 data[0];
#endif
} __packed;
#endif

#define KEY_MAX_LEN	48

/**
 * enum iwl_fmac_key_type - available key types for FMAC_SET_KEY command
 * @IWL_FMAC_KEY_TYPE_PMK: PMK from 802.1X authentication. The PMK length
 *	is 32 bytes.
 * @IWL_FMAC_KEY_TYPE_PMK_EAP_LEAP: PMK from 802.1X authentication when
 *	EAP-LEAP is used. The PMK length is 16.
 * @IWL_FMAC_KEY_TYPE_PMK_SUITE_B_192: PMK from 802.1X authentication when
 *	suite_b_192 is used. The PMK length is 48 bytes.
 */
enum iwl_fmac_key_type {
	IWL_FMAC_KEY_TYPE_PMK,
	IWL_FMAC_KEY_TYPE_PMK_EAP_LEAP,
	IWL_FMAC_KEY_TYPE_PMK_SUITE_B_192,
};

/**
 * struct iwl_fmac_mlme_set_key_cmd - set key command
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp
 * @key_type: the key type as specified in &iwl_fmac_key_type. This field
 *	defines the used length of the key buffer.
 * @aa: authenticator address
 * @key: key data. The length of the data is determined by the type
 *	of the key as specified in &key_type. See also &enum iwl_fmac_key_type.
 */
struct iwl_fmac_mlme_set_key_cmd {
	u8 vif_id;
	u8 key_type;
	u8 aa[ETH_ALEN];
	u8 key[KEY_MAX_LEN];
};

/**
 * struct iwl_fmac_start_ftm_responder_cmd - start FTM responder
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp
 * @reserved: for future use and alignment.
 * @lci_len: The length of the 1st (LCI) part in the &lci_civic buffer
 * @civic_len: The length of the 2nd (CIVIC) part in the &lci_civic buffer
 * @lci_civic: The LCI/CIVIC buffer. LCI data (if exists) followed by
 *	CIVIC data (if exists).
 */
struct iwl_fmac_start_ftm_responder_cmd {
	u8 vif_id;
	u8 reserved[3];
	__le32 lci_len;
	__le32 civic_len;
#ifndef _MSC_VER
	u8 lci_civic[0];
#endif
};

/**
 * enum iwl_fmac_ftm_target_flags - FTM target flags
 * @IWL_FMAC_FTM_TARGET_ONE_SIDED: if set, indicates that a one-sided
 *	(rather than two-sided) measurement shall be done
 * @IWL_FMAC_FTM_TARGET_ASAP: Indicates that the measurement should be
 *	performed in ASAP mode. Ignored if one-sided.
 * @IWL_FMAC_FTM_TARGET_LCI: Whether to query for LCI in the request.
 *	Ignored if one-sided.
 * @IWL_FMAC_FTM_TARGET_CIVIC: Whether to query for CIVIC in the request.
 *	Ignored if one-sided.
 */
enum iwl_fmac_ftm_target_flags {
	IWL_FMAC_FTM_TARGET_ONE_SIDED	= BIT(0),
	IWL_FMAC_FTM_TARGET_ASAP	= BIT(1),
	IWL_FMAC_FTM_TARGET_LCI		= BIT(2),
	IWL_FMAC_FTM_TARGET_CIVIC	= BIT(3),
};

/**
 * struct iwl_fmac_ftm_target - data for an FTM target (FTM responder)
 *
 * @bssid: target's BSSID.
 * @flags: flags, see &enum iwl_fmac_ftm_target_flags
 * @bw: target's bandwidth, one of @enum iwl_fmac_chan_width.
 * @freq: target's control frequency
 * @center_freq1: target's center frequency, 1st segment
 * @center_freq2: target's center frequency, 2nd segment (if relevant)
 * @burst_period: Measurement periodicity in units of 100ms.
 *	Ignored if num_of_bursts_exp is 0.
 * @num_of_bursts_exp: exponent of 2 of the number of measurement iterations.
 * @samples_per_burst: Number of measurement frames requested per burst.
 * @retries: Number of retries per sample.
 * @burst_duration: duration of an rtt burst. Valid values are 2-11 and 15,
 *	as defined in ieee80211-2016 spec, table 9-257
 * @enable_dyn_ack: Enable Dynamic ACK BW
 * @algo_type: &enum iwl_tof_algo_type
 * @reserved: reserved for future use and alignment
 */
struct iwl_fmac_ftm_target {
	u8 bssid[ETH_ALEN];
	u8 flags;
	u8 bw;
	__le16 freq;
	__le16 center_freq1;
	__le16 center_freq2;
	__le16 burst_period;
	u8 num_of_bursts_exp;
	u8 samples_per_burst;
	u8 retries;
	u8 burst_duration;
	u8 enable_dyn_ack;
	u8 algo_type;
	__le16 reserved;
} __packed;

#define IWL_FMAC_FTM_MAX_APS 5

/**
 * struct iwl_fmac_ftm_request_cmd - FTM request parameters
 *
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp
 * @mac_addr: MAC address used with randomization. The address cannot be a
 *	multicast one, i.e., bit 0 of byte 0 should not be set.
 * @mac_addr_mask: MAC address mask used with randomization. Bits that are 0 in
 *	the mask should be randomized. Bits that are 1 in the mask should be
 *	taken as is from mac_addr. The mask should not allow the generation of a
 *	multicast address, i.e., bit 0 of byte 0 must be set.
 * @timeout: Timespan within which measurement should complete. Given in tenths
 *	of a second.
 * @report_tsf: if true, report the TSF of the AP to which the vif is
 *	associated. Not relevant if the vif is not associated.
 * @num_of_targets: Number of targets (with which to perform a measurement)
 *	contained in this request.
 * @targets: the targets to perform a measurement with
 */
struct iwl_fmac_ftm_request_cmd {
	u8 vif_id;
	u8 mac_addr[ETH_ALEN];
	u8 mac_addr_mask[ETH_ALEN];
	u8 timeout;
	u8 report_tsf;
	u8 num_of_targets;
	struct iwl_fmac_ftm_target targets[IWL_FMAC_FTM_MAX_APS];
} __packed;

/**
 * enum iwl_fmac_ftm_request_status - status of an FTM measurement request
 *
 * @FMAC_FTM_REQ_OK: Successful request.
 * @FMAC_FTM_REQ_INVALID: Invalid request
 * @FMAC_FTM_REQ_BUSY: Device is in the middle of another FTM measurement
 */
enum iwl_fmac_ftm_request_status {
	FMAC_FTM_REQ_OK,
	FMAC_FTM_REQ_INVALID,
	FMAC_FTM_REQ_BUSY,
};

/**
 * struct iwl_fmac_ftm_request_resp - FTM request response
 *
 * @status: status of the request, one of @enum iwl_fmac_ftm_request_status
 * @request_id: in case the status is &FMAC_FTM_REQ_OK: the id assigned to the
 *	new requested measurement, 0 is an invalid value.
 *	in case the status is &FMAC_FTM_REQ_BUSY: the id of the measurement that
 *	is currently being executed.
 *	in case of &FMAC_FTM_REQ_INVALID, this field is ignored
 * @reserved: reserved (for alignment)
 */
struct iwl_fmac_ftm_request_resp {
	u8 status;
	u8 request_id;
	__le16 reserved;
} __packed;

/**
 * enum iwl_fmac_ftm_target_status - status of an FTM measurement attempt
 *
 * @FMAC_FTM_RESP_SUCCESS: Successful measurement, given results are valid.
 * @FMAC_FTM_RESP_TARGET_INCAPAB: Target reported incapable
 * @FMAC_FTM_RESP_TARGET_BUSY: Target reported busy
 * @FMAC_FTM_RESP_NOT_MEASURED: Target not measured due to timeout expiration
 * @FMAC_FTM_RESP_TARGET_UNAVAILABLE: Target is unavailable.
 * @FMAC_FTM_RESP_FAIL: Failed for some other reason.
 */
enum iwl_fmac_ftm_target_status {
	FMAC_FTM_RESP_SUCCESS,
	FMAC_FTM_RESP_TARGET_INCAPAB,
	FMAC_FTM_RESP_TARGET_BUSY,
	FMAC_FTM_RESP_NOT_MEASURED,
	FMAC_FTM_RESP_TARGET_UNAVAILABLE,
	FMAC_FTM_RESP_FAIL,
};

#define IWL_FMAC_MAX_LCI_CIVIC_LEN 512

/**
 * struct iwl_fmac_ftm_result_entry - FTM result
 *
 * @bssid: The measured target's bssid
 * @status: Measurement status, as specified in @enum iwl_fmac_ftm_target_status
 * @complete: Whether this measurement is the last one expected for this target.
 *	This implies that resources associated with this target may be released.
 * @rtt: The Round Trip Time that took for the last measurement for current
 *	target, in psec. Since a measurement can have an error tolerance, this
 *	value can be negative.
 * @rtt_variance: The variance of the RTT values measured for current target, in
 *	psec^2.
 * @rtt_spread: The difference between max and min RTT values measured for
 *	the current target in the current session, in psec.
 * @range: Distance from target, in cm. This might be a negative value, as this
 *	value exists within an error range.
 * @range_variance: variance of the distance, in cm^2.
 * @range_spread: The difference between max and min distance values measured
 *	for the current target, in cm
 * @host_time: The GP2 Clock [usec] in which:
 *	 - in case of error - error was detected
 *	 - in case of success - successful measurement started
 * @tsf: Same as %host_time, but in TSF of the AP the vif is associated to.
 *	This value is not an estimation. If field &report_tsf in the request is
 *	not set, or the vif isn't associated, this field is ignored.
 * @rssi: Measured RSSI, given in dBm. Valid values range: -128-0.
 * @rssi_spread: The difference between max and min measured RSSI values
 * @bw: The actual bandwidth of the measurement, one of
 *	@enum iwl_fmac_chan_width
 * @reserved: reserved (for alignment)
 * @lci_len: length of the LCI buffer, starting at the beginnig of &lci_civic.
 * @civic_len: length of the CIVIC buffer, starting at &lci_civic + &lci_len.
 * @lci_civic: the LCI and CIVIC info buffer.
 */
struct iwl_fmac_ftm_result_entry {
	u8 bssid[ETH_ALEN];
	u8 status;
	u8 complete;
	__le32 rtt;
	__le32 rtt_variance;
	__le32 rtt_spread;
	__le32 range;
	__le32 range_variance;
	__le32 range_spread;
	__le32 host_time;
	__le32 tsf;
	s8 rssi;
	u8 rssi_spread;
	u8 bw;
	u8 reserved;
	__le32 lci_len;
	__le32 civic_len;
	u8 lci_civic[IWL_FMAC_MAX_LCI_CIVIC_LEN];
} __packed;

/**
 * enum iwl_fmac_msrment_status - measurement response status values
 *
 * @FMAC_MSRMENT_STATUS_SUCCESS: Measurement performed. This does not mean
 *	every sub-measurement was successful, but only that as a whole, the
 *	operation succeeded. More detailed status should reside in the internal
 *	parts of the response, and according to the measurement type.
 * @FMAC_MSRMENT_STATUS_REFUSED: Device is refusing to perform the required
 *	measurement.
 * @FMAC_MSRMENT_STATUS_TIMEOUT: Measurement timeout has expired before request
 *	completion.
 * @FMAC_MSRMENT_STATUS_FAIL: Measurement failed.
 */
enum iwl_fmac_msrment_status {
	FMAC_MSRMENT_STATUS_SUCCESS,
	FMAC_MSRMENT_STATUS_REFUSED,
	FMAC_MSRMENT_STATUS_TIMEOUT,
	FMAC_MSRMENT_STATUS_FAIL,
};

/**
 * struct iwl_fmac_ftm_response_notify - FTM measurement response
 *
 * @request_id: the id of the request that this measurement corresponds to
 * @msrment_status: status of current measurement session, one of
 *	@enum iwl_fmac_msrment_status.
 * @num_of_aps: Number of APs measured
 * @reserved: reserved (for alignment)
 * @ap: the APs measured
 */
struct iwl_fmac_ftm_response_notify {
	u8 request_id;
	u8 msrment_status;
	u8 num_of_aps;
	u8 reserved;
	struct iwl_fmac_ftm_result_entry ap[IWL_FMAC_FTM_MAX_APS];
} __packed;

/**
 * struct iwl_fmac_ftm_abort_cmd - Abort an FTM measurement
 *
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp
 * @request_id: the id of the request to cancel
 * @reserved: reserved (for alignment)
 */
struct iwl_fmac_ftm_abort_cmd {
	u8 vif_id;
	u8 request_id;
	__le16 reserved;
} __packed;

/**
 * enum iwl_fmac_ftm_abort_response - response of an FTM abort command
 *
 * @FMAC_FTM_ABORT_OK: Abort sent successful
 * @FMAC_FTM_ABORT_WRONG_REQ_ID: Wrong request id in abort command
 * @FMAC_FTM_ABORT_NO_CUR_REQUEST: No current request exists, nothing to abort
 */
enum iwl_fmac_ftm_abort_response {
	FMAC_FTM_ABORT_OK,
	FMAC_FTM_ABORT_WRONG_REQ_ID,
	FMAC_FTM_ABORT_NO_CUR_REQUEST,
};

/*
 * struct iwl_fmac_neighbor_req_cmd - neighbor request command
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp
 * @lci: if set, the neighbor request will query for LCI information
 * @civic: if set, the neighbor request will query for CIVIC information
 * @ssid_len: the length of &ssid. Zero ssid length indicates the neighbor
 *	report request is for the current ESS.
 * @ssid: specifies the SSID for the neighbor report request.
 */
struct iwl_fmac_neighbor_req_cmd {
	u8 vif_id;
	u8 lci;
	u8 civic;
	u8 ssid_len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
} __packed;

/*
 * enum iwl_fmac_neighbor_rep_chan_width - channel width definitions
 *
 * The values in this enum correspond to the values defined in
 * IEEE802.11-2016, table 9-153.
 */
enum iwl_fmac_neighbor_rep_chan_width {
	IWL_FMAC_NEI_REP_CHAN_WIDTH_20,
	IWL_FMAC_NEI_REP_CHAN_WIDTH_40,
	IWL_FMAC_NEI_REP_CHAN_WIDTH_80,
	IWL_FMAC_NEI_REP_CHAN_WIDTH_160,
	IWL_FMAC_NEI_REP_CHAN_WIDTH_80P80,
};

/*
 * enum iwl_fmac_phy_type - phy type definitions
 *
 * The values in this enum corresponds to the values defined in
 * IEEE802.11-2016, Annex C.
 */
enum iwl_fmac_phy_type {
	IWL_FMAC_PHY_TYPE_UNSPECIFIED,
	IWL_FMAC_PHY_TYPE_DSSS = 2,
	IWL_FMAC_PHY_TYPE_OFDM = 4,
	IWL_FMAC_PHY_TYPE_HRDSSS = 5,
	IWL_FMAC_PHY_TYPE_ERP = 6,
	IWL_FMAC_PHY_TYPE_HT = 7,
	IWL_FMAC_PHY_TYPE_DMG = 8,
	IWL_FMAC_PHY_TYPE_VHT = 9,
	IWL_FMAC_PHY_TYPE_TVHT = 10,
};

/*
 * struct iwl_fmac_neighbor_rep - neighbor information for neighbor report
 * @bssid: the BSSID of the reported AP.
 * @op_class: the operating class of the reported AP.
 * @channel: the primary channel of the reported AP.
 * @bss_info: the BSSID information field as defined in
 *	IEEE802.11-2016, figure 9-296.
 * @phy_type: the PHY type of the reported AP, as specified in
 *	&enum iwl_fmac_phy_type.
 * @channel_width: the channel width, as specified in
 *	&enum iwl_fmac_neighbor_rep_chan_width.
 * @center_freq_idx_0: segment 0 center frequency in MHz. Only valid if
 *	&channel_width is not &IWL_FMAC_NEI_REP_CHAN_WIDTH_20.
 * @center_freq_idx_1: segment 1 center frequency in MHz. Only valid if
 *	&channel_width is &IWL_FMAC_NEI_REP_CHAN_WIDTH_80P80.
 * @reserved: for future use and alignment.
 * @lci_len: length of the LCI part of &lci_civic field in bytes.
 * @civic_len: length of the CIVIC part of &lci_civic field in bytes.
 * @lci_civic: &lci_len bytes of LCI information followed by &civic_len bytes of
 *	CIVIC information.
 */
struct iwl_fmac_neighbor_rep {
	u8 bssid[ETH_ALEN];
	u8 op_class;
	u8 channel;
	__le32 bss_info;
	u8 phy_type;
	u8 channel_width;
	u8 center_freq_idx_0;
	u8 center_freq_idx_1;
	__le16 reserved;
	u8 lci_len;
	u8 civic_len;
#ifndef _MSC_VER
	u8 lci_civic[0];
#endif
} __packed;

/*
 * struct iwl_fmac_neighbor_rep_notify - neighbor report notification
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp.
 * @neighbor_count: number of neighbors in &neighbors array.
 * @reserved: for alignment.
 * @neighbors: array of &iwl_fmac_neighbor_rep.
 */
struct iwl_fmac_neighbor_rep_notify {
	u8 vif_id;
	u8 neighbor_count;
	__le16 reserved;
#ifndef _MSC_VER
	struct iwl_fmac_neighbor_rep neighbors[0];
#endif
} __packed;

/**
 * struct iwl_fmac_join_ibss_cmd - Request to join IBSS network.
 * @vif_id: the interface identifier returned in &iwl_fmac_add_vif_resp.
 * @ssid_len: the length of &ssid.
 * @bssid: the BSSID in the IBSS to join to - relevant if &fixed_bssid in true.
 * @fixed_freq: do not search for IBSS on frequencies other than &chandef.
 * @fixed_bssid: do not search for IBSS with different BSSID other than &bssid.
 * @ssid: the SSID to be used for the IBSS.
 * @beacon_int: the beacon interval in TU.
 * @chandef: channel to be used if no other IBSS to join can be found.
 * @crypto: the security configuration as specified in &iwl_fmac_crypto.
 */
struct iwl_fmac_join_ibss_cmd {
	u8 vif_id;
	u8 ssid_len;
	u8 bssid[ETH_ALEN];
	u8 fixed_freq;
	u8 fixed_bssid;
	u8 ssid[32];
	__le16 beacon_int;
	struct iwl_fmac_chandef chandef;
	struct iwl_fmac_crypto crypto;
} __packed;

/**
 * enum iwl_fmac_join_ibss_resp_status - Status in &iwl_fmac_join_ibss_resp.
 */
enum iwl_fmac_join_ibss_resp_status {
	/**
	 * @IWL_FMAC_JOIN_IBSS_SUCCESS:
	 * Success to join IBSS.
	 */
	IWL_FMAC_JOIN_IBSS_SUCCESS = 0,

	/**
	 * @IWL_FMAC_JOIN_IBSS_INVALID_PARAMS:
	 * Fail to join IBSS due to invalid params.
	 */
	IWL_FMAC_JOIN_IBSS_INVALID_PARAMS,

	/**
	 * @IWL_FMAC_JOIN_IBSS_UNSUPPORTED_PARAMS:
	 * Fail to join IBSS due to unsupported params.
	 */
	IWL_FMAC_JOIN_IBSS_UNSUPPORTED_PARAMS
};

/**
 * struct iwl_fmac_start_ap_resp - Response to %FMAC_JOIN_IBSS command.
 * @status: status defined in &iwl_fmac_join_ibss_resp_status.
 */
struct iwl_fmac_join_ibss_resp {
	__le32 status;
} __packed;

/**
 * struct iwl_fmac_leave_ibss_cmd - Request to leave IBSS network.
 * @vif_id: the interface identifier used to join the IBSS.
 * @reserved: reserved for 4 byte alignment.
 */
struct iwl_fmac_leave_ibss_cmd {
	u8 vif_id;
	u8 reserved[3];
} __packed;

/**
 * struct iwl_fmac_merge_ibss - Notify host about starting a merge.
 * @vif_id: the interface identifier used to join/merge the IBSS.
 * @reserved: reserved for 4 byte alignment.
 */
struct iwl_fmac_merge_ibss {
	u8 vif_id;
	u8 reserved[3];
} __packed;

/**
 * struct iwl_fmac_ibss_joined - Notify that vif joined an IBSS.
 */
struct iwl_fmac_ibss_joined {
	/**
	 * @vif_id:
	 * the interface identifier returned in &iwl_fmac_add_vif_resp.
	 */
	u8 vif_id;

	/**
	 * @mcast_sta_id:
	 * the identifier allocation for the used for broadcast and
	 * multicast transmissions.
	 */
	u8 mcast_sta_id;

#ifdef CPTCFG_IWLFMAC_9000_SUPPORT
	/**
	 * @mcast_queue:
	 * queue allocation for broadcast and multicast transmissions.
	 * Only valid for 9000-series devices, otherwise reserved.
	 */
	u8 mcast_queue;

	/**
	 * @reserved:
	 * for alignment.
	 */
	u8 reserved;
#else
	/**
	 * @reserved: reserved
	 */
	__le16 reserved;
#endif
	/**
	 * @bssid:
	 * the BSSID of the joined IBSS.
	 */
	u8 bssid[ETH_ALEN];

	/**
	 * @chandef:
	 * chandef of the joined IBSS, see &iwl_fmac_chandef
	 */
	struct iwl_fmac_chandef chandef;

	/**
	* @keys:
	* wep keys (for a secured IBSS)
	*/
	struct iwl_fmac_keys keys;

	/**
	 * @probe_len:
	 * probe_res length.
	 */
	__le32 probe_len;

	/**
	 * @status:
	 * status defined in &iwl_fmac_ibss_joined_status.
	 */
	__le32 status;

	/**
	 * @probe_res:
	 * probe response frame used in the IBSS.
	 */
#ifndef _MSC_VER
	u8 probe_res[0];
#endif
} __packed;

/**
 * struct iwl_fmac_mic_failure - Notify fmac of TKIP MMIC failures.
 * @vif_id: the interface identifier connected to TKIP WLAN.
 * @pairwise: whether the mic failure was on unicaseet or multicast.
 * @reserved: reserved for 4 byte alignment.
 */
struct iwl_fmac_mic_failure {
	u8 vif_id;
	u8 pairwise;
	u8 reserved[2];
} __packed;

/**
 * enum iwl_fmac_sha_type - SHA function types
 * @IWL_FMAC_SHA_TYPE_SHA1: SHA1
 * @IWL_FMAC_SHA_TYPE_SHA256: SHA256
 * @IWL_FMAC_SHA_TYPE_SHA384: SHA384
 */
enum iwl_fmac_sha_type {
	IWL_FMAC_SHA_TYPE_SHA1,
	IWL_FMAC_SHA_TYPE_SHA256,
	IWL_FMAC_SHA_TYPE_SHA384,
};

#define SHA_MAX_MSG_LEN	128

/**
 * struct iwl_fmac_vector_sha - vector for FIPS SHA tests
 * @type: the SHA type to use. One of &enum iwl_fmac_sha_type.
 * @msg_len: the length of &msg in bytes.
 * @reserved: for alignment.
 * @msg: the message to generate the hash for.
 */
struct iwl_fmac_vector_sha {
	u8 type;
	u8 msg_len;
	__le16 reserved;
	u8 msg[SHA_MAX_MSG_LEN];
} __packed;

#define HMAC_KDF_MAX_KEY_LEN	192
#define HMAC_KDF_MAX_MSG_LEN	144

/**
 * struct iwl_fmac_vector_hmac_kdf - vector for FIPS HMAC/KDF tests
 * @type: the SHA type to use. One of &enum iwl_fmac_sha_type.
 * @res_len: requested result length in bytes.
 * @key_len: the length of &key in bytes.
 * @msg_len: the length of &msg in bytes.
 * @key: key for HMAC/KDF operations.
 * @msg: the message to generate the MAC for.
 */
struct iwl_fmac_vector_hmac_kdf {
	u8 type;
	u8 res_len;
	u8 key_len;
	u8 msg_len;
	u8 key[HMAC_KDF_MAX_KEY_LEN];
	u8 msg[HMAC_KDF_MAX_MSG_LEN];
} __packed;

/**
 * enum iwl_fmac_fips_test_type - FIPS test types
 * @IWL_FMAC_FIPS_TEST_SHA: test SHA functions.
 * @IWL_FMAC_FIPS_TEST_HMAC: test HMAC functions.
 * @IWL_FMAC_FIPS_TEST_KDF: test KDF functions.
 */
enum iwl_fmac_fips_test_type {
	IWL_FMAC_FIPS_TEST_SHA,
	IWL_FMAC_FIPS_TEST_HMAC,
	IWL_FMAC_FIPS_TEST_KDF,
};

union iwl_fmac_fips_test_vector {
	struct iwl_fmac_vector_sha sha_vector;
	struct iwl_fmac_vector_hmac_kdf hmac_kdf_vector;
};

#define MAX_FIPS_VECTOR_LEN	sizeof(union iwl_fmac_fips_test_vector)

/**
 * struct iwl_fmac_test_fips_cmd - FIPS test command
 * @type: test type. One of &enum iwl_fmac_fips_test_type.
 * @reserved: for alignment.
 * @vector: buffer with vector data. Union &iwl_fmac_fips_test_vector.
 */
struct iwl_fmac_test_fips_cmd {
	u8 type;
	u8 reserved[3];
	u8 vector[MAX_FIPS_VECTOR_LEN];
} __packed;

/**
 * enum iwl_fmac_fips_test_status - FIPS test result status
 * @IWL_FMAC_TEST_FIPS_STATUS_SUCCESS: The requested operation was completed
 *	successfully. The result buffer is valid.
 * @IWL_FMAC_TEST_FIPS_STATUS_FAIL: The requested operation failed.
 */
enum iwl_fmac_test_fips_status {
	IWL_FMAC_TEST_FIPS_STATUS_SUCCESS,
	IWL_FMAC_TEST_FIPS_STATUS_FAIL,
};

#define FIPS_MAX_RES_LEN		88
#define MAX_RES_LEN_HMAC_SHA1		20
#define MAX_RES_LEN_HMAC_SHA256		32
#define MAX_RES_LEN_HMAC_SHA384		48

/**
 * struct iwl_fmac_test_fips_resp - FIPS test response
 * @status: one of &enum iwl_fmac_fips_test_status.
 * @len: the length of the response in bytes.
 * @reserved: for alignment.
 * @buf: response buffer.
 *
 * Note that the response buffer has valid data only if &status is
 * &IWL_FMAC_TEST_FIPS_STATUS_SUCCESS. Otherwise it should be ignored.
 */
struct iwl_fmac_test_fips_resp {
	u8 status;
	u8 len;
	__le16 reserved;
	u8 buf[FIPS_MAX_RES_LEN];
} __packed;

/**
 * struct iwl_fmac_set_monitor_chan_cmd - Set the monifor channel
 * @vif_id: id of monitor vif to set
 * @reserved: reserved for dword alignment
 * @chandef: channel to set
 */
struct iwl_fmac_set_monitor_chan_cmd {
	u8 vif_id;
	u8 reserved[3];
	struct iwl_fmac_chandef chandef;
} __packed;

/**
 * enum iwl_fmac_nan_ndp_oper - NAN NDP operations
 *
 * @IWL_FMAC_NAN_NDP_OPER_REQ: NAN Data Path request
 * @IWL_FMAC_NAN_NDP_OPER_RES: NAN Data path response
 * @IWL_FMAC_NAN_NDP_OPER_TERM: NAN Data path termination request
 */
enum iwl_fmac_nan_ndp_oper {
	IWL_FMAC_NAN_NDP_OPER_REQ,
	IWL_FMAC_NAN_NDP_OPER_RES,
	IWL_FMAC_NAN_NDP_OPER_TERM,
};

#define IWL_FMAC_NAN_PMK_LEN     (32)
#define IWL_FMAC_NAN_PMKID_LEN   (16)

/**
 * enum iwl_fmac_nan_csid - NAN Cipher Suite Identifier (CSID)
 * @IWL_FMAC_NAN_CSID_NONE: No CSID
 * @IWL_FMAC_NAN_CSID_SK_128: shared symmetric key using CCMP 128
 * @IWL_FMAC_NAN_CSID_SK_256: shared symmetric key using GCMP 256
 */
enum iwl_fmac_nan_csid {
	IWL_FMAC_NAN_CSID_NONE = 0,
	IWL_FMAC_NAN_CSID_SK_128,
	IWL_FMAC_NAN_CSID_SK_256,
};

/**
 * enum iwl_fmac_nan_ndp_flags - NAN NDP flags
 *
 * @IWL_FMAC_NAN_NDP_FLAGS_PMK: PMK is set in &struct iwl_fmac_nan_ndp_cmd
 * @IWL_FMAC_NAN_NDP_FLAGS_PMKID: PMKID is set in &struct iwl_fmac_nan_ndp_cmd
 * @IWL_FMAC_NAN_NDP_FLAGS_QOS: QoS is set in &struct iwl_fmac_nan_ndp_cmd
 */
enum iwl_fmac_nan_ndp_flags {
	IWL_FMAC_NAN_NDP_FLAGS_PMK   = BIT(0),
	IWL_FMAC_NAN_NDP_FLAGS_PMKID = BIT(1),
	IWL_FMAC_NAN_NDP_FLAGS_QOS   = BIT(2),
};

/**
 * struct iwl_fmac_nan_ndp_cmd - NAN Data Path (NDP) parameters
 *
 * @vif_id: the virtual interface identifier as returned in
 *	&iwl_fmac_add_vif_resp.
 * @oper: The requested NDP operation. See &enum iwl_fmac_nan_ndp_oper.
 * @pub_inst_id: The identifier for the instance of the publisher function
 *     associated with the data path setup request. Valid only for NDP request
 *     (both when used as a command and when used as a notification).
 * @ndp_id: The NDP ID. Together with the NDP initiator NDI uniquely identifies
 *     the NDP.
 * @rejected: Valid only for an NDP response. True iff the corresponding
 *     response is rejected.
 * @reason_code: Valid only for NDP response when %rejected is True. Identifies
 *     the reason for the reject status.
 * @min_slots: Indicate the minimum number of further available NAN Slots needed
 *     per DW interval (512 TU). Set to 0 if no preference.
 * @csid: cipher suites ID used for the establishment of a secure NDP. Valid
 *     only for NDP request and NDP response in case the %csid is either SK-128
 *     or SK-256. See &enum iwl_fmac_nan_csid.
 * @max_latency: Indicate the maximum allowed NAN Slots between every two
 *     non-contiguous NDL CRBs. Set to 0xffff if no preference.
 * @peer_nmi: The peer's NAN Management Interface (NMI) address.
 * @init_ndi: The initiator's NAN Data Interface (NDI).
 * @resp_ndi: The responder's NAN Data Interface (NDI). Valid only for an NDP
 *     response.
 * @pmk: The PMK used for the establishment of a secure NDP. Valid only for NDP
 *     request and NDP response in case the %csid is either SK-128 or SK-256.
 *     If included, IWL_FMAC_NAN_NDP_FLAGS_PMK must be set in %flags.
 * @pmkid: The PMK Identifier. If included, IWL_FMAC_NAN_NDP_FLAGS_PMKID must be
 *     set in %flags.
 * @ssi_len: The length in octets of the service specific
 *     information. Valid only for an NDP request or an NDP response.
 * @ssi: Opaque Service specific information. Valid only for an NDP request or
 *     an NDP response.
 * @flags: see &enum iwl_fmac_nan_ndp_flags
 * @reserved: for alignment
 *
 * The structure can be used for both command to the FW, and also as an
 * asynchronous notification from the FW (when a peer NAN device starts a NDP
 * setup with us).
 */
struct iwl_fmac_nan_ndp_cmd {
	u8 vif_id;
	u8 oper;
	u8 pub_inst_id;
	u8 ndp_id;
	u8 rejected;
	u8 reason_code;
	u8 min_slots;
	u8 csid;
	__le16 max_latency;
	u8 peer_nmi[ETH_ALEN];
	u8 init_ndi[ETH_ALEN];
	u8 resp_ndi[ETH_ALEN];
	u8 pmk[IWL_FMAC_NAN_PMK_LEN];
	u8 pmkid[IWL_FMAC_NAN_PMKID_LEN];
	__le16 ssi_len;
	u8 flags;
	u8 reserved;
	u8 ssi[0];
} __packed;

/**
 * enum iwl_fmac_nan_ndp_status - NDP command status
 *
 * @IWL_FMAC_NAN_NDP_STATUS_SUCCESS: Success to process the command and initiate
 *     the requested flow.
 * @IWL_FMAC_NAN_NDP_STATUS_FAILURE: Failure to process the command or initiate
 *     the requested flow.
 */
enum iwl_fmac_nan_ndp_status {
	IWL_FMAC_NAN_NDP_STATUS_SUCCESS = 0,
	IWL_FMAC_NAN_NDP_STATUS_FAILURE,
};

/**
 * struct iwl_fmac_nan_ndp_resp - Response to an NDP command
 *
 * @status: see &enum iwl_fmac_nan_ndp_status
 * @ndp_id: in case of success to parse and process an NDP request flow, would
 *     contain the NDP identifier allocated by the FW.
 * @reserved: for alignment.
 */
struct iwl_fmac_nan_ndp_resp {
	u8 status;
	u8 ndp_id;
	__le16 reserved;
} __packed;

/**
 * struct iwl_fmac_nan_cluster_notif - Nan cluster change notification
 *
 * @id: the 2 lowest octets of the cluster ID
 * @reserved: for alignment.
 */
struct iwl_fmac_nan_cluster_notif {
	u8 id[2];
	__le16 reserved;
} __packed;

#endif /* __iwl_fw_api_fmac_h__ */
