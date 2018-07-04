/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2016 - 2017 Intel Deutschland GmbH
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
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2016 - 2017 Intel Deutschland GmbH
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
#include <linux/etherdevice.h>
#include <net/cfg80211.h>

#include "iwl-nvm-parse.h"

#include "fmac.h"

static const struct ieee80211_iface_limit iwl_fmac_limits_nan[] = {
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_NAN_DATA),
	},
#if CFG80211_VERSION >= KERNEL_VERSION(4,9,0)
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_NAN),
	},
#endif
};

static const struct ieee80211_iface_combination
iwl_fmac_iface_combinations_nan[] = {
	{
		/* TODO: num of different channel does not really matter for NAN
		 * Data interfaces, but cfg80211 does not allow 0 to be used
		 */
		.num_different_channels = 1,
		.max_interfaces = 2,
		.limits = iwl_fmac_limits_nan,
		.n_limits = ARRAY_SIZE(iwl_fmac_limits_nan),
	},
};

static struct wireless_dev *
iwl_fmac_cfg_add_virtual_intf(struct wiphy *wiphy, const char *name,
			      unsigned char name_assign_type,
			      enum nl80211_iftype type,
			      struct vif_params *params)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct net_device *dev;

	if (fmac->shutdown)
		return ERR_PTR(-ESHUTDOWN);

	if (type == NL80211_IFTYPE_P2P_DEVICE || ieee80211_viftype_nan(type))
		return iwl_fmac_create_non_netdev_iface(fmac, params, type);

	dev = iwl_fmac_create_netdev(fmac, name, name_assign_type,
				     type, params);
	if (IS_ERR(dev))
		return ERR_PTR(PTR_ERR(dev));

	return &vif_from_netdev(dev)->wdev;
}

static int iwl_fmac_cfg_del_virtual_intf(struct wiphy *wiphy,
					 struct wireless_dev *wdev)
{
	iwl_fmac_destroy_vif(vif_from_wdev(wdev));
	return 0;
}

static int iwl_fmac_cfg_change_virtual_intf(struct wiphy *wiphy,
					    struct net_device *dev,
					    enum nl80211_iftype type,
					    struct vif_params *params)
{
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);

	if (vif->wdev.iftype != type) {
		if (vif->id != FMAC_VIF_ID_INVALID)
			/* This is ok - cfg80211 will reopen this vif */
			return -EBUSY;
		vif->wdev.iftype = type;
	}

	return 0;
}

static int scan_cfg(struct cfg80211_scan_request *request,
		    struct iwl_fmac_scan_cmd *cmd, unsigned int cmd_size)
{
	int i;

	if (WARN_ON(request->n_channels > IWL_FMAC_MAX_CHANS))
		return -EINVAL;

	cmd->vif_id = vif_from_wdev(request->wdev)->id;

	/* TODO: convert request->rates to cmd->rates bitmap */

	cmd->n_ssids = request->n_ssids;
	for (i = 0; i < request->n_ssids; i++) {
		memcpy(cmd->ssids[i], request->ssids[i].ssid,
		       request->ssids[i].ssid_len);
		cmd->ssids_lengths[i] = request->ssids[i].ssid_len;
	}

	for (i = 0; i < request->n_channels; i++)
		cmd->freqs[i] = cpu_to_le16(request->channels[i]->center_freq);
	cmd->n_freqs = request->n_channels;

	cmd->random_mac = !!(request->flags & NL80211_SCAN_FLAG_RANDOM_ADDR);
	ether_addr_copy(cmd->bssid, request->bssid);
	cmd->ie_len = cpu_to_le16(request->ie_len);
	memcpy(cmd->ie, request->ie, request->ie_len);

	return 0;
}

static int iwl_fmac_cfg_scan(struct wiphy *wiphy,
			     struct cfg80211_scan_request *request)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	unsigned int cmd_size;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(FMAC_SCAN, FMAC_GROUP, 0),
		.dataflags = { IWL_HCMD_DFL_NOCOPY, },
	};
	void *ptr;
	int ret;

	mutex_lock(&fmac->mutex);

	if (WARN_ON(fmac->scan_request)) {
		ret = -EBUSY;
		goto out;
	}

	cmd_size = ALIGN(sizeof(struct iwl_fmac_scan_cmd) +
			 request->ie_len, 4);
	ptr = kmalloc(cmd_size, GFP_KERNEL);
	if (!ptr) {
		ret = -ENOMEM;
		goto out;
	}

	ret = scan_cfg(request, ptr, cmd_size);
	if (ret < 0)
		goto out;

	hcmd.data[0] = ptr;
	hcmd.len[0] = cmd_size;

	ret = iwl_fmac_send_cmd(fmac, &hcmd);
	kfree(ptr);

	if (ret == 0)
		fmac->scan_request = request;

 out:
	mutex_unlock(&fmac->mutex);

	return ret;
}

int iwl_fmac_abort_scan(struct iwl_fmac *fmac, struct iwl_fmac_vif *vif)
{
	struct iwl_notification_wait wait_scan_done;
	static const u16 scan_done_notif[] = {
		WIDE_ID(FMAC_GROUP, FMAC_SCAN_COMPLETE),
	};
	struct iwl_fmac_scan_abort_cmd cmd = {
		.vif_id = vif->id,
	};
	int ret;

	lockdep_assert_held(&fmac->mutex);

	if (iwl_fmac_is_radio_killed(fmac)) {
		struct cfg80211_scan_info info = {
			.aborted = true,
		};

		if (WARN_ON(!fmac->scan_request))
			return 0;

		IWL_DEBUG_SCAN(fmac, "Stop scan in RFKILL - skip command\n");
		cfg80211_scan_done(fmac->scan_request, &info);
		fmac->scan_request = NULL;
		return 0;
	}

	iwl_init_notification_wait(&fmac->notif_wait, &wait_scan_done,
				   scan_done_notif,
				   ARRAY_SIZE(scan_done_notif),
				   NULL, NULL);

	IWL_DEBUG_SCAN(fmac, "Preparing to stop scan\n");

	ret = iwl_fmac_send_cmd_pdu(fmac,
				    iwl_cmd_id(FMAC_SCAN_ABORT, FMAC_GROUP, 0),
				    0, sizeof(cmd), &cmd);
	if (ret) {
		IWL_DEBUG_SCAN(fmac, "couldn't stop scan\n");
		iwl_remove_notification(&fmac->notif_wait, &wait_scan_done);
		return ret;
	}

	ret = iwl_wait_notification(&fmac->notif_wait, &wait_scan_done,
				    HZ * CPTCFG_IWL_TIMEOUT_FACTOR);
	return ret;
}

#if CFG80211_VERSION >= KERNEL_VERSION(4,5,0)
static void iwl_fmac_cfg_abort_scan(struct wiphy *wiphy,
				    struct wireless_dev *wdev)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_wdev(wdev);

	mutex_lock(&fmac->mutex);
	iwl_fmac_abort_scan(fmac, vif);
	mutex_unlock(&fmac->mutex);
}
#endif

static u32 cipher_suites_to_fmac_ciphers(unsigned int n_suites, u32 *suites)
{
	int i;
	u32 suite;
	u32 ciphers = 0;

	for (i = 0; i < n_suites; i++) {
		suite = suites[i];

		if (suite == WLAN_CIPHER_SUITE_CCMP_256)
			ciphers |= IWL_FMAC_CIPHER_CCMP_256;
		if (suite == WLAN_CIPHER_SUITE_GCMP_256)
			ciphers |= IWL_FMAC_CIPHER_GCMP_256;
		if (suite == WLAN_CIPHER_SUITE_CCMP)
			ciphers |= IWL_FMAC_CIPHER_CCMP;
		if (suite == WLAN_CIPHER_SUITE_GCMP)
			ciphers |= IWL_FMAC_CIPHER_GCMP;
		if (suite == WLAN_CIPHER_SUITE_TKIP)
			ciphers |= IWL_FMAC_CIPHER_TKIP;
		if (suite == WLAN_CIPHER_SUITE_WEP104)
			ciphers |= IWL_FMAC_CIPHER_WEP104;
		if (suite == WLAN_CIPHER_SUITE_WEP40)
			ciphers |= IWL_FMAC_CIPHER_WEP40;
	}
	return ciphers ? : IWL_FMAC_CIPHER_NONE;
}

static int
cfg_crypto_to_iwlfmac_crypto(struct iwl_fmac *fmac,
			     struct cfg80211_crypto_settings *cfg_crypto,
			     struct iwl_fmac_crypto *iwl_crypto)
{
	u32 i, val;

	if (cfg_crypto->wep_keys) {
		for (i = 0; i < IWL_NUM_WEP_KEYS; i++) {
			if (cfg_crypto->wep_keys[i].cipher) {
				memcpy(iwl_crypto->u.wep.key[i],
				       cfg_crypto->wep_keys[i].key,
				       cfg_crypto->wep_keys[i].key_len);
				iwl_crypto->u.wep.key_len[i] =
					cfg_crypto->wep_keys[i].key_len;
			}
		}

		iwl_crypto->u.wep.def_key = cfg_crypto->wep_tx_key;
	} else {
		if (cfg_crypto->psk)
			memcpy(iwl_crypto->u.wpa.psk, cfg_crypto->psk,
			       WLAN_PMK_LEN);

		val = 0;
		if (cfg_crypto->wpa_versions & NL80211_WPA_VERSION_1)
			val |= IWL_FMAC_PROTO_WPA;
		if (cfg_crypto->wpa_versions & NL80211_WPA_VERSION_2)
			val |= IWL_FMAC_PROTO_RSN;
		iwl_crypto->u.wpa.proto = cpu_to_le32(val);
	}

	val = cipher_suites_to_fmac_ciphers(cfg80211_crypto_n_ciphers_group(cfg_crypto),
					    cfg_crypto->ciphers_group);
	iwl_crypto->cipher_group = cpu_to_le32(val);

	val = cipher_suites_to_fmac_ciphers(cfg_crypto->n_ciphers_pairwise,
					    cfg_crypto->ciphers_pairwise);
	iwl_crypto->ciphers_pairwise = cpu_to_le32(val);

	val = 0;
	for (i = 0; i < cfg_crypto->n_akm_suites; i++) {
		switch (cfg_crypto->akm_suites[i]) {
		case WLAN_AKM_SUITE_PSK:
			val |= IWL_FMAC_KEY_MGMT_PSK;
			break;
		case WLAN_AKM_SUITE_FT_PSK:
			val |= IWL_FMAC_KEY_MGMT_FT_PSK;
			break;
		case WLAN_AKM_SUITE_PSK_SHA256:
			val |= IWL_FMAC_KEY_MGMT_PSK_SHA256;
			break;
		case WLAN_AKM_SUITE_8021X:
			val |= IWL_FMAC_KEY_MGMT_IEEE8021X;
			break;
		case WLAN_AKM_SUITE_8021X_SHA256:
			val |= IWL_FMAC_KEY_MGMT_IEEE8021X_SHA256;
			break;
		case WLAN_AKM_SUITE_8021X_SUITE_B:
			val |= IWL_FMAC_KEY_MGMT_IEEE8021X_SUITE_B;
			break;
		case WLAN_AKM_SUITE_8021X_SUITE_B_192:
			val |= IWL_FMAC_KEY_MGMT_IEEE8021X_SUITE_B_192;
			break;
		case WLAN_AKM_SUITE_FT_8021X:
			val |= IWL_FMAC_KEY_MGMT_FT_IEEE8021X;
			break;
		default:
			IWL_ERR(fmac, "Unsupported akm_suite: 0x%x\n",
				cfg_crypto->akm_suites[i]);
			return -EINVAL;
		}
	}
	iwl_crypto->key_mgmt = cpu_to_le32(val);

	return 0;
}

static int iwl_fmac_cfg_connect(struct wiphy *wiphy, struct net_device *dev,
				struct cfg80211_connect_params *sme)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	struct iwl_fmac_connect_cmd cmd = {
		.vif_id = vif->id,
	};
	int ret;

	if (vif->mgd.connect_state != IWL_FMAC_CONNECT_IDLE)
		return -EBUSY;

	vif->mgd.connect_state = IWL_FMAC_CONNECT_CONNECTING;

	if (sme->channel) {
		cmd.flags |= cpu_to_le32(IWL_FMAC_FREQ_IN_USE);
		cmd.center_freq = cpu_to_le16(sme->channel->center_freq);
	}

	if (sme->channel_hint) {
		cmd.flags |= cpu_to_le32(IWL_FMAC_FREQ_IN_USE |
					 IWL_FMAC_FREQ_HINT);
		cmd.center_freq = cpu_to_le16(sme->channel_hint->center_freq);
	}

	if (sme->bssid)
		memcpy(cmd.bssid, sme->bssid, ETH_ALEN);

	cmd.ssid_len = sme->ssid_len;
	memcpy(cmd.ssid, sme->ssid, sme->ssid_len);

	ret = cfg_crypto_to_iwlfmac_crypto(fmac, &sme->crypto, &cmd.crypto);
	if (ret)
		return ret;

	if (sme->mfp && !iwl_fmac_has_new_tx_api(fmac))
		return -EOPNOTSUPP;

	if (sme->mfp == NL80211_MFP_REQUIRED)
		cmd.crypto.mfp = IWL_FMAC_MFP_REQUIRED;

	if (sme->mfp == NL80211_MFP_OPTIONAL)
		cmd.crypto.mfp = IWL_FMAC_MFP_OPTIONAL;

	mutex_lock(&fmac->mutex);
	ret = iwl_fmac_send_cmd_pdu(fmac,
				    iwl_cmd_id(FMAC_CONNECT, FMAC_GROUP, 0),
				    0, sizeof(cmd), &cmd);
	mutex_unlock(&fmac->mutex);

	return ret;
}

static int iwl_fmac_cfg_disconnect(struct wiphy *wiphy,
				   struct net_device *dev,
				   u16 reason_code)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	struct iwl_fmac_disconnect_cmd cmd = {
		.vif_id = vif->id,
		.reason = cpu_to_le16(reason_code),
	};
	u32 status = 0;
	int ret = 0;

	mutex_lock(&fmac->mutex);

	if (test_bit(IWL_STATUS_HW_RFKILL, &fmac->status)) {
		struct iwl_fmac_sta *sta;

		sta = rcu_dereference_protected(vif->mgd.ap_sta,
						lockdep_is_held(&fmac->mutex));
		iwl_fmac_disconnected(fmac, sta, cmd.reason, 0);
	} else {
		/*
		 * The disconnect flow would un-map the queues allocated to the
		 * AP station, so transmissions would no longer be allowed.
		 */
		netif_carrier_off(dev);
		synchronize_net();

		ret = iwl_fmac_send_cmd_pdu_status(fmac,
						   iwl_cmd_id(FMAC_DISCONNECT,
							      FMAC_GROUP, 0),
						   sizeof(cmd), &cmd, &status);
	}

	/*
	 * if we got a disconnect notification (we will if the status is
	 * successful), we'll have gotten it before the command response,
	 * so it's already queued. Force processing it so we can turn
	 * around and connect again without having to rely on scheduling
	 * decisions letting the worker run first.
	 */
	iwl_fmac_process_async_handlers(fmac);

	mutex_unlock(&fmac->mutex);

	/* The firmware will reply with a disconnection notification when
	 * the disconnection will be complete. That one will call
	 * cfg80211_disconnected().
	 */

	if (ret)
		return ret;
	if (status)
		return -ENOTCONN;

	return 0;
}

static int iwl_fmac_fill_rate_info(int rate, struct rate_info *rinfo)
{
	if (rate == 0)
		return -EINVAL;

	memset(rinfo, 0, sizeof(*rinfo));

	if (rate & RATE_MCS_HT_MSK) {
		WARN_ON(rate & RATE_MCS_VHT_MSK);
		rinfo->flags |= RATE_INFO_FLAGS_MCS;
		rinfo->mcs = rate & RATE_HT_MCS_INDEX_MSK;
	} else if (rate & RATE_MCS_VHT_MSK) {
		WARN_ON(rate & RATE_MCS_CCK_MSK);
		rinfo->flags |= RATE_INFO_FLAGS_VHT_MCS;
		rinfo->mcs = rate & RATE_VHT_MCS_RATE_CODE_MSK;
		rinfo->nss = ((rate & RATE_VHT_MCS_NSS_MSK) >>
			      RATE_VHT_MCS_NSS_POS) + 1;
	} else if (rate & RATE_MCS_HE_MSK) {
		rinfo->flags |= RATE_INFO_FLAGS_HE_MCS;
		rinfo->mcs = rate & RATE_VHT_MCS_RATE_CODE_MSK;
		rinfo->nss = ((rate & RATE_VHT_MCS_NSS_MSK) >>
			      RATE_VHT_MCS_NSS_POS) + 1;
		rinfo->he_dcm = !!(rate & RATE_HE_DUAL_CARRIER_MODE_MSK);

		switch ((rate & RATE_MCS_HE_GI_LTF_MSK) >>
			RATE_MCS_HE_GI_LTF_POS) {
		case 0:
		case 1:
			rinfo->he_gi = NL80211_RATE_INFO_HE_GI_0_8;
			break;
		case 2:
			rinfo->he_gi = NL80211_RATE_INFO_HE_GI_1_6;
			break;
		case 3:
			rinfo->he_gi = NL80211_RATE_INFO_HE_GI_3_2;
			break;
		}
	} else if (rate & RATE_MCS_CCK_MSK) {
		rinfo->legacy = (rate & RATE_LEGACY_RATE_MSK);
	} else {
		switch (rate & RATE_LEGACY_RATE_MSK) {
		case 0xD:
			rinfo->legacy = 60;
			break;
		case 0xF:
			rinfo->legacy = 90;
			break;
		case 0x5:
			rinfo->legacy = 120;
			break;
		case 0x7:
			rinfo->legacy = 180;
			break;
		case 0x9:
			rinfo->legacy = 240;
			break;
		case 0xB:
			rinfo->legacy = 360;
			break;
		case 0x1:
			rinfo->legacy = 480;
			break;
		case 0x3:
			rinfo->legacy = 540;
			break;
		default:
			WARN_ON(1);
			return -EINVAL;
		}
	}

	switch (rate & RATE_MCS_CHAN_WIDTH_MSK) {
	case RATE_MCS_CHAN_WIDTH_20:
		rinfo->bw = RATE_INFO_BW_20;
		if (rate & RATE_MCS_HE_MSK)
			rinfo->he_ru_alloc = NL80211_RATE_INFO_HE_RU_ALLOC_242;
		break;
	case RATE_MCS_CHAN_WIDTH_40:
		rinfo->bw = RATE_INFO_BW_40;
		if (rate & RATE_MCS_HE_MSK)
			rinfo->he_ru_alloc = NL80211_RATE_INFO_HE_RU_ALLOC_484;
		break;
	case RATE_MCS_CHAN_WIDTH_80:
		rinfo->bw = RATE_INFO_BW_80;
		if (rate & RATE_MCS_HE_MSK)
			rinfo->he_ru_alloc = NL80211_RATE_INFO_HE_RU_ALLOC_996;
		break;
	case RATE_MCS_CHAN_WIDTH_160:
		rinfo->bw = RATE_INFO_BW_160;
		if (rate & RATE_MCS_HE_MSK)
			rinfo->he_ru_alloc =
				NL80211_RATE_INFO_HE_RU_ALLOC_2x996;
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	if ((rate & RATE_MCS_HE_TYPE_MSK) == RATE_MCS_HE_TYPE_MU)
		rinfo->bw = RATE_INFO_BW_HE_RU;

	if (rate & RATE_MCS_SGI_MSK)
		rinfo->flags |= RATE_INFO_FLAGS_SHORT_GI;

	return 0;
}

static void iwl_fmac_set_sta_rx_sinfo(const struct iwl_fmac_sta *sta,
				      struct station_info *sinfo)
{
	struct iwl_fmac_rx_stats *last_rx_stats = NULL;
	int cpu;

	sinfo->filled |= BIT(NL80211_STA_INFO_RX_PACKETS);
	sinfo->rx_packets = 0;

	for_each_possible_cpu(cpu) {
		struct iwl_fmac_rx_stats *cpu_stats =
			per_cpu_ptr(sta->info.pcpu_rx_stats, cpu);

		if (!last_rx_stats ||
		    time_after(cpu_stats->last_rx, last_rx_stats->last_rx))
			last_rx_stats = cpu_stats;

		sinfo->rx_packets += cpu_stats->packets;
	}

	if (iwl_fmac_fill_rate_info(last_rx_stats->last_rate,
				    &sinfo->rxrate) == 0)
		sinfo->filled |= BIT(NL80211_STA_INFO_RX_BITRATE);

	sinfo->filled |= BIT(NL80211_STA_INFO_SIGNAL);
	sinfo->signal = MBM_TO_DBM(last_rx_stats->signal);
}

static void iwl_fmac_set_sta_tx_sinfo(const struct iwl_fmac_sta *sta,
				      struct station_info *sinfo)
{
	if (iwl_fmac_fill_rate_info(sta->info.tx_stats.last_rate,
				    &sinfo->txrate) == 0)
		sinfo->filled |= BIT(NL80211_STA_INFO_TX_BITRATE);

	sinfo->filled |= BIT(NL80211_STA_INFO_TX_BYTES64);
	sinfo->tx_bytes = sta->info.tx_stats.bytes;

	sinfo->filled |= BIT(NL80211_STA_INFO_TX_PACKETS);
	sinfo->tx_packets = sta->info.tx_stats.packets;

	sinfo->filled |= BIT(NL80211_STA_INFO_TX_RETRIES);
	sinfo->tx_retries = sta->info.tx_stats.retries;

	sinfo->filled |= BIT(NL80211_STA_INFO_TX_FAILED);
	sinfo->tx_failed = sta->info.tx_stats.failed;
}

static void iwl_fmac_set_sta_info(const struct iwl_fmac_sta *sta,
				  struct station_info *sinfo)
{
	struct iwl_fmac *fmac = sta->vif->fmac;

	lockdep_assert_held(&fmac->mutex);

	sinfo->filled = 0;

	sinfo->generation = fmac->sta_generation;

	sinfo->filled |= BIT(NL80211_STA_INFO_CONNECTED_TIME);
	sinfo->connected_time = ktime_get_seconds() - sta->info.connect_time;

	sinfo->sta_flags.mask = 0;
	sinfo->filled |= BIT(NL80211_STA_INFO_STA_FLAGS);

	if (sta->authorized)
		sinfo->sta_flags.set |= BIT(NL80211_STA_FLAG_AUTHORIZED);
	sinfo->sta_flags.mask |= BIT(NL80211_STA_FLAG_AUTHORIZED);

	iwl_fmac_set_sta_tx_sinfo(sta, sinfo);

	iwl_fmac_set_sta_rx_sinfo(sta, sinfo);
}

static int iwl_fmac_get_station(struct wiphy *wiphy, struct net_device *dev,
				const u8 *mac, struct station_info *sinfo)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_sta *sta;
	int ret = -ENOENT;

	mutex_lock(&fmac->mutex);
	sta = iwl_get_sta(fmac, mac);
	if (sta) {
		ret = 0;
		iwl_fmac_set_sta_info(sta, sinfo);
	}
	mutex_unlock(&fmac->mutex);

	return ret;
}

static int iwl_fmac_dump_station(struct wiphy *wiphy, struct net_device *dev,
				 int idx, u8 *mac, struct station_info *sinfo)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_sta *sta;
	int tmp;

	mutex_lock(&fmac->mutex);
	for_each_valid_sta(fmac, sta, tmp) {
		if (idx > 0) {
			idx--;
			continue;
		}

		ether_addr_copy(mac, sta->addr);
		iwl_fmac_set_sta_info(sta, sinfo);
		mutex_unlock(&fmac->mutex);
		return 0;
	}
	mutex_unlock(&fmac->mutex);

	return -ENOENT;
}

static int iwl_fmac_set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);

	if (changed & WIPHY_PARAM_FRAG_THRESHOLD)
		fmac->rts_threshold = wiphy->rts_threshold;

	return 0;
}

u8 cfg_width_to_iwl_width(enum nl80211_chan_width cfg_width)
{
	switch (cfg_width) {
	case NL80211_CHAN_WIDTH_20_NOHT:
		return IWL_CHAN_WIDTH_20_NOHT;
	case NL80211_CHAN_WIDTH_20:
		return IWL_CHAN_WIDTH_20;
	case NL80211_CHAN_WIDTH_40:
		return IWL_CHAN_WIDTH_40;
	case NL80211_CHAN_WIDTH_80:
		return IWL_CHAN_WIDTH_80;
	case NL80211_CHAN_WIDTH_160:
		return IWL_CHAN_WIDTH_160;
	default:
		break;
	}

	WARN_ON(1); /* shouldn't get here */
	return IWL_NUM_CHAN_WIDTH;
}

static int cfg_chan_to_iwl_chan(const struct cfg80211_chan_def *cfg_chandef,
				struct iwl_fmac_chandef *iwl_chandef)
{
	enum iwl_fmac_chan_width bandwidth =
			cfg_width_to_iwl_width(cfg_chandef->width);

	if (bandwidth >= IWL_NUM_CHAN_WIDTH)
		return -EINVAL;

	iwl_chandef->control_freq =
			cpu_to_le16(cfg_chandef->chan->center_freq);
	iwl_chandef->bandwidth = bandwidth;
	iwl_chandef->center_freq1 = cpu_to_le16(cfg_chandef->center_freq1);

	return 0;
}

static int iwl_width_to_cfg_width(enum iwl_fmac_chan_width iwl_width)
{
	switch (iwl_width) {
	case IWL_CHAN_WIDTH_20_NOHT:
		return NL80211_CHAN_WIDTH_20_NOHT;
	case IWL_CHAN_WIDTH_20:
		return NL80211_CHAN_WIDTH_20;
	case IWL_CHAN_WIDTH_40:
		return NL80211_CHAN_WIDTH_40;
	case IWL_CHAN_WIDTH_80:
		return NL80211_CHAN_WIDTH_80;
	case IWL_CHAN_WIDTH_160:
		return NL80211_CHAN_WIDTH_160;
	default:
		WARN_ON(1); /* shouldn't get here */
		return -1;
	}
}

static int iwl_fmac_start_ap_fw(struct iwl_fmac *fmac,
				struct iwl_fmac_vif *vif,
				struct cfg80211_ap_settings *params)
{
	enum iwl_fmac_chan_width bandwidth;
	int ret;
	struct iwl_fmac_start_ap_cmd cmd = {
		.vif_id = vif->id,
		.ssid_len = params->ssid_len,
		.beacon_int = cpu_to_le32(params->beacon_interval),
		.dtim_period = params->dtim_period,
		.inactivity_timeout = cpu_to_le32(params->inactivity_timeout),
		.chandef.control_freq =
			cpu_to_le16(params->chandef.chan->center_freq),
		.chandef.center_freq1 =
			cpu_to_le16(params->chandef.center_freq1),
	};
	struct iwl_fmac_start_ap_resp *resp;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(FMAC_START_AP, FMAC_GROUP, 0),
		.flags = CMD_WANT_SKB,
		.data = { &cmd, },
		.len = { sizeof(cmd), },
	};

	bandwidth = cfg_width_to_iwl_width(params->chandef.width);
	if (bandwidth >= IWL_NUM_CHAN_WIDTH)
		return -EINVAL;

	cmd.chandef.bandwidth = bandwidth;
	memcpy(cmd.ssid, params->ssid, params->ssid_len);

	BUILD_BUG_ON((int)NL80211_HIDDEN_SSID_NOT_IN_USE !=
		     (int)IWL_FMAC_HIDDEN_SSID_NONE);
	BUILD_BUG_ON((int)NL80211_HIDDEN_SSID_ZERO_LEN !=
		     (int)IWL_FMAC_HIDDEN_SSID_ZERO_LEN);
	BUILD_BUG_ON((int)NL80211_HIDDEN_SSID_ZERO_CONTENTS !=
		     (int)IWL_FMAC_HIDDEN_SSID_ZERO_BYTES);
	cmd.hidden_ssid = params->hidden_ssid;

	cfg_crypto_to_iwlfmac_crypto(fmac, &params->crypto, &cmd.crypto);

	if (cmd.crypto.cipher_group & cpu_to_le32(IWL_FMAC_CIPHER_TKIP) ||
	    cmd.crypto.ciphers_pairwise & cpu_to_le32(IWL_FMAC_CIPHER_TKIP))
		return -ENOTSUPP;

	mutex_lock(&fmac->mutex);

	if (vif->ap.state != IWL_FMAC_AP_STOPPED)
		ret = -EALREADY;
	else
		ret = iwl_fmac_send_cmd(fmac, &hcmd);

	if (ret)
		goto out;

	resp = (void *)((struct iwl_rx_packet *)hcmd.resp_pkt)->data;

	switch (le32_to_cpu(resp->status)) {
	case IWL_FMAC_START_AP_SUCCESS:
		break;
	case IWL_FMAC_START_AP_INVALID_PARAMS:
		ret = -EINVAL;
		break;
	case IWL_FMAC_START_AP_UNSUPPORTED_PARAMS:
		ret = -ENOTSUPP;
		break;
	default:
		WARN_ON(1);
		ret = -ENOTSUPP;
		break;
	}

	if (ret)
		goto out_free_resp;

	vif->ap.state = IWL_FMAC_AP_STARTING;

	/*
	 * TODO: schedule a timeout on getting AP_STARTED notification,
	 *	 and if it fails to arrive call cfg80211_stop_iface().
	 */
out_free_resp:
	iwl_free_resp(&hcmd);
out:
	mutex_unlock(&fmac->mutex);
	return ret;
}

static int iwl_fmac_send_beacon_cmd(struct iwl_fmac *fmac,
				    struct iwl_fmac_vif *vif,
				    const struct cfg80211_beacon_data *params,
				    bool cck)
{
	struct iwl_mac_beacon_cmd beacon_cmd = {};
	struct iwl_host_cmd cmd = {
		.id = BEACON_TEMPLATE_CMD,
		.flags = CMD_ASYNC,
		.len[0] = sizeof(beacon_cmd),
		.data[0] = &beacon_cmd,
		.dataflags[1] = IWL_HCMD_DFL_DUP,
	};
	u16 flags = 0;
	u8 *tmp;

	if (params->head)
		vif->ap.head_len = params->head_len;
	/* else keep the old value */

	if (params->tail)
		vif->ap.tail_len = params->tail_len;
	/* else keep the old value */

	tmp = vif->ap.beacon;
	vif->ap.beacon = kzalloc(vif->ap.head_len + vif->ap.tail_len,
				 GFP_KERNEL);
	if (!vif->ap.beacon)
		return -ENOMEM;

	if (params->head)
		memcpy(vif->ap.beacon, params->head, vif->ap.head_len);
	else
		memcpy(vif->ap.beacon, vif->ap.head, vif->ap.head_len);
	vif->ap.head = vif->ap.beacon;

	if (params->tail)
		memcpy(vif->ap.beacon + vif->ap.head_len, params->tail,
		       vif->ap.tail_len);
	else
		memcpy(vif->ap.beacon + vif->ap.head_len, vif->ap.tail,
		       vif->ap.tail_len);
	vif->ap.tail = vif->ap.head + vif->ap.head_len;

	kfree(tmp);

	/*
	 * TODO: handle beacon_ies (used only for OWE and
	 * other esoteric features.
	 */

	/* TODO: (e)CSA */

	if (cck)
		flags |= IWL_MAC_BEACON_CCK;

	beacon_cmd.flags = cpu_to_le16(flags);
	beacon_cmd.byte_cnt = cpu_to_le16(vif->ap.head_len + vif->ap.tail_len);
	beacon_cmd.template_id = cpu_to_le32(vif->id);

	beacon_cmd.tim_idx = cpu_to_le32(params->head_len);
	beacon_cmd.tim_size = 0;

	cmd.len[1] = vif->ap.head_len + vif->ap.tail_len;
	cmd.data[1] = vif->ap.beacon;

	return iwl_fmac_send_cmd(fmac, &cmd);
}

static int iwl_fmac_start_ap_host(struct iwl_fmac *fmac,
				  struct iwl_fmac_vif *vif,
				  const struct cfg80211_ap_settings *params)
{
	bool cck = false;
	int ret;

	if (cfg_chan_to_iwl_chan(&params->chandef, &vif->chandef))
		return -EINVAL;

	mutex_lock(&fmac->mutex);

	if (le16_to_cpu(vif->chandef.control_freq) <= 3000 &&
	    vif->wdev.iftype != NL80211_IFTYPE_P2P_GO)
		cck = true;

	ret = iwl_fmac_send_beacon_cmd(fmac, vif, &params->beacon, cck);

	vif->ap.state = IWL_FMAC_AP_STARTING;
	mutex_unlock(&fmac->mutex);

	return ret;
}

static int iwl_fmac_start_ap(struct wiphy *wiphy, struct net_device *dev,
			     struct cfg80211_ap_settings *params)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);

	if (iwlfmac_mod_params.host_based_ap)
		return iwl_fmac_start_ap_host(fmac, vif, params);
	return iwl_fmac_start_ap_fw(fmac, vif, params);
}

static int iwl_fmac_change_beacon(struct wiphy *wiphy, struct net_device *dev,
				  struct cfg80211_beacon_data *params)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	bool cck = false;

	/* TODO */
	if (!iwlfmac_mod_params.host_based_ap)
		return 0;

	if (le16_to_cpu(vif->chandef.control_freq) <= 3000 &&
	    vif->wdev.iftype != NL80211_IFTYPE_P2P_GO)
		cck = true;

	return iwl_fmac_send_beacon_cmd(fmac, vif, params, cck);
}

static int iwl_fmac_stop_ap(struct wiphy *wiphy, struct net_device *dev)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	int ret;

	mutex_lock(&fmac->mutex);
	if (vif->ap.state != IWL_FMAC_AP_STOPPED) {
		struct iwl_fmac_stop_ap_cmd cmd = {
			.vif_id = vif->id,
		};

		iwl_fmac_clear_ap_state(fmac, vif);
		ret = iwl_fmac_send_cmd_pdu(fmac,
					    iwl_cmd_id(FMAC_STOP_AP,
						       FMAC_GROUP, 0),
					    0, sizeof(cmd), &cmd);
	} else {
		/* no-op */
		ret = 0;
	}
	mutex_unlock(&fmac->mutex);

	return ret;
}

static int iwl_fmac_change_bss(struct wiphy *wiphy,
			       struct net_device *dev,
			       struct bss_parameters *params)
{
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);

	/* TODO: more */

	vif->ap.isolate = params->ap_isolate;

	return 0;
}

static int iwl_fmac_set_txq_params(struct wiphy *wiphy,
				   struct net_device *dev,
				   struct ieee80211_txq_params *params)
{
	/* TODO */
	return 0;
}

static int iwl_fmac_set_power_mgmt(struct wiphy *wiphy, struct net_device *dev,
				   bool enabled, int timeout)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	int ret;

	IWL_DEBUG_POWER(fmac, "set_power_mgmt enabled=%d\n", enabled);

	if (vif->wdev.iftype != NL80211_IFTYPE_STATION)
		return -EOPNOTSUPP;

	mutex_lock(&fmac->mutex);
	ret = iwl_fmac_send_config_u32(fmac, vif->id,
				       IWL_FMAC_CONFIG_VIF_POWER_DISABLED,
				       enabled ? 0 : 1);
	mutex_unlock(&fmac->mutex);

	return ret;
}

static int iwl_fmac_set_wdev_tx_power(struct iwl_fmac *fmac,
				      struct wireless_dev *wdev,
				      int user_power_level)
{
	struct iwl_fmac_vif *vif = vif_from_wdev(wdev);

	if (wdev->iftype == NL80211_IFTYPE_AP &&
	    iwlfmac_mod_params.host_based_ap)
		return -EOPNOTSUPP;

	if (vif->user_power_level == user_power_level)
		return 0;

	vif->user_power_level = user_power_level;

	return iwl_fmac_send_config_u32(fmac, vif->id,
					IWL_FMAC_CONFIG_VIF_TXPOWER_USER,
					user_power_level);
}

static int iwl_fmac_set_tx_power(struct wiphy *wiphy,
				 struct wireless_dev *wdev,
				 enum nl80211_tx_power_setting txp_type,
				 int mbm)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	int user_power_level;

	switch (txp_type) {
	case NL80211_TX_POWER_AUTOMATIC:
		user_power_level = IWL_FMAC_POWER_LEVEL_UNSET;
		break;
	case NL80211_TX_POWER_LIMITED:
		if (mbm < 0 || (mbm % 100))
			return -EOPNOTSUPP;
		user_power_level = MBM_TO_DBM(mbm);
		break;
	case NL80211_TX_POWER_FIXED:
	default:
		return -EOPNOTSUPP;
	}

	mutex_lock(&fmac->mutex);

	if (wdev) {
		iwl_fmac_set_wdev_tx_power(fmac, wdev, user_power_level);
	} else {
		list_for_each_entry(wdev, &wiphy->wdev_list, list)
			iwl_fmac_set_wdev_tx_power(fmac, wdev,
						   user_power_level);
	}

	mutex_unlock(&fmac->mutex);

	return 0;
}

static int iwl_fmac_get_channel(struct wiphy *wiphy,
				struct wireless_dev *wdev,
				struct cfg80211_chan_def *chandef)
{
	struct iwl_fmac_vif *vif = vif_from_wdev(wdev);

	if (!(wdev->iftype == NL80211_IFTYPE_AP &&
	      vif->ap.state == IWL_FMAC_AP_STARTED) &&
	    !(wdev->iftype == NL80211_IFTYPE_ADHOC &&
	      vif->ibss.state == IWL_FMAC_IBSS_JOINED))
		return -ENODATA;
	/*
	 * TODO: support other vif types
	 */

	memset(chandef, 0, sizeof(*chandef));

	chandef->chan =
		ieee80211_get_channel(wiphy,
				      le16_to_cpu(vif->chandef.control_freq));
	chandef->center_freq1 = le16_to_cpu(vif->chandef.center_freq1);
	chandef->width = iwl_width_to_cfg_width(vif->chandef.bandwidth);

	return 0;
}

static int iwl_fmac_suspend(struct wiphy *wiphy,
			    struct cfg80211_wowlan *wowlan)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);

	/* TODO: Wowlan */

	mutex_lock(&fmac->mutex);
	iwl_fmac_stop_device(fmac);
	mutex_unlock(&fmac->mutex);

	return 0;
}

static int iwl_fmac_resume(struct wiphy *wiphy)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);

	/*
	 * TODO: Wowlan.
	 * For now we run a full NIC restart here instead of tearing down the
	 * connections in suspend flow and restoring them here.
	 */
	iwl_fmac_nic_restart(fmac);

	return 0;
}

#if CFG80211_VERSION >= KERNEL_VERSION(3,14,0)
static int iwl_fmac_set_qos_map(struct wiphy *wiphy,
				struct net_device *dev,
				struct cfg80211_qos_map *qos_map)
{
	struct iwl_fmac_qos_map *fmac_prev_qos_map, *fmac_next_qos_map;
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);

	if (!qos_map) {
		fmac_next_qos_map = NULL;
		goto out;
	}

	fmac_next_qos_map = kzalloc(sizeof(*fmac_next_qos_map), GFP_KERNEL);
	if (!fmac_next_qos_map)
		return -ENOMEM;
	memcpy(&fmac_next_qos_map->qos_map, qos_map,
	       sizeof(fmac_next_qos_map->qos_map));

out:
	fmac_prev_qos_map =
		rcu_dereference_protected(vif->qos_map,
					  lockdep_is_held(&vif->wdev.mtx));

	rcu_assign_pointer(vif->qos_map, fmac_next_qos_map);

	if (fmac_prev_qos_map)
		kfree_rcu(fmac_prev_qos_map, rcu_head);

	return 0;
}
#endif

static int iwl_fmac_set_pmk(struct wiphy *wiphy, struct net_device *dev,
			    const struct cfg80211_pmk_conf *conf)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	struct iwl_fmac_mlme_set_key_cmd cmd = {
		.vif_id = vif->id,
	};
	int ret;

	if (!conf->pmk)
		return -EINVAL;

	/* FT authentication is currently not supported */
	if (conf->pmk_r0_name)
		return -ENOTSUPP;

	switch (conf->pmk_len) {
	case WLAN_PMK_LEN:
		cmd.key_type = IWL_FMAC_KEY_TYPE_PMK;
		break;
	case WLAN_PMK_LEN_EAP_LEAP:
		cmd.key_type = IWL_FMAC_KEY_TYPE_PMK_EAP_LEAP;
		break;
	case WLAN_PMK_LEN_SUITE_B_192:
		cmd.key_type = IWL_FMAC_KEY_TYPE_PMK_SUITE_B_192;
		break;
	default:
		return -EINVAL;
	}

	memcpy(cmd.aa, conf->aa, ETH_ALEN);
	memcpy(cmd.key, conf->pmk, conf->pmk_len);

	mutex_lock(&fmac->mutex);
	ret = iwl_fmac_send_cmd_pdu(fmac,
				    iwl_cmd_id(FMAC_SET_KEY, FMAC_GROUP, 0),
				    0, sizeof(cmd), &cmd);
	mutex_unlock(&fmac->mutex);

	return ret;
}

static int iwl_fmac_del_pmk(struct wiphy *wiphy, struct net_device *dev,
			    const u8 *aa)
{
	return -ENOTSUPP;
}

int
iwl_fmac_send_ftm_responder_start(struct iwl_fmac *fmac,
				  struct iwl_fmac_vif *vif,
				  struct cfg80211_ftm_responder_params *params)
{
	struct iwl_fmac_start_ftm_responder_cmd *cmd;
	size_t cmd_size = sizeof(*cmd) + ALIGN(params->lci_len +
					       params->civic_len, 4);
	int ret;

	lockdep_assert_held(&fmac->mutex);

	cmd = kzalloc(cmd_size, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->vif_id = vif->id;

	if (params->lci_len) {
		cmd->lci_len = cpu_to_le32(params->lci_len);
		memcpy(cmd->lci_civic, params->lci, params->lci_len);
	}

	if (params->civic_len) {
		cmd->civic_len = cpu_to_le32(params->civic_len);
		memcpy(cmd->lci_civic + params->lci_len, params->civic,
		       params->civic_len);
	}

	ret = iwl_fmac_send_cmd_pdu(fmac, iwl_cmd_id(FMAC_START_FTM_RESPONDER,
						     FMAC_GROUP, 0),
				    0, cmd_size, cmd);

	kfree(cmd);
	return ret;
}

static int
iwl_fmac_copy_responder_params(struct cfg80211_ftm_responder_params *src,
			       struct cfg80211_ftm_responder_params *dst)
{
	if (src->lci_len) {
		dst->lci_len = src->lci_len;
		dst->lci = kmemdup(src->lci, src->lci_len, GFP_KERNEL);
		if (!dst->lci)
			return -ENOMEM;
	}

	if (src->civic_len) {
		dst->civic_len = src->civic_len;
		dst->civic = kmemdup(src->civic, src->civic_len, GFP_KERNEL);
		if (!dst->civic) {
			kfree(dst->lci);
			dst->lci = NULL;
			return -ENOMEM;
		}
	}

	return 0;
}

#if CFG80211_VERSION >= KERNEL_VERSION(99,0,0)
static int
iwl_fmac_start_ftm_responder(struct wiphy *wiphy, struct net_device *dev,
			     struct cfg80211_ftm_responder_params *params)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	int ret = 0;

	mutex_lock(&fmac->mutex);
	if (vif->wdev.iftype != NL80211_IFTYPE_AP) {
		ret = -EINVAL;
		goto out;
	}

	if (vif->ap.state != IWL_FMAC_AP_STARTED) {
		ret = iwl_fmac_copy_responder_params(params,
						     &vif->ap.responder_params);
		if (!ret)
			vif->ap.start_responder = true;

		/*
		 * TODO: When a timeout is added for getting the AP_STARTED
		 * notification, clear responder params if AP fails to start.
		 */
		goto out;
	}

	ret = iwl_fmac_send_ftm_responder_start(fmac, vif, params);
out:
	mutex_unlock(&fmac->mutex);
	return ret;
}
#endif

static int
iwl_fmac_cfg_wep_keys_to_crypto(struct cfg80211_ibss_params *ibss_params,
				struct iwl_fmac_crypto *iwl_crypto)
{
	int i;
	u32 suite;
	u32 ciphers;

	if (ibss_params->wep_keys) {
		ciphers = 0;
		for (i = 0; i < CFG80211_MAX_WEP_KEYS; i++) {
			if (!ibss_params->wep_keys[i].key_len)
				continue;

			suite = ibss_params->wep_keys[i].cipher;

			if (suite == WLAN_CIPHER_SUITE_WEP104)
				ciphers |= IWL_FMAC_CIPHER_WEP104;
			else if (suite == WLAN_CIPHER_SUITE_WEP40)
				ciphers |= IWL_FMAC_CIPHER_WEP40;
			else
				return -ENOTSUPP;

			memcpy(iwl_crypto->u.wep.key[i],
			       ibss_params->wep_keys[i].key,
			       ibss_params->wep_keys[i].key_len);
			iwl_crypto->u.wep.key_len[i] =
				ibss_params->wep_keys[i].key_len;
		}
	} else {
		ciphers = IWL_FMAC_CIPHER_NONE;
	}

	iwl_crypto->ciphers_pairwise = cpu_to_le32(ciphers);
	iwl_crypto->cipher_group = cpu_to_le32(ciphers);
	iwl_crypto->u.wep.def_key = ibss_params->wep_tx_key;
	return 0;
}

static int iwl_fmac_join_ibss(struct wiphy *wiphy, struct net_device *dev,
			      struct cfg80211_ibss_params *params)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	enum iwl_fmac_chan_width bandwidth;
	int ret;
	struct iwl_fmac_join_ibss_cmd cmd = {
		.vif_id = vif->id
	};
	struct iwl_fmac_join_ibss_resp *resp;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(FMAC_JOIN_IBSS, FMAC_GROUP, 0),
		.flags = CMD_WANT_SKB,
		.data = { &cmd, },
		.len = { sizeof(cmd), },
	};

	bandwidth = cfg_width_to_iwl_width(params->chandef.width);
	if (bandwidth >= IWL_NUM_CHAN_WIDTH)
		return -EINVAL;

	if (params->bssid) {
		ether_addr_copy(cmd.bssid, params->bssid);
		cmd.fixed_bssid = 1;
	}

	cmd.beacon_int = cpu_to_le16(params->beacon_interval);
	cmd.fixed_freq = params->channel_fixed;
	cmd.ssid_len = params->ssid_len;
	memcpy(cmd.ssid, params->ssid, params->ssid_len);
	cmd.chandef.control_freq =
		cpu_to_le16(params->chandef.chan->center_freq);
	cmd.chandef.bandwidth = bandwidth;
	cmd.chandef.center_freq1 =
		cpu_to_le16((u16)params->chandef.center_freq1);

	cmd.crypto.key_mgmt = 0;
	ret = iwl_fmac_cfg_wep_keys_to_crypto(params, &cmd.crypto);
	if (ret)
		return ret;

	/* TODO: Support user setting basic_rates/mcast_rate ? */

	mutex_lock(&fmac->mutex);

	if (vif->ibss.state != IWL_FMAC_IBSS_IDLE)
		ret = -EALREADY;
	else
		ret = iwl_fmac_send_cmd(fmac, &hcmd);

	if (ret)
		goto out;

	resp = (void *)((struct iwl_rx_packet *)hcmd.resp_pkt)->data;

	switch (le32_to_cpu(resp->status)) {
	case IWL_FMAC_JOIN_IBSS_SUCCESS:
		break;
	case IWL_FMAC_JOIN_IBSS_INVALID_PARAMS:
		ret = -EINVAL;
		break;
	case IWL_FMAC_JOIN_IBSS_UNSUPPORTED_PARAMS:
		ret = -ENOTSUPP;
		break;
	default:
		WARN_ON(1);
		ret = -EIO;
		break;
	}

	vif->ibss.state = IWL_FMAC_IBSS_JOINING;

	/*
	 * TODO: schedule a timeout on getting IBSS_JOIND notification,
	 *	 and if it fails to arrive call cfg80211_stop_iface().
	 */

	iwl_free_resp(&hcmd);
out:
	mutex_unlock(&fmac->mutex);
	return ret;
}

static int iwl_fmac_leave_ibss(struct wiphy *wiphy, struct net_device *dev)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	int ret;

	mutex_lock(&fmac->mutex);
	if (vif->ibss.state != IWL_FMAC_IBSS_IDLE) {
		struct iwl_fmac_leave_ibss_cmd cmd = {
			.vif_id = vif->id
		};
		iwl_fmac_clear_ibss_state(fmac, vif);
		ret = iwl_fmac_send_cmd_pdu(fmac,
					    iwl_cmd_id(FMAC_LEAVE_IBSS,
						       FMAC_GROUP, 0),
					    0, sizeof(cmd), &cmd);
	} else {
		ret = 0;
	}
	mutex_unlock(&fmac->mutex);

	return ret;
}

static u64 iwl_fmac_msrment_cookie(struct iwl_fmac *fmac,
				   enum nl80211_msrment_type type)
{
	fmac->msrment_cookie_counter++;
	if (fmac->msrment_cookie_counter == (1ULL << 48))
		fmac->msrment_cookie_counter = 1;

	return ((u64)type << 48) | fmac->msrment_cookie_counter;
}

#if CFG80211_VERSION >= KERNEL_VERSION(99,0,0)
static int iwl_fmac_perform_msrment(struct wiphy *wiphy,
				    struct wireless_dev *dev,
				    struct cfg80211_msrment_request *req,
				    u64 *cookie)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_wdev(dev);

	*cookie = iwl_fmac_msrment_cookie(fmac, req->type);

	switch (req->type) {
	case NL80211_MSRMENT_TYPE_FTM:
		return iwl_fmac_perform_ftm(fmac, vif, *cookie, &req->u.ftm);
	default:
		return -EOPNOTSUPP;
	}
}
#endif

#if CFG80211_VERSION >= KERNEL_VERSION(99,0,0)
static int iwl_fmac_abort_msrment(struct wiphy *wiphy, struct wireless_dev *dev,
				  u64 cookie)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_wdev(dev);
	enum nl80211_msrment_type type = cookie >> 48;

	switch (type) {
	case NL80211_MSRMENT_TYPE_FTM:
		iwl_fmac_abort_ftm(fmac, vif, cookie);
	default:
		return -EOPNOTSUPP;
	}
}
#endif

static int iwl_fmac_set_monitor_channel(struct wiphy *wiphy,
					struct cfg80211_chan_def *chandef)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	int ret;
	struct iwl_fmac_set_monitor_chan_cmd cmd = {};

	ret = cfg_chan_to_iwl_chan(chandef, &cmd.chandef);
	if (ret)
		return ret;

	mutex_lock(&fmac->mutex);
	if (WARN_ON(!rcu_access_pointer(fmac->monitor_vif))) {
		ret = -EINVAL;
	} else {
		struct iwl_fmac_vif *mvif;

		mvif = rcu_dereference_protected(fmac->monitor_vif,
						 lockdep_is_held(&fmac->mutex));
		cmd.vif_id = mvif->id;
		ret = iwl_fmac_send_cmd_pdu(fmac,
					    iwl_cmd_id(FMAC_SET_MONITOR_CHAN,
						       FMAC_GROUP, 0),
					    0, sizeof(cmd), &cmd);
	}
	mutex_unlock(&fmac->mutex);

	return ret;
}

struct cfg80211_ops iwl_fmac_cfg_ops = {
	.add_virtual_intf = iwl_fmac_cfg_add_virtual_intf,
	.del_virtual_intf = iwl_fmac_cfg_del_virtual_intf,
	.change_virtual_intf = iwl_fmac_cfg_change_virtual_intf,

	.scan = iwl_fmac_cfg_scan,
#if CFG80211_VERSION >= KERNEL_VERSION(4,5,0)
	.abort_scan = iwl_fmac_cfg_abort_scan,
#endif

	.connect = iwl_fmac_cfg_connect,
	.disconnect = iwl_fmac_cfg_disconnect,

	.get_station = iwl_fmac_get_station,
	.dump_station = iwl_fmac_dump_station,

	.set_wiphy_params = iwl_fmac_set_wiphy_params,

	.set_power_mgmt = iwl_fmac_set_power_mgmt,

	/* Needed for enterprise security (or security bringup?):
	 * .add_key, .del_key, .set_default_key, .set_default_mgmt_key,
	 */

	/* WoWLAN:
	 * .set_wakeup, .set_rekey_data
	 */
	.suspend = iwl_fmac_suspend,
	.resume = iwl_fmac_resume,

	/* AP/GO:
	 * maybe .probe_client, .set_mac_acl, .set_ap_chanwidth
	 */
	.start_ap = iwl_fmac_start_ap,
	.change_beacon = iwl_fmac_change_beacon,
	.stop_ap = iwl_fmac_stop_ap,
	.change_bss = iwl_fmac_change_bss,
	.set_txq_params = iwl_fmac_set_txq_params,

#if CFG80211_VERSION >= KERNEL_VERSION(4,9,0)
	.start_nan = iwl_fmac_start_nan,
#endif
#if CFG80211_VERSION >= KERNEL_VERSION(4,9,0)
	.stop_nan = iwl_fmac_stop_nan,
#endif
#if CFG80211_VERSION >= KERNEL_VERSION(4,9,0)
	.add_nan_func = iwl_fmac_add_nan_func,
#endif
#if CFG80211_VERSION >= KERNEL_VERSION(4,9,0)
	.del_nan_func = iwl_fmac_del_nan_func,
#endif

	/*IBSS*/
	.join_ibss = iwl_fmac_join_ibss,
	.leave_ibss = iwl_fmac_leave_ibss,

	.set_tx_power = iwl_fmac_set_tx_power,
	/* .get_tx_power ? */

	.get_channel = iwl_fmac_get_channel,

	/* .dump_survey - for Android active/RX/TX/scan time statistics */

	/* .set_pmksa, .del_pmksa, .flush_pmksa */

	/* .remain_on_channel, .cancel_remain_on_channel,
	 * .mgmt_tx, .mgmt_tx_cancel_wait, .mgmt_frame_register ?
	 */

	/* .sched_scan_start, .sched_scan_stop */

	/* .start_p2p_device, .stop_p2p_device ? */

	/* .crit_proto_start, .crit_proto_stop ? */

#if CFG80211_VERSION >= KERNEL_VERSION(3,14,0)
	.set_qos_map = iwl_fmac_set_qos_map,
#endif

	.set_pmk = iwl_fmac_set_pmk,
	.del_pmk = iwl_fmac_del_pmk,

#if CFG80211_VERSION >= KERNEL_VERSION(99,0,0)
	.perform_msrment = iwl_fmac_perform_msrment,
#endif
#if CFG80211_VERSION >= KERNEL_VERSION(99,0,0)
	.abort_msrment = iwl_fmac_abort_msrment,
#endif
#if CFG80211_VERSION >= KERNEL_VERSION(99,0,0)
	.start_ftm_responder = iwl_fmac_start_ftm_responder,
#endif
	.set_monitor_channel = iwl_fmac_set_monitor_channel,
	.nan_ndp = iwl_fmac_nan_ndp,
	.nan_data_stop = iwl_fmac_nan_data_stop,

	/* new API needed:
	 *  - to allow 4-way-hs offload
	 *  - P2P offloading?
	 */
};

static u32 iwl_fmac_cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_GCMP,
	WLAN_CIPHER_SUITE_GCMP_256,
	WLAN_CIPHER_SUITE_CCMP_256,
	WLAN_CIPHER_SUITE_AES_CMAC,
	WLAN_CIPHER_SUITE_BIP_CMAC_256,
	WLAN_CIPHER_SUITE_BIP_GMAC_128,
	WLAN_CIPHER_SUITE_BIP_GMAC_256,
	WLAN_CIPHER_SUITE_TKIP,
};

static const struct wiphy_ftm_initiator_capa iwl_fmac_ftm_initiator_capa = {
	.max_total_ftm_targets = IWL_FMAC_FTM_MAX_APS,
	.max_two_sided_ftm_targets = IWL_FMAC_FTM_MAX_APS,
	.asap = true,
	.non_asap = true,
	.req_tsf = true,
	.preamble = NL80211_FTM_PREAMBLE_LEGACY | NL80211_FTM_PREAMBLE_HT |
		    NL80211_FTM_PREAMBLE_VHT,
	.bw = NL80211_FTM_BW_20 | NL80211_FTM_BW_40 | NL80211_FTM_BW_80,
	.req_lci = true,
	.req_civic = true,
};

#define SCAN_OFFLOAD_PROBE_REQ_SIZE 512

static u16 iwl_fmac_max_scan_ie_len(void)
{
	const int internal_preq_size =
		24 + /* header */
		2 + /* ssid ie (without the actual ssid) */
		3 + /* ds ie */
		2 + 2 + 12 + /* (ext) supported rates 2.4 (12 rates) */
		2 + 8 + /* supported rates 5 (8 rates ) */
		sizeof(struct ieee80211_ht_cap) + 2 + /* ht 2.4 */
		sizeof(struct ieee80211_ht_cap) + 2 + /* ht 5 */
		sizeof(struct ieee80211_vht_cap) + 2; /* vht 5 */

	return SCAN_OFFLOAD_PROBE_REQ_SIZE - internal_preq_size;
}

void iwl_fmac_setup_wiphy(struct iwl_fmac *fmac)
{
	struct wiphy *wiphy = wiphy_from_fmac(fmac);
	u8 num_mac;
	int i;

	set_wiphy_dev(wiphy, fmac->dev);
	snprintf(wiphy->fw_version, sizeof(wiphy->fw_version),
		 "%s", fmac->fw->fw_version);

	/* iface_combinations, software_iftypes */

	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
				 BIT(NL80211_IFTYPE_AP) |
				 BIT(NL80211_IFTYPE_ADHOC) |
				 BIT(NL80211_IFTYPE_MONITOR) |
				 BIT(NL80211_IFTYPE_NAN) |
				 BIT(NL80211_IFTYPE_NAN_DATA);

	wiphy->flags = WIPHY_FLAG_NETNS_OK |
		       /* WIPHY_FLAG_SUPPORTS_SCHED_SCAN | */
		       /* WIPHY_FLAG_SUPPORTS_FW_ROAM | ?? */
		       WIPHY_FLAG_PS_ON_BY_DEFAULT |
		       WIPHY_FLAG_HAVE_AP_SME |
		       WIPHY_FLAG_HAS_STATIC_WEP |
		       WIPHY_FLAG_HAS_FTM_RESPONDER;
	wiphy->features = NL80211_FEATURE_SCAN_RANDOM_MAC_ADDR |
			  NL80211_FEATURE_MAC_ON_CREATE |
			  NL80211_FEATURE_HT_IBSS |
			  NL80211_FEATURE_INACTIVITY_TIMER;
	/* LAR (DRS) is always supported in FMAC FWs */
	wiphy->regulatory_flags |= REGULATORY_WIPHY_SELF_MANAGED |
				   REGULATORY_ENABLE_RELAX_NO_IR;

	wiphy_ext_feature_set(wiphy,
			      NL80211_EXT_FEATURE_4WAY_HANDSHAKE_STA_PSK);
	wiphy_ext_feature_set(wiphy,
			      NL80211_EXT_FEATURE_4WAY_HANDSHAKE_STA_1X);
	wiphy_ext_feature_set(wiphy, NL80211_EXT_FEATURE_VHT_IBSS);

	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wiphy->hw_version = fmac->trans->hw_id;

	wiphy->cipher_suites = iwl_fmac_cipher_suites;
	/* TKIP is supported only on gen2 devices. */
	wiphy->n_cipher_suites = fmac->trans->cfg->gen2 ?
					ARRAY_SIZE(iwl_fmac_cipher_suites) :
					ARRAY_SIZE(iwl_fmac_cipher_suites) - 1;

	/* we don't support MFP on 9000 devices */
	if (iwl_fmac_has_new_tx_api(fmac))
		wiphy_ext_feature_set(wiphy, NL80211_EXT_FEATURE_MFP_OPTIONAL);
	else
		wiphy->n_cipher_suites -= 4;

	/* Extract MAC address */
	ether_addr_copy(fmac->addresses[0].addr, fmac->nvm_data->hw_addr);
	wiphy->addresses = fmac->addresses;
	wiphy->n_addresses = 1;

	/* Extract additional MAC addresses if available */
	num_mac = (fmac->nvm_data->n_hw_addrs > 1) ?
		min(IWL_MAX_ADDRESSES, fmac->nvm_data->n_hw_addrs) : 1;
#ifdef CPTCFG_IWLWIFI_SUPPORT_DEBUG_OVERRIDES
	if (fmac->trans->dbg_cfg.hw_address.len) {
		num_mac = IWL_MAX_ADDRESSES;
	}
#endif

	for (i = 1; i < num_mac; i++) {
		ether_addr_copy(fmac->addresses[i].addr,
				fmac->addresses[i - 1].addr);
		fmac->addresses[i].addr[5]++;
		wiphy->n_addresses++;
	}

	wiphy->max_scan_ssids = IWL_FMAC_MAX_SSIDS;
	wiphy->max_scan_ie_len = iwl_fmac_max_scan_ie_len();

	if (fmac->nvm_data->bands[NL80211_BAND_2GHZ].n_channels)
		wiphy->bands[NL80211_BAND_2GHZ] =
			&fmac->nvm_data->bands[NL80211_BAND_2GHZ];
	if (fmac->nvm_data->bands[NL80211_BAND_5GHZ].n_channels) {
		wiphy->bands[NL80211_BAND_5GHZ] =
			&fmac->nvm_data->bands[NL80211_BAND_5GHZ];

		/* TODO: IEEE80211_VHT_CAP_SU_BEAMFORMER_CAPABLE
		 *	 we can probably set it always...
		 */
	}

	fmac->rts_threshold = IEEE80211_MAX_RTS_THRESHOLD;
	wiphy->rts_threshold = fmac->rts_threshold;

	if (false) {
		wiphy->interface_modes |= BIT(NL80211_IFTYPE_NAN) |
			BIT(NL80211_IFTYPE_NAN_DATA);

		wiphy->iface_combinations = iwl_fmac_iface_combinations_nan;
		wiphy->n_iface_combinations =
			ARRAY_SIZE(iwl_fmac_iface_combinations_nan);

		wiphy->nan_supported_bands = BIT(NL80211_BAND_2GHZ);
		if (fmac->nvm_data->bands[NL80211_BAND_5GHZ].n_channels)
			wiphy->nan_supported_bands |= BIT(NL80211_BAND_5GHZ);
	}

	iwl_fmac_set_wiphy_vendor_commands(wiphy);
	wiphy->ftm_initiator_capa = &iwl_fmac_ftm_initiator_capa;
}
