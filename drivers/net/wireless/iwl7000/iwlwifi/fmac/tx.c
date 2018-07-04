/******************************************************************************
 *
 * This file is provided under GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright 2002-2005, Instant802 Networks, Inc.
 * Copyright 2005-2006, Devicescape Software, Inc.
 * Copyright 2006-2007	Jiri Benc <jbenc@suse.cz>
 * Copyright 2007	Johannes Berg <johannes@sipsolutions.net>
 * Copyright 2013-2014  Intel Mobile Communications GmbH
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
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ieee80211.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/tcp.h>
#include <net/ip.h>
#include <net/ipv6.h>

#include "fmac.h"

#include "fw-api.h"
#include "fw/api/binding.h"

#define MICHAEL_MIC_LEN 8

static inline void iwl_fmac_update_tx_stats(struct net_device *dev, u32 len)
{
	struct pcpu_sw_netstats *tstats = this_cpu_ptr(netdev_tstats(dev));

	u64_stats_update_begin(&tstats->syncp);
	tstats->tx_packets++;
	tstats->tx_bytes += len;
	u64_stats_update_end(&tstats->syncp);
}

static struct sk_buff *iwl_fmac_build_80211_hdr(struct sk_buff *skb,
						struct net_device *dev,
						struct iwl_fmac_skb_info *info)
{
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	struct iwl_fmac *fmac = vif->fmac;
	u16 eth_type, hdrlen;
	struct ieee80211_hdr hdr = {};
	const u8 *encaps_data;
	int encaps_len, skip_header_bytes;

	int ret;
	__le16 fc;

	/* convert Ethernet header to 802.11 header */
	eth_type = (skb->data[12] << 8) | skb->data[13];
	fc = cpu_to_le16(IEEE80211_FTYPE_DATA | IEEE80211_STYPE_DATA);

	switch (vif->wdev.iftype) {
	case NL80211_IFTYPE_STATION:
		fc |= cpu_to_le16(IEEE80211_FCTL_TODS);

		/* BSSID SA DA */
		ether_addr_copy(hdr.addr1, info->sta->addr);
		ether_addr_copy(hdr.addr2, skb->data + ETH_ALEN);
		ether_addr_copy(hdr.addr3, skb->data);
		hdrlen = 24;
		break;
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_GO:
		fc |= cpu_to_le16(IEEE80211_FCTL_FROMDS);

		/* DA BSSID SA */
		ether_addr_copy(hdr.addr1, skb->data);
		ether_addr_copy(hdr.addr2, dev->dev_addr);
		ether_addr_copy(hdr.addr3, skb->data + ETH_ALEN);
		hdrlen = 24;
		break;
	case NL80211_IFTYPE_ADHOC:
		/* DA SA BSSID */
		ether_addr_copy(hdr.addr1, skb->data);
		ether_addr_copy(hdr.addr2, skb->data + ETH_ALEN);
		ether_addr_copy(hdr.addr3, vif->ibss.bssid);
		hdrlen = 24;
		break;
#if CFG80211_VERSION >= KERNEL_VERSION(99,0,0)
	case NL80211_IFTYPE_NAN_DATA:
		/* keep code in case of fall-through (spatch generated) */
#endif
		/* DA SA CLUSTER_ID */
		ether_addr_copy(hdr.addr1, skb->data);
		ether_addr_copy(hdr.addr2, skb->data + ETH_ALEN);
		hdr.addr3[0] = 0x50;
		hdr.addr3[1] = 0x6f;
		hdr.addr3[2] = 0x9a;
		hdr.addr3[3] = 0x01;
		hdr.addr3[4] = vif->nan.cluster_id[0];
		hdr.addr3[5] = vif->nan.cluster_id[1];
		hdrlen = 24;
		break;
	default:
		IWL_WARN(vif->fmac, "Invalid iftype\n");
		ret = -EINVAL;
		goto free;
	}

	/* remove the eth hdr */
	skip_header_bytes = ETH_HLEN;
	if (eth_type == ETH_P_AARP || eth_type == ETH_P_IPX) {
		encaps_data = bridge_tunnel_header;
		encaps_len = sizeof(bridge_tunnel_header);
		skip_header_bytes -= 2;
	} else if (eth_type >= ETH_P_802_3_MIN) {
		encaps_data = rfc1042_header;
		encaps_len = sizeof(rfc1042_header);
		skip_header_bytes -= 2;
	} else {
		encaps_data = NULL;
		encaps_len = 0;
	}

	skb_pull(skb, skip_header_bytes);

	if (encaps_data)
		memcpy(skb_push(skb, encaps_len), encaps_data, encaps_len);

	if (info->key) {
		fc |= cpu_to_le16(IEEE80211_FCTL_PROTECTED);

		switch (info->key->cipher) {
		case IWL_FMAC_CIPHER_WEP40:
		case IWL_FMAC_CIPHER_WEP104:
			/* The HW adds the IV and doesn't even need to allcoate
			 * room for it.
			 */
			break;
		case IWL_FMAC_CIPHER_CCMP:
		case IWL_FMAC_CIPHER_CCMP_256:
		case IWL_FMAC_CIPHER_GCMP:
		case IWL_FMAC_CIPHER_GCMP_256:
			if (iwl_fmac_has_new_tx_api(fmac))
				break;
			skb_push(skb, info->key->iv_len);
			break;
		case IWL_FMAC_CIPHER_TKIP:
			if (fmac->trans->cfg->gen2) {
				ret = skb_linearize(skb);
				if (ret)
					goto free;

				memset(skb_put(skb, MICHAEL_MIC_LEN), 0,
				       MICHAEL_MIC_LEN);
				break;
			}
			/* Fall through */
		default:
			IWL_ERR(vif->fmac, "Cipher %d isn't supported\n",
				info->key->cipher);
		}
	}
	hdr.frame_control = fc;

	if (info->sta->qos) {
		u8 *qos_control;
		u8 tid;

		hdr.frame_control |= cpu_to_le16(IEEE80211_STYPE_QOS_DATA);
		hdrlen += 2;

		qos_control = skb_push(skb, 2);
		memcpy(skb_push(skb, hdrlen - 2), &hdr, hdrlen - 2);

		tid = skb->priority & IEEE80211_QOS_CTL_TAG1D_MASK;
		/*TODO in case of Access Controlled downgrade tid (look at
		 *ieee80211_downgrade
		 */
		/*TODO multicast and noack_map not dealt */

		*qos_control++ = tid;
		*qos_control = 0;
	} else {
		memcpy(skb_push(skb, hdrlen), &hdr, hdrlen);
	}

	return skb;
free:
	kfree_skb(skb);
	return ERR_PTR(ret);
}

static void iwl_fmac_tx_set_sta(struct iwl_fmac_vif *vif,
				struct sk_buff *skb,
				struct iwl_fmac_skb_info *info)
{
	const u8 *addr = skb->data;

	switch (vif->wdev.iftype) {
	case NL80211_IFTYPE_STATION:
		/* TODO: TDLS */
		info->sta = rcu_dereference(vif->mgd.ap_sta);
		if (!info->sta)
			IWL_ERR(vif->fmac, "AP station not initialized\n");
		break;
#if CFG80211_VERSION >= KERNEL_VERSION(99,0,0)
	case NL80211_IFTYPE_NAN_DATA:
		/* keep code in case of fall-through (spatch generated) */
#endif
		/*
		 * multicast frames over NAN Data interface should have
		 * been converted to unicast A-MSDU
		 */
		if (WARN_ON(is_multicast_ether_addr(addr)))
			return;

		/* Fall through */
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_GO:
	case NL80211_IFTYPE_ADHOC:
		if (is_multicast_ether_addr(addr))
			addr = MCAST_STA_ADDR;

		info->sta = iwl_get_sta(vif->fmac, addr);
		if (!info->sta)
			IWL_ERR(vif->fmac, "no destination STA for %pM\n",
				addr);
		break;
	default:
		WARN_ON_ONCE(1);
	}
}

static void iwl_fmac_set_ccm_gcm_pn(struct sk_buff *skb,
				    struct iwl_fmac_skb_info *info,
				    int hdrlen)
{
	u8 *crypto_hdr = skb->data + hdrlen;
	u64 pn;

	pn = atomic64_inc_return(&info->key->tx_pn);
	crypto_hdr[0] = pn;
	crypto_hdr[2] = 0;
	crypto_hdr[3] = 0x20 | (info->key->keyidx << 6);
	crypto_hdr[1] = pn >> 8;
	crypto_hdr[4] = pn >> 16;
	crypto_hdr[5] = pn >> 24;
	crypto_hdr[6] = pn >> 32;
	crypto_hdr[7] = pn >> 40;
}

static void iwl_fmac_tx_set_pn(struct sk_buff *skb,
			       struct iwl_fmac_skb_info *info)
{
	struct ieee80211_hdr *hdr = (void *)skb->data;
	int hdrlen = ieee80211_hdrlen(hdr->frame_control);

	switch (info->key->cipher) {
	case IWL_FMAC_CIPHER_GCMP:
	case IWL_FMAC_CIPHER_GCMP_256:
	case IWL_FMAC_CIPHER_CCMP:
	case IWL_FMAC_CIPHER_CCMP_256:
		iwl_fmac_set_ccm_gcm_pn(skb, info, hdrlen);
		break;
	case IWL_FMAC_CIPHER_WEP104:
	case IWL_FMAC_CIPHER_WEP40:
		break;
	case IWL_FMAC_CIPHER_TKIP:
		WARN_ON_ONCE(1);
		break;
	default:
		WARN_ON_ONCE(1);
	}
}

static void iwl_fmac_tx_set_key(struct iwl_fmac_vif *vif,
				struct sk_buff *skb,
				struct iwl_fmac_skb_info *info)
{
	struct iwl_fmac_sta *sta = info->sta;

	switch (vif->wdev.iftype) {
	case NL80211_IFTYPE_STATION:
		if (sta->encryption)
			info->key = rcu_dereference(sta->ptk[sta->ptk_idx]);
		break;
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_GO:
	case NL80211_IFTYPE_ADHOC:
#if CFG80211_VERSION >= KERNEL_VERSION(99,0,0)
	case NL80211_IFTYPE_NAN_DATA:
		/* keep code in case of fall-through (spatch generated) */
#endif
		if (sta->encryption) {
			if (is_multicast_ether_addr(skb->data)) {
				info->key =
					rcu_dereference(sta->gtk[sta->gtk_idx]);
				WARN_ON_ONCE(!info->key);
			} else {
				info->key =
					rcu_dereference(sta->ptk[sta->ptk_idx]);
				WARN_ON_ONCE(!info->key);
			}
		}
		break;
	default:
		WARN_ON_ONCE(1);
	}
}

static void iwl_fmac_tx_add_stream(struct iwl_fmac *fmac,
				   struct iwl_fmac_sta *sta, u8 tid,
				   struct sk_buff *skb)
{
	struct sk_buff_head *deferred_tx_frames;
	u8 ac;

	lockdep_assert_held(&sta->lock);

	sta->deferred_traffic_tid_map |= BIT(tid);
	set_bit(sta->sta_id, fmac->sta_deferred_frames);

	deferred_tx_frames = &sta->tids[tid].deferred_tx_frames;

	skb_queue_tail(deferred_tx_frames, skb);

	ac = tid_to_ac[tid];

	/*
	 * The first deferred frame stops the ac netdev queue, so we
	 * should never get a second deferred frame for the RA/TID.
	 */
	if (!WARN(skb_queue_len(deferred_tx_frames) != 1,
		  "RATID %d/%d has %d deferred frames\n", sta->sta_id, tid,
		  skb_queue_len(deferred_tx_frames))) {
		iwl_fmac_stop_ac_queue(fmac, &sta->vif->wdev, ac);
		schedule_work(&fmac->add_stream_wk);
	}
}

#ifdef CPTCFG_IWLWIFI_VENDOR_MODE
static int iwl_fmac_tx_vendor_mode(struct iwl_fmac *fmac, struct sk_buff *skb,
				   struct iwl_fmac_skb_info *info)
{
	struct iwl_fmac_sta *sta = info->sta;
	struct iwl_device_cmd *dev_cmd;
	u8 tid = skb->priority & IEEE80211_QOS_CTL_TAG1D_MASK;
	u16 txq_id;

	if (WARN_ON_ONCE(!sta))
		return -1;

	if (WARN_ON_ONCE(sta->sta_id == IWL_FMAC_STATION_COUNT))
		return -1;

	if (!info->dev_cmd) {
		struct iwl_tx_cmd_gen3 *tx_cmd;
		struct ethhdr *hdr = (void *)skb->data;
		u32 csum_assist = IWL_VENDOR_MODE_CSUM;

		dev_cmd = iwl_trans_alloc_tx_cmd(fmac->trans);
		if (unlikely(!dev_cmd))
			return -1;

		memset(dev_cmd, 0, sizeof(*dev_cmd));
		dev_cmd->hdr.cmd = TX_CMD;

		if (skb->ip_summed == CHECKSUM_PARTIAL) {
			/* enable vendor mode checksum support */
			csum_assist |= IWL_VENDOR_ENABLE_CSUM |
					  IWL_VENDOR_UDP_CTRL_ZERO;

			/* signal packet checksums offset */
			csum_assist |= skb->csum_start <<
				       IWL_VENDOR_CSUM_START_POS;
			csum_assist |= (skb->csum_start + skb->csum_offset) <<
				       IWL_VENDOR_CSUM_RES_POS;
		}

		/* pad ethernet header for THC */
		skb_push(skb, 2);

		tx_cmd = (void *)dev_cmd->payload;
		tx_cmd->offload_assist = cpu_to_le32(csum_assist);
		tx_cmd->len = cpu_to_le16(skb->len);
		memcpy(tx_cmd->payload, hdr, sizeof(*hdr));

		info->dev_cmd = dev_cmd;
	} else {
		dev_cmd = info->dev_cmd;
	}

	spin_lock(&sta->lock);

	txq_id = sta->tids[tid].txq_id;

	/* Check if TXQ needs to be allocated */
	if (unlikely(txq_id == IWL_FMAC_INVALID_TXQ_ID)) {
		/* This is a bit hacky - we hold a pointer to the station
		 * which is an RCU protected pointer and enqueue the skb
		 * for later processing. This is fine though since if the
		 * station were to be destroyed, it'd destroy the queue as
		 * well and hence the skb would be freed.
		 */
		iwl_fmac_tx_add_stream(fmac, sta, tid, skb);

		spin_unlock(&sta->lock);
		return 0;
	}

	IWL_DEBUG_TX(fmac, "TX to [%d|%d] Q:%d\n", sta->sta_id, tid, txq_id);

	if (iwl_trans_tx(fmac->trans, skb, dev_cmd, txq_id))
		goto drop_unlock_sta;

	spin_unlock(&sta->lock);

	return 0;

drop_unlock_sta:
	iwl_trans_free_tx_cmd(fmac->trans, dev_cmd);
	spin_unlock(&sta->lock);
}
#endif

static int iwl_fmac_convert_nan_mcast_to_amsdu(struct sk_buff *skb,
					       struct net_device *dev,
					       struct sk_buff_head *queue)
{
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	struct ethhdr *eth = (struct ethhdr *)skb->data;
	struct iwl_fmac_sta *sta;
	struct ieee80211_qos_hdr hdr = {};
	__be16 len;
	int ret, tmp;

	hdr.frame_control = cpu_to_le16(IEEE80211_FTYPE_DATA |
					IEEE80211_STYPE_QOS_DATA);

	/* addr1 would be set below for each station */
	memcpy(hdr.addr2, eth->h_source, ETH_ALEN);
	hdr.addr3[0] = 0x50;
	hdr.addr3[1] = 0x6f;
	hdr.addr3[2] = 0x9a;
	hdr.addr3[3] = 0x01;
	hdr.addr3[4] = vif->nan.cluster_id[0];
	hdr.addr3[5] = vif->nan.cluster_id[1];
	hdr.qos_ctrl = cpu_to_le16(IEEE80211_QOS_CTL_A_MSDU_PRESENT);

	len = cpu_to_be16(skb->len - sizeof(*eth) +
			  sizeof(rfc1042_header) +
			  sizeof(len));

	for_each_valid_sta(vif->fmac, sta, tmp) {
		struct sk_buff *cloned_skb;
		struct iwl_fmac_skb_info *info;
		u8 *data;
		u8 iv_len = 0;

		if (vif != sta->vif)
			continue;

		if (unlikely(ether_addr_equal(eth->h_source, sta->addr)))
			/* do not send back to source */
			continue;

		if (unlikely(!sta->qos)) {
			ret = -EINVAL;
			goto amsdu_fail;
		}

		cloned_skb = skb_clone(skb, GFP_ATOMIC);
		if (!cloned_skb) {
			ret = -ENOMEM;
			goto amsdu_fail;
		}

		info = (void *)cloned_skb->cb;
		memset(info, 0, sizeof(*info));

		/*
		 * To get the key, the station must be set and the skb
		 * should contain a 802.3 header with the station address, so
		 * the unicast key would be fetched.
		 */
		info->sta = sta;
		memcpy(cloned_skb->data, sta->addr, ETH_ALEN);
		iwl_fmac_tx_set_key(vif, cloned_skb, info);

		/* consider space for the IV if it is also needed */
		if (info->key && !iwl_fmac_has_new_tx_api(vif->fmac))
			iv_len = info->key->iv_len;

		/*
		 * note that a single NAN data interface can support various
		 * stations with various security configurations, so the size of
		 * the skb_push can differ between stations.
		 */
		data = skb_push(cloned_skb, sizeof(hdr) +
				sizeof(rfc1042_header) + sizeof(len) + iv_len);

		/* update the address and push the header  */
		memcpy(hdr.addr1, sta->addr, ETH_ALEN);
		memcpy(data, &hdr, sizeof(hdr));
		data += sizeof(hdr);

		/* skip the iv */
		if (iv_len) {
			struct ieee80211_hdr *chdr =
				(struct ieee80211_hdr *)cloned_skb->data;

			chdr->frame_control |=
				cpu_to_le16(IEEE80211_FCTL_PROTECTED);

			iwl_fmac_tx_set_pn(cloned_skb, info);
			data += iv_len;
		}

		/* build the sub frame header */
		memcpy(data, eth, 2 * ETH_ALEN);
		data += (2 * ETH_ALEN);

		/* set the subframe length */
		memcpy(data, &len, sizeof(len));
		data += sizeof(len);

		/* place rfc1042 header */
		memcpy(data, rfc1042_header, sizeof(rfc1042_header));
		data += sizeof(rfc1042_header);

		/* no need to handle padding with a single subframe */

		skb_reset_mac_header(cloned_skb);

		__skb_queue_tail(queue, cloned_skb);
	}

	dev_kfree_skb(skb);
	return 0;

amsdu_fail:
	__skb_queue_purge(queue);
	return ret;
}

static unsigned int iwl_fmac_max_amsdu_size(struct iwl_fmac *fmac,
					    struct iwl_fmac_sta *sta,
					    unsigned int tid)
{
	unsigned int txf;
	int lmac = IWL_LMAC_24G_INDEX;

	if (fw_has_capa(&fmac->fw->ucode_capa,
			IWL_UCODE_TLV_CAPA_CDB_SUPPORT) &&
	    sta->band == NL80211_BAND_5GHZ)
		lmac = IWL_LMAC_5G_INDEX;

	if (sta->he)
		txf = iwl_fmac_11ax_tid_to_tx_fifo[tid];
	else
		txf = iwl_fmac_tid_to_tx_fifo[tid];

	/*
	 * Don't send an AMSDU that will be longer than the TXF.
	 * Add a security margin of 256 for the TX command + headers.
	 * We also want to have the start of the next packet inside the
	 * fifo to be able to send bursts.
	 */
	return min_t(unsigned int, sta->amsdu_size,
		     fmac->fwrt.smem_cfg.lmac[lmac].txfifo_size[txf] - 256);
}

netdev_tx_t iwl_fmac_dev_start_xmit(struct sk_buff *skb,
				    struct net_device *dev)
{
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	struct iwl_fmac_skb_info *info;

	/* TODO: This code needs rework towards AP / TDLS.
	 *	 We need to get the station before we build the header
	 */

	/* If the skb is shared clone it before you change it */
	if (skb_shared(skb)) {
		struct sk_buff *sk_tmp = skb;

		skb = skb_clone(skb, GFP_ATOMIC);
		kfree_skb(sk_tmp);

		if (!skb)
			return NETDEV_TX_OK;
	}

	rcu_read_lock();

	/*
	 * NAN Data interface does not support transmission of multicast data
	 * frames. These frames need to be encapsulated within a unicast A-MSDU
	 * for each station the NAN Data interface is peered with.
	 */
	if (unlikely(ieee80211_viftype_nan_data(vif->wdev.iftype) &&
		     is_multicast_ether_addr(skb->data))) {
		struct sk_buff_head queue;
		int ret;

		__skb_queue_head_init(&queue);

		ret = iwl_fmac_convert_nan_mcast_to_amsdu(skb, dev, &queue);
		if (ret) {
			dev_kfree_skb(skb);
			goto out;
		}

		while ((skb = __skb_dequeue(&queue))) {
			iwl_fmac_update_tx_stats(dev, skb->len);
			if (iwl_fmac_tx_skb(vif->fmac, skb))
				dev_kfree_skb(skb);
		}
		goto out;
	}

	BUILD_BUG_ON(sizeof(struct iwl_fmac_skb_info) > sizeof(skb->cb));

	info = (void *)skb->cb;
	memset(info, 0, sizeof(*info));

	iwl_fmac_tx_set_sta(vif, skb, info);
	if (!info->sta) {
		kfree_skb(skb);
		goto out;
	}

	iwl_fmac_tx_set_key(vif, skb, info);

#ifdef CPTCFG_IWLWIFI_VENDOR_MODE
	if (iwl_fmac_tx_vendor_mode(vif->fmac, skb, info))
		dev_kfree_skb(skb);
	goto out;
#endif
	skb = iwl_fmac_build_80211_hdr(skb, dev, info);
	if (IS_ERR(skb))
		goto out;

	if (info->key && !iwl_fmac_has_new_tx_api(vif->fmac))
		iwl_fmac_tx_set_pn(skb, info);

	iwl_fmac_update_tx_stats(dev, skb->len);
	if (iwl_fmac_tx_skb(vif->fmac, skb))
		dev_kfree_skb(skb);
out:
	rcu_read_unlock();
	return NETDEV_TX_OK;
}

static void iwl_fmac_reclaim_and_free(struct iwl_fmac *fmac,
				      u8 sta_id, u8 tid, u16 txq_id, u16 ssn)
{
	struct sk_buff_head reclaimed_skbs;
	struct iwl_fmac_sta *sta;

	__skb_queue_head_init(&reclaimed_skbs);

	/* we can free until ssn % q.n_bd not inclusive */
	iwl_trans_reclaim(fmac->trans, txq_id, ssn, &reclaimed_skbs);

	while (!skb_queue_empty(&reclaimed_skbs)) {
		struct sk_buff *skb = __skb_dequeue(&reclaimed_skbs);
		struct iwl_fmac_skb_info *skb_info = (void *)skb->cb;

		iwl_trans_free_tx_cmd(fmac->trans, skb_info->dev_cmd);

		dev_kfree_skb(skb);
	}

	rcu_read_lock();
	sta = rcu_dereference(fmac->stas[sta_id]);

	/*
	 * The station typically shouldn't be NULL, since that would mean we
	 * have TX frames released from a queue it owned after the station
	 * was removed.
	 * This can, however, legitimately happen while we remove the station,
	 * we have to remove it from the array before we flush the queues so
	 * that we stop transmitting to those queues, and then the completions
	 * might only happen after it's removed (while removing queues.)
	 */
	if (!sta)
		goto out;

	/* If this is an aggregation queue, we use the ssn since:
	 * ssn = wifi seq_num % 256.
	 * The seq_ctl is the sequence control of the packet to which
	 * this Tx response relates. But if there is a hole in the
	 * bitmap of the BA we received, this Tx response may allow to
	 * reclaim the hole and all the subsequent packets that were
	 * already acked. In that case, seq_ctl != ssn, and the next
	 * packet to be reclaimed will be ssn and not seq_ctl. In that
	 * case, several packets will be reclaimed even if
	 * frame_count = 1.
	 *
	 * The ssn is the index (% 256) of the latest packet that has
	 * treated (acked / dropped) + 1.
	 */

	/* TODO: when TXQ is shared, next_reclaimed is the one after this one */

	if (sta->qos && tid != IWL_MAX_TID_COUNT)
		sta->tids[tid].next_reclaimed = ssn;

out:
	rcu_read_unlock();
}

static struct agg_tx_status *
iwl_fmac_get_agg_status(struct iwl_fmac *fmac, struct iwl_mvm_tx_resp *tx_resp)
{
	if (iwl_fmac_has_new_tx_api(fmac))
		return &((struct iwl_mvm_tx_resp *)tx_resp)->status;
	else
		return ((struct iwl_mvm_tx_resp_v3 *)tx_resp)->status;
}

/**
 * iwl_fmac_get_scd_ssn - returns the SSN of the SCD
 * @tx_resp: the Tx response from the fw (agg or non-agg)
 *
 * When the fw sends an AMPDU, it fetches the MPDUs one after the other. Since
 * it can't know that everything will go well until the end of the AMPDU, it
 * can't know in advance the number of MPDUs that will be sent in the current
 * batch. This is why it writes the agg Tx response while it fetches the MPDUs.
 * Hence, it can't know in advance what the SSN of the SCD will be at the end
 * of the batch. This is why the SSN of the SCD is written at the end of the
 * whole struct at a variable offset. This function knows how to cope with the
 * variable offset and returns the SSN of the SCD.
 */
static u32 iwl_fmac_get_scd_ssn(struct iwl_fmac *fmac,
				struct iwl_mvm_tx_resp *tx_resp)
{
	return le32_to_cpup((__le32 *)iwl_fmac_get_agg_status(fmac, tx_resp) +
			    tx_resp->frame_count) & 0xfff;
}

static void iwl_fmac_update_sta_tx_stats(struct iwl_fmac *fmac,
					 struct iwl_mvm_tx_resp *tx_resp)
{
	struct iwl_fmac_sta *sta;
	struct iwl_fmac_tx_stats *stats;
	int sta_id = IWL_MVM_TX_RES_GET_RA(tx_resp->ra_tid);
	u32 status =
		le16_to_cpu(iwl_fmac_get_agg_status(fmac, tx_resp)->status);

	rcu_read_lock();
	sta = rcu_dereference(fmac->stas[sta_id]);
	if (unlikely(!sta)) {
		rcu_read_unlock();
		return;
	}

	stats = &sta->info.tx_stats;
	stats->last_rate = le32_to_cpu(tx_resp->initial_rate);
	stats->bytes += le16_to_cpu(tx_resp->byte_cnt);
	stats->packets += tx_resp->frame_count;
	if (tx_resp->frame_count == 1) {
		stats->retries += tx_resp->failure_frame;
		if (!(status & TX_STATUS_SUCCESS ||
		      status & TX_STATUS_DIRECT_DONE))
			stats->failed++;
	}
	rcu_read_unlock();
}

static void iwl_fmac_rx_tx_cmd_single(struct iwl_fmac *fmac,
				      struct iwl_rx_packet *pkt)
{
	u16 sequence = le16_to_cpu(pkt->hdr.sequence);
	int txq_id = SEQ_TO_QUEUE(sequence);
	/* struct iwl_mvm_tx_resp_v3 is almost the same */
	struct iwl_mvm_tx_resp *tx_resp = (void *)pkt->data;
	int sta_id = IWL_MVM_TX_RES_GET_RA(tx_resp->ra_tid);
	int tid = IWL_MVM_TX_RES_GET_TID(tx_resp->ra_tid);
	u32 status;
	u16 ssn = iwl_fmac_get_scd_ssn(fmac, tx_resp);

	status = le16_to_cpu(iwl_fmac_get_agg_status(fmac, tx_resp)->status);

	if (iwl_fmac_has_new_tx_api(fmac))
		txq_id = le16_to_cpu(tx_resp->tx_queue);

	iwl_fmac_reclaim_and_free(fmac, sta_id, tid, txq_id, ssn);

	IWL_DEBUG_TX_REPLY(fmac, "TXQ %d status 0x%08x\n", txq_id, status);

	IWL_DEBUG_TX_REPLY(fmac,
			   "\t\t\t\tinitial_rate 0x%x retries %d, idx=%d ssn=%d\n",
			   le32_to_cpu(tx_resp->initial_rate),
			   tx_resp->failure_frame, SEQ_TO_INDEX(sequence),
			   ssn);
}

void iwl_fmac_rx_tx_cmd(struct iwl_fmac *fmac, struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_mvm_tx_resp *tx_resp = (void *)pkt->data;

	if (tx_resp->frame_count == 1)
		iwl_fmac_rx_tx_cmd_single(fmac, pkt);

	/* Ignore aggregated TX responses. They only carry data
	 * for TLC which is now offloaded.
	 */

	iwl_fmac_update_sta_tx_stats(fmac, tx_resp);
}

static void
iwl_fmac_update_sta_ba_tx_stats(struct iwl_fmac *fmac,
				struct iwl_mvm_compressed_ba_notif *notif)
{
	struct iwl_fmac_sta *sta;
	struct iwl_fmac_tx_stats *stats;

	rcu_read_lock();
	sta = rcu_dereference(fmac->stas[notif->sta_id]);
	if (unlikely(!sta)) {
		rcu_read_unlock();
		return;
	}

	stats = &sta->info.tx_stats;
	stats->last_rate = le32_to_cpu(notif->tx_rate);
	stats->bytes += le32_to_cpu(notif->query_byte_cnt);
	stats->packets += le16_to_cpu(notif->query_frame_cnt);
	stats->retries += notif->retry_cnt;
	rcu_read_unlock();
}

void iwl_fmac_rx_ba_notif(struct iwl_fmac *fmac, struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_mvm_ba_notif *ba_notif;
	u16 scd_flow;
	u16 scd_ssn;

	if (iwl_fmac_has_new_tx_api(fmac)) {
		struct iwl_mvm_compressed_ba_notif *ba_res = (void *)pkt->data;
		u8 tid;
		u16 queue;
		u16 tfd_idx;

		if (!le16_to_cpu(ba_res->tfd_cnt))
			goto out;

		/*
		 * TODO:
		 * When supporting multi TID aggregations - we need to move
		 * next_reclaimed to be per TXQ and not per TID or handle it
		 * in a different way.
		 * This will go together with SN and AddBA offload and cannot
		 * be handled properly for now.
		 */
		WARN_ON(le16_to_cpu(ba_res->ra_tid_cnt) != 1);
		tid = ba_res->ra_tid[0].tid;
		if (tid == IWL_MGMT_TID)
			tid = IWL_MAX_TID_COUNT;
		queue = le16_to_cpu(ba_res->tfd[0].q_num);
		tfd_idx = le16_to_cpu(ba_res->tfd[0].tfd_index);
		iwl_fmac_reclaim_and_free(fmac, ba_res->sta_id, tid,
					  queue, tfd_idx);

		iwl_fmac_update_sta_ba_tx_stats(fmac, ba_res);

out:
		IWL_DEBUG_TX_REPLY(fmac,
				   "BA_NOTIFICATION Received from sta_id = %d, flags %x, sent:%d, acked:%d\n",
				   ba_res->sta_id, le32_to_cpu(ba_res->flags),
				   le16_to_cpu(ba_res->txed),
				   le16_to_cpu(ba_res->done));
		return;
	}

	ba_notif = (void *)pkt->data;
	scd_ssn = le16_to_cpu(ba_notif->scd_ssn);
	scd_flow = le16_to_cpu(ba_notif->scd_flow);

	iwl_fmac_reclaim_and_free(fmac, ba_notif->sta_id, ba_notif->tid,
				  scd_flow, scd_ssn);

	IWL_DEBUG_TX_REPLY(fmac, "ba_notif from %pM, sta_id = %d\n",
			   ba_notif->sta_addr, ba_notif->sta_id);
	IWL_DEBUG_TX_REPLY(fmac, "tid %d, seq %d, bitmap 0x%llx, scd flow %d, ssn %d, sent %d, acked %d\n",
			   ba_notif->tid, le16_to_cpu(ba_notif->seq_ctl),
			   (unsigned long long)le64_to_cpu(ba_notif->bitmap),
			   scd_flow, scd_ssn, ba_notif->txed,
			   ba_notif->txed_2_done);
}

static u8 iwl_fmac_bt_coex_tx_prio(struct iwl_fmac *fmac,
				   struct ieee80211_hdr *hdr, u8 ac)
{
	/* TODO: make sure that LMAC ignore those in case we operate in 5GHz */

	if (unlikely(is_multicast_ether_addr(hdr->addr1)))
		return 3;

	switch (ac) {
	case AC_BE:
		return 01;
	case AC_VI:
		return 2;
	case AC_VO:
		return 3;
	default:
		return 0;
	}

	return 0;
}

#define OPT_HDR(type, skb, off) \
	(type *)(skb_network_header(skb) + (off))

static u16 iwl_fmac_tx_csum(struct iwl_fmac *fmac, struct sk_buff *skb,
			    struct ieee80211_hdr *hdr,
			    struct iwl_fmac_skb_info *info,
			    u16 offload_assist)
{
#if IS_ENABLED(CONFIG_INET)
	u16 mh_len = ieee80211_hdrlen(hdr->frame_control);
	u8 protocol = 0;

	/* Do not compute checksum if already computed by stack */
	if (skb->ip_summed != CHECKSUM_PARTIAL)
		goto out;

	/* We do not expect to be requested to csum stuff we do not support */
	if (WARN_ONCE(!fw_has_capa(&fmac->fw->ucode_capa,
				   IWL_UCODE_TLV_CAPA_CSUM_SUPPORT) ||
		      (skb->protocol != htons(ETH_P_IP) &&
		       skb->protocol != htons(ETH_P_IPV6)),
		      "No support for requested checksum\n")) {
		skb_checksum_help(skb);
		goto out;
	}

	if (skb->protocol == htons(ETH_P_IP)) {
		protocol = ip_hdr(skb)->protocol;
	} else {
#if IS_ENABLED(CONFIG_IPV6)
		struct ipv6hdr *ipv6h =
			(struct ipv6hdr *)skb_network_header(skb);
		unsigned int off = sizeof(*ipv6h);

		protocol = ipv6h->nexthdr;
		while (protocol != NEXTHDR_NONE && ipv6_ext_hdr(protocol)) {
			struct ipv6_opt_hdr *hp;

			/* only supported extension headers */
			if (protocol != NEXTHDR_ROUTING &&
			    protocol != NEXTHDR_HOP &&
			    protocol != NEXTHDR_DEST) {
				skb_checksum_help(skb);
				goto out;
			}

			hp = OPT_HDR(struct ipv6_opt_hdr, skb, off);
			protocol = hp->nexthdr;
			off += ipv6_optlen(hp);
		}
		/* if we get here - protocol now should be TCP/UDP */
#endif
	}

	if (WARN_ON_ONCE(protocol != IPPROTO_TCP && protocol != IPPROTO_UDP)) {
		skb_checksum_help(skb);
		goto out;
	}

	/* enable L4 csum */
	offload_assist |= BIT(TX_CMD_OFFLD_L4_EN);

	/*
	 * Set offset to IP header (snap).
	 * We don't support tunneling so no need to take care of inner header.
	 * Size is in words.
	 */
	offload_assist |= (4 << TX_CMD_OFFLD_IP_HDR);

	/* Do IPv4 csum for AMSDU only (no IP csum for Ipv6) */
	if (skb->protocol == htons(ETH_P_IP) &&
	    (offload_assist & BIT(TX_CMD_OFFLD_AMSDU))) {
		ip_hdr(skb)->check = 0;
		offload_assist |= BIT(TX_CMD_OFFLD_L3_EN);
	}

	/* reset UDP/TCP header csum */
	if (protocol == IPPROTO_TCP)
		tcp_hdr(skb)->check = 0;
	else
		udp_hdr(skb)->check = 0;

	/*
	 * mac header len should include IV, unless the IV is added by the
	 * firmware like in WEP.
	 * In new Tx API, the IV is always added by the firmware.
	 */
	if (!iwl_fmac_has_new_tx_api(fmac) && info->key &&
	    info->key->cipher != IWL_FMAC_CIPHER_WEP40 &&
	    info->key->cipher != IWL_FMAC_CIPHER_WEP104)
		mh_len += info->key->iv_len;
	mh_len /= 2;
	offload_assist |= mh_len << TX_CMD_OFFLD_MH_SIZE;

out:
#endif

	return offload_assist;
}

/*
 * Sets most of the Tx cmd's fields
 */
static void iwl_fmac_set_tx_cmd(struct iwl_fmac *fmac, struct sk_buff *skb,
				struct iwl_tx_cmd *tx_cmd,
				struct iwl_fmac_skb_info *info)
{
	struct ieee80211_hdr *hdr = (void *)skb->data;
	__le16 fc = hdr->frame_control;
	u32 tx_flags = le32_to_cpu(tx_cmd->tx_flags);
	u32 len = skb->len + FCS_LEN;
	u16 offload_assist = 0;
	u8 ac;

	if (ieee80211_is_probe_resp(fc))
		tx_flags |= TX_CMD_FLG_TSF;

	if (ieee80211_has_morefrags(fc))
		tx_flags |= TX_CMD_FLG_MORE_FRAG;

	if (ieee80211_is_data_qos(fc)) {
		u8 *qc = ieee80211_get_qos_ctl(hdr);

		tx_cmd->tid_tspec = qc[0] & 0xf;
		if (*qc & IEEE80211_QOS_CTL_A_MSDU_PRESENT)
			offload_assist |= BIT(TX_CMD_OFFLD_AMSDU);
	} else {
		if (ieee80211_is_data(fc))
			tx_cmd->tid_tspec = IWL_TID_NON_QOS;
		else
			tx_cmd->tid_tspec = IWL_MAX_TID_COUNT;

		tx_flags |= TX_CMD_FLG_SEQ_CTL;
	}

	/* Default to 0 (BE) when tid_spec is set to IWL_TID_NON_QOS */
	if (tx_cmd->tid_tspec < IWL_MAX_TID_COUNT)
		ac = tid_to_ac[tx_cmd->tid_tspec];
	else
		ac = tid_to_ac[0];

	tx_flags |= iwl_fmac_bt_coex_tx_prio(fmac, hdr, ac) <<
		TX_CMD_FLG_BT_PRIO_POS;

	if (ieee80211_is_data(fc) && len > fmac->rts_threshold &&
	    !is_multicast_ether_addr(ieee80211_get_DA(hdr)))
		tx_flags |= TX_CMD_FLG_PROT_REQUIRE;

	/* TODO: check additional conditions when no ack required */
	if (!is_multicast_ether_addr(ieee80211_get_DA(hdr)))
		tx_flags |= TX_CMD_FLG_ACK;

	tx_cmd->tx_flags = cpu_to_le32(tx_flags);
	/* Total # bytes to be transmitted */
	tx_cmd->len = cpu_to_le16((u16)skb->len);
	tx_cmd->life_time = cpu_to_le32(TX_CMD_LIFE_TIME_INFINITE);
	tx_cmd->sta_id = info->sta->sta_id;

	/* padding is inserted later in transport */
	if (ieee80211_hdrlen(fc) % 4 &&
	    !(offload_assist & BIT(TX_CMD_OFFLD_AMSDU)))
		offload_assist |= BIT(TX_CMD_OFFLD_PAD);

	tx_cmd->offload_assist =
		cpu_to_le16(iwl_fmac_tx_csum(fmac, skb, hdr, info,
					     offload_assist));
}

/*
 * Sets the fields in the Tx cmd that are rate related
 */
static void iwl_fmac_set_tx_cmd_rate(struct iwl_fmac *fmac,
				     struct iwl_tx_cmd *tx_cmd,
				     struct iwl_fmac_sta *sta, __le16 fc)
{
	/* Set retry limit on RTS packets */
	tx_cmd->rts_retry_limit = IWL_RTS_DFAULT_RETRY_LIMIT;

	/* Set retry limit on DATA packets and Probe Responses*/
	tx_cmd->data_retry_limit = IWL_DEFAULT_TX_RETRY;

	/*
	 * for data packets, rate info comes from the table inside the fw. This
	 * table is controlled by LINK_QUALITY commands
	 */

	if (ieee80211_is_data(fc) && sta) {
		tx_cmd->initial_rate_index = 0;
		tx_cmd->tx_flags |= cpu_to_le32(TX_CMD_FLG_STA_RATE);
		return;
	}

	WARN_ON_ONCE(1);
	/* TODO: implement all other cases */
}

/*
 * Sets the fields in the Tx cmd that are crypto related
 */
static void iwl_fmac_set_crypto(struct sk_buff *skb,
				struct iwl_tx_cmd *tx_cmd,
				struct iwl_fmac_skb_info *info,
				int hdrlen)
{
	struct iwl_fmac_sta_key *key = info->key;

	if (!key)
		return;

	switch (key->cipher) {
	case IWL_FMAC_CIPHER_GCMP:
	case IWL_FMAC_CIPHER_GCMP_256:
		tx_cmd->sec_ctl |= TX_CMD_SEC_GCMP | TX_CMD_SEC_KEY_FROM_TABLE;
		tx_cmd->key[0] = key->hw_keyidx;
		break;
	case IWL_FMAC_CIPHER_CCMP:
	case IWL_FMAC_CIPHER_CCMP_256:
		tx_cmd->sec_ctl |= TX_CMD_SEC_CCM | TX_CMD_SEC_KEY_FROM_TABLE;
		tx_cmd->key[0] = key->hw_keyidx;
		break;
	case IWL_FMAC_CIPHER_WEP40:
	case IWL_FMAC_CIPHER_WEP104:
		tx_cmd->sec_ctl |= TX_CMD_SEC_WEP | TX_CMD_SEC_KEY_FROM_TABLE |
			((key->keyidx << TX_CMD_SEC_WEP_KEY_IDX_POS) &
			  TX_CMD_SEC_WEP_KEY_IDX_MSK);
		tx_cmd->key[0] = key->hw_keyidx;
		break;
	case IWL_FMAC_CIPHER_TKIP:
		tx_cmd->sec_ctl |= TX_CMD_SEC_TKIP | TX_CMD_SEC_KEY_FROM_TABLE;
		tx_cmd->key[0] = key->hw_keyidx;
		break;
	default:
		WARN_ON_ONCE(1);
	}
}

/*
 * Allocates and sets the Tx cmd the driver data pointers in the skb
 */
static struct iwl_device_cmd *
iwl_fmac_set_tx_params(struct iwl_fmac *fmac, struct sk_buff *skb,
		       struct iwl_fmac_skb_info *info, int hdrlen)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	struct iwl_device_cmd *dev_cmd;

	if (info->dev_cmd)
		return info->dev_cmd;

	dev_cmd = iwl_trans_alloc_tx_cmd(fmac->trans);

	if (unlikely(!dev_cmd))
		return NULL;

	memset(dev_cmd, 0, sizeof(*dev_cmd));
	dev_cmd->hdr.cmd = TX_CMD;

	if (iwl_fmac_has_new_tx_api(fmac)) {
		u32 offload_assist = 0;
		bool amsdu = false;

		if (ieee80211_is_data_qos(hdr->frame_control)) {
			u8 *qc = ieee80211_get_qos_ctl(hdr);

			if (*qc & IEEE80211_QOS_CTL_A_MSDU_PRESENT) {
				offload_assist |= BIT(TX_CMD_OFFLD_AMSDU);
				amsdu = true;
			}
		}

		if (ieee80211_hdrlen(hdr->frame_control) % 4 && !amsdu)
			offload_assist |= BIT(TX_CMD_OFFLD_PAD);

		offload_assist = iwl_fmac_tx_csum(fmac, skb, hdr, info,
						  offload_assist);

		if (fmac->trans->cfg->device_family >=
		    IWL_DEVICE_FAMILY_22560) {
			struct iwl_tx_cmd_gen3 *tx_cmd =
				(void *)dev_cmd->payload;

			tx_cmd->offload_assist = cpu_to_le32(offload_assist);
			tx_cmd->len = cpu_to_le16(skb->len);
			memcpy(tx_cmd->hdr, hdr, hdrlen);
		} else {
			struct iwl_tx_cmd_gen2 *tx_cmd =
				(void *)dev_cmd->payload;

			tx_cmd->offload_assist = cpu_to_le16(offload_assist);
			tx_cmd->len = cpu_to_le16(skb->len);
			memcpy(tx_cmd->hdr, hdr, hdrlen);
		}
	} else {
		struct iwl_tx_cmd *tx_cmd = (void *)dev_cmd->payload;

		iwl_fmac_set_crypto(skb, tx_cmd, info,
				    ieee80211_hdrlen(hdr->frame_control));

		iwl_fmac_set_tx_cmd(fmac, skb, tx_cmd, info);

		iwl_fmac_set_tx_cmd_rate(fmac, tx_cmd, info->sta,
					 hdr->frame_control);

		/* TODO: we'll need more stuff here */
	}

	info->dev_cmd = dev_cmd;

	return dev_cmd;
}

static int iwl_fmac_tx_mpdu(struct iwl_fmac *fmac, struct sk_buff *skb,
			    struct iwl_fmac_skb_info *info)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	struct iwl_fmac_sta *sta = info->sta;
	struct iwl_device_cmd *dev_cmd;
	__le16 fc;
	u16 seq_number = 0;
	u8 tid = 0; /* If no TID is given - use TID 0 */
	u16 txq_id;
	int hdrlen;

	if (WARN_ON_ONCE(!sta))
		return -1;

	if (WARN_ON_ONCE(sta->sta_id == IWL_FMAC_STATION_COUNT))
		return -1;

	fc = hdr->frame_control;
	hdrlen = ieee80211_hdrlen(fc);

	dev_cmd = iwl_fmac_set_tx_params(fmac, skb, info, hdrlen);
	if (!dev_cmd)
		goto drop;

	spin_lock(&sta->lock);

	if (ieee80211_is_data_qos(fc)) {
		u8 *qc = ieee80211_get_qos_ctl(hdr);

		if (!sta->authorized)
			goto drop_unlock_sta;

		tid = qc[0] & IEEE80211_QOS_CTL_TID_MASK;
		if (WARN_ON_ONCE(tid >= IWL_MAX_TID_COUNT))
			goto drop_unlock_sta;

		seq_number = sta->tids[tid].seq_number;
		seq_number &= IEEE80211_SCTL_SEQ;
		hdr->seq_ctrl &= cpu_to_le16(IEEE80211_SCTL_FRAG);
		hdr->seq_ctrl |= cpu_to_le16(seq_number);
		/* TODO: check for AMPDU flag */
	}

	txq_id = sta->tids[tid].txq_id;

	/* Copy MAC header from skb into command buffer */
	if (fmac->trans->cfg->device_family >=
	    IWL_DEVICE_FAMILY_22560) {
		struct iwl_tx_cmd_gen3 *tx_cmd = (void *)dev_cmd->payload;

		memcpy(tx_cmd->hdr, hdr, hdrlen);
	} else if (iwl_fmac_has_new_tx_api(fmac)) {
		struct iwl_tx_cmd_gen2 *tx_cmd = (void *)dev_cmd->payload;

		memcpy(tx_cmd->hdr, hdr, hdrlen);
	} else {
		struct iwl_tx_cmd *tx_cmd = (void *)dev_cmd->payload;

		memcpy(tx_cmd->hdr, hdr, hdrlen);
	}

	/* TODO: TDLS - default to TID 0 for non-QoS packets */

	/* Check if TXQ needs to be allocated */
	if (unlikely(txq_id == IWL_FMAC_INVALID_TXQ_ID)) {
		/* This is a bit hacky - we hold a pointer to the station
		 * which is an RCU protected pointer and enqueue the skb
		 * for later processing. This is fine though since if the
		 * station were to be destroyed, it'd destroy the queue as
		 * well and hence the skb would be freed.
		 */
		iwl_fmac_tx_add_stream(fmac, sta, tid, skb);

		spin_unlock(&sta->lock);

		/* TODO: handle DQA inactive queue re-activations */

		return 0;
	}

	/*
	 * TODO: Keep track of the time of the last frame for this RA/TID for
	 * timeouts and freeing queues
	 */

	IWL_DEBUG_TX(fmac, "TX to [%d|%d] Q:%d - seq: 0x%x\n", sta->sta_id,
		     tid, txq_id, IEEE80211_SEQ_TO_SN(seq_number));

	if (iwl_trans_tx(fmac->trans, skb, dev_cmd, txq_id))
		goto drop_unlock_sta;

	if (ieee80211_is_data_qos(fc) && !ieee80211_has_morefrags(fc))
		sta->tids[tid].seq_number = seq_number + 0x10;

	spin_unlock(&sta->lock);

	/* TODO: Increase pending frames count if this isn't AMPDU */

	/* TODO: keep count of pending frames */

	return 0;

drop_unlock_sta:
	iwl_trans_free_tx_cmd(fmac->trans, dev_cmd);
	spin_unlock(&sta->lock);
drop:
	return -1;
}

#ifdef CONFIG_INET
static int
iwl_fmac_tx_segment(struct sk_buff *skb, unsigned int num_subframes,
		    netdev_features_t netdev_flags,
		    struct sk_buff_head *mpdus_skb)
{
	struct sk_buff *tmp, *next;
	struct ieee80211_hdr *hdr = (void *)skb->data;
	char cb[sizeof(skb->cb)];
	unsigned int payload;
	unsigned int mss = skb_shinfo(skb)->gso_size;
	bool ipv4 = (skb->protocol == htons(ETH_P_IP));
	u16 ip_base_id = ipv4 ? ntohs(ip_hdr(skb)->id) : 0, i = 0;

	skb_shinfo(skb)->gso_size = num_subframes * mss;
	memcpy(cb, skb->cb, sizeof(cb));

	next = skb_gso_segment(skb, netdev_flags);
	skb_shinfo(skb)->gso_size = mss;
	if (WARN_ON_ONCE(IS_ERR(next)))
		return -EINVAL;
	else if (next)
		consume_skb(skb);

	while (next) {
		tmp = next;
		next = tmp->next;

		memcpy(tmp->cb, cb, sizeof(tmp->cb));
		/*
		 * Compute the length of all the data added for the A-MSDU.
		 * This will be used to compute the length to write in the TX
		 * command. We have: SNAP + IP + TCP for n -1 subframes and
		 * ETH header for n subframes.
		 */
		payload = skb_tail_pointer(tmp) - skb_transport_header(tmp) -
			tcp_hdrlen(tmp) + tmp->data_len;

		if (ipv4)
			ip_hdr(tmp)->id = htons(ip_base_id + i * num_subframes);

		if (payload > mss) {
			skb_shinfo(tmp)->gso_size = mss;
		} else {
			if (ieee80211_is_data_qos(hdr->frame_control)) {
				u8 *qc;

				if (ipv4)
					ip_send_check(ip_hdr(tmp));

				qc = ieee80211_get_qos_ctl((void *)tmp->data);
				*qc &= ~IEEE80211_QOS_CTL_A_MSDU_PRESENT;
			}
			skb_shinfo(tmp)->gso_size = 0;
		}

		tmp->prev = NULL;
		tmp->next = NULL;

		__skb_queue_tail(mpdus_skb, tmp);
		i++;
	}

	return 0;
}
#endif

static int iwl_fmac_tx_tso(struct iwl_fmac *fmac, struct sk_buff *skb,
			   struct sk_buff_head *mpdus_skb)
{
#ifdef CONFIG_INET
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	struct iwl_fmac_skb_info *info = (void *)skb->cb;
	struct iwl_fmac_sta *sta = info->sta;
	netdev_features_t netdev_flags = NETIF_F_CSUM_MASK | NETIF_F_SG;
	unsigned int subframes, tcp_payload_len, subf_len, max_len;
	unsigned int mss = skb_shinfo(skb)->gso_size;
	u16 pad;
	u8 tid;

	if (!ieee80211_is_data_qos(hdr->frame_control) ||
	    !sta->amsdu_size || !sta->amsdu_enabled)
		return iwl_fmac_tx_segment(skb, 1, netdev_flags, mpdus_skb);

	/*
	 * Do not build AMSDU for IPv6 with extension headers.
	 * Ask stack to segment and checkum the generated MPDUs for us.
	 */
	if (skb->protocol == htons(ETH_P_IPV6) &&
	    ((struct ipv6hdr *)skb_network_header(skb))->nexthdr !=
	    IPPROTO_TCP) {
		netdev_flags &= ~NETIF_F_CSUM_MASK;
		return iwl_fmac_tx_segment(skb, 1, netdev_flags, mpdus_skb);
	}

	tid = *ieee80211_get_qos_ctl(hdr) & IEEE80211_QOS_CTL_TID_MASK;
	if (WARN_ON_ONCE(tid >= IWL_MAX_TID_COUNT))
		return -EINVAL;

	max_len = iwl_fmac_max_amsdu_size(fmac, sta, tid);

	/* Sub frame header + SNAP + IP header + TCP header + MSS */
	subf_len = sizeof(struct ethhdr) + 8 + skb_transport_header(skb) -
		skb_network_header(skb) + tcp_hdrlen(skb) + mss;
	pad = (4 - subf_len) & 0x3;

	/*
	 * If we have N subframes in the A-MSDU, then the A-MSDU's size is
	 * N * subf_len + (N - 1) * pad.
	 */
	subframes = (max_len + pad) / (subf_len + pad);

	/*
	 * The most severe restriction we can have on the number of subframes
	 * is 8. This can be advertised by the peer through the extended
	 * capabilities. Since we are very unlikely to reach that limit anyway
	 * don't parse the extended capability and just limit ourselves to 8.
	 */
	subframes = min_t(unsigned int, subframes, 8);

	/*
	 * Make sure we have enough TBs for the A-MSDU:
	 *	2 for each subframe
	 *	1 more for each fragment
	 *	1 more for the potential data in the header
	 */
	if ((subframes * 2 + skb_shinfo(skb)->nr_frags + 1) >
	    fmac->trans->max_skb_frags)
		subframes = 1;

	if (subframes > 1)
		*ieee80211_get_qos_ctl(hdr) |= IEEE80211_QOS_CTL_A_MSDU_PRESENT;


	tcp_payload_len = skb_tail_pointer(skb) - skb_transport_header(skb) -
		tcp_hdrlen(skb) + skb->data_len;

	/* This skb fits in one single A-MSDU */
	if (subframes * mss >= tcp_payload_len) {
		__skb_queue_tail(mpdus_skb, skb);
		return 0;
	}

	/* Use segmentation to create SKBs that can fit in an A-MSDU. */
	return iwl_fmac_tx_segment(skb, subframes, netdev_flags, mpdus_skb);
#else /* CONFIG_INET */
	/* Impossible to get TSO without CONFIG_INET */
	WARN_ON(1);
	return -1;
#endif
}

int iwl_fmac_tx_skb(struct iwl_fmac *fmac, struct sk_buff *skb)
{
	struct iwl_fmac_skb_info *info;
	struct sk_buff_head mpdus_skbs;
	unsigned int payload_len;
	int ret;

	__skb_queue_head_init(&mpdus_skbs);

	payload_len = skb_tail_pointer(skb) - skb_transport_header(skb) -
		tcp_hdrlen(skb) + skb->data_len;

	if (!skb_is_gso(skb) || payload_len <= skb_shinfo(skb)->gso_size) {
		__skb_queue_tail(&mpdus_skbs, skb);
	} else {
		ret = iwl_fmac_tx_tso(fmac, skb, &mpdus_skbs);
		if (ret)
			return ret;
	}

	if (WARN_ON(skb_queue_empty(&mpdus_skbs)))
		return -EINVAL;

	while (!skb_queue_empty(&mpdus_skbs)) {
		skb = __skb_dequeue(&mpdus_skbs);

		info = (void *)skb->cb;

		ret = iwl_fmac_tx_mpdu(fmac, skb, info);
		if (ret) {
			__skb_queue_purge(&mpdus_skbs);
			return ret;
		}
	}

	return 0;
}

void iwl_fmac_tx_send_frame(struct iwl_fmac *fmac,
			    struct iwl_fmac_send_frame_notif *send_frame)
{
	struct wiphy *wiphy = wiphy_from_fmac(fmac);
	u16 len = le16_to_cpu(send_frame->len);
	struct wireless_dev *wdev;
	struct iwl_fmac_vif *vif = NULL;
	struct sk_buff *skb;
	size_t copy_size;

	if (WARN_ON(send_frame->proto != cpu_to_be16(ETH_P_PAE)))
		return;

	rcu_read_lock();

	list_for_each_entry(wdev, &wiphy->wdev_list, list) {
		vif = vif_from_wdev(wdev);

		if (vif->id == send_frame->vif_id)
			break;
	}

	if (WARN_ON(!vif ||
		    vif->wdev.iftype != NL80211_IFTYPE_STATION))
		goto out;

	skb = alloc_skb(len + sizeof(send_frame->src_addr) +
			sizeof(send_frame->dst_addr) +
			sizeof(send_frame->proto) +
			wdev->netdev->needed_tailroom +
			wdev->netdev->needed_headroom, GFP_ATOMIC);
	if (!skb)
		goto out;

	skb_reserve(skb, wdev->netdev->needed_headroom);

	copy_size = offsetof(struct iwl_fmac_send_frame_notif, data) -
		offsetof(struct iwl_fmac_send_frame_notif, dst_addr) + len;

	memcpy(skb_put(skb, copy_size), send_frame->dst_addr, copy_size);

	iwl_fmac_dev_start_xmit(skb, wdev->netdev);

out:
	rcu_read_unlock();
}
