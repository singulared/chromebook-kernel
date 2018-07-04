/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2017 Intel Deutschland GmbH
 * Copyright (C) 2018 Intel Corporation
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
 * Copyright(c) 2017 Intel Deutschland GmbH
 * Copyright (C) 2018 Intel Corporation
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
#include "iwl-io.h"
#include "iwl-prph.h"
#include "fmac.h"

void iwl_fmac_reset_ftm_data(struct iwl_fmac *fmac)
{
	fmac->ftm_data.cur_cookie = 0;
	fmac->ftm_data.cur_req_id = 0;
	kfree(fmac->ftm_data.active_targets);
	fmac->ftm_data.active_targets = NULL;
	fmac->ftm_data.num_active_targets = 0;
}

int iwl_fmac_perform_ftm(struct iwl_fmac *fmac, struct iwl_fmac_vif *vif,
			 u64 cookie, struct cfg80211_ftm_request *req)
{
	struct iwl_fmac_ftm_request_cmd cmd = {};
	struct iwl_fmac_ftm_request_resp *resp;
	struct iwl_rx_packet *pkt;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(FMAC_FTM_REQUEST, FMAC_GROUP, 0),
		.flags = CMD_WANT_SKB,
		.dataflags = { IWL_HCMD_DFL_NOCOPY, },
	};
	int i, ret;

	if (req->num_of_targets > IWL_FMAC_FTM_MAX_APS) {
		IWL_ERR(fmac,
			"FTM: number of targets requested isn't supported!\n");
		return -E2BIG;
	}

	if (fmac->ftm_data.cur_cookie) {
		IWL_ERR(fmac,
			"FTM request: device busy! current cookie: %llu\n",
			fmac->ftm_data.cur_cookie);
		return -EBUSY;
	}

	cmd.vif_id = vif->id;
	memcpy(cmd.mac_addr, req->macaddr_template, ETH_ALEN);
	memcpy(cmd.mac_addr_mask, req->macaddr_mask, ETH_ALEN);
	cmd.timeout = req->timeout;
	cmd.report_tsf = req->report_tsf;
	cmd.num_of_targets = req->num_of_targets;
	for (i = 0; i < cmd.num_of_targets; i++) {
		struct cfg80211_ftm_target *req_tgt = &req->targets[i];
		struct iwl_fmac_ftm_target *cmd_tgt = &cmd.targets[i];
		struct cfg80211_chan_def *req_cd = &req_tgt->chan_def;

		memcpy(cmd_tgt->bssid, req_tgt->bssid, ETH_ALEN);
		cmd_tgt->flags = req_tgt->one_sided ?
				 IWL_FMAC_FTM_TARGET_ONE_SIDED : 0;
		cmd_tgt->flags |= req_tgt->asap ? IWL_FMAC_FTM_TARGET_ASAP : 0;
		cmd_tgt->flags |= req_tgt->lci ? IWL_FMAC_FTM_TARGET_LCI : 0;
		cmd_tgt->flags |= req_tgt->civic ? IWL_FMAC_FTM_TARGET_CIVIC :
						   0;
		cmd_tgt->bw = cfg_width_to_iwl_width(req_cd->width);
		cmd_tgt->freq = cpu_to_le16(req_cd->chan->center_freq);
		cmd_tgt->center_freq1 = cpu_to_le16(req_cd->center_freq1);
		cmd_tgt->center_freq2 = cpu_to_le16(req_cd->center_freq2);
		cmd_tgt->burst_period = cpu_to_le16(req_tgt->burst_period);
		cmd_tgt->num_of_bursts_exp = req_tgt->num_of_bursts_exp;
		cmd_tgt->samples_per_burst = req_tgt->samples_per_burst;
		cmd_tgt->retries = req_tgt->retries;
		cmd_tgt->burst_duration = req_tgt->burst_duration;
		cmd_tgt->enable_dyn_ack = fmac->ftm_data.enable_dyn_ack;
		cmd_tgt->algo_type = fmac->ftm_data.algo_type;
	}

	hcmd.len[0] = sizeof(cmd);
	hcmd.data[0] = &cmd;

	mutex_lock(&fmac->mutex);

	fmac->ftm_data.num_active_targets = req->num_of_targets;
	fmac->ftm_data.active_targets =
		kmemdup(req->targets, sizeof(struct cfg80211_ftm_target) *
			req->num_of_targets, GFP_KERNEL);
	if (!fmac->ftm_data.active_targets) {
		ret = -ENOMEM;
		goto out;
	}

	ret = iwl_fmac_send_cmd(fmac, &hcmd);
	if (ret) {
		IWL_ERR(fmac, "Couldn't send FTM request command: %d\n", ret);
		goto out;
	}

	pkt = hcmd.resp_pkt;

	if (WARN_ON(iwl_rx_packet_payload_len(pkt) != sizeof(*resp))) {
		ret = -EIO;
		goto out_free_resp;
	}

	resp = (void *)pkt->data;
	if (resp->status == FMAC_FTM_REQ_INVALID) {
		IWL_ERR(fmac, "FTM request: invalid params!\n");
		ret = -EINVAL;
		goto out_free_resp;
	} else if (resp->status == FMAC_FTM_REQ_BUSY) {
		IWL_ERR(fmac, "FTM request: device busy!\n");
		ret = -EBUSY;
		goto out_free_resp;
	}

	fmac->ftm_data.cur_cookie = cookie;
	fmac->ftm_data.cur_req_id = resp->request_id;
	ret = 0;

out_free_resp:
	iwl_free_resp(&hcmd);
out:
	if (ret)
		iwl_fmac_reset_ftm_data(fmac);
	mutex_unlock(&fmac->mutex);
	return ret;
}

int iwl_fmac_abort_ftm(struct iwl_fmac *fmac, struct iwl_fmac_vif *vif,
		       u64 cookie)
{
	struct iwl_fmac_ftm_abort_cmd cmd;
	u32 status;
	int ret;

	mutex_lock(&fmac->mutex);

	if (cookie != fmac->ftm_data.cur_cookie) {
		IWL_ERR(fmac,
			"FTM abort: Invalid cookie: %llu. current cookie: %llu\n",
			cookie, fmac->ftm_data.cur_cookie);
		ret = -EINVAL;
		goto out;
	}

	cmd.vif_id = vif->id;
	cmd.request_id = fmac->ftm_data.cur_req_id;

	ret = iwl_fmac_send_cmd_pdu_status(fmac, iwl_cmd_id(FMAC_FTM_ABORT,
							    FMAC_GROUP, 0),
					   sizeof(cmd), &cmd, &status);

	if (ret)
		goto out;

	if (WARN_ON(status == FMAC_FTM_ABORT_WRONG_REQ_ID)) {
		ret = -EINVAL;
		goto out;
	}

	iwl_fmac_reset_ftm_data(fmac);

out:
	mutex_unlock(&fmac->mutex);
	return ret;
}

static void iwl_fmac_debug_range_resp(struct iwl_fmac *fmac,
				      struct cfg80211_msrment_response *resp)
{
	u8 num_of_entries = resp->u.ftm.num_of_entries;
	int i;

	IWL_DEBUG_INFO(fmac, "Range response received\n");
	IWL_DEBUG_INFO(fmac, "status: %d, cookie: %lld, num of entries: %hhx\n",
		       resp->status, resp->cookie, num_of_entries);

	for (i = 0; i < num_of_entries; i++) {
		struct cfg80211_ftm_result *res = &resp->u.ftm.entries[i];

		IWL_DEBUG_INFO(fmac, "entry %d\n", i);
		IWL_DEBUG_INFO(fmac, "\tstatus: %d\n", res->status);
		IWL_DEBUG_INFO(fmac, "\tcomplete: %s\n",
			       res->complete ? "true" : "false");
		IWL_DEBUG_INFO(fmac, "\tBSSID: %pM\n", res->target->bssid);
		IWL_DEBUG_INFO(fmac, "\thost time: %llu\n", res->host_time);
		IWL_DEBUG_INFO(fmac, "\ttsf: %llu\n", res->tsf);
		IWL_DEBUG_INFO(fmac, "\tburst index: %hhu\n", res->burst_index);
		IWL_DEBUG_INFO(fmac, "\tmeasurement num: %u\n",
			       res->measurement_num);
		IWL_DEBUG_INFO(fmac, "\tsuccess num: %u\n", res->success_num);
		IWL_DEBUG_INFO(fmac, "\tnum per burst: %hhu\n",
			       res->num_per_burst);
		IWL_DEBUG_INFO(fmac, "\tretry after duration: %u\n",
			       res->retry_after_duration);
		IWL_DEBUG_INFO(fmac, "\tburst duration: %u\n",
			       res->burst_duration);
		IWL_DEBUG_INFO(fmac, "\tnegotiated burst: %u\n",
			       res->negotiated_burst_num);
		IWL_DEBUG_INFO(fmac, "\trssi: %hhd\n", res->rssi);
		IWL_DEBUG_INFO(fmac, "\trssi spread: %hhu\n", res->rssi_spread);
		IWL_DEBUG_INFO(fmac, "\trtt: %lld\n", res->rtt);
		IWL_DEBUG_INFO(fmac, "\trtt var: %llu\n", res->rtt_variance);
		IWL_DEBUG_INFO(fmac, "\trtt spread: %llu\n", res->rtt_spread);
		IWL_DEBUG_INFO(fmac, "\tdistance: %lld\n", res->distance);
		IWL_DEBUG_INFO(fmac, "\tdistance variance: %llu\n",
			       res->distance_variance);
		IWL_DEBUG_INFO(fmac, "\tdistance spread: %llu\n",
			       res->distance_spread);
		IWL_DEBUG_INFO(fmac, "\tfilled: %x\n", res->filled);
	}
}

static inline enum rate_info_bw
iwl_fmac_chan_width_to_rate_info_bw(enum iwl_fmac_chan_width bw)
{
	switch (bw) {
	case IWL_CHAN_WIDTH_20:
	case IWL_CHAN_WIDTH_20_NOHT:
		return RATE_INFO_BW_20;
	case IWL_CHAN_WIDTH_40:
		return RATE_INFO_BW_40;
	case IWL_CHAN_WIDTH_80:
		return RATE_INFO_BW_80;
	default:
		break;
	}

	return -1;
}

static u64 iwl_fmac_ftm_get_host_time(struct iwl_fmac *fmac, u32 msrment_gp2_ts)
{
	u32 diff, curr_gp2 = iwl_read_prph(fmac->trans, DEVICE_SYSTEM_TIME_REG);
	u64 now_from_boot_ns = ktime_get_boot_ns();

	if (curr_gp2 >= msrment_gp2_ts)
		diff = curr_gp2 - msrment_gp2_ts;
	else
		diff = curr_gp2 + (U32_MAX - msrment_gp2_ts + 1);

	return now_from_boot_ns - (u64)diff * 1000;
}

static enum nl80211_ftm_response_status
iwl_fmac_get_target_status(enum iwl_fmac_ftm_target_status status)
{
	switch (status) {
	case FMAC_FTM_RESP_SUCCESS:
		return NL80211_FTM_RESP_SUCCESS;
	case FMAC_FTM_RESP_TARGET_INCAPAB:
		return NL80211_FTM_RESP_TARGET_INCAPAB;
	case FMAC_FTM_RESP_TARGET_BUSY:
		return NL80211_FTM_RESP_TARGET_BUSY;
	case FMAC_FTM_RESP_NOT_MEASURED:
		return NL80211_FTM_RESP_NOT_MEASURED;
	case FMAC_FTM_RESP_TARGET_UNAVAILABLE:
		return NL80211_FTM_RESP_TARGET_UNAVAILABLE;
	case FMAC_FTM_RESP_FAIL:
	default:
		return NL80211_FTM_RESP_FAIL;
	}
}

static struct cfg80211_ftm_target *
iwl_fmac_ftm_find_target_in_request(struct iwl_fmac *fmac, const u8 *bssid)
{
	int i;

	for (i = 0; i < fmac->ftm_data.num_active_targets; i++)
		if (ether_addr_equal_unaligned(
		    fmac->ftm_data.active_targets[i].bssid, bssid))
			return &fmac->ftm_data.active_targets[i];

	return NULL;
}

static enum nl80211_msrment_status
iwl_fmac_get_msrment_status(enum iwl_fmac_msrment_status status)
{
	switch (status) {
	case FMAC_MSRMENT_STATUS_SUCCESS:
		return NL80211_MSRMENT_STATUS_SUCCESS;
	case FMAC_MSRMENT_STATUS_REFUSED:
		return NL80211_MSRMENT_STATUS_REFUSED;
	case FMAC_MSRMENT_STATUS_TIMEOUT:
		return NL80211_MSRMENT_STATUS_TIMEOUT;
	case FMAC_MSRMENT_STATUS_FAIL:
	default:
		return NL80211_MSRMENT_STATUS_FAIL;
	}
}

void iwl_fmac_rx_ftm_response(struct iwl_fmac *fmac,
			      struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_fmac_ftm_response_notify *notif = (void *)pkt->data;
	struct cfg80211_msrment_response res = {0};
	int i;

	lockdep_assert_held(&fmac->mutex);

	if (notif->request_id != fmac->ftm_data.cur_req_id) {
		IWL_ERR(fmac, "Request id mismatch, got %d, active %d\n",
			notif->request_id, fmac->ftm_data.cur_req_id);
		return;
	}

	if (notif->num_of_aps > fmac->ftm_data.num_active_targets) {
		IWL_ERR(fmac, "FTM range response invalid\n");
		return;
	}

	res.cookie = fmac->ftm_data.cur_cookie;
	res.type = NL80211_MSRMENT_TYPE_FTM;
	res.status = iwl_fmac_get_msrment_status(notif->msrment_status);
	res.u.ftm.entries = kzalloc(sizeof(*res.u.ftm.entries) *
				    notif->num_of_aps, GFP_KERNEL);
	if (!res.u.ftm.entries) {
		iwl_fmac_reset_ftm_data(fmac);
		return;
	}

	for (i = 0; i < notif->num_of_aps && i < IWL_FMAC_FTM_MAX_APS; i++) {
		struct cfg80211_ftm_result *result =
			&res.u.ftm.entries[res.u.ftm.num_of_entries];
		struct iwl_fmac_ftm_result_entry *fw_ap = &notif->ap[i];
		struct cfg80211_ftm_target *target;
		u32 timestamp;

		target = iwl_fmac_ftm_find_target_in_request(fmac,
							     fw_ap->bssid);
		if (!target) {
			IWL_WARN(fmac,
				 "Unknown bssid (target #%d) in FTM response, bssid: %pM\n",
				 i, fw_ap->bssid);
			continue;
		}

		result->status = iwl_fmac_get_target_status(fw_ap->status);
		result->target = target;
		timestamp = le32_to_cpu(fw_ap->host_time);
		result->host_time =
			iwl_fmac_ftm_get_host_time(fmac, timestamp);
		result->rssi = fw_ap->rssi;
		result->rssi_spread = fw_ap->rssi_spread;
		result->tx_rate_info.bw =
			iwl_fmac_chan_width_to_rate_info_bw(fw_ap->bw);
		result->rtt = (s32)le32_to_cpu(fw_ap->rtt);
		result->rtt_variance = le32_to_cpu(fw_ap->rtt_variance);
		result->rtt_spread = le32_to_cpu(fw_ap->rtt_spread);
		result->distance = (s32)le32_to_cpu(fw_ap->range);
		result->distance_variance = le32_to_cpu(fw_ap->range_variance);
		result->distance_spread = le32_to_cpu(fw_ap->range_spread);
		result->tsf = le32_to_cpu(fw_ap->tsf);
		if (fw_ap->lci_len && target->lci) {
			result->lci_len = le32_to_cpu(fw_ap->lci_len);
			result->lci = fw_ap->lci_civic;
		}
		if (fw_ap->civic_len && target->civic) {
			result->civic_len = le32_to_cpu(fw_ap->civic_len);
			result->civic = fw_ap->lci_civic +
					le32_to_cpu(fw_ap->lci_len);
		}

#define FTM_RESP_BIT(attr) BIT(NL80211_FTM_RESP_ENTRY_ATTR_##attr)
		result->filled |= FTM_RESP_BIT(HOST_TIME) |
				  FTM_RESP_BIT(TSF) |
				  FTM_RESP_BIT(RSSI) |
				  FTM_RESP_BIT(RSSI_SPREAD) |
				  FTM_RESP_BIT(TX_RATE_INFO) |
				  FTM_RESP_BIT(RTT_VAR) |
				  FTM_RESP_BIT(RTT_SPREAD) |
				  FTM_RESP_BIT(DISTANCE) |
				  FTM_RESP_BIT(DISTANCE_VAR) |
				  FTM_RESP_BIT(DISTANCE_SPREAD);
		if (result->lci)
			result->filled |= FTM_RESP_BIT(LCI);
		if (result->civic)
			result->filled |= FTM_RESP_BIT(CIVIC);
#undef FTM_RESP_BIT
		res.u.ftm.num_of_entries++;
	}

	iwl_fmac_debug_range_resp(fmac, &res);

	cfg80211_measurement_response(wiphy_from_fmac(fmac), &res, GFP_KERNEL);
	kfree(res.u.ftm.entries);

	iwl_fmac_reset_ftm_data(fmac);
}
