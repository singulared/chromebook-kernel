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
#include <linux/firmware.h>
#include <linux/rtnetlink.h>

#include "fmac.h"

struct iwl_fmac_nan_func {
	struct list_head list;
	struct cfg80211_nan_func *func;
};

int iwl_fmac_nan_config_cmd(struct iwl_fmac *fmac, struct iwl_fmac_vif *vif,
			    struct cfg80211_nan_conf *conf)
{
	struct iwl_fmac_cfg_nan_cmd cmd = {
		.master_pref = conf->master_pref,
		.dual = ieee80211_nan_has_band(conf, NL80211_BAND_5GHZ),
	};
	int ret;

	ether_addr_copy(cmd.vif_addr, vif->addr);
	ret = iwl_fmac_send_cmd_pdu(fmac, iwl_cmd_id(FMAC_START_NAN,
						     FMAC_GROUP, 0),
				    0, sizeof(cmd),  &cmd);

	return ret;
}

int iwl_fmac_start_nan(struct wiphy *wiphy, struct wireless_dev *wdev,
		       struct cfg80211_nan_conf *conf)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_wdev(wdev);
	int ret;

	mutex_lock(&fmac->mutex);

	if (!atomic_read(&fmac->open_count)) {
		ret = iwl_fmac_run_rt_fw(fmac);
		if (ret)
			goto out;
	}

	if (vif->id != FMAC_VIF_ID_INVALID) {
		ret = -EALREADY;
		goto out;
	}

	ret = iwl_fmac_nan_config_cmd(fmac, vif, conf);
	if (ret)
		goto out;

	atomic_inc(&fmac->open_count);
	vif->nan.conf = *conf;
	vif->id = FMAC_NAN_DUMMY_VIF_ID;
	fmac->nan_vif = vif;
	INIT_LIST_HEAD(&fmac->nan_vif->nan.funcs);
out:
	if (!atomic_read(&fmac->open_count))
		iwl_fmac_stop_device(fmac);

	mutex_unlock(&fmac->mutex);
	return ret;
}

static void iwl_fmac_nan_data_stop_locked(struct wiphy *wiphy,
					  struct net_device *dev)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_netdev(dev);
	struct iwl_fmac_nan_ndp_state *cur, *tmp;

	lockdep_assert_held(&fmac->mutex);

	if (WARN_ON(!ieee80211_viftype_nan_data(vif->wdev.iftype)))
		return;

	netif_carrier_off(dev);

	list_for_each_entry_safe(cur, tmp, &vif->nan_data.ndps_list, list) {
		struct iwl_fmac_nan_ndp_cmd ndp = {
			.oper = IWL_FMAC_NAN_NDP_OPER_TERM,
			.ndp_id = cur->ndp_id,
		};
		struct iwl_host_cmd hcmd = {
			.id = iwl_cmd_id(FMAC_NAN_NDP, FMAC_GROUP, 0),
			.len[0] = sizeof(ndp),
			.data[0] = &ndp,
		};
		int ret;

		ndp.vif_id = vif->id;
		memcpy(ndp.peer_nmi, cur->peer_nmi, ETH_ALEN);
		memcpy(ndp.init_ndi, cur->init_ndi, ETH_ALEN);

		/* The actual NDP termination should be indicated
		 * asynchronously, but remove the entry here, as it is possible
		 * that the NAN device is being stopped
		 */
		list_del(&cur->list);
		kfree(cur);

		ret = iwl_fmac_send_cmd(fmac, &hcmd);
		WARN(ret, "Couldn't send FMAC_NAN_NDP\n");
	}
}

static void iwl_fmac_nan_stop_all_data(struct wiphy *wiphy)
{
	struct wireless_dev *wdev;

	list_for_each_entry(wdev, &wiphy->wdev_list, list)
		if (ieee80211_viftype_nan_data(wdev->iftype))
			iwl_fmac_nan_data_stop_locked(wiphy, wdev->netdev);
}

void iwl_fmac_stop_nan(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_wdev(wdev);
	struct iwl_fmac_cfg_nan_cmd cmd = {};
	struct iwl_fmac_nan_func *func, *tmp;
	int ret;

	mutex_lock(&fmac->mutex);

	if (vif->id == FMAC_VIF_ID_INVALID || WARN_ON(vif != fmac->nan_vif))
		goto out;

	/* terminate all NDPs */
	iwl_fmac_nan_stop_all_data(wiphy);

	ret = iwl_fmac_send_cmd_pdu(fmac, iwl_cmd_id(FMAC_STOP_NAN,
						     FMAC_GROUP, 0),
				    0, sizeof(cmd),  &cmd);
	if (ret)
		goto out;

	atomic_dec(&fmac->open_count);
	vif->id = FMAC_VIF_ID_INVALID;

	/* clean all the functions */
	list_for_each_entry_safe(func, tmp, &fmac->nan_vif->nan.funcs, list) {
		list_del(&func->list);
		cfg80211_free_nan_func(func->func);
		kfree(func);
	}

	fmac->nan_vif = NULL;

	if (!atomic_read(&fmac->open_count))
		iwl_fmac_stop_device(fmac);

out:
	mutex_unlock(&fmac->mutex);
}

static u8
iwl_fmac_get_match_filter_len(struct cfg80211_nan_func_filter *filters,
			      u8 num_filters)
{
	int i;
	unsigned int len;

	len = num_filters;
	for (i = 0; i < num_filters; i++)
		len += filters[i].len;

	if (WARN_ON_ONCE(len > U8_MAX))
		return 0;

	return len;
}

static void iwl_fmac_copy_filters(struct cfg80211_nan_func_filter *filters,
				  u8 num_filters, u8 *cmd_data)
{
	int i;
	u8 offset = 0;

	for (i = 0; i < num_filters; i++) {
		if (!filters[i].len)
			continue;

		cmd_data[offset++] = filters[i].len;
		memcpy(cmd_data + offset, filters[i].filter, filters[i].len);
		offset += filters[i].len;
	}
}

static enum iwl_fmac_nan_func_type
iwl_fw_nan_func_type(enum nl80211_nan_function_type type)
{
	switch (type) {
	case NL80211_NAN_FUNC_PUBLISH:
		return IWL_FMAC_NAN_DE_FUNC_PUBLISH;
	case NL80211_NAN_FUNC_SUBSCRIBE:
		return IWL_FMAC_NAN_DE_FUNC_SUBSCRIBE;
	case NL80211_NAN_FUNC_FOLLOW_UP:
		return IWL_FMAC_NAN_DE_FUNC_FOLLOW_UP;
	default:
		return IWL_FMAC_NAN_DE_FUNC_NOT_VALID;
	}
}

int iwl_fmac_add_nan_func(struct wiphy *wiphy, struct wireless_dev *wdev,
			  struct cfg80211_nan_func *nan_func)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_add_set_nan_func_cmd *cmd;
	struct iwl_fmac_nan_add_func_res *resp;
	struct iwl_fmac_vif *vif = vif_from_wdev(wdev);
	struct iwl_rx_packet *pkt;
	struct iwl_fmac_nan_func *fmac_func;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(FMAC_ADD_NAN_FUNC, FMAC_GROUP, 0),
		.flags = CMD_WANT_SKB,
	};
	int ret = 0;
	u8 *cmd_data;
	u16 flags = 0;
	u8 tx_filt_len, rx_filt_len;
	u8 data_len = 0;

	if (!ieee80211_viftype_nan(wdev->iftype))
		return -EOPNOTSUPP;

	if (vif->id == FMAC_VIF_ID_INVALID)
		return -ENETDOWN;

	data_len = ALIGN(nan_func->serv_spec_info_len, 4);
	if (nan_func->srf_bf_len)
		data_len += ALIGN(nan_func->srf_bf_len + 1, 4);
	else if (nan_func->srf_num_macs)
		data_len += ALIGN(nan_func->srf_num_macs * ETH_ALEN + 1, 4);

	rx_filt_len = iwl_fmac_get_match_filter_len(nan_func->rx_filters,
						    nan_func->num_rx_filters);

	tx_filt_len = iwl_fmac_get_match_filter_len(nan_func->tx_filters,
						    nan_func->num_tx_filters);

	data_len += ALIGN(rx_filt_len, 4);
	data_len += ALIGN(tx_filt_len, 4);

	cmd = kzalloc(data_len + sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOBUFS;

	cmd->data_len = data_len;
	hcmd.len[0] = data_len + sizeof(*cmd);
	hcmd.data[0] = cmd;

	cmd_data = ((u8 *)cmd) + sizeof(*cmd);

	cmd->type = iwl_fw_nan_func_type(nan_func->type);
	if (cmd->type == IWL_FMAC_NAN_DE_FUNC_NOT_VALID)
		return -EINVAL;

	memcpy(&cmd->service_id, nan_func->service_id, sizeof(cmd->service_id));

	if (nan_func->subscribe_active ||
	    nan_func->publish_type == NL80211_NAN_UNSOLICITED_PUBLISH)
		flags |= IWL_FMAC_NAN_DE_FUNC_FLAG_UNSOLICITED_OR_ACTIVE;

	if (nan_func->close_range)
		flags |= IWL_FMAC_NAN_DE_FUNC_FLAG_CLOSE_RANGE;

	if (nan_func->type == NL80211_NAN_FUNC_FOLLOW_UP ||
	    (nan_func->type == NL80211_NAN_FUNC_PUBLISH &&
	     !nan_func->publish_bcast))
		flags |= IWL_FMAC_NAN_DE_FUNC_FLAG_UNICAST;

	cmd->flags = cpu_to_le16(flags);
	cmd->ttl = cpu_to_le32(nan_func->ttl);
	cmd->serv_info_len = nan_func->serv_spec_info_len;
	if (nan_func->serv_spec_info_len)
		memcpy(cmd_data, nan_func->serv_spec_info,
		       nan_func->serv_spec_info_len);

	cmd_data += ALIGN(cmd->serv_info_len, 4);
	if (nan_func->srf_bf_len) {
		u8 srf_ctl = 0;

		srf_ctl |= SRF_BF_TYPE;
		srf_ctl |= (nan_func->srf_bf_idx << 2) & SRF_BLOOM_FILTER_IDX;
		if (nan_func->srf_include)
			srf_ctl |= SRF_INCLUDE;

		cmd->srf_len = nan_func->srf_bf_len + 1;
		cmd_data[0] = srf_ctl;
		memcpy(cmd_data + 1, nan_func->srf_bf, nan_func->srf_bf_len);
	} else if (nan_func->srf_num_macs) {
		u8 srf_ctl = 0;
		int i;

		if (nan_func->srf_include)
			srf_ctl |= SRF_INCLUDE;

		cmd->srf_len = nan_func->srf_num_macs * ETH_ALEN + 1;
		cmd_data[0] = srf_ctl;

		for (i = 0; i < nan_func->srf_num_macs; i++) {
			memcpy(cmd_data + 1 + i * ETH_ALEN,
			       nan_func->srf_macs[i].addr, ETH_ALEN);
		}
	}

	cmd_data += ALIGN(cmd->srf_len, 4);

	if (rx_filt_len)
		iwl_fmac_copy_filters(nan_func->rx_filters,
				      nan_func->num_rx_filters, cmd_data);
	cmd->rx_filter_len = rx_filt_len;
	cmd_data += ALIGN(cmd->rx_filter_len, 4);

	if (tx_filt_len)
		iwl_fmac_copy_filters(nan_func->tx_filters,
				      nan_func->num_tx_filters, cmd_data);
	cmd->tx_filter_len = tx_filt_len;

	if (nan_func->type == NL80211_NAN_FUNC_FOLLOW_UP) {
		cmd->flw_up_id = nan_func->followup_id;
		cmd->flw_up_req_id = nan_func->followup_reqid;
		memcpy(cmd->flw_up_dest, nan_func->followup_dest.addr,
		       ETH_ALEN);
	}

	mutex_lock(&fmac->mutex);

	fmac_func = kzalloc(sizeof(*fmac_func), GFP_KERNEL);
	if (!fmac_func)
		goto out_free;

	ret = iwl_fmac_send_cmd(fmac, &hcmd);
	if (ret) {
		IWL_ERR(fmac, "Couldn't send FMAC_ADD_NAN_FUNC: %d\n",
			ret);
		goto remove_func;
	}

	pkt = hcmd.resp_pkt;

	if (WARN_ON(iwl_rx_packet_payload_len(pkt) != sizeof(*resp))) {
		ret = -EIO;
		goto out_free_resp;
	}

	resp = (void *)pkt->data;

	if (resp->status == IWL_FMAC_NAN_DE_FUNC_STATUS_INSUFFICIENT_ENTRIES ||
	    resp->status == IWL_FMAC_NAN_DE_FUNC_STATUS_INSUFFICIENT_MEMORY) {
		ret = -ENOBUFS;
		goto out_free_resp;
	}

	if (resp->status != IWL_FMAC_NAN_DE_FUNC_STATUS_SUCCESSFUL) {
		ret = -EIO;
		goto out_free_resp;
	}

	/* save the instance ID and the track the function */
	nan_func->instance_id = resp->instance_id;
	fmac_func->func = nan_func;
	list_add(&fmac_func->list, &fmac->nan_vif->nan.funcs);

	iwl_free_resp(&hcmd);
	goto out_free;

out_free_resp:
	iwl_free_resp(&hcmd);
remove_func:
	kfree(fmac_func);
out_free:
	mutex_unlock(&fmac->mutex);
	kfree(cmd);
	return ret;
}

static struct cfg80211_nan_func *
iwl_fmac_find_nan_func_by_cookie(struct iwl_fmac *fmac, u64 cookie)
{
	struct iwl_fmac_nan_func *func;

	lockdep_assert_held(&fmac->mutex);

	list_for_each_entry(func, &fmac->nan_vif->nan.funcs, list) {
		if (func->func->cookie == cookie)
			return func->func;
	}

	return NULL;
}

void iwl_fmac_del_nan_func(struct wiphy *wiphy, struct wireless_dev *wdev,
			   u64 cookie)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_del_nan_func_cmd cmd = {0};
	struct iwl_fmac_vif *vif = vif_from_wdev(wdev);
	struct cfg80211_nan_func *func;
	int ret;

	if (!ieee80211_viftype_nan(wdev->iftype) ||
	    vif->id == FMAC_VIF_ID_INVALID)
		return;

	mutex_lock(&fmac->mutex);

	func = iwl_fmac_find_nan_func_by_cookie(fmac, cookie);
	if (!func || !func->instance_id)
		goto out;

	cmd.instance_id = func->instance_id;

	ret = iwl_fmac_send_cmd_pdu(fmac,
				    iwl_cmd_id(FMAC_DEL_NAN_FUNC,
					       FMAC_GROUP, 0),
				    0, sizeof(cmd),  &cmd);
	if (ret)
		IWL_ERR(fmac, "Failed to remove NAN func instance_id: %d\n",
			cmd.instance_id);

out:
	mutex_unlock(&fmac->mutex);
}

static struct iwl_fmac_nan_func *
iwl_fmac_find_nan_func_by_inst_id(struct iwl_fmac *fmac, u8 inst_id)
{
	struct iwl_fmac_nan_func *func;

	lockdep_assert_held(&fmac->mutex);

	list_for_each_entry(func, &fmac->nan_vif->nan.funcs, list) {
		if (func->func->instance_id == inst_id)
			return func;
	}

	return NULL;
}

void iwl_fmac_nan_de_term_notif(struct iwl_fmac *fmac,
				struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_fmac_nan_de_term_event *ev = (void *)pkt->data;
	struct iwl_fmac_nan_func *func;
	enum nl80211_nan_func_term_reason nl_reason;
	u64 cookie;

	lockdep_assert_held(&fmac->mutex);

	if (WARN_ON_ONCE(!fmac->nan_vif))
		return;

	switch (ev->reason) {
	case IWL_FMAC_NAN_DE_TERM_TTL_REACHED:
		nl_reason = NL80211_NAN_FUNC_TERM_REASON_TTL_EXPIRED;
		break;
	case IWL_FMAC_NAN_DE_TERM_DRIVER_REQ:
		nl_reason = NL80211_NAN_FUNC_TERM_REASON_USER_REQUEST;
		break;
	case IWL_FMAC_NAN_DE_TERM_FAILURE:
		nl_reason = NL80211_NAN_FUNC_TERM_REASON_ERROR;
		break;
	default:
		WARN_ON_ONCE(1);
		return;
	}

	func = iwl_fmac_find_nan_func_by_inst_id(fmac, ev->instance_id);
	if (WARN_ON(!func))
		return;

	cookie = func->func->cookie;

	list_del(&func->list);
	cfg80211_free_nan_func(func->func);
	kfree(func);
	cfg80211_nan_func_terminated(&fmac->nan_vif->wdev, ev->instance_id,
				     nl_reason, cookie, GFP_KERNEL);
}

static u8 iwl_cfg_nan_func_type(u8 fw_type)
{
	switch (fw_type) {
	case IWL_FMAC_NAN_DE_FUNC_PUBLISH:
		return NL80211_NAN_FUNC_PUBLISH;
	case IWL_FMAC_NAN_DE_FUNC_SUBSCRIBE:
		return NL80211_NAN_FUNC_SUBSCRIBE;
	case IWL_FMAC_NAN_DE_FUNC_FOLLOW_UP:
		return NL80211_NAN_FUNC_FOLLOW_UP;
	default:
		return NL80211_NAN_FUNC_MAX_TYPE + 1;
	}
}

void iwl_fmac_nan_match(struct iwl_fmac *fmac, struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_fmac_nan_disc_evt_notification *ev = (void *)pkt->data;
	struct iwl_fmac_nan_func *func;
	struct cfg80211_nan_match_params match = {0};

	lockdep_assert_held(&fmac->mutex);

	if (!fmac->nan_vif)
		return;

	match.type = iwl_cfg_nan_func_type(ev->type);

	if (WARN_ONCE(match.type > NL80211_NAN_FUNC_MAX_TYPE,
	    "NAN match:Invalid func type %u\n", match.type))
		return;

	func = iwl_fmac_find_nan_func_by_inst_id(fmac, ev->instance_id);
	if (WARN_ON(!func))
		return;

	match.inst_id = ev->instance_id;
	match.peer_inst_id = ev->peer_instance;
	match.addr = ev->peer_mac_addr;
	match.info = ev->buf;
	match.info_len = ev->service_info_len;
	match.cookie = func->func->cookie;

	cfg80211_nan_match(&fmac->nan_vif->wdev, &match, GFP_KERNEL);
}

void iwl_fmac_term_all_nan_func(struct iwl_fmac *fmac,
				struct wireless_dev *wdev,
				enum nl80211_nan_func_term_reason reason)
{
	struct iwl_fmac_vif *vif = vif_from_wdev(wdev);
	struct iwl_fmac_nan_func *func, *tmp;

	lockdep_assert_held(&fmac->mutex);

	if (vif->id == FMAC_VIF_ID_INVALID)
		return;

	list_for_each_entry_safe(func, tmp, &fmac->nan_vif->nan.funcs, list) {
		list_del(&func->list);

		cfg80211_nan_func_terminated(wdev, func->func->instance_id,
					     reason, func->func->cookie,
					     GFP_KERNEL);

		cfg80211_free_nan_func(func->func);
		kfree(func);
	}
}

int iwl_fmac_nan_ndp(struct wiphy *wiphy, struct wireless_dev *wdev,
		     struct cfg80211_nan_ndp_params *params)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);
	struct iwl_fmac_vif *vif = vif_from_wdev(wdev);
	struct iwl_fmac_nan_ndp_cmd *ndp;
	struct iwl_fmac_nan_ndp_state *ndp_state = NULL;
	struct iwl_fmac_nan_ndp_resp *resp;
	struct iwl_rx_packet *pkt;
	struct iwl_host_cmd hcmd = {
		.id = iwl_cmd_id(FMAC_NAN_NDP, FMAC_GROUP, 0),
		.flags = CMD_WANT_SKB,
	};
	u8 data_len;
	int ret = 0;

	if (!fmac->nan_vif || fmac->nan_vif->id == FMAC_VIF_ID_INVALID)
		return -ENETDOWN;

	data_len = ALIGN(params->serv_spec_info_len, 4);
	ndp = kzalloc(data_len + sizeof(*ndp), GFP_KERNEL);
	if (!ndp)
		return -ENOBUFS;

	hcmd.len[0] = data_len + sizeof(*ndp);
	hcmd.data[0] = ndp;

	/* Note that the vif_id can be either the NAN Device id or the
	 * NAN Data interface id. Its the responsibility of the FW to handle the
	 * command based on the vif_id and the other parameters.
	 */
	ndp->vif_id = vif->id;
	memcpy(ndp->peer_nmi, params->peer_nmi, ETH_ALEN);
	memcpy(ndp->init_ndi, params->init_ndi, ETH_ALEN);

	if (params->oper == NL80211_NAN_NDP_OPER_REQ) {
		ndp->oper = IWL_FMAC_NAN_NDP_OPER_REQ;
		ndp->pub_inst_id = params->pub_inst_id;

		/* allocate the structure here, to avoid the need to stop the
		 * NDP if allocation later fails.
		 */
		ndp_state = kzalloc(sizeof(*ndp_state), GFP_KERNEL);
		if (!ndp_state) {
			ret = -ENOBUFS;
			goto out_free_ndp;
		}
	} else {
		ndp->ndp_id = params->ndp_id;

		if (params->oper == NL80211_NAN_NDP_OPER_TERM) {
			ndp->oper = IWL_FMAC_NAN_NDP_OPER_TERM;
			goto send_cmd;
		}

		ndp->oper = IWL_FMAC_NAN_NDP_OPER_RES;
		ndp->rejected = params->rejected;
		if (ndp->rejected) {
			ndp->reason_code = params->reason_code;
			goto send_cmd;
		}

		memcpy(ndp->resp_ndi, params->resp_ndi, ETH_ALEN);
	}

	/* QoS parameters */
	ndp->min_slots = params->qos.min_slots;
	ndp->max_latency = cpu_to_le16(params->qos.max_latency);

	/* Security parameters */
	if (params->sec.cipher_suite_ids) {
		if (params->sec.cipher_suite_ids ==
		    NL80211_NAN_CS_ID_SK_CCM_128)
			ndp->csid = IWL_FMAC_NAN_CSID_SK_128;
		else if (params->sec.cipher_suite_ids ==
			 NL80211_NAN_CS_ID_SK_GCM_256)
			ndp->csid = IWL_FMAC_NAN_CSID_SK_256;

		memcpy(ndp->pmk, params->pmk, IWL_FMAC_NAN_PMK_LEN);
		ndp->flags |= IWL_FMAC_NAN_NDP_FLAGS_PMK;

		/* currently support only cipher context type=PMKID which would
		 * be calculated in FW, so no need to copy it even if given.
		 */
	}

	if (params->serv_spec_info_len) {
		ndp->ssi_len = cpu_to_le16(params->serv_spec_info_len);
		memcpy(ndp + sizeof(*ndp), params->serv_spec_info,
		       params->serv_spec_info_len);
	}

send_cmd:
	mutex_lock(&fmac->mutex);
	ret = iwl_fmac_send_cmd(fmac, &hcmd);
	if (ret) {
		IWL_ERR(fmac, "Couldn't send FMAC_NAN_NDP: %d\n",
			ret);
		mutex_unlock(&fmac->mutex);
		goto out_free_ndp;
	}

	pkt = hcmd.resp_pkt;
	if (WARN_ON(iwl_rx_packet_payload_len(pkt) != sizeof(*resp))) {
		ret = -EIO;
		goto out_free_resp;
	}

	resp = (void *)pkt->data;

	if (resp->status == IWL_FMAC_NAN_NDP_STATUS_FAILURE) {
		ret = -EIO;
		goto out_free_resp;
	}

	if (ndp->oper == IWL_FMAC_NAN_NDP_OPER_REQ) {
		params->ndp_id = resp->ndp_id;

		/* update the ndp_state in order to track active NDPs */
		ndp_state->ndp_id = resp->ndp_id;
		ndp_state->active = false;
		memcpy(ndp_state->peer_nmi, ndp->peer_nmi, ETH_ALEN);
		memcpy(ndp_state->init_ndi, ndp->init_ndi, ETH_ALEN);
		list_add(&ndp_state->list, &vif->nan_data.ndps_list);
	}
out_free_resp:
	mutex_unlock(&fmac->mutex);
	iwl_free_resp(&hcmd);
out_free_ndp:
	kfree(ndp);
	if (ret)
		kfree(ndp_state);
	return ret;
}

void iwl_fmac_nan_data_stop(struct wiphy *wiphy,
			    struct net_device *dev)
{
	struct iwl_fmac *fmac = iwl_fmac_from_wiphy(wiphy);

	mutex_lock(&fmac->mutex);
	iwl_fmac_nan_data_stop_locked(wiphy, dev);
	mutex_unlock(&fmac->mutex);
}

static void
iwl_fmac_nan_ndp_update_state(struct iwl_fmac *fmac,
			      struct cfg80211_nan_ndp_params *params,
			      struct iwl_fmac_vif *vif)
{
	struct iwl_fmac_nan_ndp_state *cur;
	bool found = false;

	lockdep_assert_held(&fmac->mutex);

	/* Do not nothing. The state would be updated in the response
	 * notification.
	 */
	if (params->oper == NL80211_NAN_NDP_OPER_REQ)
		return;

	if (WARN_ON(!ieee80211_viftype_nan_data(vif->wdev.iftype)))
		return;

	list_for_each_entry(cur, &vif->nan_data.ndps_list, list) {
		if (ether_addr_equal(cur->peer_nmi, params->peer_nmi) &&
		    ether_addr_equal(cur->init_ndi, params->init_ndi) &&
		    cur->ndp_id == params->ndp_id) {
			found = true;
			break;
		}
	}

	/* handle termination or rejected notification */
	if (params->oper == NL80211_NAN_NDP_OPER_TERM ||
	    (params->oper == NL80211_NAN_NDP_OPER_RES && params->rejected)) {
		if (found) {
			list_del(&cur->list);
			kfree(cur);
		}

		list_for_each_entry(cur, &vif->nan_data.ndps_list, list) {
			if (cur->active)
				return;
		}

		netif_carrier_off(vif->wdev.netdev);
		return;
	}

	/* A response notification can be triggered either as an initiator, in
	 * which case the entry was already added for tracking, and as a
	 * responder in which case a new entry needs to be created.
	 */
	if (!found) {
		cur = kzalloc(sizeof(*cur), GFP_KERNEL);
		if (WARN_ON(!cur))
			return;

		cur->ndp_id = params->ndp_id;
		memcpy(cur->peer_nmi, params->peer_nmi, ETH_ALEN);
		memcpy(cur->init_ndi, params->init_ndi, ETH_ALEN);
		list_add(&cur->list, &vif->nan_data.ndps_list);
	}

	memcpy(cur->resp_ndi, params->resp_ndi, ETH_ALEN);
	cur->active = true;

	netif_carrier_on(vif->wdev.netdev);
}

void iwl_fmac_nan_ndp_notify(struct iwl_fmac *fmac,
			     struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_fmac_nan_ndp_cmd *ndp = (void *)pkt->data;
	struct cfg80211_nan_ndp_params *params;
	struct wiphy *wiphy = wiphy_from_fmac(fmac);
	struct wireless_dev *wdev;
	struct iwl_fmac_vif *vif = NULL;

	lockdep_assert_held(&fmac->mutex);

	if (!fmac->nan_vif)
		return;

	list_for_each_entry(wdev, &wiphy->wdev_list, list) {
		vif = vif_from_wdev(wdev);
		if (vif->id == ndp->vif_id)
			break;
		vif = NULL;
	}

	/* NAN device interface does not have an ID assigned by FW */
	if (!vif)
		vif = fmac->nan_vif;

	params = kzalloc(sizeof(*params) + le16_to_cpu(ndp->ssi_len),
			 GFP_KERNEL);
	if (!params)
		return;

	memcpy(params->peer_nmi, ndp->peer_nmi, ETH_ALEN);
	memcpy(params->init_ndi, ndp->init_ndi, ETH_ALEN);
	params->ndp_id = ndp->ndp_id;

	switch (ndp->oper) {
	case IWL_FMAC_NAN_NDP_OPER_REQ:
		params->oper = NL80211_NAN_NDP_OPER_REQ;
		params->pub_inst_id = ndp->pub_inst_id;
		break;
	case IWL_FMAC_NAN_NDP_OPER_RES:
		params->oper = NL80211_NAN_NDP_OPER_RES;
		params->rejected = ndp->rejected;
		if (params->rejected) {
			params->reason_code = ndp->reason_code;
			goto notify;
		} else {
			memcpy(params->resp_ndi, ndp->resp_ndi, ETH_ALEN);
		}
		break;
	case IWL_FMAC_NAN_NDP_OPER_TERM:
		params->oper = NL80211_NAN_NDP_OPER_TERM;
		goto notify;
	default:
		WARN(1, "NAN: Invalid NAN NDP OPER\n");
		kfree(params);
		return;
	}

	params->qos.min_slots = ndp->min_slots;
	params->qos.max_latency = le16_to_cpu(ndp->max_latency);

	switch (ndp->csid) {
	case IWL_FMAC_NAN_CSID_NONE:
		break;
	case IWL_FMAC_NAN_CSID_SK_128:
		params->sec.cipher_suite_ids = NL80211_NAN_CS_ID_SK_CCM_128;
		break;
	case IWL_FMAC_NAN_CSID_SK_256:
		params->sec.cipher_suite_ids = NL80211_NAN_CS_ID_SK_GCM_256;
	default:
		WARN(1, "NAN: Invalid NAN Cipher Suite ID\n");
		kfree(params);
		return;
	}

	if (ndp->flags & IWL_FMAC_NAN_NDP_FLAGS_PMKID) {
		params->sec.n_ctx_ids = 1;
		params->sec.ctx_ids =
			kzalloc(sizeof(struct cfg80211_nan_sec_ctx_id),
				GFP_KERNEL);
		if (WARN_ON(!params->sec.ctx_ids)) {
			kfree(params);
			return;
		}

		params->sec.ctx_ids[0].data = ndp->pmkid;
		params->sec.ctx_ids[0].type = NL80211_NAN_SEC_CTX_TYPE_PMKID;
		params->sec.ctx_ids[0].len = IWL_FMAC_NAN_PMKID_LEN;
	}

	if (ndp->ssi_len) {
		params->serv_spec_info_len =
			le16_to_cpu(ndp->ssi_len);
		params->serv_spec_info = ndp->ssi;
	}

notify:
	iwl_fmac_nan_ndp_update_state(fmac, params, vif);
	cfg80211_nan_ndp_notify(&fmac->nan_vif->wdev, params, GFP_KERNEL);
	kfree(params->sec.ctx_ids);
	kfree(params);
}

void iwl_fmac_nan_cluster_notify(struct iwl_fmac *fmac,
				 struct iwl_rx_cmd_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_fmac_nan_cluster_notif *notif = (void *)pkt->data;
	struct wiphy *wiphy = wiphy_from_fmac(fmac);
	struct wireless_dev *wdev;

	if (!fmac->nan_vif)
		return;

	rcu_read_lock();

	list_for_each_entry_rcu(wdev, &wiphy->wdev_list, list) {
		struct iwl_fmac_vif *vif = vif_from_wdev(wdev);

		if (!ieee80211_viftype_nan_data(wdev->iftype))
			continue;

		/*
		 * this is not atomic, but for NAN, cluster ID can change all
		 * the time and as Tx/Rx flows need to handle such changes, the
		 * atomicity is not really required.
		 */
		vif->nan.cluster_id[0] = notif->id[0];
		vif->nan.cluster_id[1] = notif->id[1];
	}

	rcu_read_unlock();
}
