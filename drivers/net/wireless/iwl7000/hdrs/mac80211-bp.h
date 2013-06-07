#include <linux/if_ether.h>
#include <net/cfg80211.h>

#define IEEE80211_MAX_CHAINS 4

/* some backport stuff for mac80211 from cfg80211/... */
struct cfg80211_wowlan_wakeup {
	bool disconnect, magic_pkt, gtk_rekey_failure,
	     eap_identity_req, four_way_handshake,
	     rfkill_release, packet_80211,
	     tcp_match, tcp_connlost, tcp_nomoretokens;
	s32 pattern_idx;
	u32 packet_present_len, packet_len;
	const void *packet;
};

/* nl80211 */
struct nl80211_wowlan_tcp_data_token_feature {
	__u32 min_len, max_len, bufsize;
};


struct wiphy_wowlan_tcp_support {
	const struct nl80211_wowlan_tcp_data_token_feature *tok;
	u32 data_payload_max;
	u32 data_interval_max;
	u32 wake_payload_max;
	bool seq;
};

struct nl80211_wowlan_tcp_data_seq {
	__u32 start, offset, len;
};

struct nl80211_wowlan_tcp_data_token {
	__u32 offset, len;
	__u8 token_stream[];
};

struct cfg80211_wowlan_tcp {
	struct socket *sock;
	__be32 src, dst;
	u16 src_port, dst_port;
	u8 dst_mac[ETH_ALEN];
	int payload_len;
	const u8 *payload;
	struct nl80211_wowlan_tcp_data_seq payload_seq;
	u32 data_interval;
	u32 wake_len;
	const u8 *wake_data, *wake_mask;
	u32 tokens_size;
	/* must be last, variable member */
	struct nl80211_wowlan_tcp_data_token payload_tok;
};

enum nl80211_mesh_power_mode {
	__DUMMY_MESH_POWER_MODE,
};

#define MONITOR_FLAG_ACTIVE 0
#define NL80211_FEATURE_USERSPACE_MPM 0

static inline u32
ieee80211_mandatory_rates(struct ieee80211_supported_band *sband)
{
	struct ieee80211_rate *bitrates;
	u32 mandatory_rates = 0;
	enum ieee80211_rate_flags mandatory_flag;
	int i;

	if (WARN_ON(!sband))
		return 1;

	if (sband->band == IEEE80211_BAND_2GHZ)
		mandatory_flag = IEEE80211_RATE_MANDATORY_B;
	else
		mandatory_flag = IEEE80211_RATE_MANDATORY_A;

	bitrates = sband->bitrates;
	for (i = 0; i < sband->n_bitrates; i++)
		if (bitrates[i].flags & mandatory_flag)
			mandatory_rates |= BIT(i);
	return mandatory_rates;
}

static inline void netdev_attach_ops(struct net_device *dev,
				     const struct net_device_ops *ops)
{
	dev->netdev_ops = ops;
}

#define IEEE80211_RADIOTAP_MCS_STBC_SHIFT	5

static inline bool
ieee80211_operating_class_to_band(u8 operating_class,
				  enum ieee80211_band *band)
{
	switch (operating_class) {
	case 112:
	case 115 ... 127:
		*band = IEEE80211_BAND_5GHZ;
		return true;
	case 81:
	case 82:
	case 83:
	case 84:
		*band = IEEE80211_BAND_2GHZ;
		return true;
	case 180:
		*band = IEEE80211_BAND_60GHZ;
		return true;
	}

	/* stupid compiler */
	*band = IEEE80211_BAND_2GHZ;

	return false;
}

#define ASSOC_REQ_DISABLE_VHT 0

static inline void
cfg80211_report_wowlan_wakeup(struct wireless_dev *wdev,
			      struct cfg80211_wowlan_wakeup *wakeup, gfp_t gfp)
{
}
