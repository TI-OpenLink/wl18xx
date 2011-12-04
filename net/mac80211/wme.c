/*
 * Copyright 2004, Instant802 Networks, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/if_arp.h>
#include <linux/types.h>
#include <net/ip.h>
#include <net/pkt_sched.h>

#include <net/mac80211.h>
#include "ieee80211_i.h"
#include "wme.h"
#include "driver-ops.h"

/* Default mapping in classifier to work with default
 * queue setup.
 */
const int ieee802_1d_to_ac[8] = {
	IEEE80211_AC_BE,
	IEEE80211_AC_BK,
	IEEE80211_AC_BK,
	IEEE80211_AC_BE,
	IEEE80211_AC_VI,
	IEEE80211_AC_VI,
	IEEE80211_AC_VO,
	IEEE80211_AC_VO
};

const u8 tid_couple[8] = {3, 2, 1, 0, 5, 4, 7, 6};

static int wme_downgrade_ac(struct sk_buff *skb)
{
	switch (skb->priority) {
	case 6:
	case 7:
		skb->priority = 5; /* VO -> VI */
		return 0;
	case 4:
	case 5:
		skb->priority = 3; /* VI -> BE */
		return 0;
	case 0:
	case 3:
		skb->priority = 2; /* BE -> BK */
		return 0;
	default:
		return -1;
	}
}

#define MSEC_UPGRADE_DELTA (1000)

u16 ieee80211_downgrade_queue(struct ieee80211_sub_if_data *sdata,
			      struct sk_buff *skb)
{
	int ac = ieee802_1d_to_ac[skb->priority];
	/* in case we are a client verify acm is not set for this ac */
	while (unlikely(sdata->wmm_acm & BIT(skb->priority)) &&
			(!(sdata->wmm_admitted & BIT(skb->priority)) ||
			 sdata->wmm_ac_downgraded & BIT(ac))) {

		u32 time_diff = (jiffies_to_msecs(jiffies) - sdata->downgrade_ts[ac]);
		if ((sdata->wmm_ac_downgraded & BIT(ac)) &&
				( time_diff > MSEC_UPGRADE_DELTA)) {
			sdata->wmm_ac_downgraded &= ~BIT(ac);
			continue;
		}
		if (wme_downgrade_ac(skb)) {
			/*
			 * This should not really happen. The AP has marked all
			 * lower ACs to require admission control which is not
			 * a reasonable configuration. Allow the frame to be
			 * transmitted using AC_BK as a workaround.
			 */
			break;
		}
	}

	/* look up which queue to use for frames with this 1d tag */
	return ieee802_1d_to_ac[skb->priority];
}

/* Indicate which queue to use for this fully formed 802.11 frame */
u16 ieee80211_select_queue_80211(struct ieee80211_sub_if_data *sdata,
				 struct sk_buff *skb,
				 struct ieee80211_hdr *hdr)
{
	struct ieee80211_local *local = sdata->local;
	u8 *p;

	if (local->hw.queues < IEEE80211_NUM_ACS)
		return 0;

	if (!ieee80211_is_data(hdr->frame_control)) {
		skb->priority = 7;
		return ieee802_1d_to_ac[skb->priority];
	}
	if (!ieee80211_is_data_qos(hdr->frame_control)) {
		skb->priority = 0;
		return ieee802_1d_to_ac[skb->priority];
	}

	p = ieee80211_get_qos_ctl(hdr);
	skb->priority = *p & IEEE80211_QOS_CTL_TAG1D_MASK;

	return ieee80211_downgrade_queue(sdata, skb);
}

/* Indicate which queue to use. */
u16 ieee80211_select_queue(struct ieee80211_sub_if_data *sdata,
			   struct sk_buff *skb)
{
	struct ieee80211_local *local = sdata->local;
	struct sta_info *sta = NULL;
	const u8 *ra = NULL;
	bool qos = false;

	if (local->hw.queues < IEEE80211_NUM_ACS || skb->len < 6) {
		skb->priority = 0; /* required for correct WPA/11i MIC */
		return 0;
	}

	rcu_read_lock();
	switch (sdata->vif.type) {
	case NL80211_IFTYPE_AP_VLAN:
		sta = rcu_dereference(sdata->u.vlan.sta);
		if (sta) {
			qos = test_sta_flag(sta, WLAN_STA_WME);
			break;
		}
	case NL80211_IFTYPE_AP:
		ra = skb->data;
		break;
	case NL80211_IFTYPE_WDS:
		ra = sdata->u.wds.remote_addr;
		break;
#ifdef CONFIG_MAC80211_MESH
	case NL80211_IFTYPE_MESH_POINT:
		qos = true;
		break;
#endif
	case NL80211_IFTYPE_STATION:
		ra = sdata->u.mgd.bssid;
		break;
	case NL80211_IFTYPE_ADHOC:
		ra = skb->data;
		break;
	default:
		break;
	}

	if (!sta && ra && !is_multicast_ether_addr(ra)) {
		sta = sta_info_get(sdata, ra);
		if (sta)
			qos = test_sta_flag(sta, WLAN_STA_WME);
	}
	rcu_read_unlock();

	if (!qos) {
		skb->priority = 0; /* required for correct WPA/11i MIC */
		return IEEE80211_AC_BE;
	}

	/* use the data classifier to determine what 802.1d tag the
	 * data frame has */
	skb->priority = cfg80211_classify8021d(skb);

	return ieee80211_downgrade_queue(sdata, skb);
}

void ieee80211_set_qos_hdr(struct ieee80211_sub_if_data *sdata,
			   struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (void *)skb->data;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

	/* Fill in the QoS header if there is one. */
	if (ieee80211_is_data_qos(hdr->frame_control)) {
		u8 *p = ieee80211_get_qos_ctl(hdr);
		u8 ack_policy, tid;

		tid = skb->priority & IEEE80211_QOS_CTL_TAG1D_MASK;

		/* preserve EOSP bit */
		ack_policy = *p & IEEE80211_QOS_CTL_EOSP;

		if (is_multicast_ether_addr(hdr->addr1) ||
		    sdata->noack_map & BIT(tid)) {
			ack_policy |= IEEE80211_QOS_CTL_ACK_POLICY_NOACK;
			info->flags |= IEEE80211_TX_CTL_NO_ACK;
		}

		/* qos header is 2 bytes */
		*p++ = ack_policy | tid;
		*p = ieee80211_vif_is_mesh(&sdata->vif) ?
			(IEEE80211_QOS_CTL_MESH_CONTROL_PRESENT >> 8) : 0;
	}
}

static int
ieee80211_build_tspec_ie(struct ieee80211_tspec_params *tspec_params, u8* buff)
{
	struct ieee80211_tspec_ie *tspec;
	__le16 tsinfo = 0;

	*buff++ = WLAN_EID_VENDOR_SPECIFIC;
	*buff++ = WMM_TSPEC_IE_LEN -2;

	tspec = (struct ieee80211_tspec_ie*)buff;
	memset(tspec, 0, sizeof(*tspec));
	tspec->oui[0] = 0x00; tspec->oui[1] = 0x50; tspec->oui[2] = 0xf2;
	tspec->oui_type = 2;
	tspec->oui_subtype = 2;
	tspec->version = 1;

	tsinfo = (tspec_params->tid & IEEE80211_WMM_IE_TSPEC_TID_MASK) <<
			IEEE80211_WMM_IE_TSPEC_TID_SHIFT;
	tsinfo |= (tspec_params->direction & IEEE80211_WMM_IE_TSPEC_DIR_MASK) <<
			IEEE80211_WMM_IE_TSPEC_DIR_SHIFT;
	tsinfo |= (1 << 7);
	tsinfo |= (tspec_params->psb & IEEE80211_WMM_IE_TSPEC_PSB_MASK) <<
			IEEE80211_WMM_IE_TSPEC_PSB_SHIFT;
	tsinfo |= (tspec_params->user_priority & IEEE80211_WMM_IE_TSPEC_UP_MASK) <<
			IEEE80211_WMM_IE_TSPEC_UP_SHIFT;

	tspec->tsinfo = tsinfo;
	tspec->nominal_msdu = tspec_params->nominal_msdu_size;
	tspec->max_msdu = tspec_params->maximum_msdu_size;
	tspec->min_service_int = tspec_params->minimum_service_interval;
	tspec->max_service_int = tspec_params->maximum_service_interval;
	tspec->inactivity_int = tspec_params->inactivity_interval;
	tspec->suspension_int = tspec_params->suspension_interval;
	tspec->service_start_time = tspec_params->service_start_time;
	tspec->min_data_rate = tspec_params->minimum_data_rate;
	tspec->mean_data_rate = tspec_params->mean_data_rate;
	tspec->peak_data_rate = tspec_params->peak_data_rate;
	tspec->max_burst_size = tspec_params->maximum_burst_size;
	tspec->delay_bound = tspec_params->delay_bound;
	tspec->min_phy_rate = tspec_params->minimum_phy_rate;
	tspec->sba = tspec_params->surplus_bandwidth_allowance;

	return WMM_TSPEC_IE_LEN;
}

int ieee80211_delts_request(struct ieee80211_sub_if_data *sdata,
		u8 status_code, struct ieee80211_tspec_params *tspec_params) {
	struct ieee80211_local *local = sdata->local;
	struct sk_buff *skb;
	struct ieee80211_mgmt *mgmt;
	u8 *tspec_buff;

	if (!sdata->local->ops->set_wme_medium_time)
		return -EOPNOTSUPP;

	if (!(sdata->wmm_acm & BIT(tspec_params->tid)) ||
	    !(sdata->wmm_admitted & BIT(tspec_params->tid)))
		return -EINVAL;

	skb = dev_alloc_skb(local->hw.extra_tx_headroom +
			sizeof(*mgmt) + 2 + sizeof(struct ieee80211_tspec_ie));
	if (!skb)
		return -ENOMEM;

	skb_reserve(skb, local->hw.extra_tx_headroom);
	mgmt = (struct ieee80211_mgmt *) skb_put(skb, 24 + 1 +
			sizeof(mgmt->u.action.u.wme_action));
	memset(mgmt, 0, 24);
	memcpy(mgmt->da, sdata->u.mgd.bssid, ETH_ALEN);
	memcpy(mgmt->sa, sdata->vif.addr, ETH_ALEN);
	memcpy(mgmt->bssid, sdata->u.mgd.bssid, ETH_ALEN);
	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
			IEEE80211_STYPE_ACTION);

	mgmt->u.action.category = WLAN_CATEGORY_WMM;
	mgmt->u.action.u.wme_action.action_code =
			WLAN_WMM_ACTION_CODE_DELTS;
	mgmt->u.action.u.wme_action.dialog_token = 0xa0;
	mgmt->u.action.u.wme_action.status_code = status_code;

	tspec_buff = skb_put(skb, 2 + sizeof(struct ieee80211_tspec_ie));
	ieee80211_build_tspec_ie(tspec_params, tspec_buff);
	ieee80211_tx_skb(sdata, skb);

	sdata->local->ops->set_wme_medium_time(&sdata->local->hw,
				&sdata->vif, 0, 0,
				ieee802_1d_to_ac[tspec_params->tid]);

	sdata->wmm_admitted &= ~BIT(tspec_params->tid);
	sdata->wmm_admitted &= ~BIT(tid_couple[tspec_params->tid]);
	sdata->wmm_ac_downgraded &=
			~BIT(ieee802_1d_to_ac[tspec_params->tid]);

	ieee80211_tspec_done(sdata, (u8*)mgmt,
			 24 + 1 + sizeof(mgmt->u.action.u.wme_action)
			 + 2 + sizeof(struct ieee80211_tspec_ie));
	return 0;
}

void ieee80211_tspec_done(struct ieee80211_sub_if_data *sdata,
						  u8* buf, u8 len)
{
	if (!buf || !len)
		return;

	cfg80211_send_rx_wme(sdata->dev, buf, len);
}

static void ieee80211_send_addts_tspec(struct ieee80211_sub_if_data *sdata,
		struct ieee80211_tspec_params *tspec_params,
		u8* extra_ies, u8 extra_ies_len) {

	struct ieee80211_local *local = sdata->local;
	struct sk_buff *skb;
	struct ieee80211_mgmt *mgmt;
	u8 *tspec_buff;
	u8 *ie;

	skb = dev_alloc_skb(
			local->hw.extra_tx_headroom + sizeof(*mgmt)
					+ 2 + sizeof(struct ieee80211_tspec_ie) +
					extra_ies_len);
	if (!skb)
		return;

	skb_reserve(skb, local->hw.extra_tx_headroom);

	mgmt = (struct ieee80211_mgmt *) skb_put(skb,
			24 + 1 + sizeof(mgmt->u.action.u.wme_action));
	memset(mgmt, 0, 24);
	memcpy(mgmt->da, sdata->u.mgd.bssid, ETH_ALEN);
	memcpy(mgmt->sa, sdata->vif.addr, ETH_ALEN);
	memcpy(mgmt->bssid, sdata->u.mgd.bssid, ETH_ALEN);
	mgmt->frame_control = cpu_to_le16(
			IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_ACTION);

	mgmt->u.action.category = WLAN_CATEGORY_WMM;
	mgmt->u.action.u.wme_action.action_code =
			WLAN_WMM_ACTION_CODE_ADDTS_REQUEST;
	mgmt->u.action.u.wme_action.dialog_token = 0xa0;
	mgmt->u.action.u.wme_action.status_code = 0;

	tspec_buff = skb_put(skb, 2 + sizeof(struct ieee80211_tspec_ie));
	ieee80211_build_tspec_ie(tspec_params, tspec_buff);

	ie = skb_put(skb, extra_ies_len);
	memcpy(ie, extra_ies, extra_ies_len);

	ieee80211_tx_skb(sdata, skb);
}

int ieee80211_addts_request(struct ieee80211_sub_if_data *sdata,
		struct ieee80211_tspec_params *tspec_params,
		u8* extra_ies, u8 extra_ies_len) {

	if (!sdata->local->ops->set_wme_medium_time)
		return -EOPNOTSUPP;

	ieee80211_send_addts_tspec(sdata, tspec_params,
			extra_ies, extra_ies_len);
	return 0;
}


void ieee80211_set_wme_ac_admitted(struct ieee80211_sub_if_data *sdata,
				   u8 ac, bool admitted)
{
	switch (ac) {
	case IEEE80211_AC_BK: /* AC_BK */
		if (admitted)
			sdata->wmm_admitted |= BIT(1) | BIT(2); /* BK/- */
		else
			sdata->wmm_admitted &= ~(BIT(1) | BIT(2)); /* BK/- */
		break;
	case IEEE80211_AC_VI: /* AC_VI */
		if (admitted)
			sdata->wmm_admitted |= BIT(4) | BIT(5); /* CL/VI */
		else
			sdata->wmm_admitted &= ~(BIT(4) | BIT(5)); /* CL/VI */
		break;
	case IEEE80211_AC_VO: /* AC_VO */
		if (admitted)
			sdata->wmm_admitted |= BIT(6) | BIT(7); /* VO/NC */
		else
			sdata->wmm_admitted &= ~(BIT(6) | BIT(7)); /* VO/NC */
		break;
	case IEEE80211_AC_BE: /* AC_BE */
	default:
		if (admitted)
			sdata->wmm_admitted |= BIT(0) | BIT(3); /* BE/EE */
		else
			sdata->wmm_admitted &= ~(BIT(0) | BIT(3)); /* BE/EE */
		break;
	}
}

void
wme_rx_action_tspec(struct ieee80211_sub_if_data *sdata,
		u8 action_code,	u8 status_code, u8 *buf) {
	struct ieee80211_tspec_ie *tspec;
	u8 tid;

	tspec = (struct ieee80211_tspec_ie*) buf;
	tid = (tspec->tsinfo >> IEEE80211_WMM_IE_TSPEC_TID_SHIFT)
			& IEEE80211_WMM_IE_TSPEC_TID_MASK;

	switch (action_code) {
	case 0:
		break;
	case 1:
		if (!sdata->local->ops->set_wme_medium_time)
			return;
		if (status_code
				== IEEE80211_TSPEC_STATUS_ADMISS_ACCEPTED) {
			sdata->wmm_admitted |= BIT(tid);
			sdata->wmm_admitted |= BIT(tid_couple[tid]);
			sdata->wmm_ac_downgraded &= ~BIT(ieee802_1d_to_ac[tid]);
			sdata->local->ops->set_wme_medium_time(&sdata->local->hw,
					&sdata->vif, tspec->medium_time, tspec->min_phy_rate,
					ieee802_1d_to_ac[tid]);
		}
		break;
	case 2:
		sdata->wmm_admitted &= ~BIT(tid);
		sdata->wmm_admitted &= ~BIT(tid_couple[tid]);
		sdata->wmm_ac_downgraded &= ~BIT(ieee802_1d_to_ac[tid]);
		break;
	default:
		return;
	}
	return;
}

void ieee80211_set_wme_medium_time_crossed(struct ieee80211_vif *vif,
					   struct sk_buff *skb)
{
	struct ieee80211_sub_if_data *sdata = vif_to_sdata(vif);
	int ac = ieee802_1d_to_ac[skb->priority];

	/*ieee80211_set_wme_ac_admitted(sdata, ac, false);*/
	sdata->downgrade_ts[ac] = jiffies_to_msecs(jiffies);
	sdata->wmm_ac_downgraded |= BIT(ac);

}
EXPORT_SYMBOL(ieee80211_set_wme_medium_time_crossed);
