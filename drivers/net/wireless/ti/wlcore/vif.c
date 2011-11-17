/*
 * This file is part of wlcore
 *
 * Copyright (C) 2011 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <net/mac80211.h>

#include "wlcore.h"
#include "vif.h"
#include "cmd.h"
#include "acx.h"

static u8 wlcore_vif_to_bss_type(struct ieee80211_vif *vif)
{
	u8 type;

	/*
	 * In terms of firmware API, a P2P client is the same as a STA
	 * and a P2P GO is the same as an AP.  The bss_type here is
	 * the one understood by the firmware.
	 */
	switch (ieee80211_vif_type_p2p(vif)) {
	case NL80211_IFTYPE_P2P_CLIENT:
		/* fall-through */
	case NL80211_IFTYPE_STATION:
		type = BSS_TYPE_STA_BSS;
		break;
	case NL80211_IFTYPE_ADHOC:
		type = BSS_TYPE_IBSS;
		break;
	case NL80211_IFTYPE_P2P_GO:
		/* fall-through */
	case NL80211_IFTYPE_AP:
		type = BSS_TYPE_AP_BSS;
		break;
	default:
		type = BSS_TYPE_INVALID;
		break;
	}

	return type;
}

u8 wlcore_vif_to_role_type(struct ieee80211_vif *vif)
{
	u8 type;

	/* TODO: combine with wlcore_vif_to_bss_type()? */

	switch (ieee80211_vif_type_p2p(vif)) {
	case NL80211_IFTYPE_P2P_CLIENT:
		type = WLCORE_ROLE_P2P_CL;
		break;
	case NL80211_IFTYPE_STATION:
		type = WLCORE_ROLE_STA;
		break;
	case NL80211_IFTYPE_ADHOC:
		type = WLCORE_ROLE_IBSS;
		break;
	case NL80211_IFTYPE_P2P_GO:
		type = WLCORE_ROLE_P2P_GO;
		break;
	case NL80211_IFTYPE_AP:
		type = WLCORE_ROLE_AP;
		break;
	default:
		type = WLCORE_ROLE_INVALID;
		break;
	}

	return type;
}

static int wlcore_vif_alloc_rate_policy(struct wlcore *wl, u8 *idx)
{
	u8 policy = find_first_zero_bit(wl->rate_policies_map,
					WLCORE_MAX_RATE_POLICIES);
	if (policy >= WLCORE_MAX_RATE_POLICIES)
		return -EBUSY;

	__set_bit(policy, wl->rate_policies_map);
	*idx = policy;
	return 0;
}

static void wlcore_vif_free_rate_policy(struct wlcore *wl, u8 *idx)
{
	if (WARN_ON(*idx >= WLCORE_MAX_RATE_POLICIES))
		return;

	__clear_bit(*idx, wl->rate_policies_map);
	*idx = WLCORE_MAX_RATE_POLICIES;
}

int wlcore_vif_add(struct wlcore *wl, struct ieee80211_vif *vif)
{
	struct wlcore_vif *wlvif = wlcore_vif_to_wlvif(vif);

	wlvif->bss_type = wlcore_vif_to_bss_type(vif);
	if (wlvif->bss_type == BSS_TYPE_INVALID)
		return -EOPNOTSUPP;

	/* clear everything but the persistent data */
	memset(wlvif, 0, offsetof(struct wlcore_vif, persistent));

	wlvif->role_id = WLCORE_INVALID_ROLE_ID;
	wlvif->dev_role_id = WLCORE_INVALID_ROLE_ID;
	wlvif->dev_hlid = WLCORE_INVALID_LINK_ID;

	if (wlvif->bss_type == BSS_TYPE_STA_BSS ||
	    wlvif->bss_type == BSS_TYPE_IBSS) {
		/* init sta/ibss data */
		wlvif->sta.hlid = WLCORE_INVALID_LINK_ID;
		wlcore_vif_alloc_rate_policy(wl, &wlvif->sta.basic_rate_idx);
		wlcore_vif_alloc_rate_policy(wl, &wlvif->sta.ap_rate_idx);
		wlcore_vif_alloc_rate_policy(wl, &wlvif->sta.p2p_rate_idx);

		/* TODO: add op to allocate HW specific extra policies */
	} else {
		/* init ap data */

		/* TODO: add AP */
	}

	wlvif->bitrate_masks[IEEE80211_BAND_2GHZ] = wl->conf->tx.basic_rate;
	wlvif->bitrate_masks[IEEE80211_BAND_5GHZ] = wl->conf->tx.basic_rate_5;
	wlvif->basic_rate_set = WLCORE_TX_RATE_MASK_BASIC;
	wlvif->basic_rate = WLCORE_TX_RATE_MASK_BASIC;
	wlvif->rate_set = WLCORE_TX_RATE_MASK_BASIC;
	wlvif->beacon_int = WLCORE_DEFAULT_BEACON_INT;

	/*
	 * mac80211 configures some values globally, while we treat them
	 * per-interface. thus, on init, we have to copy them from wl
	 */
	wlvif->band = wl->band;
	wlvif->channel = wl->channel;
	wlvif->power_level = wl->power_level;
	wlvif->channel_type = wl->channel_type;

	/* TODO: Add RX streaming enable/disable works */

	/* TODO: Add PS poll work */
	INIT_LIST_HEAD(&wlvif->list);

	/* TODO: Add rx streaming timer */

	wlvif->wl = wl;

	list_add(&wlvif->list, &wl->wlvif_list);

	return 0;
}

void wlcore_vif_remove(struct wlcore *wl, struct ieee80211_vif *vif)
{
	struct wlcore_vif *wlvif = wlcore_vif_to_wlvif(vif);

	if (wlvif->bss_type == BSS_TYPE_STA_BSS ||
	    wlvif->bss_type == BSS_TYPE_IBSS) {
		/* init sta/ibss data */
		wlvif->sta.hlid = WLCORE_INVALID_LINK_ID;
		wlcore_vif_free_rate_policy(wl, &wlvif->sta.basic_rate_idx);
		wlcore_vif_free_rate_policy(wl, &wlvif->sta.ap_rate_idx);
		wlcore_vif_free_rate_policy(wl, &wlvif->sta.p2p_rate_idx);

		/* TODO: add op to free HW specific extra policies */
	} else {
		/* free ap data */

		/* TODO: add AP */
	}

	list_del(&wlvif->list);
}

static int wlcore_vif_init_sta(struct wlcore *wl, struct wlcore_vif *wlvif)
{
	/* TODO: add ext radio params op */

	/* TODO: add PS config */

	/* TODO: add FM WLAN coexistence */

	/* TODO: add rate policies config */

	return 0;
}

int wlcore_vif_init(struct wlcore *wl, struct ieee80211_vif *vif)
{
	struct wlcore_vif *wlvif = wlcore_vif_to_wlvif(vif);
	/* struct conf_tx_ac_category *conf_ac; */
	/* struct conf_tx_tid *conf_tid; */
	int ret, i;

	ret = wlcore_acx_sleep_auth(wl, WLCORE_PSM_CAM);
	if (ret < 0)
		return ret;

	ret = wlcore_vif_init_sta(wl, wlvif);
	if (ret < 0)
		return ret;

#if 0
	ret = wlcore_init_sta_role(wl, wlvif);
	if (ret < 0)
		return ret;

	wlcore_init_phy_vif_config(wl, wlvif);

	/* Default TID/AC configuration */
	BUG_ON(wl->conf.tx.tid_conf_count != wl->conf.tx.ac_conf_count);
	for (i = 0; i < wl->conf.tx.tid_conf_count; i) {
		conf_ac = &wl->conf.tx.ac_conf[i];
		ret = wlcore_acx_ac_cfg(wl, wlvif, conf_ac->ac,
					conf_ac->cw_min, conf_ac->cw_max,
					conf_ac->aifsn, conf_ac->tx_op_limit);
		if (ret < 0)
			return ret;

		conf_tid = &wl->conf.tx.tid_conf[i];
		ret = wlcore_acx_tid_cfg(wl, wlvif,
					 conf_tid->queue_id,
					 conf_tid->channel_type,
					 conf_tid->tsid,
					 conf_tid->ps_scheme,
					 conf_tid->ack_policy,
					 conf_tid->apsd_conf[0],
					 conf_tid->apsd_conf[1]);
		if (ret < 0)
			return ret;
	}

	/* Configure HW encryption */
	ret = wlcore_acx_feature_cfg(wl, wlvif);
	if (ret < 0)
		return ret;

	ret = wlcore_sta_hw_init_post_mem(wl, vif);
	if (ret < 0)
		return ret;

	/* Configure initiator BA sessions policies */
	ret = wlcore_set_ba_policies(wl, wlvif);
	if (ret < 0)
		return ret;
#endif
	/* TODO: add cs op */

	return 0;
}
