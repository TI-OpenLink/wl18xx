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
}
