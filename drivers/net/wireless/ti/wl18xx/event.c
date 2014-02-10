/*
 * This file is part of wl12xx
 *
 * Copyright (C) 2012 Texas Instruments. All rights reserved.
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

#include <net/genetlink.h>
#include "event.h"
#include "scan.h"
#include "../wlcore/cmd.h"
#include "../wlcore/debug.h"
#include "../wlcore/testmode.h"

int wl18xx_wait_for_event(struct wl1271 *wl, enum wlcore_wait_event event,
			  bool *timeout)
{
	u32 local_event;

	switch (event) {
	case WLCORE_EVENT_PEER_REMOVE_COMPLETE:
		local_event = PEER_REMOVE_COMPLETE_EVENT_ID;
		break;

	case WLCORE_EVENT_DFS_CONFIG_COMPLETE:
		local_event = DFS_CHANNELS_CONFIG_COMPLETE_EVENT;
		break;

	default:
		/* event not implemented */
		return 0;
	}
	return wlcore_cmd_wait_for_event_or_timeout(wl, local_event, timeout);
}

#ifdef CONFIG_NL80211_TESTMODE
static int wlcore_smart_config_sync_event(struct wl1271 *wl, u8 sync_channel,
					  u8 sync_band)
{
	struct sk_buff *skb;
	enum ieee80211_band band;
	int freq;

	if (sync_band == WLCORE_BAND_5GHZ)
		band = IEEE80211_BAND_5GHZ;
	else
		band = IEEE80211_BAND_2GHZ;

	freq = ieee80211_channel_to_frequency(sync_channel, band);

	wl1271_debug(DEBUG_EVENT,
		     "SMART_CONFIG_SYNC_EVENT_ID, freq: %d (chan: %d band %d)",
		     freq, sync_channel, sync_band);
	skb = cfg80211_testmode_alloc_event_skb(wl->hw->wiphy, 20, GFP_KERNEL);

	if (nla_put_u8(skb, WL1271_TM_ATTR_SMART_CONFIG_EVENT,
		       WLCORE_TM_SC_EVENT_SYNC) ||
	    nla_put_u32(skb, WL1271_TM_ATTR_FREQ, freq)) {
		kfree_skb(skb);
		return -EMSGSIZE;
	}
	cfg80211_testmode_event(skb, GFP_KERNEL);
	return 0;
}

static int wlcore_smart_config_decode_event(struct wl1271 *wl,
					    u8 ssid_len, u8 *ssid,
					    u8 pwd_len, u8 *pwd)
{
	struct sk_buff *skb;

	wl1271_debug(DEBUG_EVENT, "SMART_CONFIG_DECODE_EVENT_ID");
	wl1271_dump_ascii(DEBUG_EVENT, "SSID:", ssid, ssid_len);
	wl1271_dump_ascii(DEBUG_EVENT, "PWD:",pwd, pwd_len);

	skb = cfg80211_testmode_alloc_event_skb(wl->hw->wiphy,
			ssid_len + pwd_len + 20, GFP_KERNEL);

	if (nla_put_u8(skb, WL1271_TM_ATTR_SMART_CONFIG_EVENT,
		       WLCORE_TM_SC_EVENT_DECODE) ||
	    nla_put(skb, WL1271_TM_ATTR_SSID, ssid_len, ssid) ||
	    nla_put(skb, WL1271_TM_ATTR_PSK, pwd_len, pwd)) {
		kfree_skb(skb);
		return -EMSGSIZE;
	}
	cfg80211_testmode_event(skb, GFP_KERNEL);
	return 0;
}
#else
static int wlcore_smart_config_sync_event(struct wl1271 *wl, u8 sync_channel,
					  u8 sync_band)
{
	wl1271_error("got SMART_CONFIG event, but CONFIG_NL80211_TESTMODE is not configured!");
	return -EINVAL;
}

static int wlcore_smart_config_decode_event(struct wl1271 *wl,
					    u8 ssid_len, u8 *ssid,
					    u8 pwd_len, u8 *pwd)
{
	wl1271_error("got SMART_CONFIG event, but CONFIG_NL80211_TESTMODE is not configured!");
	return -EINVAL;
}
#endif

int wl18xx_process_mailbox_events(struct wl1271 *wl)
{
	struct wl18xx_event_mailbox *mbox = wl->mbox;
	u32 vector;

	vector = le32_to_cpu(mbox->events_vector);
	wl1271_debug(DEBUG_EVENT, "MBOX vector: 0x%x", vector);

	if (vector & SCAN_COMPLETE_EVENT_ID) {
		wl1271_debug(DEBUG_EVENT, "scan results: %d",
			     mbox->number_of_scan_results);

		if (wl->scan_wlvif)
			wl18xx_scan_completed(wl, wl->scan_wlvif);
	}

	if (vector & PERIODIC_SCAN_REPORT_EVENT_ID) {
		wl1271_debug(DEBUG_EVENT,
			     "PERIODIC_SCAN_REPORT_EVENT (results %d)",
			     mbox->number_of_sched_scan_results);

		wlcore_scan_sched_scan_results(wl);
	}

	if (vector & PERIODIC_SCAN_COMPLETE_EVENT_ID)
		wlcore_event_sched_scan_completed(wl, 1);

	if (vector & RSSI_SNR_TRIGGER_0_EVENT_ID)
		wlcore_event_rssi_trigger(wl, mbox->rssi_snr_trigger_metric);

	if (vector & BA_SESSION_RX_CONSTRAINT_EVENT_ID)
		wlcore_event_ba_rx_constraint(wl,
				le16_to_cpu(mbox->rx_ba_role_id_bitmap),
				le16_to_cpu(mbox->rx_ba_allowed_bitmap));

	if (vector & BSS_LOSS_EVENT_ID)
		wlcore_event_beacon_loss(wl,
					 le16_to_cpu(mbox->bss_loss_bitmap));

	if (vector & CHANNEL_SWITCH_COMPLETE_EVENT_ID)
		wlcore_event_channel_switch(wl,
			le16_to_cpu(mbox->channel_switch_role_id_bitmap),
			true);

	if (vector & DUMMY_PACKET_EVENT_ID)
		wlcore_event_dummy_packet(wl);

	/*
	 * "TX retries exceeded" has a different meaning according to mode.
	 * In AP mode the offending station is disconnected.
	 */
	if (vector & MAX_TX_FAILURE_EVENT_ID)
		wlcore_event_max_tx_failure(wl,
				le32_to_cpu(mbox->tx_retry_exceeded_bitmap));

	if (vector & INACTIVE_STA_EVENT_ID)
		wlcore_event_inactive_sta(wl,
				le32_to_cpu(mbox->inactive_sta_bitmap));

	if (vector & REMAIN_ON_CHANNEL_COMPLETE_EVENT_ID)
		wlcore_event_roc_complete(wl);

	if (vector & RX_BA_WIN_SIZE_CHANGE_EVENT_ID) {
		struct wl12xx_vif *wlvif;
		struct ieee80211_vif *vif;
		u8 role_id = mbox->rx_ba_role_id;
		u8 link_id = mbox->rx_ba_link_id;
		u8 win_size = mbox->rx_ba_win_size;
		int prev_win_size;

		wl1271_debug(DEBUG_EVENT,
			     "%s. role_id=%u link_id=%u win_size=%u",
			     "RX_BA_WIN_SIZE_CHANGE_EVENT_ID",
			     role_id, link_id, win_size);

		wlvif = wl->links[link_id].wlvif;
		if (unlikely(!wlvif)) {
			wl1271_error("%s. link_id wlvif is null",
				     "RX_BA_WIN_SIZE_CHANGE_EVENT_ID");

			goto out_event;
		}

		if (unlikely(wlvif->role_id != role_id)) {
			wl1271_error("%s. wlvif has different role_id=%d",
				     "RX_BA_WIN_SIZE_CHANGE_EVENT_ID",
				     wlvif->role_id);

			goto out_event;
		}

		prev_win_size = wlcore_rx_ba_max_subframes(wl, link_id);
		if (unlikely(prev_win_size < 0)) {
			wl1271_error("%s. cannot get link rx_ba_max_subframes",
				     "RX_BA_WIN_SIZE_CHANGE_EVENT_ID");

			goto out_event;
		}

		if ((u8) prev_win_size <= win_size) {
			/* This not supposed to happen unless a FW bug */
			wl1271_error("%s. prev_win_size(%d) <= win_size(%d)",
				       "RX_BA_WIN_SIZE_CHANGE_EVENT_ID",
					prev_win_size, win_size);

			goto out_event;
		}

		/*
		 * Call MAC routine to update win_size and stop all link active
		 * BA sessions. This routine returns 0 on failure or previous
		 * win_size on success
		 */
		vif = wl12xx_wlvif_to_vif(wlvif);
		ieee80211_change_rx_ba_max_subframes(vif,
			(wlvif->bss_type != BSS_TYPE_AP_BSS ?
				vif->bss_conf.bssid :
				wl->links[link_id].addr),
			win_size);
	}

	if (vector & SMART_CONFIG_SYNC_EVENT_ID)
		wlcore_smart_config_sync_event(wl, mbox->sc_sync_channel,
					       mbox->sc_sync_band);

	if (vector & SMART_CONFIG_DECODE_EVENT_ID)
		wlcore_smart_config_decode_event(wl,
						 mbox->sc_ssid_len,
						 mbox->sc_ssid,
						 mbox->sc_pwd_len,
						 mbox->sc_pwd);

out_event:

	return 0;
}
