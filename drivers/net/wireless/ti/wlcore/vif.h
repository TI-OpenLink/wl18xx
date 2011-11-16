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

#ifndef __VIF_H__
#define __VIF_H__

enum {
	BSS_TYPE_IBSS		= 0,
	BSS_TYPE_STA_BSS	= 2,
	BSS_TYPE_AP_BSS		= 3,
	BSS_TYPE_INVALID	= 0xFF
};

struct wlcore_vif {
	struct wlcore *wl;
	struct list_head list;
	unsigned long flags;
	u8 bss_type;
	u8 role_id;

	/* sta/ibss specific */
	u8 dev_role_id;
	u8 dev_hlid;

	union {
		struct {
			u8 hlid;
			u8 ba_rx_bitmap;

			u8 basic_rate_idx;
			u8 ap_rate_idx;
			u8 p2p_rate_idx;
			u8 wide_chan_rate_idx;
		} sta;
		/* TODO: add AP */
	};

	/* the hlid of the last transmitted skb */
	int last_tx_hlid;

	unsigned long links_map[BITS_TO_LONGS(WLCORE_MAX_LINKS)];

	u8 ssid[IEEE80211_MAX_SSID_LEN + 1];
	u8 ssid_len;

	/* The current band */
	enum ieee80211_band band;
	int channel;

	u32 bitrate_masks[IEEE80211_NUM_BANDS];
	u32 basic_rate_set;

	/*
	 * currently configured rate set:
	 *	bits  0-15 - 802.11abg rates
	 *	bits 16-23 - 802.11n   MCS index mask
	 * support only 1 stream, thus only 8 bits for the MCS rates (0-7).
	 */
	u32 basic_rate;
	u32 rate_set;

	/* probe-req template for the current AP */
	struct sk_buff *probereq;

	/* Beaconing interval (needed for ad-hoc) */
	u32 beacon_int;

	/* Default key (for WEP) */
	u32 default_key;

	/* Our association ID */
	u16 aid;

	/* Session counter for the chipset */
	int session_counter;

	struct completion *ps_compl;
	struct delayed_work pspoll_work;

	/* counter for ps-poll delivery failures */
	int ps_poll_failures;

	/* retry counter for PSM entries */
	u8 psm_entry_retry;

	/* in dBm */
	int power_level;

	int rssi_thold;
	int last_rssi_event;

	/* RX BA constraint value */
	bool ba_support;
	bool ba_allowed;

	/* Rx Streaming */
	struct work_struct rx_streaming_enable_work;
	struct work_struct rx_streaming_disable_work;
	struct timer_list rx_streaming_timer;

	enum nl80211_channel_type channel_type;

	/*
	 * This struct must be last!
	 * data that has to be saved acrossed reconfigs (e.g. recovery)
	 * should be declared in this struct.
	 */
	struct {
		u8 persistent[0];
		/*
		 * Security sequence number
		 *     bits 0-15: lower 16 bits part of sequence number
		 *     bits 16-47: higher 32 bits part of sequence number
		 *     bits 48-63: not in use
		 */
		u64 tx_security_seq;

		/* 8 bits of the last sequence number in use */
		u8 tx_security_last_seq_lsb;
	};
};

static inline
struct wlcore_vif *wlcore_vif_to_wlvif(struct ieee80211_vif *vif)
{
	return (struct wlcore_vif *)vif->drv_priv;
}

static inline
struct ieee80211_vif *wlcore_wlvif_to_vif(struct wlcore_vif *wlvif)
{
	return container_of((void *)wlvif, struct ieee80211_vif, drv_priv);
}

u8 wlcore_vif_to_role_type(struct ieee80211_vif *vif);
int wlcore_vif_add(struct wlcore *wl, struct ieee80211_vif *vif);
void wlcore_vif_remove(struct wlcore *wl, struct ieee80211_vif *vif);

#endif /* __VIF_H__ */
