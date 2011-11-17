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

#ifndef __CMD_H__
#define __CMD_H__

#include <linux/ieee80211.h>
#include <linux/if_arp.h>

#include "vif.h"

enum wlcore_commands {
	CMD_INTERROGATE			= 1,
	CMD_CONFIGURE			= 2,
	CMD_ENABLE_RX			= 3,
	CMD_ENABLE_TX			= 4,
	CMD_DISABLE_RX			= 5,
	CMD_DISABLE_TX			= 6,
	CMD_SCAN			= 8,
	CMD_STOP_SCAN			= 9,
	CMD_SET_KEYS			= 12,
	CMD_READ_MEMORY			= 13,
	CMD_WRITE_MEMORY		= 14,
	CMD_SET_TEMPLATE		= 19,
	CMD_TEST			= 23,
	CMD_NOISE_HIST			= 28,
	CMD_QUIET_ELEMENT_SET_STATE	= 29,
	CMD_SET_BCN_MODE		= 33,
	CMD_MEASUREMENT			= 34,
	CMD_STOP_MEASUREMENT		= 35,
	CMD_SET_PS_MODE			= 37,
	CMD_CHANNEL_SWITCH		= 38,
	CMD_STOP_CHANNEL_SWICTH		= 39,
	CMD_AP_DISCOVERY		= 40,
	CMD_STOP_AP_DISCOVERY		= 41,
	CMD_HEALTH_CHECK		= 45,
	CMD_DEBUG			= 46,
	CMD_TRIGGER_SCAN_TO		= 47,
	CMD_CONNECTION_SCAN_CFG		= 48,
	CMD_CONNECTION_SCAN_SSID_CFG	= 49,
	CMD_START_PERIODIC_SCAN		= 50,
	CMD_STOP_PERIODIC_SCAN		= 51,
	CMD_SET_PEER_STATE		= 52,
	CMD_REMAIN_ON_CHANNEL		= 53,
	CMD_CANCEL_REMAIN_ON_CHANNEL	= 54,

	CMD_CONFIG_FWLOGGER		= 55,
	CMD_START_FWLOGGER		= 56,
	CMD_STOP_FWLOGGER		= 57,

	/* AP commands */
	CMD_ADD_PEER			= 62,
	CMD_REMOVE_PEER			= 63,

	/* Role API */
	CMD_ROLE_ENABLE			= 70,
	CMD_ROLE_DISABLE		= 71,
	CMD_ROLE_START			= 72,
	CMD_ROLE_STOP			= 73,

	/* WIFI Direct */
	CMD_WFD_START_DISCOVERY		= 80,
	CMD_WFD_STOP_DISCOVERY		= 81,
	CMD_WFD_ATTRIBUTE_CONFIG	= 82,

	CMD_NOP				= 100,
};

enum {
	CMD_STATUS_MAILBOX_IDLE		 =  0,
	CMD_STATUS_SUCCESS		 =  1,
	CMD_STATUS_UNKNOWN_CMD		 =  2,
	CMD_STATUS_UNKNOWN_IE		 =  3,
	CMD_STATUS_REJECT_MEAS_SG_ACTIVE = 11,
	CMD_STATUS_RX_BUSY		 = 13,
	CMD_STATUS_INVALID_PARAM	 = 14,
	CMD_STATUS_TEMPLATE_TOO_LARGE	 = 15,
	CMD_STATUS_OUT_OF_MEMORY	 = 16,
	CMD_STATUS_STA_TABLE_FULL	 = 17,
	CMD_STATUS_RADIO_ERROR		 = 18,
	CMD_STATUS_WRONG_NESTING	 = 19,
	CMD_STATUS_TIMEOUT		 = 21,
	CMD_STATUS_FW_RESET		 = 22,
	CMD_STATUS_TEMPLATE_OOM		 = 23,
	CMD_STATUS_NO_RX_BA_SESSION	 = 24,
};

#define WLCORE_COMMAND_TIMEOUT		2000
#define WLCORE_CMD_FAST_POLL_COUNT	50

struct wlcore_cmd_header {
	__le16 id;
	__le16 status;
	/* payload */
	u8 data[0];
} __packed;

#define WLCORE_TEMPL_DFLT_SIZE 252
#define WLCORE_TEMPL_MAX_SIZE  548

#define WLCORE_RATE_AUTOMATIC  0

/*
 * We have most structs we need for the templates in
 * linux/ieee80211.h, but we still need to define a few of our own.
 */
struct wlcore_arp_rsp_tmpl {
	struct ieee80211_hdr_3addr hdr;

	u8 llc_hdr[sizeof(rfc1042_header)];
	__be16 llc_type;

	struct arphdr arp_hdr;
	u8 sender_hw[ETH_ALEN];
	__be32 sender_ip;
	u8 target_hw[ETH_ALEN];
	__be32 target_ip;
} __packed;

/*
 * TODO: we have this in linux/ieee80211.h, but it's part of a union
 * and I didn't find any clean way to get the size of it
 */
struct wlcore_deauth_tmpl {
	struct ieee80211_hdr_3addr header;
	__le16 reason;
} __packed;

enum {
	TEMPL_KLV_IDX_NULL_DATA = 0,
	TEMPL_KLV_IDX_MAX = 4
};

enum {
	TEMPL_NULL_DATA = 0,
	TEMPL_BEACON,
	TEMPL_CFG_PROBE_REQ_2_4,
	TEMPL_CFG_PROBE_REQ_5,
	TEMPL_PROBE_RESPONSE,
	TEMPL_QOS_NULL_DATA,
	TEMPL_PS_POLL,
	TEMPL_KLV,
	TEMPL_DISCONNECT,
	TEMPL_PROBE_REQ_2_4, /* for firmware internal use only  */
	TEMPL_PROBE_REQ_5,   /* for firmware internal use only  */
	TEMPL_BAR,           /* for firmware internal use only  */
	TEMPL_CTS,           /* FastCTS for BT/WLAN coexistence */
	TEMPL_AP_BEACON,
	TEMPL_AP_PROBE_RESPONSE,
	TEMPL_ARP_RSP,
	TEMPL_DEAUTH_AP,
	TEMPL_TEMPORARY,
	TEMPL_LINK_MEASUREMENT_REPORT,
};

struct wlcore_cmd_template_set {
	struct wlcore_cmd_header header;

	__le16 len;
	u8 template_type;
	u8 index;  /* relevant only for KLV_TEMPLATE type */
	__le32 enabled_rates;
	u8 short_retry_limit;
	u8 long_retry_limit;
	u8 aflags;
	u8 reserved;
	u8 template_data[WLCORE_TEMPL_MAX_SIZE];
} __packed;

struct wlcore_cmd_enable_disable_rx_tx {
	struct wlcore_cmd_header header;

	u8 channel; /* deprecated */
	u8 padding[3];
} __packed;

enum {
	WLCORE_ROLE_STA = 0,
	WLCORE_ROLE_IBSS,
	WLCORE_ROLE_AP,
	WLCORE_ROLE_DEVICE,
	WLCORE_ROLE_P2P_CL,
	WLCORE_ROLE_P2P_GO,

	WLCORE_ROLE_INVALID = 0xFF
};

struct wlcore_cmd_role_enable {
	struct wlcore_cmd_header header;

	u8 role_id;
	u8 role_type;
	u8 mac_address[ETH_ALEN];
} __packed;

struct wlcore_cmd_role_disable {
	struct wlcore_cmd_header header;

	u8 role_id;
	u8 padding[3];
} __packed;

enum wlcore_band {
	WLCORE_BAND_2_4GHZ		= 0,
	WLCORE_BAND_5GHZ		= 1,
	WLCORE_BAND_JAPAN_4_9_GHZ	= 2,
};

struct wlcore_cmd_role_start {
	struct wlcore_cmd_header header;

	u8 role_id;
	u8 band;
	u8 channel;
	u8 channel_type;

	union {
		struct {
			u8 hlid;
			u8 session;
			u8 padding_1[54];
		} __packed device;
		/* sta & p2p_cli use the same struct */
		struct {
			u8 bssid[ETH_ALEN];
			u8 hlid; /* data hlid */
			u8 session;
			__le32 remote_rates; /* remote supported rates */

			/*
			 * The target uses this field to determine the rate at
			 * which to transmit control frame responses (such as
			 * ACK or CTS frames).
			 */
			__le32 basic_rate_set;
			__le32 local_rates; /* local supported rates */

			u8 ssid_type;
			u8 ssid_len;
			u8 ssid[IEEE80211_MAX_SSID_LEN];

			__le16 beacon_interval; /* in TBTTs */
		} __packed sta;
		struct {
			u8 bssid[ETH_ALEN];
			u8 hlid; /* data hlid */
			u8 dtim_interval;
			__le32 remote_rates; /* remote supported rates */

			__le32 basic_rate_set;
			__le32 local_rates; /* local supported rates */

			u8 ssid_type;
			u8 ssid_len;
			u8 ssid[IEEE80211_MAX_SSID_LEN];

			__le16 beacon_interval; /* in TBTTs */

			u8 padding_1[4];
		} __packed ibss;
		/* ap & p2p_go use the same struct */
		struct {
			__le16 aging_period; /* in secs */
			u8 beacon_expiry; /* in ms */
			u8 bss_index;
			/* The host link id for the AP's global queue */
			u8 global_hlid;
			/* The host link id for the AP's broadcast queue */
			u8 broadcast_hlid;

			__le16 beacon_interval; /* in TBTTs */

			__le32 basic_rate_set;
			__le32 local_rates; /* local supported rates */

			u8 dtim_interval;

			u8 ssid_type;
			u8 ssid_len;
			u8 ssid[IEEE80211_MAX_SSID_LEN];

			u8 padding_1[5];
		} __packed ap;
	};
} __packed;

/*
 * There are three types of disconnections:
 *
 * DISCONNECT_IMMEDIATE: the fw doesn't send any frames
 * DISCONNECT_DEAUTH:    the fw generates a DEAUTH request with the reason
 *                       we have passed
 * DISCONNECT_DISASSOC:  the fw generates a DESASSOC request with the reason
 *                       we have passed
 */
enum {
	DISCONNECT_IMMEDIATE,
	DISCONNECT_DEAUTH,
	DISCONNECT_DISASSOC
};

struct wlcore_cmd_role_stop {
	struct wlcore_cmd_header header;

	u8 role_id;
	u8 disc_type; /* only STA and P2P_CLI */
	__le16 reason; /* only STA and P2P_CLI */
} __packed;

struct wlcore_cmd_roc {
	struct wlcore_cmd_header header;

	u8 role_id;
	u8 channel;
	u8 band;
	u8 padding;
};

struct wlcore_cmd_croc {
	struct wlcore_cmd_header header;

	u8 role_id;
	u8 padding[3];
};

static inline bool wlcore_is_roc(struct wlcore *wl)
{
	u8 role_id;

	role_id = find_first_bit(wl->roc_map, WLCORE_MAX_ROLES);
	if (role_id >= WLCORE_MAX_ROLES)
		return false;

	return true;
}

int wlcore_cmd_send(struct wlcore *wl, u16 id, void *buf, size_t len,
		    size_t res_len);
int wlcore_cmd_configure(struct wlcore *wl, u16 id, void *buf, size_t len);

int wlcore_cmd_interrogate(struct wlcore *wl, u16 id, void *buf, size_t len);
int wlcore_cmd_template_set(struct wlcore *wl, u16 template_id, void *buf,
			    size_t buf_len, int index, u32 rates);
int wlcore_cmd_enable_rx_tx(struct wlcore *wl);
int wlcore_cmd_role_enable(struct wlcore *wl, u8 *addr, u8 role_type,
			   u8 *role_id);
int wlcore_cmd_role_disable(struct wlcore *wl, u8 *role_id);
int wlcore_roc(struct wlcore *wl, struct wlcore_vif *wlvif, u8 role_id);
int wlcore_croc(struct wlcore *wl, u8 role_id);
int wlcore_start_dev(struct wlcore *wl, struct wlcore_vif *wlvif);
int wlcore_stop_dev(struct wlcore *wl, struct wlcore_vif *wlvif);

#endif /* __CMD_H__ */
