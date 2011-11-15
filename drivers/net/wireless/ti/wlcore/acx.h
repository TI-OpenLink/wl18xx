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

#ifndef __ACX_H__
#define __ACX_H__

#include "cmd.h"

enum {
	ACX_WAKE_UP_CONDITIONS			= 0x0002,
	ACX_MEM_CFG				= 0x0003,
	ACX_SLOT				= 0x0004,
	ACX_AC_CFG				= 0x0007,
	ACX_MEM_MAP				= 0x0008,
	ACX_AID					= 0x000A,
	ACX_MEDIUM_USAGE			= 0x000F,
	ACX_TX_QUEUE_CFG			= 0x0011,
	ACX_STATISTICS				= 0x0013,
	ACX_PWR_CONSUMPTION_STATISTICS		= 0x0014,
	ACX_FEATURE_CFG				= 0x0015,
	ACX_TID_CFG				= 0x001A,
	ACX_PS_RX_STREAMING			= 0x001B,
	ACX_BEACON_FILTER_OPT			= 0x001F,
	ACX_NOISE_HIST				= 0x0021,
	ACX_HDK_VERSION				= 0x0022,
	ACX_PD_THRESHOLD			= 0x0023,
	ACX_TX_CONFIG_OPT			= 0x0024,
	ACX_CCA_THRESHOLD			= 0x0025,
	ACX_EVENT_MBOX_MASK			= 0x0026,
	ACX_CONN_MONIT_PARAMS			= 0x002D,
	ACX_DISABLE_BROADCASTS			= 0x0030,
	ACX_BCN_DTIM_OPTIONS			= 0x0031,
	ACX_SG_ENABLE				= 0x0032,
	ACX_SG_CFG				= 0x0033,
	ACX_FM_COEX_CFG				= 0x0034,
	ACX_BEACON_FILTER_TABLE			= 0x0038,
	ACX_ARP_IP_FILTER			= 0x0039,
	ACX_ROAMING_STATISTICS_TBL		= 0x003B,
	ACX_RATE_POLICY				= 0x003D,
	ACX_CTS_PROTECTION			= 0x003E,
	ACX_SLEEP_AUTH				= 0x003F,
	ACX_PREAMBLE_TYPE			= 0x0040,
	ACX_ERROR_CNT				= 0x0041,
	ACX_IBSS_FILTER				= 0x0044,
	ACX_SERVICE_PERIOD_TIMEOUT		= 0x0045,
	ACX_TSF_INFO				= 0x0046,
	ACX_CONFIG_PS_WMM			= 0x0049,
	ACX_ENABLE_RX_DATA_FILTER		= 0x004A,
	ACX_SET_RX_DATA_FILTER			= 0x004B,
	ACX_GET_DATA_FILTER_STATISTICS		= 0x004C,
	ACX_RX_CONFIG_OPT			= 0x004E,
	ACX_FRAG_CFG				= 0x004F,
	ACX_BET_ENABLE				= 0x0050,
	ACX_RSSI_SNR_TRIGGER			= 0x0051,
	ACX_RSSI_SNR_WEIGHTS			= 0x0052,
	ACX_KEEP_ALIVE_MODE			= 0x0053,
	ACX_SET_KEEP_ALIVE_CONFIG		= 0x0054,
	ACX_BA_SESSION_INITIATOR_POLICY		= 0x0055,
	ACX_BA_SESSION_RX_SETUP			= 0x0056,
	ACX_PEER_HT_CAP				= 0x0057,
	ACX_HT_BSS_OPERATION			= 0x0058,
	ACX_COEX_ACTIVITY			= 0x0059,
	ACX_BURST_MODE				= 0x005C,
	ACX_SET_RATE_MAMAGEMENT_PARAMS		= 0x005D,
	ACX_GET_RATE_MAMAGEMENT_PARAMS		= 0x005E,
	ACX_SET_RATE_ADAPT_PARAMS		= 0x0060,
	ACX_SET_DCO_ITRIM_PARAMS		= 0x0061,
	ACX_GEN_FW_CMD				= 0x0070,
	ACX_MAX_TX_FAILURE_THRESHOLD_CFG	= 0x0072,
	ACX_UPDATE_INCONNECTION_STA_LIST	= 0x0073,
	ACX_DOT11_RX_MSDU_LIFE_TIME		= 0x1004,
	ACX_DOT11_CUR_TX_PWR			= 0x100D,
	ACX_DOT11_RX_DOT11_MODE			= 0x1012,
	ACX_DOT11_RTS_THRESHOLD			= 0x1013,
	ACX_DOT11_GROUP_ADDRESS_TBL		= 0x1014,
	ACX_SET_RADIO_PARAMS			= 0x1015,
	ACX_PM_CONFIG				= 0x1016,
	ACX_CONFIG_PS				= 0x1017,
	ACX_CONFIG_HANGOVER			= 0x1018,
};


enum {
	EVENT_RSSI_SNR_TRIGGER_0		= BIT(0),
	EVENT_RSSI_SNR_TRIGGER_1		= BIT(1),
	EVENT_RSSI_SNR_TRIGGER_2		= BIT(2),
	EVENT_RSSI_SNR_TRIGGER_3		= BIT(3),
	EVENT_RSSI_SNR_TRIGGER_4		= BIT(4),
	EVENT_RSSI_SNR_TRIGGER_5		= BIT(5),
	EVENT_RSSI_SNR_TRIGGER_6		= BIT(6),
	EVENT_RSSI_SNR_TRIGGER_7		= BIT(7),
	EVENT_MEASUREMENT_START			= BIT(8),
	EVENT_MEASUREMENT_COMPLETE		= BIT(9),
	EVENT_SCAN_COMPLETE			= BIT(10),
	EVENT_WFD_DISCOVERY_COMPLETE		= BIT(11),
	EVENT_AP_DISCOVERY_COMPLETE		= BIT(12),
	EVENT_PS_REPORT				= BIT(13),
	EVENT_PSPOLL_DELIVERY_FAILURE		= BIT(14),
	EVENT_DISCONNECT_COMPLETE		= BIT(15),
	/* BIT(16) is reserved */
	EVENT_CHANNEL_SWITCH_COMPLETE		= BIT(17),
	EVENT_BSS_LOSE				= BIT(18),
	EVENT_REGAINED_BSS			= BIT(19),
	EVENT_MAX_TX_RETRY			= BIT(20),
	EVENT_DUMMY_PACKET			= BIT(21),
	EVENT_SOFT_GEMINI_SENSE			= BIT(22),
	EVENT_CHANGE_AUTO_MODE_TIMEOUT		= BIT(23),
	EVENT_SOFT_GEMINI_AVALANCHE		= BIT(24),
	EVENT_PLT_RX_CALIBRATION_COMPLETE	= BIT(25),
	EVENT_INACTIVE_STA			= BIT(26),
	EVENT_PEER_REMOVE_COMPLETE		= BIT(27),
	EVENT_PERIODIC_SCAN_COMPLETE		= BIT(28),
	EVENT_PERIODIC_SCAN_REPORT		= BIT(29),
	EVENT_BA_SESSION_RX_CONSTRAINT		= BIT(30),
	EVENT_REMAIN_ON_CHANNEL_COMPLETE	= BIT(31),
};

#define WLCORE_DEFAULT_EVENTS	EVENT_BSS_LOSE				| \
				EVENT_SCAN_COMPLETE			| \
				EVENT_PS_REPORT				| \
				EVENT_DISCONNECT_COMPLETE		| \
				EVENT_RSSI_SNR_TRIGGER_0		| \
				EVENT_PSPOLL_DELIVERY_FAILURE		| \
				EVENT_SOFT_GEMINI_SENSE			| \
				EVENT_PERIODIC_SCAN_REPORT		| \
				EVENT_PERIODIC_SCAN_COMPLETE		| \
				EVENT_DUMMY_PACKET			| \
				EVENT_PEER_REMOVE_COMPLETE		| \
				EVENT_BA_SESSION_RX_CONSTRAINT		| \
				EVENT_REMAIN_ON_CHANNEL_COMPLETE	| \
				EVENT_INACTIVE_STA			| \
				EVENT_MAX_TX_RETRY

struct acx_header {
	struct wlcore_cmd_header cmd;

	/* acx (or information element) header */
	__le16 id;

	/* payload length (not including headers */
	__le16 len;
} __packed;

struct acx_event_mask {
	struct acx_header header;

	__le32 event_mask;
	__le32 high_event_mask; /* unused (must be set to 0xFFFFFFFF) */
} __packed;

struct acx_config_memory {
	struct acx_header header;

	u8 rx_mem_block_num;
	u8 tx_min_mem_block_num;
	u8 num_stations;
	u8 num_ssid_profiles;
	__le32 total_tx_descriptors;
	u8 dyn_mem_enable;
	u8 tx_free_req;
	u8 rx_free_req;
	u8 tx_min;
	u8 fwlog_blocks;
	u8 padding[3];
} __packed;

struct acx_rx_msdu_lifetime {
	struct acx_header header;

	/*
	 * The maximum amount of time, in TU, before the
	 * firmware discards the MSDU.
	 */
	__le32 lifetime;
} __packed;

struct acx_rx_irq_config {
	struct acx_header header;

	__le16 mblk_threshold;
	__le16 threshold;
	__le16 timeout;
	u8 queue_type;
	u8 reserved;
} __packed;

struct wlcore_acx_mem_map {
	struct acx_header header;

	__le32 code_start;
	__le32 code_end;

	__le32 wep_defkey_start;
	__le32 wep_defkey_end;

	__le32 sta_table_start;
	__le32 sta_table_end;

	__le32 packet_template_start;
	__le32 packet_template_end;

	/* Address of the TX result interface (control block) */
	__le32 tx_result;
	__le32 tx_result_queue_start;

	__le32 queue_memory_start;
	__le32 queue_memory_end;

	__le32 packet_memory_pool_start;
	__le32 packet_memory_pool_end;

	__le32 debug_buffer1_start;
	__le32 debug_buffer1_end;

	__le32 debug_buffer2_start;
	__le32 debug_buffer2_end;

	/* Number of blocks FW allocated for TX packets */
	__le32 num_tx_mem_blocks;

	/* Number of blocks FW allocated for RX packets */
	__le32 num_rx_mem_blocks;

	/* the following 4 fields are valid in SLAVE mode only */
	__le32 tx_cbuf;
	__le32 rx_cbuf;
	__le32 rx_ctrl;
	__le32 tx_ctrl;
} __packed;

struct acx_tx_config_options {
	struct acx_header header;
	__le16 complete_timeout;	/* msec */
	__le16 complete_threshold;	/* number of packets */
} __packed;

struct acx_cca_threshold {
	struct acx_header header;

	__le16 rx_cca_threshold; /* deprecated? */
	u8 tx_energy_detection;
	u8 pad;
} __packed;

struct acx_frag_threshold {
	struct acx_header header;
	__le16 frag_threshold;
	u8 padding[2];
} __packed;

int wlcore_acx_event_mbox_mask(struct wlcore *wl, u32 event_mask);
int wlcore_acx_mem_cfg(struct wlcore *wl);
int wlcore_acx_get_mem_map(struct wlcore *wl,
			   struct wlcore_acx_mem_map *map);
int wlcore_acx_tx_config(struct wlcore *wl);
int wlcore_acx_cca_threshold(struct wlcore *wl);
int wlcore_acx_frag_threshold(struct wlcore *wl, u32 frag_threshold);
int wlcore_acx_rx_msdu_lifetime(struct wlcore *wl);
int wlcore_acx_rx_irq_config(struct wlcore *wl);

#endif /* __ACX_H__ */
