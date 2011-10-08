/*
 * This file is part of wl1271
 *
 * Copyright (C) 1998-2009 Texas Instruments. All rights reserved.
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
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

#ifndef __RX_H__
#define __RX_H__

#include <linux/bitops.h>

#define WL1271_RX_MAX_RSSI -30
#define WL1271_RX_MIN_RSSI -95

#define SHORT_PREAMBLE_BIT   BIT(0)
#define OFDM_RATE_BIT        BIT(6)
#define PBCC_RATE_BIT        BIT(7)

#define PLCP_HEADER_LENGTH 8
#define RX_DESC_PACKETID_SHIFT 11
#define RX_MAX_PACKET_ID 3

#define NUM_RX_PKT_DESC_MOD_MASK   7

#define RX_DESC_VALID_FCS         0x0001
#define RX_DESC_MATCH_RXADDR1     0x0002
#define RX_DESC_MCAST             0x0004
#define RX_DESC_STAINTIM          0x0008
#define RX_DESC_VIRTUAL_BM        0x0010
#define RX_DESC_BCAST             0x0020
#define RX_DESC_MATCH_SSID        0x0040
#define RX_DESC_MATCH_BSSID       0x0080
#define RX_DESC_ENCRYPTION_MASK   0x0300
#define RX_DESC_MEASURMENT        0x0400
#define RX_DESC_SEQNUM_MASK       0x1800
#define	RX_DESC_MIC_FAIL	  0x2000
#define	RX_DESC_DECRYPT_FAIL	  0x4000

/*
 * RX Descriptor flags:
 *
 * Bits 0-1 - band
 * Bit  2   - STBC
 * Bit  3   - A-MPDU
 * Bit  4   - HT
 * Bits 5-7 - encryption
 */
#define WL1271_RX_DESC_BAND_MASK    0x03
#define WL1271_RX_DESC_ENCRYPT_MASK 0xE0

#define WL1271_RX_DESC_BAND_BG      0x00
#define WL1271_RX_DESC_BAND_J       0x01
#define WL1271_RX_DESC_BAND_A       0x02

#define WL1271_RX_DESC_STBC         BIT(2)
#define WL1271_RX_DESC_A_MPDU       BIT(3)
#define WL1271_RX_DESC_HT           BIT(4)

#define WL1271_RX_DESC_ENCRYPT_WEP  0x20
#define WL1271_RX_DESC_ENCRYPT_TKIP 0x40
#define WL1271_RX_DESC_ENCRYPT_AES  0x60
#define WL1271_RX_DESC_ENCRYPT_GEM  0x80

/*
 * RX Descriptor status
 *
 * Bits 0-2 - error code
 * Bits 3-5 - process_id tag (AP mode FW)
 * Bits 6-7 - reserved
 */
#define WL1271_RX_DESC_STATUS_MASK      0x03

#define WL1271_RX_DESC_SUCCESS          0x00
#define WL1271_RX_DESC_DECRYPT_FAIL     0x01
#define WL1271_RX_DESC_MIC_FAIL         0x02
#define WL1271_RX_DESC_DRIVER_RX_Q_FAIL 0x03

#define RX_MEM_BLOCK_MASK            0xFF
#define RX_BUF_SIZE_MASK             0xFFF00
#define RX_BUF_SIZE_SHIFT_DIV        6

/* 18xxTODO: make this depend on the SDIO block size? */
#define ALIGNED_RX_BUF_SIZE_MASK     0xFFFF00
#define ALIGNED_RX_BUF_SIZE_SHIFT    8


/* If set, the start of IP payload is not 4 bytes aligned */
#define RX_BUF_UNALIGNED_PAYLOAD     BIT(20)

enum {
	WL12XX_RX_CLASS_UNKNOWN,
	WL12XX_RX_CLASS_MANAGEMENT,
	WL12XX_RX_CLASS_DATA,
	WL12XX_RX_CLASS_QOS_DATA,
	WL12XX_RX_CLASS_BCN_PRBRSP,
	WL12XX_RX_CLASS_EAPOL,
	WL12XX_RX_CLASS_BA_EVENT,
	WL12XX_RX_CLASS_AMSDU,
	WL12XX_RX_CLASS_LOGGER,
};

struct wl1271_rx_descriptor {
	__le16 length;
	u8  status;
	u8  flags;
	u8  rate;
	u8  channel;
	s8  rssi;
	u8  snr;
	__le32 timestamp;
	u8  packet_class;
	u8  hlid;
	u8  pad_len;
	u8  reserved;
} __packed;

#define CFG_RX_FCS		BIT(2)
#define CFG_RX_ALL_GOOD		BIT(3)
#define CFG_UNI_FILTER_EN	BIT(4)
#define CFG_BSSID_FILTER_EN	BIT(5)
#define CFG_MC_FILTER_EN	BIT(6)
#define CFG_MC_ADDR0_EN		BIT(7)
#define CFG_MC_ADDR1_EN		BIT(8)
#define CFG_BC_REJECT_EN	BIT(9)
#define CFG_SSID_FILTER_EN	BIT(10)
#define CFG_RX_INT_FCS_ERROR	BIT(11)
#define CFG_RX_INT_ENCRYPTED	BIT(12)
#define CFG_RX_WR_RX_STATUS	BIT(13)
#define CFG_RX_FILTER_NULTI	BIT(14)
#define CFG_RX_RESERVE		BIT(15)
#define CFG_RX_TIMESTAMP_TSF	BIT(16)

/* 1 = use RX_BSSID_FILTER for data frames. */
#define CFG_RX_FLTR_BSSID_DATA  BIT(17)

/* 1 = use RX_BSSID_FILTER for management frames */
#define CFG_RX_FLTR_BSSID_MGMT  BIT(18)

/* syncing MAC DMV and the tail bytes fro PHY. should be 0. */
#define CFG_RX_NOT_WAIT_PPDU_END BIT(19)
#define CFG_RX_SIMU_AP_STA      BIT(30)

#define CFG_RX_RSV_EN		BIT(0)
#define CFG_RX_RCTS_ACK		BIT(1)
#define CFG_RX_PRSP_EN		BIT(2)
#define CFG_RX_PREQ_EN		BIT(3)
#define CFG_RX_MGMT_EN		BIT(4)
#define CFG_RX_FCS_ERROR	BIT(5)
#define CFG_RX_DATA_EN		BIT(6)
#define CFG_RX_CTL_EN		BIT(7)
#define CFG_RX_CF_EN		BIT(8)
#define CFG_RX_BCN_EN		BIT(9)
#define CFG_RX_AUTH_EN		BIT(10)
#define CFG_RX_ASSOC_EN		BIT(11)

/* 18xxTODO: put in wl->conf ? */
#define WL18XX_DEFAULT_STA_RX_CONFIG (CFG_UNI_FILTER_EN | \
				      CFG_BSSID_FILTER_EN | \
				      CFG_MC_FILTER_EN)

#define WL18XX_DEFAULT_STA_RX_FILTER (CFG_RX_RCTS_ACK | CFG_RX_PRSP_EN |  \
				      CFG_RX_MGMT_EN | CFG_RX_DATA_EN |   \
				      CFG_RX_CTL_EN | CFG_RX_BCN_EN |     \
				      CFG_RX_AUTH_EN | CFG_RX_ASSOC_EN)

#define WL18XX_DEFAULT_AP_RX_CONFIG  0

#define WL18XX_DEFAULT_AP_RX_FILTER  (CFG_RX_RCTS_ACK | CFG_RX_PREQ_EN | \
				      CFG_RX_MGMT_EN | CFG_RX_DATA_EN | \
				      CFG_RX_CTL_EN | CFG_RX_AUTH_EN | \
				      CFG_RX_ASSOC_EN)

void wl12xx_rx(struct wl1271 *wl, struct wl_fw_status *status);
u8 wl1271_rate_to_idx(int rate, enum ieee80211_band band);

#endif
