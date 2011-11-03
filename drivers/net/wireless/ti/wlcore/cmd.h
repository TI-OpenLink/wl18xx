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

int wlcore_cmd_send(struct wlcore *wl, u16 id, void *buf, size_t len,
		    size_t res_len);
int wlcore_cmd_configure(struct wlcore *wl, u16 id, void *buf, size_t len);

#endif /* __CMD_H__ */
