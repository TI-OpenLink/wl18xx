/*
 * This file is part of wlcore
 *
 * Copyright (C) 1998-2009, 2011 Texas Instruments. All rights reserved.
 * Copyright (C) 2008-2009 Nokia Corporation
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

#ifndef __WLCORE_H__
#define __WLCORE_H__

#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <net/mac80211.h>

enum {
	WLCORE_HW_BIT_RATE_1MBPS   = BIT(0),
	WLCORE_HW_BIT_RATE_2MBPS   = BIT(1),
	WLCORE_HW_BIT_RATE_5_5MBPS = BIT(2),
	WLCORE_HW_BIT_RATE_6MBPS   = BIT(3),
	WLCORE_HW_BIT_RATE_9MBPS   = BIT(4),
	WLCORE_HW_BIT_RATE_11MBPS  = BIT(5),
	WLCORE_HW_BIT_RATE_12MBPS  = BIT(6),
	WLCORE_HW_BIT_RATE_18MBPS  = BIT(7),
	WLCORE_HW_BIT_RATE_22MBPS  = BIT(8),
	WLCORE_HW_BIT_RATE_24MBPS  = BIT(9),
	WLCORE_HW_BIT_RATE_36MBPS  = BIT(10),
	WLCORE_HW_BIT_RATE_48MBPS  = BIT(11),
	WLCORE_HW_BIT_RATE_54MBPS  = BIT(12),
	WLCORE_HW_BIT_RATE_MCS_0   = BIT(13),
	WLCORE_HW_BIT_RATE_MCS_1   = BIT(14),
	WLCORE_HW_BIT_RATE_MCS_2   = BIT(15),
	WLCORE_HW_BIT_RATE_MCS_3   = BIT(16),
	WLCORE_HW_BIT_RATE_MCS_4   = BIT(17),
	WLCORE_HW_BIT_RATE_MCS_5   = BIT(18),
	WLCORE_HW_BIT_RATE_MCS_6   = BIT(19),
	WLCORE_HW_BIT_RATE_MCS_7   = BIT(20)
};

#define WLCORE_TX_RATE_MASK_BASIC	(WLCORE_HW_BIT_RATE_1MBPS  | \
					 WLCORE_HW_BIT_RATE_2MBPS)
#define WLCORE_TX_RATE_MASK_BASIC_P2P	(WLCORE_HW_BIT_RATE_6MBPS  | \
					 WLCORE_HW_BIT_RATE_12MBPS | \
					 WLCORE_HW_BIT_RATE_24MBPS)

enum {
	WLCORE_HW_RATE_INDEX_1MBPS   = 0,
	WLCORE_HW_RATE_INDEX_2MBPS   = 1,
	WLCORE_HW_RATE_INDEX_5_5MBPS = 2,
	WLCORE_HW_RATE_INDEX_6MBPS   = 3,
	WLCORE_HW_RATE_INDEX_9MBPS   = 4,
	WLCORE_HW_RATE_INDEX_11MBPS  = 5,
	WLCORE_HW_RATE_INDEX_12MBPS  = 6,
	WLCORE_HW_RATE_INDEX_18MBPS  = 7,
	WLCORE_HW_RATE_INDEX_22MBPS  = 8,
	WLCORE_HW_RATE_INDEX_24MBPS  = 9,
	WLCORE_HW_RATE_INDEX_36MBPS  = 10,
	WLCORE_HW_RATE_INDEX_48MBPS  = 11,
	WLCORE_HW_RATE_INDEX_54MBPS  = 12,
	WLCORE_HW_RATE_INDEX_MAX     = WLCORE_HW_RATE_INDEX_54MBPS,
};

enum {
	WLCORE_HW_RXTX_RATE_MCS7 = 0,
	WLCORE_HW_RXTX_RATE_MCS6,
	WLCORE_HW_RXTX_RATE_MCS5,
	WLCORE_HW_RXTX_RATE_MCS4,
	WLCORE_HW_RXTX_RATE_MCS3,
	WLCORE_HW_RXTX_RATE_MCS2,
	WLCORE_HW_RXTX_RATE_MCS1,
	WLCORE_HW_RXTX_RATE_MCS0,
	WLCORE_HW_RXTX_RATE_54,
	WLCORE_HW_RXTX_RATE_48,
	WLCORE_HW_RXTX_RATE_36,
	WLCORE_HW_RXTX_RATE_24,
	WLCORE_HW_RXTX_RATE_22,
	WLCORE_HW_RXTX_RATE_18,
	WLCORE_HW_RXTX_RATE_12,
	WLCORE_HW_RXTX_RATE_11,
	WLCORE_HW_RXTX_RATE_9,
	WLCORE_HW_RXTX_RATE_6,
	WLCORE_HW_RXTX_RATE_5_5,
	WLCORE_HW_RXTX_RATE_2,
	WLCORE_HW_RXTX_RATE_1,
	WLCORE_HW_RXTX_RATE_MAX,
	WLCORE_HW_RXTX_RATE_UNSUPPORTED = 0xff
};

#define WLCORE_DEFAULT_BEACON_INT	100
#define WLCORE_DEFAULT_DTIM_PERIOD	1

enum wlcore_flags {
	WLCORE_FLAG_GPIO_POWER,
};

struct wlcore_if_ops {
	void (*read)(struct device *child, int addr, void *buf, size_t len,
		     bool fixed);
	void (*write)(struct device *child, int addr, void *buf, size_t len,
		     bool fixed);
	void (*reset)(struct device *child);
	void (*init)(struct device *child);
	int (*power)(struct device *child, bool enable);
	void (*set_block_size) (struct device *child, unsigned int blksz);
};

struct wlcore;

struct wlcore_ops {
	int (*get_chip_id)(struct wlcore *wl);
	int (*config_pll)(struct wlcore *wl);
	int (*cfg_host_if)(struct wlcore *wl);
	void (*preboot_conf)(struct wlcore *wl);
};

enum wlcore_partitions {
	PART_TOP_PRCM_ELP_SOC,
	PART_DOWN,
	PART_BOOT,
	PART_PHY_INIT,

	PART_TABLE_LEN,
};

struct wlcore_partition {
	u32 size;
	u32 start;
};

struct wlcore_partition_set {
	struct wlcore_partition mem;
	struct wlcore_partition reg;
	struct wlcore_partition mem2;
	struct wlcore_partition mem3;
};

enum wlcore_registers {
	REG_ECPU_CONTROL,
	REG_INTERRUPT_NO_CLEAR,
	REG_INTERRUPT_ACK,
	REG_COMMAND_MAILBOX_PTR,
	REG_EVENT_MAILBOX_PTR,
	REG_INTERRUPT_TRIG_L,
	REG_INTERRUPT_TRIG_H,
	REG_INTERRUPT_MASK,

	REG_TABLE_LEN,
};

enum wlcore_triggers {
	TRIG_CMD,
	TRIG_EVENT_ACK,

	TRIG_TABLE_LEN,
};

/* flag to halt the embedded CPU, used with REG_ECPU_CONTROL */
#define ECPU_CONTROL_HALT	0x00000101

/* interrupt flags, used with REG_INTERRUPT_* */
#define INTR_WATCHDOG		BIT(0)
#define INTR_INIT_COMPLETE	BIT(1)
#define INTR_EVENT_A		BIT(2)
#define INTR_EVENT_B		BIT(3)
#define INTR_CMD_COMPLETE	BIT(4)
#define INTR_HW_AVAILABLE	BIT(5)
#define INTR_DATA		BIT(6)
#define INTR_TRACE_A		BIT(7)
#define INTR_TRACE_B		BIT(8)
#define INTR_ALL		0xFFFFFFFF

#define INTR_MASK_DEFAULT	(INTR_WATCHDOG     | \
				 INTR_EVENT_A      | \
				 INTR_EVENT_B      | \
				 INTR_HW_AVAILABLE | \
				 INTR_DATA)

#define INIT_LOOP	20000
#define INIT_LOOP_DELAY	50

enum {
	FW_VER_CHIP,
	FW_VER_IF_TYPE,
	FW_VER_MAJOR,
	FW_VER_SUBTYPE,
	FW_VER_MINOR,

	NUM_FW_VER
};

#define WLCORE_FW_VER_MAX_LEN	20
#define WLCORE_NO_SUBBANDS	8
#define WLCORE_NO_POWER_LEVELS	4

/* TODO: static data may differ from chip to chip, should move to an op */
struct wlcore_static_data {
	u8 mac_address[ETH_ALEN];
	u8 padding[2];
	u8 fw_version[WLCORE_FW_VER_MAX_LEN];
	u32 hw_version;
	u8 tx_power_table[WLCORE_NO_SUBBANDS][WLCORE_NO_POWER_LEVELS];
} __packed;

struct wlcore_conf_tx_rate_class {

	/*
	 * The rates enabled for this rate class.
	 *
	 * Range: CONF_HW_BIT_RATE_* bit mask
	 */
	u32 enabled_rates;

	/*
	 * The dot11 short retry limit used for TX retries.
	 *
	 * Range: u8
	 */
	u8 short_retry_limit;

	/*
	 * The dot11 long retry limit used for TX retries.
	 *
	 * Range: u8
	 */
	u8 long_retry_limit;

	/*
	 * Flags controlling the attributes of TX transmission.
	 *
	 * Range: bit 0: Truncate - when set, FW attempts to send a frame stop
	 *               when the total valid per-rate attempts have
	 *               been exhausted; otherwise transmissions
	 *               will continue at the lowest available rate
	 *               until the appropriate one of the
	 *               short_retry_limit, long_retry_limit,
	 *               dot11_max_transmit_msdu_life_time, or
	 *               max_tx_life_time, is exhausted.
	 *            1: Preamble Override - indicates if the preamble type
	 *               should be used in TX.
	 *            2: Preamble Type - the type of the preamble to be used by
	 *               the policy (0 - long preamble, 1 - short preamble.
	 */
	u8 aflags;
};

struct wlcore_conf_tx {
	/* TX retry limits for templates */
	u8 tmpl_short_retry_limit;
	u8 tmpl_long_retry_limit;

	/*
	 * Maximum time to wait for the packets threshold to be
	 * reached before issuing the TX complete interrupt.
	 */
	u16 complete_timeout;

	/*
	 * Number of packets to be completed before issuing the TX
	 * complete interrupt
	 */
	u16 complete_threshold;

	/* enable/disable TX energy detection for TELEC */
	u8 energy_detection;

	/* the rate used for control messages and scanning on the 2.4GHz band */
	u32 basic_rate;

	/* the rate used for control messages and scanning on the 5GHz band */
	u32 basic_rate_5;

	struct wlcore_conf_tx_rate_class sta_rc;
};

enum {
	RX_QUEUE_TYPE_LOW_PRIORITY,	/* All except the high priority */
	RX_QUEUE_TYPE_HIGH_PRIORITY,	/* Management and voice packets */
};

struct wlcore_conf_rx {
	/*
	 * The maximum amount of time, in TU, before the
	 * firmware discards the MSDU.
	 *
	 * Range: 0 - 0xFFFFFFFF
	 */
	u32 msdu_lifetime;

	/*
	 * Occupied Rx mem-blocks number which requires interrupting the host
	 * (0 = no buffering, 0xffff = disabled).
	 *
	 * Range: u16
	 */
	u16 irq_blk_threshold;

	/*
	 * Rx packets number which requires interrupting the host
	 * (0 = no buffering).
	 *
	 * Range: u16
	 */
	u16 irq_pkt_threshold;

	/*
	 * Max time in msec the FW may delay RX-Complete interrupt.
	 *
	 * Range: 1 - 100
	 */
	u16 irq_timeout;

	/*
	 * The RX queue type.
	 *
	 * Range: RX_QUEUE_TYPE_LOW_PRIORITY, RX_QUEUE_TYPE_HIGH_PRIORITY
	 */
	u8 queue_type;
};

struct wlcore_conf_hw_mem {
	/* Number of stations supported in IBSS mode */
	u8 num_stations;

	/* Number of ssid profiles used in IBSS mode */
	u8 ssid_profiles;

	/* Number of memory buffers allocated to rx pool */
	u8 rx_block_num;

	/* Minimum number of blocks allocated to tx pool */
	u8 tx_min_block_num;

	/* Disable/Enable dynamic memory */
	u8 dynamic_memory;

	/*
	 * Minimum required free tx memory blocks in order to assure optimum
	 * performance
	 *
	 * Range: 0-120
	 */
	u8 min_req_tx_blocks;

	/*
	 * Minimum required free rx memory blocks in order to assure optimum
	 * performance
	 *
	 * Range: 0-120
	 */
	u8 min_req_rx_blocks;

	/*
	 * Minimum number of mem blocks (free+used) guaranteed for TX
	 *
	 * Range: 0-120
	 */
	u8 tx_min;

	/* Number of TX descriptors */
	u8 num_tx_descriptors;
};

#define CONF_BCN_IE_OUI_LEN		3
#define CONF_BCN_IE_VER_LEN		2

/* 32 normal IEs plus 6 vendor-specific ones */
#define CONF_MAX_BCN_FILT_IE_COUNT	38

struct wlcore_conf_bcn_filter {
	/* IE type to let through */
	u8 ie;

	/* pass-through rule (ACX_BCN_RUL_PASS_ON_*) */
	u8 rule;

	/* vendor specific OUI */
	u8 oui[CONF_BCN_IE_OUI_LEN];

	/* vendor specific type */
	u8 type;

	/* vendor specific version */
	u8 version[CONF_BCN_IE_VER_LEN];
};

struct wlcore_conf_conn_settings {
	/*
	 * enable or disable the beacon filtering */
	bool enable_bcn_filter;

	/* configure beacon filter pass-thru rules */
	u8 bcn_filter_count;
	struct wlcore_conf_bcn_filter bcn_filter[CONF_MAX_BCN_FILT_IE_COUNT];

	/*
	 * The number of consecutive beacons to lose, before the firmware
	 * becomes out of sync.
	 */
	u32 sync_fail_threshold;

	/*
	 * After out-of-synch, the number of TU's to wait without a further
	 * received beacon (or probe response) before issuing the BSS_EVENT_LOSE
	 * event.
	 */
	u32 bss_lose_timeout;

	/* beacon receive timeout */
	u32 beacon_rx_timeout;

	/* broadcast receive timeout */
	u32 broadcast_timeout;

	/* enable/disable reception of broadcast packets in power save mode */
	bool rx_broadcast_in_ps;

	/* consecutive PS Poll failures before sending event to driver */
	u8 ps_poll_threshold;

	/*
	 * Specifies the maximum number of times to try PSM entry if it fails
	 * (if sending the appropriate null-func message fails.)
	 */
	u8 psm_entry_retries;

	/*
	 * Specifies the maximum number of times to try PSM exit if it fails
	 * (if sending the appropriate null-func message fails.)
	 */
	u8 psm_exit_retries;
};

struct wlcore_conf_pm {
	u32 host_clk_settling_time;	/* range: 0 - 30000 us */
	bool host_fast_wakeup_support;
};

/* this should be defined in acx.h, but it would cause circular deps */
#define ACX_RATE_MGMT_NUM_OF_RATES	13

struct wlcore_conf_rate_policy {
	u16 rate_retry_score;
	u16 per_add;
	u16 per_th1;
	u16 per_th2;
	u16 max_per;
	u8 inverse_curiosity_factor;
	u8 tx_fail_low_th;
	u8 tx_fail_high_th;
	u8 per_alpha_shift;
	u8 per_add_shift;
	u8 per_beta1_shift;
	u8 per_beta2_shift;
	u8 rate_check_up;
	u8 rate_check_down;
	u8 rate_retry_policy[ACX_RATE_MGMT_NUM_OF_RATES];
};

struct wlcore_conf_hangover {
	u32 recover_time;
	u8 hangover_period;
	u8 dynamic_mode;
	u8 early_termination_mode;
	u8 max_period;
	u8 min_period;
	u8 increase_delta;
	u8 decrease_delta;
	u8 quiet_time;
	u8 increase_time;
	u8 window_size;
};

struct wlcore_conf_roam_trigger {
	/* the minimum interval between two trigger events (0 - 60000 ms) */
	u16 trigger_pacing;

	/* the weight for rssi/beacon average calculation */
	u8 avg_weight_rssi_beacon;

	/* the weight for rssi/data frame average calculation */
	u8 avg_weight_rssi_data;

	/* the weight for snr/beacon average calculation */
	u8 avg_weight_snr_beacon;

	/* the weight for snr/data frame average calculation */
	u8 avg_weight_snr_data;
};

struct wlcore_conf {
	struct wlcore_conf_tx tx;
	struct wlcore_conf_rx rx;
	struct wlcore_conf_hw_mem hw_mem;
	struct wlcore_conf_conn_settings conn;
	struct wlcore_conf_pm pm_config;
	struct wlcore_conf_rate_policy rate;
	struct wlcore_conf_hangover hangover;
	struct wlcore_conf_roam_trigger roam_trigger;

	/* private data used only by the lower driver */
	u8 priv_data[0] __attribute__((__aligned__(sizeof(void *))));
};

#define WLCORE_MAX_ROLES		4
#define WLCORE_MAX_LINKS		12
#define WLCORE_MAX_RATE_POLICIES	16

#define WLCORE_INVALID_ROLE_ID	0xFF
#define WLCORE_INVALID_LINK_ID	0xFF

/* TODO: separate local stuff from lower-driver accessible parts */
struct wlcore {
	struct ieee80211_hw *hw;
	struct device *dev;

	spinlock_t wl_lock;
	struct mutex mutex;

	struct wlcore_if_ops *if_ops;
	struct wlcore_ops *ops;

	/* Platform limitations */
	unsigned int platform_quirks;

	const char *devname;
	int irq;

	bool mac80211_registered;
	unsigned long flags;

	unsigned long links_map[BITS_TO_LONGS(WLCORE_MAX_LINKS)];
	unsigned long roles_map[BITS_TO_LONGS(WLCORE_MAX_ROLES)];
	unsigned long roc_map[BITS_TO_LONGS(WLCORE_MAX_ROLES)];
	unsigned long rate_policies_map[
		BITS_TO_LONGS(WLCORE_MAX_RATE_POLICIES)];

	struct list_head wlvif_list;

	struct ieee80211_supported_band bands[IEEE80211_NUM_BANDS];

	/* pointer to the lower driver partition table */
	const struct wlcore_partition_set *ptable;

	/* partition currently in use -- needed for address translation */
	struct wlcore_partition_set curr_part;

	/* pointer to the lower driver register table */
	const int *rtable;

	/* pointer to the lower driver table of interrupt trigger values */
	const u64 *trig_table;

	struct wlcore_conf *conf;

	u32 chip_id;

	const char *fw_name;
	u8 *fw;
	size_t fw_len;

	const char *nvs_name;
	u8 *nvs;
	size_t nvs_len;

	char fw_ver_str[ETHTOOL_BUSINFO_LEN];
	unsigned int fw_ver[NUM_FW_VER];

	int cmd_box_addr;
	int event_box_addr;

	struct wlcore_acx_mem_map *mem_map;

	u32 tx_blocks_available;

	int power_level;	/* in dBm */
	int channel;
	enum ieee80211_band band;
	enum nl80211_channel_type channel_type;

	/* TODO: is this really still needed? */
	__le32 buffer_32;
};

struct wlcore *wlcore_alloc_hw(void);
int wlcore_free_hw(struct wlcore *wl);
int wlcore_register_hw(struct wlcore *wl);
void wlcore_unregister_hw(struct wlcore *wl);

#endif
