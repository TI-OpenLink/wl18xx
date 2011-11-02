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
};

enum wlcore_partitions {
	PART_TOP_PRCM_ELP_SOC,
	PART_DOWN,
	PART_BOOT,

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

	REG_TABLE_LEN,
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

#define INIT_LOOP	20000
#define INIT_LOOP_DELAY	50

/* TODO: separate local stuff from lower-driver accessible parts */
struct wlcore {
	struct ieee80211_hw *hw;
	struct device *dev;

	spinlock_t wl_lock;
	struct mutex mutex;

	struct wlcore_if_ops *if_ops;
	struct wlcore_ops *ops;

	bool mac80211_registered;
	unsigned long flags;

	struct ieee80211_supported_band bands[IEEE80211_NUM_BANDS];

	/* pointer to the lower driver partition table */
	const struct wlcore_partition_set *ptable;

	/* partition currently in use -- needed for address translation */
	struct wlcore_partition_set curr_part;

	/* pointer to the lower driver register table */
	const int *rtable;

	u32 chip_id;

	const char *fw_name;
	u8 *fw;
	size_t fw_len;

	const char *nvs_name;
	u8 *nvs;
	size_t nvs_len;

	int cmd_box_addr;
	int event_box_addr;

	/* TODO: is this really still needed? */
	__le32 buffer_32;
};

struct wlcore *wlcore_alloc_hw(void);
int wlcore_free_hw(struct wlcore *wl);
int wlcore_register_hw(struct wlcore *wl);
void wlcore_unregister_hw(struct wlcore *wl);

#endif
