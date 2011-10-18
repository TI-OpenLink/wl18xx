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

struct wlcore {
	struct ieee80211_hw *hw;
	struct device *dev;

	spinlock_t wl_lock;
	struct mutex mutex;

	struct wlcore_if_ops *if_ops;

	bool mac80211_registered;
	unsigned long flags;

	struct ieee80211_supported_band bands[IEEE80211_NUM_BANDS];

	struct wlcore_partition_set part;

	/* TODO: is this really still needed? */
	__le32 buffer_32;
};

struct wlcore *wlcore_alloc_hw(void);
int wlcore_free_hw(struct wlcore *wl);
int wlcore_register_hw(struct wlcore *wl);
void wlcore_unregister_hw(struct wlcore *wl);

#endif
