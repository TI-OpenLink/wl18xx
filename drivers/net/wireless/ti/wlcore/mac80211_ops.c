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

#include "debug.h"
#include "mac80211_ops.h"

void wlcore_tx(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	wlcore_debug(DEBUG_MAC80211, "wlcore_tx");
}

int wlcore_start(struct ieee80211_hw *hw)
{
	wlcore_debug(DEBUG_MAC80211, "wlcore_start");

	return 0;
}

void wlcore_stop(struct ieee80211_hw *hw)
{
	wlcore_debug(DEBUG_MAC80211, "wlcore_stop");
}

int wlcore_add_interface(struct ieee80211_hw *hw,
			 struct ieee80211_vif *vif)
{
	wlcore_debug(DEBUG_MAC80211, "wlcore_add_interface");

	return 0;
}

void wlcore_remove_interface(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif)
{
	wlcore_debug(DEBUG_MAC80211, "wlcore_remove_interface");
}

int wlcore_config(struct ieee80211_hw *hw, u32 changed)
{
	wlcore_debug(DEBUG_MAC80211, "wlcore_config");

	return 0;
}

void wlcore_configure_filter(struct ieee80211_hw *hw,
			     unsigned int changed_flags,
			     unsigned int *total_flags,
			     u64 multicast)
{
	wlcore_debug(DEBUG_MAC80211, "wlcore_configure_filter");

	/* clear all flags, since we don't pass anything up yet */
	*total_flags = 0;
}
