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
#include "wlcore.h"
#include "mac80211_ops.h"
#include "boot.h"
#include "init.h"
#include "cmd.h"
#include "vif.h"

void wlcore_tx(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	wlcore_debug(DEBUG_MAC80211, "wlcore_tx");
}

int wlcore_start(struct ieee80211_hw *hw)
{
	struct wlcore *wl = hw->priv;
	int ret;

	wlcore_debug(DEBUG_MAC80211, "wlcore_start");

	mutex_lock(&wl->mutex);

	ret = wlcore_boot(wl);
	if (ret < 0)
		goto out;

	ret = wlcore_hw_init(wl);
	if (ret < 0)
		goto out_shutdown;

	goto out;

out_shutdown:
	wlcore_shutdown(wl);
out:
	mutex_unlock(&wl->mutex);
	return ret;
}

void wlcore_stop(struct ieee80211_hw *hw)
{
	struct wlcore *wl = hw->priv;

	wlcore_debug(DEBUG_MAC80211, "wlcore_stop");
	mutex_lock(&wl->mutex);

	/*
	 * TODO: should we memset everything to zero and reinitialize
	 * everything as in add_interface? as in wlcore_alloc_hw()?
	 * Maybe even dealloc and alloc_hw again to be sure? Or is
	 * there some persistent data that must remain?
	 */

	wl->channel = 0;
	wl->power_level = 0;
	wl->channel_type = NL80211_CHAN_NO_HT;
	wl->band = IEEE80211_BAND_2GHZ;

	wlcore_shutdown(wl);

	mutex_unlock(&wl->mutex);
}

int wlcore_add_interface(struct ieee80211_hw *hw,
			 struct ieee80211_vif *vif)
{
	struct wlcore *wl = hw->priv;
	struct wlcore_vif *wlvif = wlcore_vif_to_wlvif(vif);
	u8 role_type;
	int ret;

	wlcore_debug(DEBUG_MAC80211, "wlcore_add_interface");
	mutex_lock(&wl->mutex);

	ret = wlcore_vif_add(wl, vif);
	if (ret < 0)
		goto out;

	role_type = wlcore_vif_to_role_type(vif);
	if (role_type == WLCORE_ROLE_INVALID) {
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (wlvif->bss_type == BSS_TYPE_STA_BSS ||
	    wlvif->bss_type == BSS_TYPE_IBSS) {
		/*
		 * The device role is a special role used for
		 * rx and tx frames prior to association (as
		 * the STA role can get packets only from
		 * its associated bssid)
		 */
		ret = wlcore_cmd_role_enable(wl, vif->addr,
					     WLCORE_ROLE_DEVICE,
					     &wlvif->dev_role_id);
		if (ret < 0)
			goto out;
	}

	ret = wlcore_cmd_role_enable(wl, vif->addr,
				     role_type, &wlvif->role_id);
	if (ret < 0)
		goto out;
out:
	mutex_unlock(&wl->mutex);
	return ret;
}

void wlcore_remove_interface(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif)
{
	struct wlcore *wl = hw->priv;
	struct wlcore_vif *wlvif = wlcore_vif_to_wlvif(vif);
	int ret;

	wlcore_debug(DEBUG_MAC80211, "wlcore_remove_interface");
	mutex_lock(&wl->mutex);

	/*
	 * TODO: check if there was any good reason for not disabling
	 * the device role with IBSS or if it was just a bug in
	 * wl12xx.
	 */
	if (wlvif->bss_type == BSS_TYPE_STA_BSS ||
	    wlvif->bss_type == BSS_TYPE_IBSS) {
		ret = wlcore_cmd_role_disable(wl, &wlvif->dev_role_id);
		if (ret < 0)
			goto deinit;
	}

	ret = wlcore_cmd_role_disable(wl, &wlvif->role_id);
	if (ret < 0)
		goto deinit;

deinit:
	wlcore_vif_remove(wl, vif);
	mutex_unlock(&wl->mutex);
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
