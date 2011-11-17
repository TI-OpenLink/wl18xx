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

	ret = wlcore_vif_init(wl, vif);
	if (ret < 0)
		goto out;

	/* TODO: if we fail we should remove_vif and disable roles? */

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
	struct wlcore *wl = hw->priv;
	struct wlcore_vif *wlvif;
	struct ieee80211_conf *conf = &hw->conf;
	int channel, ret = 0;

	channel = ieee80211_frequency_to_channel(conf->channel->center_freq);

	wlcore_debug(DEBUG_MAC80211, "wlcore_config ch %d psm %s power %d %s"
		     " changed 0x%x",
		     channel,
		     conf->flags & IEEE80211_CONF_PS ? "on" : "off",
		     conf->power_level,
		     conf->flags & IEEE80211_CONF_IDLE ? "idle" : "in use",
		     changed);

	/* TODO: add wait tx_flush() */

	mutex_lock(&wl->mutex);

	/* we support configuring the channel and band even while off */
	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		wl->band = conf->channel->band;
		wl->channel = channel;
		wl->channel_type = conf->channel_type;
	}

	if (changed & IEEE80211_CONF_CHANGE_POWER)
		wl->power_level = conf->power_level;

	/* TODO: add check if we're off */

	/* TODO: add ELP wake-up */

#if 0
	/* configure each interface */
	wlcore_for_each_wlvif(wl, wlvif) {
		ret = wlcore_config_vif(wl, wlvif, conf, changed);
		if (ret < 0)
			goto out_sleep;
	}
#endif

out_sleep:
	/* TODO: add ELP sleep */

out:
	mutex_unlock(&wl->mutex);

	return ret;
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

static void wlcore_bss_info_changed_ap(struct wlcore *wl,
				       struct ieee80211_vif *vif,
				       struct ieee80211_bss_conf *bss_conf,
				       u32 changed)
{
	/* TODO: fill me up! */
}

static int wlcore_sta_handle_idle(struct wlcore *wl, struct wlcore_vif *wlvif,
				  bool idle)
{
	int ret;

	if (idle) {
		/* no need to croc if we weren't busy (e.g. during boot) */
		if (wlcore_is_roc(wl)) {
			ret = wlcore_stop_dev(wl, wlvif);
			if (ret < 0)
				goto out;
		}
		/* TODO: do more stuff */

	} else {
		/* TODO: stop sched_scan */

		ret = wlcore_start_dev(wl, wlvif);
		if (ret < 0)
			goto out;
	}

out:
	return ret;
}

static void wlcore_bss_info_changed_sta(struct wlcore *wl,
					struct ieee80211_vif *vif,
					struct ieee80211_bss_conf *bss_conf,
					u32 changed)
{
	struct wlcore_vif *wlvif = wlcore_vif_to_wlvif(vif);
	int ret;

	/* TODO: top me up! */

	if (changed & BSS_CHANGED_IDLE) {
		ret = wlcore_sta_handle_idle(wl, wlvif, bss_conf->idle);
		if (ret < 0)
			wlcore_warning("idle mode change failed %d", ret);
	}
}

void wlcore_bss_info_changed(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif,
			     struct ieee80211_bss_conf *bss_conf,
			     u32 changed)
{
	struct wlcore *wl = hw->priv;
	struct wlcore_vif *wlvif = wlcore_vif_to_wlvif(vif);

	wlcore_debug(DEBUG_MAC80211, "mac80211 bss info changed 0x%x",
		     (int)changed);

	mutex_lock(&wl->mutex);

	/* TODO: check wl->state and wlvif->flags */

	/* TODO: ELP wake-up */

	if (wlvif->bss_type == BSS_TYPE_AP_BSS)
		wlcore_bss_info_changed_ap(wl, vif, bss_conf, changed);
	else
		wlcore_bss_info_changed_sta(wl, vif, bss_conf, changed);

	/* TODO: ELP sleep */

	mutex_unlock(&wl->mutex);
}
