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

#include <linux/module.h>
#include <net/mac80211.h>

#include "wlcore.h"
#include "debug.h"
#include "mac80211_ops.h"

u32 wlcore_debug_level = DEBUG_ALL;

/* can't be const, mac80211 writes to this */
static struct ieee80211_rate wlcore_rates_2ghz[] = {
	{ .bitrate = 10,
	  .hw_value = WLCORE_HW_BIT_RATE_1MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_1MBPS, },
	{ .bitrate = 20,
	  .hw_value = WLCORE_HW_BIT_RATE_2MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_2MBPS,
	  .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 55,
	  .hw_value = WLCORE_HW_BIT_RATE_5_5MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_5_5MBPS,
	  .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 110,
	  .hw_value = WLCORE_HW_BIT_RATE_11MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_11MBPS,
	  .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 60,
	  .hw_value = WLCORE_HW_BIT_RATE_6MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_6MBPS, },
	{ .bitrate = 90,
	  .hw_value = WLCORE_HW_BIT_RATE_9MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_9MBPS, },
	{ .bitrate = 120,
	  .hw_value = WLCORE_HW_BIT_RATE_12MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_12MBPS, },
	{ .bitrate = 180,
	  .hw_value = WLCORE_HW_BIT_RATE_18MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_18MBPS, },
	{ .bitrate = 240,
	  .hw_value = WLCORE_HW_BIT_RATE_24MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_24MBPS, },
	{ .bitrate = 360,
	 .hw_value = WLCORE_HW_BIT_RATE_36MBPS,
	 .hw_value_short = WLCORE_HW_BIT_RATE_36MBPS, },
	{ .bitrate = 480,
	  .hw_value = WLCORE_HW_BIT_RATE_48MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_48MBPS, },
	{ .bitrate = 540,
	  .hw_value = WLCORE_HW_BIT_RATE_54MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_54MBPS, },
};

/* can't be const, mac80211 writes to this */
static struct ieee80211_channel wlcore_channels_2ghz[] = {
	{ .hw_value = 1, .center_freq = 2412, .max_power = 25 },
	{ .hw_value = 2, .center_freq = 2417, .max_power = 25 },
	{ .hw_value = 3, .center_freq = 2422, .max_power = 25 },
	{ .hw_value = 4, .center_freq = 2427, .max_power = 25 },
	{ .hw_value = 5, .center_freq = 2432, .max_power = 25 },
	{ .hw_value = 6, .center_freq = 2437, .max_power = 25 },
	{ .hw_value = 7, .center_freq = 2442, .max_power = 25 },
	{ .hw_value = 8, .center_freq = 2447, .max_power = 25 },
	{ .hw_value = 9, .center_freq = 2452, .max_power = 25 },
	{ .hw_value = 10, .center_freq = 2457, .max_power = 25 },
	{ .hw_value = 11, .center_freq = 2462, .max_power = 25 },
	{ .hw_value = 12, .center_freq = 2467, .max_power = 25 },
	{ .hw_value = 13, .center_freq = 2472, .max_power = 25 },
	{ .hw_value = 14, .center_freq = 2484, .max_power = 25 },
};

/* 11n STA capabilities */
#define HW_RX_HIGHEST_RATE	72

#define WL12XX_HT_CAP { \
	.cap = IEEE80211_HT_CAP_GRN_FLD | IEEE80211_HT_CAP_SGI_20 | \
	       (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT), \
	.ht_supported = true, \
	.ampdu_factor = IEEE80211_HT_MAX_AMPDU_8K, \
	.ampdu_density = IEEE80211_HT_MPDU_DENSITY_8, \
	.mcs = { \
		.rx_mask = { 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0, }, \
		.rx_highest = cpu_to_le16(HW_RX_HIGHEST_RATE), \
		.tx_params = IEEE80211_HT_MCS_TX_DEFINED, \
		}, \
}

/* can't be const, mac80211 writes to this */
static struct ieee80211_supported_band wlcore_band_2ghz = {
	.channels = wlcore_channels_2ghz,
	.n_channels = ARRAY_SIZE(wlcore_channels_2ghz),
	.bitrates = wlcore_rates_2ghz,
	.n_bitrates = ARRAY_SIZE(wlcore_rates_2ghz),
	.ht_cap	= WL12XX_HT_CAP,
};

/* 5 GHz data rates for WL1273 */
static struct ieee80211_rate wlcore_rates_5ghz[] = {
	{ .bitrate = 60,
	  .hw_value = WLCORE_HW_BIT_RATE_6MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_6MBPS, },
	{ .bitrate = 90,
	  .hw_value = WLCORE_HW_BIT_RATE_9MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_9MBPS, },
	{ .bitrate = 120,
	  .hw_value = WLCORE_HW_BIT_RATE_12MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_12MBPS, },
	{ .bitrate = 180,
	  .hw_value = WLCORE_HW_BIT_RATE_18MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_18MBPS, },
	{ .bitrate = 240,
	  .hw_value = WLCORE_HW_BIT_RATE_24MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_24MBPS, },
	{ .bitrate = 360,
	 .hw_value = WLCORE_HW_BIT_RATE_36MBPS,
	 .hw_value_short = WLCORE_HW_BIT_RATE_36MBPS, },
	{ .bitrate = 480,
	  .hw_value = WLCORE_HW_BIT_RATE_48MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_48MBPS, },
	{ .bitrate = 540,
	  .hw_value = WLCORE_HW_BIT_RATE_54MBPS,
	  .hw_value_short = WLCORE_HW_BIT_RATE_54MBPS, },
};

/* 5 GHz band channels for WL1273 */
static struct ieee80211_channel wlcore_channels_5ghz[] = {
	{ .hw_value = 7, .center_freq = 5035, .max_power = 25 },
	{ .hw_value = 8, .center_freq = 5040, .max_power = 25 },
	{ .hw_value = 9, .center_freq = 5045, .max_power = 25 },
	{ .hw_value = 11, .center_freq = 5055, .max_power = 25 },
	{ .hw_value = 12, .center_freq = 5060, .max_power = 25 },
	{ .hw_value = 16, .center_freq = 5080, .max_power = 25 },
	{ .hw_value = 34, .center_freq = 5170, .max_power = 25 },
	{ .hw_value = 36, .center_freq = 5180, .max_power = 25 },
	{ .hw_value = 38, .center_freq = 5190, .max_power = 25 },
	{ .hw_value = 40, .center_freq = 5200, .max_power = 25 },
	{ .hw_value = 42, .center_freq = 5210, .max_power = 25 },
	{ .hw_value = 44, .center_freq = 5220, .max_power = 25 },
	{ .hw_value = 46, .center_freq = 5230, .max_power = 25 },
	{ .hw_value = 48, .center_freq = 5240, .max_power = 25 },
	{ .hw_value = 52, .center_freq = 5260, .max_power = 25 },
	{ .hw_value = 56, .center_freq = 5280, .max_power = 25 },
	{ .hw_value = 60, .center_freq = 5300, .max_power = 25 },
	{ .hw_value = 64, .center_freq = 5320, .max_power = 25 },
	{ .hw_value = 100, .center_freq = 5500, .max_power = 25 },
	{ .hw_value = 104, .center_freq = 5520, .max_power = 25 },
	{ .hw_value = 108, .center_freq = 5540, .max_power = 25 },
	{ .hw_value = 112, .center_freq = 5560, .max_power = 25 },
	{ .hw_value = 116, .center_freq = 5580, .max_power = 25 },
	{ .hw_value = 120, .center_freq = 5600, .max_power = 25 },
	{ .hw_value = 124, .center_freq = 5620, .max_power = 25 },
	{ .hw_value = 128, .center_freq = 5640, .max_power = 25 },
	{ .hw_value = 132, .center_freq = 5660, .max_power = 25 },
	{ .hw_value = 136, .center_freq = 5680, .max_power = 25 },
	{ .hw_value = 140, .center_freq = 5700, .max_power = 25 },
	{ .hw_value = 149, .center_freq = 5745, .max_power = 25 },
	{ .hw_value = 153, .center_freq = 5765, .max_power = 25 },
	{ .hw_value = 157, .center_freq = 5785, .max_power = 25 },
	{ .hw_value = 161, .center_freq = 5805, .max_power = 25 },
	{ .hw_value = 165, .center_freq = 5825, .max_power = 25 },
};

static struct ieee80211_supported_band wlcore_band_5ghz = {
	.channels = wlcore_channels_5ghz,
	.n_channels = ARRAY_SIZE(wlcore_channels_5ghz),
	.bitrates = wlcore_rates_5ghz,
	.n_bitrates = ARRAY_SIZE(wlcore_rates_5ghz),
	.ht_cap	= WL12XX_HT_CAP,
};

static const struct ieee80211_ops wlcore_ops = {
	.tx		  = wlcore_tx,
	.start		  = wlcore_start,
	.stop		  = wlcore_stop,
	.config		  = wlcore_config,
	.add_interface	  = wlcore_add_interface,
	.remove_interface = wlcore_remove_interface,
	.configure_filter = wlcore_configure_filter,
};

static void wlcore_init_ieee80211(struct wlcore *wl, struct device *dev)
{
	/*
	 * We keep local copies of the band structs because we need to
	 * modify them on a per-device basis.
	 */
	memcpy(&wl->bands[IEEE80211_BAND_2GHZ], &wlcore_band_2ghz,
	       sizeof(wlcore_band_2ghz));
	memcpy(&wl->bands[IEEE80211_BAND_5GHZ], &wlcore_band_5ghz,
	       sizeof(wlcore_band_5ghz));

	wl->hw->wiphy->bands[IEEE80211_BAND_2GHZ] =
		&wl->bands[IEEE80211_BAND_2GHZ];
	wl->hw->wiphy->bands[IEEE80211_BAND_5GHZ] =
		&wl->bands[IEEE80211_BAND_5GHZ];

	wl->hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);

	SET_IEEE80211_DEV(wl->hw, dev);
}

struct wlcore *wlcore_alloc_hw(void)
{
	struct ieee80211_hw *hw;
	struct wlcore *wl;
	int ret;

	hw = ieee80211_alloc_hw(sizeof(*wl), &wlcore_ops);
	if (!hw) {
		wlcore_error("could not alloc ieee80211_hw");
		ret = -ENOMEM;
		goto out;
	}

	wl = hw->priv;
	memset(wl, 0, sizeof(*wl));

	wl->hw = hw;

	spin_lock_init(&wl->wl_lock);

	mutex_init(&wl->mutex);

	return wl;

out:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(wlcore_alloc_hw);

int wlcore_free_hw(struct wlcore *wl)
{
	ieee80211_free_hw(wl->hw);

	return 0;
}
EXPORT_SYMBOL_GPL(wlcore_free_hw);

int wlcore_register_hw(struct wlcore *wl)
{
	int ret;

	if (wl->mac80211_registered)
		return 0;

	/* TODO: check that all the required parameters are correctly set */

	wlcore_init_ieee80211(wl, wl->dev);

	ret = ieee80211_register_hw(wl->hw);
	if (ret < 0) {
		wlcore_error("unable to register mac80211 hw: %d", ret);
		return ret;
	}

	wl->mac80211_registered = true;

	return 0;
}
EXPORT_SYMBOL_GPL(wlcore_register_hw);

void wlcore_unregister_hw(struct wlcore *wl)
{
	ieee80211_unregister_hw(wl->hw);
	wl->mac80211_registered = false;
}
EXPORT_SYMBOL_GPL(wlcore_unregister_hw);

static int __init wlcore_init(void)
{
	return 0;
}
module_init(wlcore_init);

static void __exit wlcore_exit(void)
{
}
module_exit(wlcore_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Luciano Coelho <coelho@ti.com>");
