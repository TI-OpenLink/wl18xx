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

static const struct ieee80211_ops wlcore_ops = {
	.tx		  = wlcore_tx,
	.start		  = wlcore_start,
	.stop		  = wlcore_stop,
	.config		  = wlcore_config,
	.add_interface	  = wlcore_add_interface,
	.remove_interface = wlcore_remove_interface,
	.configure_filter = wlcore_configure_filter,
};

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
