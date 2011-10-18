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

#include "wlcore.h"
#include "io.h"
#include "boot.h"
#include "debug.h"

bool wlcore_boot(struct wlcore *wl)
{
	bool booted = false;
	int ret;
	wlcore_debug(DEBUG_BOOT, "wlcore_boot");

	msleep(20); /* WL1271_PRE_POWER_ON_SLEEP */
	ret = wlcore_io_power_on(wl);
	if (ret < 0)
		goto out;
	msleep(200); /* WL1271_POWER_ON_SLEEP */
	wlcore_io_reset(wl);
	wlcore_io_init(wl);

	/*
	 * TODO: check if this works with wl12xx too and that it is
	 * okay to do it before set_partition
	 */
	wlcore_raw_write32(wl, HW_ACCESS_ELP_CTRL_REG_ADDR, ELPCTRL_WAKE_UP);

	/* TODO: we don't need this here, we only need to set the partition */
	ret = wl->ops->get_chip_id(wl);
	if (ret < 0)
		goto power_off;
	wl->chip_id = ret;

	booted = true;

	goto out;

power_off:
	wlcore_io_power_off(wl);
out:
	return booted;
}

void wlcore_shutdown(struct wlcore *wl)
{
	wlcore_debug(DEBUG_BOOT, "wlcore_shutdown");

	wlcore_io_power_off(wl);
}
