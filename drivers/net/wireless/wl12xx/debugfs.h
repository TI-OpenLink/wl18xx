/*
 * This file is part of wl1271
 *
 * Copyright (C) 2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
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

#ifndef __DEBUGFS_H__
#define __DEBUGFS_H__

#include "wl12xx.h"

int wlcore_debugfs_init(struct wl1271 *wl);
void wlcore_debugfs_exit(struct wl1271 *wl);
void wlcore_debugfs_reset(struct wl1271 *wl);
int wlcore_debugfs_add_fw_files(struct wl1271 *wl);

#endif /* WL1271_DEBUGFS_H */
