/*
 * This file is part of wl18xx
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

#include "../wlcore/wlcore.h"
#include "acx.h"

int wl18xx_acx_host_if_cfg_bitmap(struct wlcore *wl, u32 host_cfg_bitmap,
				  u32 sdio_blk_size)
{
	struct wl18xx_acx_host_config_bitmap *bitmap_conf;
	int ret;

	bitmap_conf = kzalloc(sizeof(*bitmap_conf), GFP_KERNEL);
	if (!bitmap_conf) {
		ret = -ENOMEM;
		goto out;
	}

	bitmap_conf->host_cfg_bitmap = cpu_to_le32(host_cfg_bitmap);
	bitmap_conf->host_sdio_block_size = cpu_to_le32(sdio_blk_size);
	bitmap_conf->extra_mem_blocks = cpu_to_le32(WL18XX_TX_HW_BLOCK_SPARE);
	bitmap_conf->length_field_size = cpu_to_le32(WL18XX_TX_LEN_FIELD_SIZE);

	ret = wlcore_cmd_configure(wl, ACX_HOST_IF_CFG_BITMAP,
				   bitmap_conf, sizeof(*bitmap_conf));
	if (ret < 0) {
		printk(KERN_DEBUG "wl1271 bitmap config opt failed: %d", ret);
		goto out;
	}

out:
	kfree(bitmap_conf);

	return ret;
}
