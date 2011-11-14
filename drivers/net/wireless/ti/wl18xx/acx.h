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

#ifndef __WL18XX_ACX_H__
#define __WL18XX_ACX_H__

#include "../wlcore/acx.h"

enum {
	ACX_HOST_IF_CFG_BITMAP			= 0x0071,
	ACX_SIM_CONFIG				= 0x0074,
	ACX_CSUM_CONFIG				= 0x1020,
};

#define WL18XX_TX_HW_BLOCK_SPARE	2
/*
 * TODO: this is silly, but we need to pass the size (in bits - 1!) of
 * the length field in the TX descriptor.  To make this slightly
 * cleaner, we should use sizeof the actual field when the TX
 * descriptor structure is implemented.
 */
#define WL18XX_TX_LEN_FIELD_SIZE	15

#define HOST_IF_CFG_RX_FIFO_ENABLE	BIT(0)
#define HOST_IF_CFG_TX_EXTRA_BLKS_SWAP	BIT(1)
#define HOST_IF_CFG_TX_PAD_TO_SDIO_BLK	BIT(3)
#define HOST_IF_CFG_RX_PAD_TO_SDIO_BLK	BIT(4)

struct wl18xx_acx_host_config_bitmap {
	struct acx_header header;

	__le32 host_cfg_bitmap;
	__le32 host_sdio_block_size;
	__le32 extra_mem_blocks;
	__le32 length_field_size;
} __packed;

int wl18xx_acx_host_if_cfg_bitmap(struct wlcore *wl, u32 host_cfg_bitmap,
				  u32 sdio_blk_size);

#endif /* __WL18XX_ACX_H__ */
