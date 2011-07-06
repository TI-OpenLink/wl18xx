/*
 * This file is part of wl1271
 *
 * Copyright (C) 2008-2010 Nokia Corporation
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include "wl12xx.h"
#include "wl12xx_80211.h"
#include "io.h"
#include "tx.h"

bool wl1271_set_block_size(struct wl1271 *wl)
{
	if (wl->if_ops->set_block_size) {
		wl->if_ops->set_block_size(wl, WL12XX_BUS_BLOCK_SIZE);
		return true;
	}

	return false;
}

void wl1271_disable_interrupts(struct wl1271 *wl)
{
	wl->if_ops->disable_irq(wl);
}

void wl1271_enable_interrupts(struct wl1271 *wl)
{
	wl->if_ops->enable_irq(wl);
}

/* Set the SPI partitions to access the chip addresses
 *
 * To simplify driver code, a fixed (virtual) memory map is defined for
 * register and memory addresses. Because in the chipset, in different stages
 * of operation, those addresses will move around, an address translation
 * mechanism is required.
 *
 * There are four partitions (three memory and one register partition),
 * which are mapped to two different areas of the hardware memory.
 *
 *                                Virtual address
 *                                     space
 *
 *                                    |    |
 *                                 ...+----+--> mem.start
 *          Physical address    ...   |    |
 *               space       ...      |    | [PART_0]
 *                        ...         |    |
 *  00000000  <--+----+...         ...+----+--> mem.start + mem.size
 *               |    |         ...   |    |
 *               |MEM |      ...      |    |
 *               |    |   ...         |    |
 *  mem.size  <--+----+...            |    | {unused area)
 *               |    |   ...         |    |
 *               |REG |      ...      |    |
 *  mem.size     |    |         ...   |    |
 *      +     <--+----+...         ...+----+--> reg.start
 *  reg.size     |    |   ...         |    |
 *               |MEM2|      ...      |    | [PART_1]
 *               |    |         ...   |    |
 *                                 ...+----+--> reg.start + reg.size
 *                                    |    |
 *
 */
int wl1271_set_partition(struct wl1271 *wl,
			 struct wl1271_partition_set *p)
{
	/* copy partition info */
	memcpy(&wl->part, p, sizeof(*p));

	wl1271_debug(DEBUG_SPI, "mem_start %08X mem_size %08X",
		     p->mem.start, p->mem.size);
	wl1271_debug(DEBUG_SPI, "reg_start %08X reg_size %08X",
		     p->reg.start, p->reg.size);
	wl1271_debug(DEBUG_SPI, "mem2_start %08X mem2_size %08X",
		     p->mem2.start, p->mem2.size);
	wl1271_debug(DEBUG_SPI, "mem3_start %08X mem3_size %08X",
		     p->mem3.start, p->mem3.size);

	/* write partition info to the chipset */
	wl1271_raw_write32(wl, HW_PART0_START_ADDR, p->mem.start);
	wl1271_raw_write32(wl, HW_PART0_SIZE_ADDR, p->mem.size);
	wl1271_raw_write32(wl, HW_PART1_START_ADDR, p->reg.start);
	wl1271_raw_write32(wl, HW_PART1_SIZE_ADDR, p->reg.size);
	wl1271_raw_write32(wl, HW_PART2_START_ADDR, p->mem2.start);
	wl1271_raw_write32(wl, HW_PART2_SIZE_ADDR, p->mem2.size);
	wl1271_raw_write32(wl, HW_PART3_START_ADDR, p->mem3.start);

	return 0;
}
EXPORT_SYMBOL_GPL(wl1271_set_partition);

void wl1271_io_reset(struct wl1271 *wl)
{
	if (wl->if_ops->reset)
		wl->if_ops->reset(wl);
}

void wl1271_io_init(struct wl1271 *wl)
{
	if (wl->if_ops->init)
		wl->if_ops->init(wl);
}

void wl1271_top_reg_write(struct wl1271 *wl, int addr, u16 val)
{
	u32 tmp_val;

    wl1271_info("Orit Wl18xx - wl1271_top_reg_write addr 0x%x val 0x%x", addr, val);

	if (!(addr % 4))
	{
	    wl1271_info("Orit Wl18xx - address aligned");
		tmp_val = wl1271_read32(wl, addr);
	    wl1271_info("Orit Wl18xx - read from 0x%x val 0x%x", addr, tmp_val);
		val = (tmp_val & 0xffff0000) | val;
	    wl1271_info("Orit Wl18xx - write to addr 0x%x val 0x%x", addr, val);
		wl1271_write32(wl, addr, val);

	} else {
	    wl1271_info("Orit Wl18xx - address not aligned read from addr 0x%x", addr - 2);
		tmp_val = wl1271_read32(wl, (addr - 2));
	    wl1271_info("Orit Wl18xx - read from 0x%x val 0x%x", addr-2, tmp_val);
		val = (tmp_val & 0xffff) | (val << 16);
	    wl1271_info("Orit Wl18xx - write to addr 0x%x val 0x%x", (addr-2), val);
		wl1271_write32(wl, (addr-2), val);
	}
}

u16 wl1271_top_reg_read(struct wl1271 *wl, int addr)
{
	u32 val;

    wl1271_info("Orit Wl18xx - wl1271_top_reg_read addr 0x%x", addr);

	if (!(addr % 4))
	{
	    wl1271_info("Orit Wl18xx - address aligned");
		val = wl1271_read32(wl, addr);
	    wl1271_info("Orit Wl18xx - read val 0x%x", val);
	    wl1271_info("Orit Wl18xx - return val 0x%x", val & 0xffff);
		return val & 0xffff;

	} else {
	    wl1271_info("Orit Wl18xx - address not aligned read from addr 0x%x", addr - 2);
		val = wl1271_read32(wl, (addr - 2));
	    wl1271_info("Orit Wl18xx - read val 0x%x", val);
	    wl1271_info("Orit Wl18xx - return val 0x%x", (val & 0xffff0000) >> 16);
		return (val & 0xffff0000) >> 16;
	}
}

