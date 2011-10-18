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
#include "debug.h"
#include "io.h"

/* Set the partitions to access the chip addresses
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
int wlcore_set_partition(struct wlcore *wl,
			 struct wlcore_partition_set *p)
{
	/* copy partition info */
	memcpy(&wl->part, p, sizeof(*p));

	wlcore_debug(DEBUG_IO, "mem_start %08X mem_size %08X",
		     p->mem.start, p->mem.size);
	wlcore_debug(DEBUG_IO, "reg_start %08X reg_size %08X",
		     p->reg.start, p->reg.size);
	wlcore_debug(DEBUG_IO, "mem2_start %08X mem2_size %08X",
		     p->mem2.start, p->mem2.size);
	wlcore_debug(DEBUG_IO, "mem3_start %08X mem3_size %08X",
		     p->mem3.start, p->mem3.size);

	/* write partition info to the chipset */
	wlcore_raw_write32(wl, HW_PART0_START_ADDR, p->mem.start);
	wlcore_raw_write32(wl, HW_PART0_SIZE_ADDR, p->mem.size);
	wlcore_raw_write32(wl, HW_PART1_START_ADDR, p->reg.start);
	wlcore_raw_write32(wl, HW_PART1_SIZE_ADDR, p->reg.size);
	wlcore_raw_write32(wl, HW_PART2_START_ADDR, p->mem2.start);
	wlcore_raw_write32(wl, HW_PART2_SIZE_ADDR, p->mem2.size);
	wlcore_raw_write32(wl, HW_PART3_START_ADDR, p->mem3.start);

	return 0;
}
EXPORT_SYMBOL_GPL(wlcore_set_partition);
