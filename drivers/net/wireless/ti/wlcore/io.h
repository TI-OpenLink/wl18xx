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

#ifndef __IO_H__
#define __IO_H__

#include "debug.h"

#define HW_PARTITION_REGISTERS_ADDR     0x1FFC0
#define HW_PART0_SIZE_ADDR              (HW_PARTITION_REGISTERS_ADDR)
#define HW_PART0_START_ADDR             (HW_PARTITION_REGISTERS_ADDR + 4)
#define HW_PART1_SIZE_ADDR              (HW_PARTITION_REGISTERS_ADDR + 8)
#define HW_PART1_START_ADDR             (HW_PARTITION_REGISTERS_ADDR + 12)
#define HW_PART2_SIZE_ADDR              (HW_PARTITION_REGISTERS_ADDR + 16)
#define HW_PART2_START_ADDR             (HW_PARTITION_REGISTERS_ADDR + 20)
#define HW_PART3_START_ADDR             (HW_PARTITION_REGISTERS_ADDR + 24)

/* Raw target IO, address is not translated */
static inline void wlcore_raw_write(struct wlcore *wl, int addr, void *buf,
				    size_t len, bool fixed)
{
	wl->if_ops->write(wl->dev, addr, buf, len, fixed);
}

static inline void wlcore_raw_write32(struct wlcore *wl, int addr, u32 val)
{
	wl->buffer_32 = cpu_to_le32(val);
	wlcore_raw_write(wl, addr, &wl->buffer_32,
			 sizeof(wl->buffer_32), false);
}

static inline void wlcore_raw_read(struct wlcore *wl, int addr, void *buf,
				   size_t len, bool fixed)
{
	wl->if_ops->read(wl->dev, addr, buf, len, fixed);
}

static inline u32 wlcore_raw_read32(struct wlcore *wl, int addr)
{
	wlcore_raw_read(wl, addr, &wl->buffer_32,
			    sizeof(wl->buffer_32), false);

	return le32_to_cpu(wl->buffer_32);
}

/* Translated target IO */
static inline int wlcore_translate_addr(struct wlcore *wl, int addr)
{
	/*
	 * To translate, first check to which window of addresses the
	 * particular address belongs. Then subtract the starting address
	 * of that window from the address. Then, add offset of the
	 * translated region.
	 *
	 * The translated regions occur next to each other in physical device
	 * memory, so just add the sizes of the preceding address regions to
	 * get the offset to the new region.
	 */
	if ((addr >= wl->part.mem.start) &&
	    (addr < wl->part.mem.start + wl->part.mem.size))
		return addr - wl->part.mem.start;
	else if ((addr >= wl->part.reg.start) &&
		 (addr < wl->part.reg.start + wl->part.reg.size))
		return addr - wl->part.reg.start + wl->part.mem.size;
	else if ((addr >= wl->part.mem2.start) &&
		 (addr < wl->part.mem2.start + wl->part.mem2.size))
		return addr - wl->part.mem2.start + wl->part.mem.size +
		       wl->part.reg.size;
	else if ((addr >= wl->part.mem3.start) &&
		 (addr < wl->part.mem3.start + wl->part.mem3.size))
		return addr - wl->part.mem3.start + wl->part.mem.size +
		       wl->part.reg.size + wl->part.mem2.size;

	WARN(1, "HW address 0x%x out of range", addr);
	return 0;
}

static inline void wlcore_read(struct wlcore *wl, int addr, void *buf,
			       size_t len, bool fixed)
{
	int physical;

	physical = wlcore_translate_addr(wl, addr);

	wlcore_raw_read(wl, physical, buf, len, fixed);
}

static inline void wlcore_write(struct wlcore *wl, int addr, void *buf,
				size_t len, bool fixed)
{
	int physical;

	physical = wlcore_translate_addr(wl, addr);

	wlcore_raw_write(wl, physical, buf, len, fixed);
}

static inline u32 wlcore_read32(struct wlcore *wl, int addr)
{
	return wlcore_raw_read32(wl, wlcore_translate_addr(wl, addr));
}

static inline void wlcore_write32(struct wlcore *wl, int addr, u32 val)
{
	wlcore_raw_write32(wl, wlcore_translate_addr(wl, addr), val);
}

static inline void wlcore_io_reset(struct wlcore *wl)
{
	if (wl->if_ops->reset)
		wl->if_ops->reset(wl->dev);
}

static inline void wlcore_io_init(struct wlcore *wl)
{
	if (wl->if_ops->init)
		wl->if_ops->init(wl->dev);
}

static inline void wlcore_io_power_off(struct wlcore *wl)
{
	wl->if_ops->power(wl->dev, false);
	clear_bit(WLCORE_FLAG_GPIO_POWER, &wl->flags);
}

static inline int wlcore_io_power_on(struct wlcore *wl)
{
	int ret = wl->if_ops->power(wl->dev, true);
	if (ret == 0)
		set_bit(WLCORE_FLAG_GPIO_POWER, &wl->flags);

	return ret;
}

int wlcore_set_partition(struct wlcore *wl,
			 struct wlcore_partition_set *p);

#endif /* __IO_H__ */
