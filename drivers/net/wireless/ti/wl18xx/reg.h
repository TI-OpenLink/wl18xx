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

#ifndef __REG_H__
#define __REG_H__

#define WL18XX_REG_BOOT_PART_START 0x00802000
#define WL18XX_REG_BOOT_PART_SIZE  0x00014578

#define WL18XX_REGISTERS_BASE      0x00800000
#define WL18XX_CODE_BASE           0x00000000
#define WL18XX_DATA_BASE           0x00400000
#define WL18XX_DOUBLE_BUFFER_BASE  0x00600000
#define WL18XX_MCU_KEY_SEARCH_BASE 0x00700000
#define WL18XX_PHY_BASE            0x00900000
#define WL18XX_TOP_OCP_BASE        0x00A00000
#define WL18XX_PACKET_RAM_BASE     0x00B00000
#define WL18XX_HOST_BASE           0x00C00000

#define WL18XX_CHIP_ID_B               (WL18XX_REGISTERS_BASE + 0x01542C)

#define CHIP_ID_185x_PG10              (0x06030101)

#endif /* __REG_H__ */
