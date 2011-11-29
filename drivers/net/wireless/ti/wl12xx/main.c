/*
 * This file is part of wl1271
 *
 * Copyright (C) 2008-2010 Nokia Corporation
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

#include <linux/err.h>

#include "../wlcore/wlcore.h"
#include "../wlcore/debug.h"

#include "reg.h"

static struct wlcore_ops wl12xx_ops = {
};

static struct wlcore_partition_set wl12xx_ptable[PART_TABLE_LEN] = {
	[PART_DOWN] = {
		.mem = {
			.start = 0x00000000,
			.size  = 0x000177c0
		},
		.reg = {
			.start = REGISTERS_BASE,
			.size  = 0x00008800
		},
		.mem2 = {
			.start = 0x00000000,
			.size  = 0x00000000
		},
		.mem3 = {
			.start = 0x00000000,
			.size  = 0x00000000
		},
	},

	[PART_BOOT] = { /* in wl12xx we can use the down partition here */
		.mem = {
			.start = 0x00000000,
			.size  = 0x000177c0
		},
		.reg = {
			.start = REGISTERS_BASE,
			.size  = 0x00008800
		},
		.mem2 = {
			.start = 0x00000000,
			.size  = 0x00000000
		},
		.mem3 = {
			.start = 0x00000000,
			.size  = 0x00000000
		},
	},

	[PART_WORK] = {
		.mem = {
			.start = 0x00040000,
			.size  = 0x00014fc0
		},
		.reg = {
			.start = REGISTERS_BASE,
			.size  = 0x0000a000
		},
		.mem2 = {
			.start = 0x003004f8,
			.size  = 0x00000004
		},
		.mem3 = {
			.start = 0x00040404,
			.size  = 0x00000000
		},
	},

	[PART_DRPW] = {
		.mem = {
			.start = 0x00040000,
			.size  = 0x00014fc0
		},
		.reg = {
			.start = DRPW_BASE,
			.size  = 0x00006000
		},
		.mem2 = {
			.start = 0x00000000,
			.size  = 0x00000000
		},
		.mem3 = {
			.start = 0x00000000,
			.size  = 0x00000000
		}
	}
};

static const int wl12xx_rtable[REG_TABLE_LEN] = {
	[REG_ECPU_CONTROL]		= WL12XX_REG_ECPU_CONTROL,
	[REG_INTERRUPT_NO_CLEAR]	= WL12XX_REG_INTERRUPT_NO_CLEAR,
	[REG_INTERRUPT_ACK]		= WL12XX_REG_INTERRUPT_ACK,
	[REG_COMMAND_MAILBOX_PTR]	= WL12XX_REG_COMMAND_MAILBOX_PTR,
	[REG_EVENT_MAILBOX_PTR]		= WL12XX_REG_EVENT_MAILBOX_PTR,
	[REG_INTERRUPT_TRIG_L]		= WL12XX_REG_INTERRUPT_TRIG,
	[REG_INTERRUPT_TRIG_H]		= WL12XX_REG_INTERRUPT_TRIG_H,
	[REG_INTERRUPT_MASK]		= WL12XX_REG_INTERRUPT_MASK,
	[REG_PC_ON_RECOVERY]		= SCR_PAD4,

	[REG_OCP_POR_CTR]		= OCP_POR_CTR,
	[REG_OCP_DATA_WRITE]		= OCP_DATA_WRITE,
	[REG_OCP_DATA_READ]		= OCP_DATA_READ,
	[REG_OCP_CMD]			= OCP_CMD,
	[REG_PC_ON_RECOVERY]		= SCR_PAD4,
	[REG_CHIP_ID_B]			= CHIP_ID_B,
	[REG_HOST_WR_ACCESS]		= WL1271_HOST_WR_ACCESS,
	[REG_RX_DRIVER_COUNTER_ADDRESS]	= RX_DRIVER_COUNTER_ADDRESS,
	[REG_HI_CFG]			= HI_CFG,
	[REG_SLV_SOFT_RESET]		= ACX_REG_SLV_SOFT_RESET,
	[REG_ENABLE]			= ENABLE,
	[REG_SPARE_A2]			= SPARE_A2,
	[REG_PLL_PARAMETERS]		= PLL_PARAMETERS,
	[REG_WU_COUNTER_PAUSE]		= WU_COUNTER_PAUSE,
	[REG_WELP_ARM_COMMAND]		= WELP_ARM_COMMAND,
	[REG_DRPW_SCRATCH_START]	= DRPW_SCRATCH_START,
	[REG_EEPROMLESS_IND]		= ACX_EEPROMLESS_IND_REG,
	[REG_SCR_PAD2]			= SCR_PAD2,

	/* data access memory addresses, used with partition translation */
	[REG_SLV_MEM_DATA]		= WL1271_SLV_MEM_DATA,
	[REG_SLV_REG_DATA]		= WL1271_SLV_REG_DATA,

	/* raw register addresses, used without partition translation */
	[REG_RAW_HW_ACCESS_ELP_CTRL]	= HW_ACCESS_ELP_CTRL_REG_ADDR,

	/* raw data access memory addresses */
	[REG_RAW_FW_STATUS_ADDR]	= FW_STATUS_ADDR,
};

int __devinit wl12xx_probe(struct platform_device *pdev)
{
	struct wl1271 *wl;
	struct ieee80211_hw *hw;

	hw = wlcore_alloc_hw();
	if (IS_ERR(hw)) {
		wl1271_error("can't allocate hw");
		return PTR_ERR(hw);
	}

	wl = hw->priv;
	wl->ops = &wl12xx_ops;
	wl->ptable = wl12xx_ptable;
	wl->rtable = wl12xx_rtable;

	return wlcore_probe(wl, pdev);
}

static const struct platform_device_id wl12xx_id_table[] __devinitconst = {
	{ "wl12xx", 0 },
	{  } /* Terminating Entry */
};
MODULE_DEVICE_TABLE(platform, wl12xx_id_table);

static struct platform_driver wl12xx_driver = {
	.probe		= wl12xx_probe,
	.remove		= __devexit_p(wlcore_remove),
	.id_table	= wl12xx_id_table,
	.driver = {
		.name	= "wl12xx_driver",
		.owner	= THIS_MODULE,
	}
};

static int __init wl12xx_init(void)
{
	return platform_driver_register(&wl12xx_driver);
}
module_init(wl12xx_init);

static void __exit wl12xx_exit(void)
{
	platform_driver_unregister(&wl12xx_driver);
}
module_exit(wl12xx_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Luciano Coelho <coelho@ti.com>");
