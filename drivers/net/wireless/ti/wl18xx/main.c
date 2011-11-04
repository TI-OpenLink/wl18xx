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

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/wl12xx.h>

#include "../wlcore/wlcore.h"
#include "../wlcore/io.h"
#include "reg.h"
#include "conf.h"

#define WL18XX_FW_NAME  "ti-connectivity/wl18xx-fw-multirole-roc.bin"
#define WL18XX_NVS_NAME "ti-connectivity/wl18xx-nvs.bin"

static struct wl18xx_conf wl18xx_default_conf = {
	.phy = {
		.phy_standalone			= 0x00,
		.primary_clock_setting_time	= 0x05,
		.clock_valid_on_wake_up		= 0x00,
		.secondary_clock_setting_time	= 0x05,
		.rdl				= 0x01,
		.auto_detect			= 0x00,
		.dedicated_fem			= FEM_NONE,
		.low_band_component		= COMPONENT_2_WAY_SWITCH,
		.low_band_component_type	= 0x05,
		.high_band_component		= COMPONENT_2_WAY_SWITCH,
		.high_band_component_type	= 0x05,
		.number_of_assembled_ant2_4	= 0x01,
		.number_of_assembled_ant5	= 0x01,
		.external_pa_dc2dc		= 0x00,
		.tcxo_ldo_voltage		= 0x00,
		.xtal_itrim_val			= 0x04,
		.srf_state			= 0x00,
		.io_configuration		= 0x01,
		.sdio_configuration		= 0x00,
		.settings			= 0x00,
		.enable_clpc			= 0x00,
		.enable_tx_low_pwr_on_siso_rdl	= 0x00,
		.rx_profile			= 0x00,
	},
};

static const struct wlcore_partition_set wl18xx_ptable[PART_TABLE_LEN] = {
	[PART_TOP_PRCM_ELP_SOC] = {
		.mem  = { .start = 0x00A02000, .size  = 0x00010000 },
		.reg  = { .start = 0x00807000, .size  = 0x00005000 },
		.mem2 = { .start = 0x00800000, .size  = 0x0000B000 },
		.mem3 = { .start = 0x00000000, .size  = 0x00000000 },
	},
	[PART_DOWN] = {
		.mem  = { .start = 0x00000000, .size  = 0x00014000 },
		.reg  = { .start = 0x00810000, .size  = 0x0000BFFF },
		.mem2 = { .start = 0x00000000, .size  = 0x00000000 },
		.mem3 = { .start = 0x00000000, .size  = 0x00000000 },
	},
	[PART_BOOT] = {
		.mem  = { .start = 0x00700000, .size = 0x0000030c },
		.reg  = { .start = 0x00802000, .size = 0x00014578 },
		.mem2 = { .start = 0x00B00404, .size = 0x00001000 },
		.mem3 = { .start = 0x00C00000, .size = 0x00000400 },
	},
	[PART_PHY_INIT] = {
		.mem  = { .start = WL18XX_PHY_INIT_MEM,
			  .size = sizeof(struct wl18xx_conf) },
		.reg  = { .start = 0x00000000, .size = 0x00000000 },
		.mem2 = { .start = 0x00000000, .size = 0x00000000 },
		.mem3 = { .start = 0x00000000, .size = 0x00000000 },
	},
};

static const int wl18xx_rtable[REG_TABLE_LEN] = {
	[REG_ECPU_CONTROL]		= WL18XX_REGISTERS_BASE + 0x02004,
	[REG_INTERRUPT_NO_CLEAR]	= WL18XX_REGISTERS_BASE + 0x050E8,
	[REG_INTERRUPT_ACK]		= WL18XX_REGISTERS_BASE + 0x050F0,
	[REG_COMMAND_MAILBOX_PTR]	= WL18XX_SCR_PAD0,
	[REG_EVENT_MAILBOX_PTR]		= WL18XX_SCR_PAD1,
	[REG_INTERRUPT_TRIG_L]		= WL18XX_REGISTERS_BASE + 0x5074,
	[REG_INTERRUPT_TRIG_H]		= WL18XX_REGISTERS_BASE + 0x5078,
	[REG_INTERRUPT_MASK]		= WL18XX_REGISTERS_BASE + 0x50DC,
};

static const u64 wl18xx_trig_table[TRIG_TABLE_LEN] = {
	[TRIG_CMD]		= 1ULL << 60,
	[TRIG_EVENT_ACK]	= 1ULL << 61,
};

static int wl18xx_get_chip_id(struct wlcore *wl)
{
	u32 id;

	printk(KERN_DEBUG "wl18xx_get_chip_id\n");

	wlcore_select_partition(wl, PART_BOOT);

	id = wlcore_read32(wl, WL18XX_CHIP_ID_B);

	switch (id) {
	case CHIP_ID_185x_PG10:
		printk(KERN_DEBUG "chip id 0x%x (185x PG10)\n", id);

		wl->fw_name  = WL18XX_FW_NAME;
		wl->nvs_name = WL18XX_NVS_NAME;
		break;
	default:
		printk(KERN_DEBUG "unsupported chip id: 0x%x\n", id);
		id = -ENODEV;
		break;
	}

	return id;
}

static int wl18xx_config_pll(struct wlcore *wl)
{
	/*
	 * TODO: this is all *very* hacky, must define proper regs and
	 * set them according to the documentation
	 */
	u32 board_type = BOARD_TYPE_DVP_EVB;

	printk(KERN_DEBUG "wl18xx_config_pll (board_type = 0x%02x)\n",
	       board_type);

	wlcore_write32(wl, WL18XX_SCR_PAD2, board_type);

	if (board_type != BOARD_TYPE_FPGA) {
		wlcore_select_partition(wl, PART_TOP_PRCM_ELP_SOC);
		wlcore_write32(wl, 0x00A02360, 0xD0078);
		wlcore_write32(wl, 0x00A0236c, 0x12);
		wlcore_write32(wl, 0x00A02390, 0x20118);
	}

	/* lock PLL */
	wlcore_write32(wl, WL18XX_WELP_ARM_COMMAND, WELP_ARM_COMMAND_VAL);
	udelay(500);

	wlcore_select_partition(wl, PART_BOOT);
	wlcore_write_reg(wl, REG_INTERRUPT_MASK, INTR_ALL);

	/* TODO: check if this is really needed for wl18xx */
	wlcore_write32(wl, WL18XX_RXTX_ENABLE, 0x0);
	/* disable auto calibration on start*/
	wlcore_write32(wl, WL18XX_SPARE_A2, 0xffff);

	return 0;
}

static void wl18xx_preboot_conf(struct wlcore *wl)
{
	struct wl18xx_conf_phy *params =
		&((struct wl18xx_conf *)wl->conf->priv_data)->phy;

	wlcore_select_partition(wl, PART_PHY_INIT);
	wlcore_write(wl, WL18XX_PHY_INIT_MEM, (u8*) params,
		     sizeof(*params), false);

	/*
	 * 18xxTODO: should probably revert the set_partition thing, or
	 * do all set_partition in a single function. otherwise confusing.
	 */
}

static int wl18xx_conf_init(struct wlcore *wl)
{
	wl->conf = kmemdup(&wl18xx_default_conf, sizeof(wl18xx_default_conf),
			   GFP_KERNEL);
	if (!wl->conf)
		return -ENOMEM;

	return 0;
}

static struct wlcore_ops wl18xx_ops = {
	.get_chip_id	 = wl18xx_get_chip_id,
	.config_pll	 = wl18xx_config_pll,
	.preboot_conf	 = wl18xx_preboot_conf,
};

static int __devinit wl18xx_probe(struct platform_device *pdev)
{
	struct wl12xx_platform_data *pdata = pdev->dev.platform_data;
	struct wlcore *wl;
	int ret = -ENODEV;

	printk(KERN_DEBUG "wl18xx_probe\n");

	wl = wlcore_alloc_hw();
	if (IS_ERR(wl)) {
		ret = PTR_ERR(wl);
		goto out;
	}

	/*
	 * HACK! We're casting ops here, it should be fine for now,
	 * because they're the same, but this needs to be properly
	 * fixed
	 */
	wl->if_ops = (struct wlcore_if_ops *) pdata->ops;
	wl->ops = &wl18xx_ops;
	wl->dev = &pdev->dev;
	wl->irq = platform_get_irq(pdev, 0);
	wl->devname = pdev->name;
	wl->platform_quirks = pdata->platform_quirks;
	wl->ptable = &wl18xx_ptable[0];
	wl->rtable = &wl18xx_rtable[0];
	wl->trig_table = &wl18xx_trig_table[0];
	ret = wl18xx_conf_init(wl);
	if (ret)
		goto out_free_hw;

	platform_set_drvdata(pdev, wl);

	ret = wlcore_register_hw(wl);
	if (ret)
		goto out_free_conf;

out_free_conf:
	kfree(wl->conf);
out_free_hw:
	wlcore_free_hw(wl);
out:
	return ret;
}

static int __devexit wl18xx_remove(struct platform_device *pdev)
{
	struct wlcore *wl = platform_get_drvdata(pdev);

	printk(KERN_DEBUG "wl18xx_remove\n");

	kfree(wl->conf);
	wlcore_unregister_hw(wl);
	wlcore_free_hw(wl);

	return 0;
}

static const struct platform_device_id wl18xx_id_table[] __devinitconst = {
	{ "wl18xx", 0 },
	{  } /* Terminating Entry */
};
MODULE_DEVICE_TABLE(platform, wl18xx_id_table);

static struct platform_driver wl18xx_driver = {
	.probe		= wl18xx_probe,
	.remove		= __devexit_p(wl18xx_remove),
	.id_table	= wl18xx_id_table,
	.driver = {
		.name	= "wl18xx_driver",
		.owner	= THIS_MODULE,
	}
};

static int __init wl18xx_init(void)
{
	printk(KERN_DEBUG "wl18xx_init\n");

	return platform_driver_register(&wl18xx_driver);
}
module_init(wl18xx_init);

static void __exit wl18xx_exit(void)
{
	platform_driver_unregister(&wl18xx_driver);
}
module_exit(wl18xx_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Luciano Coelho <coelho@ti.com>");
MODULE_FIRMWARE(WL18XX_FW_NAME);
