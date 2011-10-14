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

#include "../wlcore/wlcore.h"

static int __devinit wl18xx_probe(struct platform_device *pdev)
{
	struct wlcore *wl;
	int ret = -ENODEV;

	printk(KERN_DEBUG "wl18xx_probe\n");

	wl = wlcore_alloc_hw();
	if (IS_ERR(wl)) {
		ret = PTR_ERR(wl);
		goto out;
	}

	wl->dev = &pdev->dev;

	platform_set_drvdata(pdev, wl);

	ret = wlcore_register_hw(wl);
	if (ret)
		goto out_free_hw;

out_free_hw:
	wlcore_free_hw(wl);
out:
	return ret;
}

static int __devexit wl18xx_remove(struct platform_device *pdev)
{
	struct wlcore *wl = platform_get_drvdata(pdev);

	printk(KERN_DEBUG "wl18xx_remove\n");

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
