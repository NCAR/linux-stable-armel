/*
 * Map driver for the Eurotech TITAN
 *
 * Amit Walambe <amit.walambe@eurotech.com>
 *
 * Copyright (C) 2007 Arcom Control Systems Ltd.
 * Copyright (C) 2010 Eurotech Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/cfi.h>
#include <linux/slab.h>
#include <mach/pxa27x.h>
#include <mach/pxa2xx-regs.h>
#include <mach/titan.h>

/* NOTE:
 * Caching the FLASH causes the PXA to generate burst reads (line fill).
 * This starves the PXA LCD DMA engine, causing a noticible screen
 * flicker.
 * Therefore flash should only be set to cacheable if the application
 * warrants it. */
#undef MAP_TITAN_FLASH_CACHED	/* define as 1 to enable caching */

/* Could be as much as 64M */
#define TITAN_FLASH_SIZE 64*1024*1024

struct titan_flash_info {
        struct mtd_info *mtd;
        struct map_info map;
        struct mtd_partition *partitions;
        struct resource *res;
};

const char *part_probes[] = { "cmdlinepart", "RedBoot", NULL };

#ifdef MAP_TITAN_FLASH_CACHED
static void titan_map_inval_cache(struct map_info *map, unsigned long from,
				 ssize_t len)
{
	consistent_sync((char *)map->cached + from, len, DMA_FROM_DEVICE);
}
#endif

static int titan_flash_remove(struct platform_device *dev)
{
	struct titan_flash_info *info = dev_get_drvdata(&dev->dev);

	dev_set_drvdata(&dev->dev, NULL);

	if(!info)
		return 0;

	if (info->mtd) {
		del_mtd_partitions(info->mtd);
		map_destroy(info->mtd);
	}
	if (info->map.virt)
		iounmap(info->map.virt);
	if (info->map.cached)
		iounmap(info->map.cached);

	if (info->partitions)
		kfree(info->partitions);

	if (info->res) {
		release_resource(info->res);
		kfree(info->res);
	}

	return 0;
}

static int __init titan_mtd_probe(struct platform_device *dev)
{
	struct titan_flash_info *info;
	int err;

	info = kzalloc(sizeof(struct titan_flash_info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto error;
	}

	dev_set_drvdata(&dev->dev, info);

	info->map.name        = "titan-flash";
	info->map.bankwidth   = 2;
	info->map.phys        = TITAN_FLASH_PHYS,
	info->map.size        = TITAN_FLASH_SIZE;
#ifdef MAP_TITAN_FLASH_CACHED
	info->map.inval_cache = titan_map_inval_cache;
#endif

	simple_map_init(&info->map);

        info->res = request_mem_region(info->map.phys, info->map.size, "titan-flash");
	if (!info->res) {
		printk(KERN_ERR "titan-flash: request memory resource failed\n");
		err = -EAGAIN;
		goto error;
	}

	info->map.virt = ioremap_nocache(info->map.phys, info->map.size);
	if (!info->map.virt) {
		printk(KERN_ERR "titan-flash: failed to ioremap_nocache\n");
		err = -EIO;
		goto error;
	}

#ifdef MAP_TITAN_FLASH_CACHED
	info->map.cached = ioremap_cached(info->map.phys, info->map.size);
	if (!info->map.cached)
		printk(KERN_WARNING "titan-flash: failed to ioremap cached flash device\n");
#endif

	info->mtd = do_map_probe("cfi_probe", &info->map);
	if (!info->mtd) {
		printk("titan-flash: probe failed\n");
		err = -ENXIO;
		goto error;
	}
	info->mtd->owner = THIS_MODULE;

	err = parse_mtd_partitions(info->mtd, part_probes, &info->partitions, 0);
	if (err > 0) {
		err = add_mtd_partitions (info->mtd, info->partitions, err);
		if (err)
			printk(KERN_ERR "titan-flash: could not parse partitions\n");
	}

	if (err)
		goto error;

	return 0;

  error:
	titan_flash_remove(dev);
	return err;
}

#ifdef CONFIG_PM
static int titan_mtd_suspend(struct platform_device *dev, pm_message_t state)
{
	struct titan_flash_info *info = dev_get_drvdata(&dev->dev);
	int ret = 0;

	if (info)
		ret = info->mtd->suspend(info->mtd);

	return ret;
}

static int titan_mtd_resume(struct platform_device *dev)
{
	struct titan_flash_info *info = dev_get_drvdata(&dev->dev);

	if (info)
		info->mtd->resume(info->mtd);

	return 0;
}
#endif

static struct platform_driver titan_mtd_driver = {
	.probe		= titan_mtd_probe,
	.remove	 	= __exit_p(titan_mtd_remove),
	.driver		= {
		.name		= "flash",
	},
#ifdef CONFIG_PM
	.suspend	= titan_mtd_suspend,
	.resume	 	= titan_mtd_resume,
#endif
};

static int __init titan_mtd_init(void)
{
	return platform_driver_register(&titan_mtd_driver);
}

static void __exit titan_mtd_exit(void)
{
	platform_driver_unregister(&titan_mtd_driver);
}

module_init(titan_mtd_init);
module_exit(titan_mtd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eurotech Ltd.");
MODULE_DESCRIPTION("MTD map driver for the Eurotech TITAN");
