/*
 * PXA2xx external 8 bit SRAM MTD map driver.
 *
 * Copyright (C) 2005-2012 Eurotech Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <asm/sizes.h>

#include <mach/hardware.h>

/*
 * The PXA2xx memory bus only supports 16 bit peripherals. For 8 bit devices
 * we only take the lowest byte of each 16 bit word.
 */

static map_word pxa2xx_8bit_sram_read(struct map_info *map, unsigned long ofs)
{
	map_word ret;
	ret.x[0] = __raw_readw(map->virt + (ofs<<1)) & 0xFF;
	return ret;
}

static void pxa2xx_8bit_sram_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	__u8 *dst = (__u8 *)to;

	while(len) {
		*dst = __raw_readw(map->virt + (from<<1)) & 0xff;
		len--; dst++; from++;
	}
}

static void pxa2xx_8bit_sram_write(struct map_info *map, const map_word d, unsigned long ofs)
{
	__raw_writew(d.x[0], map->virt + (ofs<<1));
}

static void pxa2xx_8bit_sram_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
	__u8 *src = (__u8 *)from;

	while(len) {
		__raw_writew(*src, map->virt + (to<<1));
		len--; to++; src++;
	}
}

struct pxa2xx_8bit_sram {
	struct mtd_info *mtd;
	struct map_info  map;
	struct resource *res;
};

static int pxa2xx_8bit_sram_remove(struct platform_device *dev)
{
	struct pxa2xx_8bit_sram *info = platform_get_drvdata(dev);

	if (!info)
		return 0;

	if (info->mtd) {
		del_mtd_device(info->mtd);
		map_destroy(info->mtd);
	}
	if (info->map.virt)
		iounmap(info->map.virt);
	if (info->res) {
		release_resource(info->res);
		kfree(info->res);
	}

	kfree(info);

	return 0;
}


static int pxa2xx_8bit_sram_probe(struct platform_device *dev)
{
	struct pxa2xx_8bit_sram *info;
	int err;

	info = kzalloc(sizeof(struct pxa2xx_8bit_sram), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto error;
	}

	platform_set_drvdata(dev, info);

	info->map.phys = NO_XIP;
	info->map.size = (dev->resource->end - dev->resource->start + 1) / 2;

	info->map.bankwidth = 1;
	info->map.name = dev->name;
	info->map.read = pxa2xx_8bit_sram_read;
	info->map.copy_from = pxa2xx_8bit_sram_copy_from;
	info->map.write = pxa2xx_8bit_sram_write;
	info->map.copy_to = pxa2xx_8bit_sram_copy_to;

	info->res = request_mem_region(dev->resource->start, info->map.size * 2, "pxa2xx_8bit_sram");
	if (!info->res) {
		printk(KERN_ERR "%s: memory region in use\n", info->map.name);
		err = -ENOMEM;
		goto error;
	}

	info->map.virt = ioremap(info->res->start, info->map.size * 2);
	if (!info->map.virt) {
		printk(KERN_ERR "%s: ioremap failed\n", info->map.name);
		err = -EIO;
		goto error;
	}

	info->mtd = do_map_probe("map_ram", &info->map);
	if (!info->mtd) {
		printk(KERN_ERR "%s: SRAM not found\n", info->map.name);
		err = -ENXIO;
		goto error;
	}

	info->mtd->owner = THIS_MODULE;

	add_mtd_device(info->mtd);

	return 0;

  error:
	pxa2xx_8bit_sram_remove(dev);
	return err;
}

static struct platform_driver pxa2xx_8bit_sram_driver = {
        .probe          = pxa2xx_8bit_sram_probe,
        .remove         = pxa2xx_8bit_sram_remove,
	.driver		= {
		.name	= "pxa2xx-8bit-sram",
	}
};

static int __init pxa2xx_8bit_sram_init(void)
{
        return platform_driver_register(&pxa2xx_8bit_sram_driver);
}

static void __exit pxa2xx_8bit_sram_exit(void)
{
        platform_driver_unregister(&pxa2xx_8bit_sram_driver);
}

module_init(pxa2xx_8bit_sram_init);
module_exit(pxa2xx_8bit_sram_exit);

MODULE_AUTHOR("Eurotech Linux Team <linux@eurotech.com>");
MODULE_DESCRIPTION("PXA2xx 8bit SRAM MTD map driver");
MODULE_LICENSE("GPL");
