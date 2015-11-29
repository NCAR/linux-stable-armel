/*
 *   arch/arm/mach-pxa/include/mach/titan.h
 *
 * Author: <linux@eurotech.com>
 * Created:	Feb 08, 2007
 * Copyright: (C) 2007 Arcom Control Systems Ltd.
 * Copyright: (C) 2010-2012 Eurotech Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MACH_TITAN_H
#define _MACH_TITAN_H

/* Physical addresses */
#define TITAN_FLASH_PHYS	PXA_CS0_PHYS
#define TITAN_ETH0_PHYS		PXA_CS1_PHYS
#define TITAN_CPLD_PHYS		(PXA_CS4_PHYS+0x1000000)
#define TITAN_SRAM_PHYS		PXA_CS5_PHYS
#define TITAN_PC104IO_PHYS	(0x30000000)

/* Virtual addresses */
#define TITAN_CPLD_BASE		(0xf0000000)
#define TITAN_PC104IO_BASE	(0xf1000000)

/* GPIOs */
#define TITAN_AC97_GPIO		(0)
#define TITAN_WAKEUP_GPIO	(1)
#define TITAN_UARTA_GPIO	(10)
#define TITAN_UARTB_GPIO	(11)
#define TITAN_ETH0_GPIO		(14)
#define TITAN_ISA_IRQ		(17)	/* PC104 */
#define TITAN_BKLEN_GPIO	(19)
#define TITAN_USB2_PWR_EN_GPIO	(22)
#define TITAN_MMC_WP_GPIO       (52)
#define TITAN_MMC_CD_GPIO       (53)
#define TITAN_AC97_RST_GPIO	(95)
#define TITAN_MMC_SD_PEN_N	(99)	/* TITAN-V2 only */
#define TITAN_LCD_EN_GPIO	(101)
#define TITAN_GPIO_IRQ		(116)
#define PXA_ISA_IRQ_EXTN(x)     (PXA_ISA_IRQ(x) + 3)
#define TITAN_USER_GPIO_BASE    (128)
#define TITAN_USER_GPIO(x)      (TITAN_USER_GPIO_BASE + (x))

/* CPLD registers */

/* ... Physical <--> Virtual address mapping */
	/*                              PHYSICAL_ADDR     VIRTUAL_ADDR
	 *      TITAN_BOARD_VERSION ___ 0x1200_0000 <---> 0xF000_0000
	 *      TITAN_HI_IRQ_STATUS ___ 0x1280_0000 <---> 0xF000_1000
	 *      TITAN_CPLD_VERSION  ___ 0x1300_0000 <---> 0xF000_2000
	 *      TITAN_LO_IRQ_STATUS ___ 0x1380_0000 <---> 0xF000_3000
	 *      TITAN_MISC          ___ 0x1400_0000 <---> 0xF000_4000    */
#define TITAN_CPLD_REG_PHYS_SIZE	0x800000
#define TITAN_CPLD_REG_VIRT_SIZE	0x1000
#define __TITAN_CPLD_DIV	(TITAN_CPLD_REG_PHYS_SIZE / TITAN_CPLD_REG_VIRT_SIZE)
#define TITAN_CPLD_P2V(x)       ((((x) - TITAN_CPLD_PHYS)/__TITAN_CPLD_DIV) + TITAN_CPLD_BASE)
#define TITAN_CPLD_V2P(x)       ((((x) - TITAN_CPLD_BASE)*__TITAN_CPLD_DIV) + TITAN_CPLD_PHYS)

#ifndef __ASSEMBLY__
#  define __TITAN_CPLD_REG(x)   (*((volatile u16 *)TITAN_CPLD_P2V(x)))
#else
#  define __TITAN_CPLD_REG(x)	TITAN_CPLD_P2V(x)
#endif

/* ... Physical addresses */
#define _TITAN_BOARD_VERSION	(TITAN_CPLD_PHYS + 0x00000000)
#define _TITAN_HI_IRQ_STATUS	(TITAN_CPLD_PHYS + 0x00800000)
#define _TITAN_CPLD_VERSION	(TITAN_CPLD_PHYS + 0x01000000)
#define _TITAN_LO_IRQ_STATUS	(TITAN_CPLD_PHYS + 0x01800000)
#define _TITAN_MISC		(TITAN_CPLD_PHYS + 0x02000000)

/* ... Virtual addresses */
#define TITAN_BOARD_VERSION	__TITAN_CPLD_REG(_TITAN_BOARD_VERSION)
#define TITAN_HI_IRQ_STATUS	__TITAN_CPLD_REG(_TITAN_HI_IRQ_STATUS)
#define TITAN_CPLD_VERSION	__TITAN_CPLD_REG(_TITAN_CPLD_VERSION)
#define TITAN_LO_IRQ_STATUS	__TITAN_CPLD_REG(_TITAN_LO_IRQ_STATUS)
#define TITAN_MISC		__TITAN_CPLD_REG(_TITAN_MISC)

/* CPLD register bits */
#define TITAN_MISC_ISA_RST	0x01

/* Word aligned 256K SRAM sparsely mapped over 512K (disregard high byte),
 * See drivers/mtd/maps/pxa2xx_8bit_sram.c for specifics */
#define TITAN_SRAM_SIZE		(256 * 2 * 1024)

#endif
