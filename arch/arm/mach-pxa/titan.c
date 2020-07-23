/*
 *  Support for the Eurotech TITAN.
 *
 *  Author: <linux@eurotech.com>
 *
 *  Copyright (C) 2007 Arcom Control Systems Ltd.
 *  Copyright (C) 2008-2012 Eurotech Ltd.
 *
 *  Contributors: Amit Walambe <amit.awalambe@eurotech.com>
 *                Graham Osborne <graham.osborne@eurotech.com>
 *
 *  Many thanks to Marc Zyngier <maz@misterjones.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/cpufreq.h>
#include <linux/delay.h>

#if (CONFIG_ARCOM_TITAN_VER & 1)
# include <linux/dm9000.h>
#endif
#if (CONFIG_ARCOM_TITAN_VER & 2)
# include <linux/smsc911x.h>
#endif

#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <linux/serial_8250.h>

#include <linux/platform_data/pca953x.h>
#include <linux/mmc/host.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/spi.h>

#include <asm/mach-types.h>
#include <asm/suspend.h>
#include <asm/system_info.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/i2c/pxa-i2c.h>

#include <mach/audio.h>
#include <mach/irqs.h>
#include <mach/mfp-pxa27x.h>
#include <linux/platform_data/mmc-pxamci.h>
#include <mach/pxa27x-udc.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/platform_data/video-pxafb.h>
#include <mach/pm.h>
#include <linux/platform_data/usb-ohci-pxa27x.h>
#include <mach/pxa27x.h>
#include <mach/pxa2xx-regs.h>
#include <mach/regs-uart.h>
#include <mach/udc.h>
#include <mach/smemc.h> /* MSCn */

#include <mach/titan.h>

#include "generic.h"

/*
 * Time period to check for paused pc104 interrupt.
 */
#define PC104_WATCHDOG_JIFFIES (HZ / 10)

/*
 * Information associated with GPIO interrupt handler for PC104.
 */
static struct pc104_device
{
        struct timer_list watchdog;
        spinlock_t lock;
        unsigned short quiet; /* mask of interrupts that are quiet */
        unsigned long lastbark;
        unsigned int nbark;
        unsigned int npend0;
        unsigned int nok0;
        unsigned long lastpend;
        unsigned int ndiff;
        unsigned long lastdiff;
        unsigned int nok;
} pc104_dev;

/*
 * Interrupt handling
 */
static unsigned short titan_irq_enabled_mask;

/*
 * Index into the IRQ map arrays. 0 for V1 boards, 1 for V2.
 */
static int irq_map_index;

/*
 * Mapping from bit number to IRQ.
 */
static const unsigned int titan_isa_irqs[2][11] = {
        { 3, 4, 5, 6, 7, 10, 11, 12,  9, 14, 15 },  /* V1 */
        { 3, 4, 5, 6, 7,  9, 10, 11, 12, 14, 15 }   /* V2 */
};

/*
 * IRQ to bit mask
 */
static const unsigned short titan_isa_irq_map[2][16] = {
        {
                0,		/* ISA irq #0, invalid */
                0,		/* ISA irq #1, invalid */
                0,		/* ISA irq #2, invalid */
                1 << 0,		/* ISA irq #3 */
                1 << 1,		/* ISA irq #4 */
                1 << 2,		/* ISA irq #5 */
                1 << 3,		/* ISA irq #6 */
                1 << 4,		/* ISA irq #7 */
                0,		/* ISA irq #8, invalid */
                1 << 8,		/* ISA irq #9 */
                1 << 5,		/* ISA irq #10 */
                1 << 6,		/* ISA irq #11 */ 
                1 << 7,		/* ISA irq #12 */
                0,		/* ISA irq #13, invalid */
                1 << 9,		/* ISA irq #14 */
                1 << 10,	/* ISA irq #15 */
        },  /* V1 */
        {
                0,
                0,
                0,
                0x001,          /* ISA irq #3 */
                0x002,          /* ISA irq #4 */
                0x004,          /* ISA irq #5 */
                0x008,          /* ISA irq #6 */
                0x010,          /* ISA irq #7 */
                0,
                0x020,          /* ISA irq #9 */
                0x040,          /* ISA irq #10 */
                0x080,          /* ISA irq #11 */
                0x100,          /* ISA irq #12 */
                0,
                0x200,          /* ISA irq #14 */
                0x400           /* ISA irq #15 */
        }   /* V2 */
};

static inline unsigned short titan_irq_to_bitmask(unsigned int irq)
{
	return titan_isa_irq_map[irq_map_index][irq - PXA_ISA_IRQ(0)];
}

static inline unsigned int titan_bit_to_irq(int bit)
{
	return titan_isa_irqs[irq_map_index][bit] + PXA_ISA_IRQ(0);
}

/* According to the Titan Technical Manual:
 *     Once the PXA270 microprocessor has serviced a PC/104 interrupt,
 *     the corresponding add-on-board clears the interrupt by driving
 *     the IRQ signal low. When the TITAN hardware sees the interrupt
 *     go low the corresponding bit is automatically cleared from the
 *     I1_REG or I2_REG register.
 * In 2.6.35 kernel code for the Titan, the IRQ ack function wrote a one
 * to the appropriate IRQ bit in TITAN_LO_IRQ_STATUS or TITAN_HI_IRQ_STATUS,
 * which according to the above, seems unnecessary. Testing indicates that
 * it has no effect. Note that the chip ack function is only called
 * from handle_edge_irq or handle_level_irq, not by handle_simple_irq.
 */
#ifdef TITAN_ACK_NEEDED
/*
 * An ack function, if is is determined to be needed.
 */
static void titan_ack_pc104_irq(unsigned int irq)
{
        unsigned short ibits = titan_irq_to_bitmask(irq);
        if (ibits & 0xff)
                TITAN_LO_IRQ_STATUS = (ibits) & 0xff;
        else
                TITAN_HI_IRQ_STATUS = (ibits >> 8) & 0x07;
}
#else
static void titan_ack_pc104_irq(struct irq_data *d)
{
}
#endif

static void titan_mask_pc104_irq(struct irq_data *d)
{
	titan_irq_enabled_mask &= ~(titan_irq_to_bitmask(d->irq));
}

static void titan_unmask_pc104_irq(struct irq_data *d)
{
	titan_irq_enabled_mask |= titan_irq_to_bitmask(d->irq);
}

static inline unsigned short titan_pc104_irq_pending(void)
{
        unsigned short ibits, mbits;

        ibits = ((TITAN_LO_IRQ_STATUS) & 0xff) | (TITAN_HI_IRQ_STATUS & 0x7) << 8;

        /* masked bits */
        mbits = ibits & titan_irq_enabled_mask;

        /*
         * Warn about unexpected values from the CPLD. This helped detect
         * the V1 vs V2 difference, and could warn of spurious interrupts.
         */
        if (ibits != mbits) {
                unsigned long j = jiffies;
                pc104_dev.ndiff++;
                if (j - pc104_dev.lastdiff > 300 * HZ) {
                        int td = ((long)j - (long)pc104_dev.lastdiff) / HZ;
			printk(KERN_INFO "PC104 IRQ bits=%#hx, mask=%#hx, #bad=%u, %d/sec, #ok=%u, %d/sec\n",
				ibits, titan_irq_enabled_mask,
                                pc104_dev.ndiff, pc104_dev.ndiff / td,
                                pc104_dev.nok, pc104_dev.nok / td );
                        pc104_dev.ndiff = 0;
                        pc104_dev.nok = 0;
                        pc104_dev.lastdiff = j;
                }
        }
        /* OK means that no bits were set that were not for enabled IRQS */
        else if (ibits) pc104_dev.nok++;
        return mbits;
}

static struct irq_chip titan_pc104_irq_chip = {
	.name           = "PC104",
        .irq_ack        = titan_ack_pc104_irq,
	.irq_mask	= titan_mask_pc104_irq,
	.irq_unmask	= titan_unmask_pc104_irq,
};

static void __init titan_init_irq(void)
{
	int level;


	pxa27x_init_irq();

        /*
         * Much of the configuration of the GPIOs and their associated IRQs is
         * done by pxa_gpio_init in drivers/gpio/gpio-pxa.c.
         */

	/* Peripheral IRQs */
	irq_set_irq_type(PXA_GPIO_TO_IRQ(TITAN_AC97_GPIO),	IRQ_TYPE_EDGE_RISING);
	irq_set_irq_type(PXA_GPIO_TO_IRQ(TITAN_WAKEUP_GPIO),	IRQ_TYPE_EDGE_FALLING);
	irq_set_irq_type(PXA_GPIO_TO_IRQ(TITAN_GPIO_IRQ),	IRQ_TYPE_EDGE_FALLING);
	irq_set_irq_type(PXA_GPIO_TO_IRQ(TITAN_UARTA_GPIO),	IRQ_TYPE_EDGE_RISING);
	irq_set_irq_type(PXA_GPIO_TO_IRQ(TITAN_UARTB_GPIO),	IRQ_TYPE_EDGE_RISING);
	irq_set_irq_type(PXA_GPIO_TO_IRQ(TITAN_ETH0_GPIO),	IRQ_TYPE_EDGE_FALLING);

	/* Setup ISA IRQs */

        /* Which PC104 IRQ mapping to use, 0=V1, 1=V2 */
        irq_map_index = (((TITAN_BOARD_VERSION & 0xff) >> 4) > 1) ? 1 : 0; 

        /* probably not necessary, but we'll do it anyway... */
        TITAN_LO_IRQ_STATUS = 0;
        TITAN_HI_IRQ_STATUS = 0;

	for (level = 0; level < ARRAY_SIZE(titan_isa_irqs[0]); level++) {
                unsigned int isa_irq = titan_bit_to_irq(level);

                printk(KERN_INFO "Map PC104 IRQ %d to IRQ %d\n",
                        titan_isa_irqs[irq_map_index][level], isa_irq);
                irq_set_chip_and_handler(isa_irq, &titan_pc104_irq_chip,
                        handle_simple_irq);
                set_irq_flags(isa_irq, IRQF_VALID | IRQF_PROBE);
	}

        printk(KERN_INFO "Titan PC104 GPIO=%d, irq=%d, nirqs=%d\n",
                TITAN_ISA_IRQ,PXA_GPIO_TO_IRQ(TITAN_ISA_IRQ),
                TITAN_NR_IRQS);
	irq_set_irq_type(PXA_GPIO_TO_IRQ(TITAN_ISA_IRQ),
                IRQ_TYPE_EDGE_RISING);
}

/*
 * Called by handler for the TITAN_ISA_IRQ GPIO interrupt.
 * Calls handlers for pending interrupts.
 */
static inline irqreturn_t
titan_gpio_pc104_do_pending(unsigned short pending)
{
    do {
            while (pending) {
                    int bit = __ffs(pending);
                    unsigned int uirq = titan_bit_to_irq(bit);
                    generic_handle_irq(uirq);
#ifdef TITAN_ACK_NEEDED
                    titan_ack_pc104_irq(uirq);
#endif
                    pending &= ~titan_irq_to_bitmask(uirq);
            }
            pending = titan_pc104_irq_pending();
    } while (pending);
    return IRQ_HANDLED;
}

#ifdef TRY_THREADED_HANDLER
/*
 * Handler for the TITAN_ISA_IRQ GPIO interrupt
 * if we're using a threaded handler.
 * schedules the threaded handler.
 */
static irqreturn_t
titan_gpio_pc104_handler(int irq, void* devid)
{
        struct pc104_device* dev = (struct pc104_device*) devid;
        if (!titan_irq_enabled_mask) return IRQ_NONE;
        return IRQ_WAKE_THREAD;
}
#endif

/*
 * Handler for the TITAN_ISA_IRQ GPIO interrupt.
 * Checks the bits in the interrupt status registers to see what 
 * PC104/ISA needs servicing and calls those handlers, until
 * the pending bits are clear..
 */
static irqreturn_t
#ifdef TRY_THREADED_HANDLER
titan_gpio_pc104_thread_handler(int irq, void* devid)
#else
titan_gpio_pc104_handler(int irq, void* devid)
#endif
{
        unsigned short pending;

#ifdef TRY_THREADED_HANDLER
        unsigned long flags;
#endif

        struct pc104_device* dev = (struct pc104_device*) devid;
        irqreturn_t result;

        pending = titan_pc104_irq_pending();
        if (!pending) {
                unsigned long j = jiffies;
                dev->npend0++;
                if (j - dev->lastpend > 300 * HZ) {
                        int td = ((long)j - (long)pc104_dev.lastpend) / HZ;
			printk(KERN_INFO "PC104 IRQ bits=0, mask=%#hx, #zero=%u, %d/sec, #ok=%u, %d/sec\n",
                                titan_irq_enabled_mask,
                                dev->npend0, dev->npend0 / td,
                                dev->nok0, dev->nok0 / td );
                        dev->npend0 = 0;
                        dev->nok0 = 0;
                        dev->lastpend = j;
                }
                /*
                 * Under heavy PC104 interrupt load, say > 500/sec,
                 * this pending value may be be 0 from time to
                 * time.  Setting it to titan_irq_enabled_mask, so that
                 * all ISRs for enabled PC104 interrupts are called,
                 * reduces the chance of interrupts being dropped.
                 * The gotcha is that an ISR can be called
                 * when its device does not need attention. If the
                 * device and driver support working in a shared-interrupt
                 * type manner, where they can detect if the device needs
                 * service, and return IRQ_NONE otherwise, then this
                 * shouldn't be a problem. If not, then setting 
                 * pending here to the enabled interrupts may have to
                 * be re-thought, or selectable via a kernel parameter.
                 */
                pending = titan_irq_enabled_mask; 
        } 
        else dev->nok0++;

#ifdef TRY_THREADED_HANDLER
        local_irq_save(flags);
#endif
        dev->quiet &= ~pending;
        result = titan_gpio_pc104_do_pending(pending);

#ifdef TRY_THREADED_HANDLER
        local_irq_restore(flags);
#endif
        return result;
}

static void pc104_irq_watchdog(unsigned long data)
{
        struct pc104_device* dev = (struct pc104_device*) data;
        unsigned short quiet;

#ifndef TRY_THREADED_HANDLER
        unsigned long flags;
        spin_lock_irqsave(&dev->lock, flags);
#endif

        quiet = dev->quiet;

        if (quiet) {
                dev->nbark++;
                if (jiffies - dev->lastbark > 300 * HZ) {
                        char info[128];
                        char* cp = info;
                        char* ep = info + sizeof(info) - 10;
                        *cp = 0;
                        while (quiet && cp < ep) {
                                int bit = __ffs(quiet);
                                unsigned int uirq = titan_bit_to_irq(bit);
                                cp += sprintf(cp,"%3u",uirq);
                                quiet &= ~titan_irq_to_bitmask(uirq);
                        }

                        printk(KERN_INFO "pc104_irq_watchdog bark! #%d, quiet IRQs=%s\n",
                                dev->nbark,info);
                        dev->lastbark = jiffies;
                        dev->nbark = 0;
                }
                // call handlers for all enabled, but quiet, interrupts
                titan_gpio_pc104_do_pending(quiet);
        }
        dev->quiet = titan_irq_enabled_mask;
        mod_timer(&dev->watchdog, jiffies + PC104_WATCHDOG_JIFFIES); 

#ifndef TRY_THREADED_HANDLER
        spin_unlock_irqrestore(&dev->lock, flags);
#endif
}

/*
 * Request interrupt (129) for the the TITAN_ISA_IRQ GPIO (17).
 */
static int titan_pc104_init(void)
{
	int err;

        memset(&pc104_dev, 0, sizeof(pc104_dev));

        spin_lock_init(&pc104_dev.lock);
        pc104_dev.quiet = titan_irq_enabled_mask;

#ifdef TRY_THREADED_HANDLER
        err = request_threaded_irq(PXA_GPIO_TO_IRQ(TITAN_ISA_IRQ),
                    titan_gpio_pc104_handler,
                    titan_gpio_pc104_thread_handler,
                    IRQF_TRIGGER_RISING,
                    "GPIO_17-PC104", &pc104_dev);
#else
        err = request_irq(PXA_GPIO_TO_IRQ(TITAN_ISA_IRQ),
                    titan_gpio_pc104_handler,
                    IRQF_TRIGGER_RISING,
                    "GPIO_17-PC104", &pc104_dev);
#endif
	if (err) {
		printk(KERN_ERR "Error %d in request_irq of IRQ %d for PC104 CPLD on GPIO %d\n",
                        err, PXA_GPIO_TO_IRQ(TITAN_ISA_IRQ), TITAN_ISA_IRQ);
                return err;
        }

        /* start the watchdog timer */
        init_timer(&pc104_dev.watchdog);
        pc104_dev.watchdog.function = pc104_irq_watchdog;
        pc104_dev.watchdog.data = (unsigned long)&pc104_dev;
        pc104_dev.watchdog.expires = jiffies + PC104_WATCHDOG_JIFFIES / 10;
        pc104_dev.lastbark = 0;
        add_timer(&pc104_dev.watchdog);

        printk(KERN_INFO "Setup IRQ %d for PC104 CPLD on GPIO %d (with watchdog)\n",
                PXA_GPIO_TO_IRQ(TITAN_ISA_IRQ), TITAN_ISA_IRQ);
        return err;
}

/*
 * Request the interrupt for the TITAN_ISA_IRQ GPIO late in the boot sequence,
 * after the postcore_initcall(pxa_gpio_init) in drivers/gpio/gpio-pxa.c has been done.
 */
device_initcall(titan_pc104_init);

/*
 * Platform devices
 */
static struct resource titan_mtd_resources[] = {
        [0] = { /* NOR Flash (up to 64MB) */
                .start  = TITAN_FLASH_PHYS,
                .end    = TITAN_FLASH_PHYS + SZ_64M - 1,
                .flags  = IORESOURCE_MEM,
        },
        [1] = { /* SRAM */
                .start  = TITAN_SRAM_PHYS,
                .end    = TITAN_SRAM_PHYS + SZ_512K - 1,
                .flags  = IORESOURCE_MEM,
        },
};

static struct physmap_flash_data titan_flash_data[] = {
        [0] = {
                .width          = 2,
                .parts          = NULL,
                .nr_parts       = 0,
        },
};

static struct platform_device titan_mtd_devices[] = {
        [0] = {
                .name           = "physmap-flash",
                .id             = 0,
                .dev            = {
                        .platform_data = &titan_flash_data[0],
                },
                .resource       = &titan_mtd_resources[0],
                .num_resources  = 1,
        },
};

/* Serial */
static struct resource titan_serial_resources[] = {
	/* Internal UARTs */
	{
		.start	= 0x40100000,
		.end	= 0x4010001f,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 0x40200000,
		.end	= 0x4020001f,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 0x40700000,
		.end	= 0x4070001f,
		.flags	= IORESOURCE_MEM,
	},
	/* External UARTs */
	{
		.start	= 0x10000000,
		.end	= 0x1000000f,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 0x10800000,
		.end	= 0x1080000f,
		.flags	= IORESOURCE_MEM,
	},
};

static struct plat_serial8250_port serial_platform_data[] = {
	/* Internal UARTs */
	{ /* FFUART */
		.membase	= (void *)&FFUART,
		.mapbase	= __PREG(FFUART),
		.irq		= IRQ_FFUART,
		.uartclk	= 921600 * 16,
		.regshift	= 2,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
	},
	{ /* BTUART */
		.membase	= (void *)&BTUART,
		.mapbase	= __PREG(BTUART),
		.irq		= IRQ_BTUART,
		.uartclk	= 921600 * 16,
		.regshift	= 2,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
	},
	{ /* STUART */
		.membase	= (void *)&STUART,
		.mapbase	= __PREG(STUART),
		.irq		= IRQ_STUART,
		.uartclk	= 921600 * 16,
		.regshift	= 2,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
	},
	/* External UARTs */
	{ /* COM4 */
		.mapbase	= 0x10000000,
		.irq		= PXA_GPIO_TO_IRQ(TITAN_UARTA_GPIO),
                .irqflags       = IRQF_TRIGGER_RISING,
		.uartclk	= 14745600,
		.regshift	= 1,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
	},
	{ /* COM5 */
		.mapbase	= 0x10800000,
		.irq		= PXA_GPIO_TO_IRQ(TITAN_UARTB_GPIO),
                .irqflags       = IRQF_TRIGGER_RISING,
		.uartclk	= 14745600,
		.regshift	= 1,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
	},
	{ },
};

static struct platform_device titan_serial_device = {
	.name = "serial8250",
	.id   = PLAT8250_DEV_PLATFORM,
	.dev  = {
		.platform_data = serial_platform_data,
	},
	.num_resources	= ARRAY_SIZE(titan_serial_resources),
	.resource	= titan_serial_resources,
};


#if (CONFIG_ARCOM_TITAN_VER & 1)
/* DM9000 Ethernet */
static struct resource titan_dm9000_resource[] = {
	[0] = {
		.start = TITAN_ETH0_PHYS,
		.end   = TITAN_ETH0_PHYS + 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = TITAN_ETH0_PHYS + 2,
		.end   = TITAN_ETH0_PHYS + 3,
		.flags = IORESOURCE_MEM
	},
	[2] = {
		.start = PXA_GPIO_TO_IRQ(TITAN_ETH0_GPIO),
		.end   = PXA_GPIO_TO_IRQ(TITAN_ETH0_GPIO),
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE
	},
};

struct dm9000_plat_data titan_dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY,
};

static struct platform_device titan_dm9000_device = {
	.name		= "dm9000",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(titan_dm9000_resource),
	.resource	= titan_dm9000_resource,
	.dev		= {
		.platform_data = &titan_dm9000_platdata,
	}
};
#endif

#if (CONFIG_ARCOM_TITAN_VER & 2)
/* LAN9221 Ethernet */
static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags          = SMSC911X_USE_16BIT | SMSC911X_FORCE_INTERNAL_PHY,
	.phy_interface  = PHY_INTERFACE_MODE_MII,
};

static struct resource smsc911x_resources[] = {
	{
		.start          = TITAN_ETH0_PHYS,
		.end            = TITAN_ETH0_PHYS + 0x10f,
		.flags          = IORESOURCE_MEM,
	}, {
		.start          = PXA_GPIO_TO_IRQ(TITAN_ETH0_GPIO),
		.end            = PXA_GPIO_TO_IRQ(TITAN_ETH0_GPIO),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device smsc911x_device = {
	.name           = "smsc911x",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(smsc911x_resources),
	.resource       = smsc911x_resources,
	.dev            = {
		.platform_data = &smsc911x_config,
	},
};
#endif

/* Audio */
static pxa2xx_audio_ops_t titan_ac97_info = {
       .reset_gpio	= TITAN_AC97_RST_GPIO,
};

/* External SRAM */
static struct resource titan_sram_resource = {
	.start		= TITAN_SRAM_PHYS,
	.end		= TITAN_SRAM_PHYS + TITAN_SRAM_SIZE - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device titan_sram_device = {
	.name		= "pxa2xx-8bit-sram",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &titan_sram_resource,
};

/* SPI interface on SSP3 */
static struct pxa2xx_spi_master pxa2xx_spi_ssp3_master_info = {
	.num_chipselect = 1,
	.enable_dma     = 1,
};

static struct platform_device pxa2xx_spi_ssp3_device = {
	.name       = "pxa2xx-spi",
	.id         = 3,
	.dev = {
		.platform_data = &pxa2xx_spi_ssp3_master_info,
	},
};

static struct platform_device *titan_devices[] __initdata = {
	&titan_serial_device,
	&titan_mtd_devices[0],
	&titan_sram_device,
	&pxa2xx_spi_ssp3_device,
};

/*
 * MMC/SD Device
 *
 * The card detect interrupt isn't debounced so we delay it by 250ms
 * to give the card a chance to fully insert/eject.
 */
#if (CONFIG_ARCOM_TITAN_VER & 1)
static struct pxamci_platform_data titan_v1_mci_platform_data = {
	.ocr_mask		= MMC_VDD_32_33|MMC_VDD_33_34,
	.detect_delay_ms	= HZ/4,
	.gpio_card_detect	= TITAN_MMC_CD_GPIO,
	.gpio_card_ro		= TITAN_MMC_WP_GPIO,
	.gpio_card_ro_invert	= 1,
	.gpio_power		= -1,	/* Use JP6 to prevent HOTSWAP issues */
};
#endif

#if (CONFIG_ARCOM_TITAN_VER & 2)
static struct pxamci_platform_data titan_v2_mci_platform_data = {
	.ocr_mask		= MMC_VDD_32_33|MMC_VDD_33_34,
	.detect_delay_ms	= HZ/4,
	.gpio_card_detect	= TITAN_MMC_CD_GPIO,
	.gpio_card_ro		= TITAN_MMC_WP_GPIO,
	.gpio_card_ro_invert	= 1,
	.gpio_power		= TITAN_MMC_SD_PEN_N,
	.gpio_power_invert	= 1,
};
#endif

/*
 * USB host
 */
static int titan_ohci_init(struct device *dev)
{
	int err;
	if ((err = gpio_request(TITAN_USB2_PEN_GPIO, "USB2_PWREN"))) {
		dev_err(dev, "Can't request USB2_PWREN\n");
		return err;
	}

	if ((err = gpio_direction_output(TITAN_USB2_PEN_GPIO, 1))) {
		gpio_free(TITAN_USB2_PEN_GPIO);
		dev_err(dev, "Can't enable USB2_PWREN\n");
		return err;
	}

	/* Switch on port 2. */
	gpio_set_value(TITAN_USB2_PEN_GPIO, 1);

	/* Port 2 is shared between the host and client interface. */
	UP2OCR = UP2OCR_HXOE | UP2OCR_HXS | UP2OCR_DMPDE | UP2OCR_DPPDE;

	return 0;
}

static void titan_ohci_exit(struct device *dev)
{
	/* Power-off port 2 */
	gpio_set_value(TITAN_USB2_PEN_GPIO, 0);
	gpio_direction_output(TITAN_USB2_PEN_GPIO, 0);
	gpio_free(TITAN_USB2_PEN_GPIO);
}

static struct pxaohci_platform_data titan_ohci_platform_data = {
	.port_mode	= PMM_NPS_MODE,
	/* Clear Power Control Polarity Low and set Power Sense
	 * Polarity Low. Supply power to USB ports. */
	.flags          = ENABLE_PORT_ALL | ~(POWER_CONTROL_LOW) | POWER_SENSE_LOW,
	.init		= titan_ohci_init,
	.exit		= titan_ohci_exit,
};

/*
 * Flat Panel
 */
static void titan_lcd_power(int on, struct fb_var_screeninfo *var)
{
       gpio_set_value(TITAN_LCD_EN_GPIO, on);
}

static void titan_backlight_power(int on)
{
       gpio_set_value(TITAN_BKLEN_GPIO, on);
}

static int titan_setup_fb_gpios(void)
{
       int err;

       if ((err = gpio_request(TITAN_LCD_EN_GPIO, "LCD_EN")))
               goto out_err;

       if ((err = gpio_direction_output(TITAN_LCD_EN_GPIO, 0)))
               goto out_err_lcd;

       if ((err = gpio_request(TITAN_BKLEN_GPIO, "BKLEN")))
               goto out_err_lcd;

       if ((err = gpio_direction_output(TITAN_BKLEN_GPIO, 0)))
               goto out_err_bkl;

       return 0;

out_err_bkl:
       gpio_free(TITAN_BKLEN_GPIO);
out_err_lcd:
       gpio_free(TITAN_LCD_EN_GPIO);
out_err:
       return err;
}

static struct pxafb_mode_info titan_fb_modeinfo = {
	.pixclock       = 39722,

	.xres           = 640,
	.yres           = 480,

	.bpp            = 16,

	.hsync_len      = 63,
	.left_margin    = 16,
	.right_margin   = 81,

	.vsync_len      = 2,
	.upper_margin   = 12,
	.lower_margin   = 31,

	.sync		= 0,
};

static struct pxafb_mach_info titan_fb_info = {
	.modes			= &titan_fb_modeinfo,
	.num_modes		= 1,
	.lccr0			= LCCR0_Act | LCCR0_Color,
	.lccr3		        = LCCR3_OutEnH | LCCR3_PixFlEdg | LCCR3_Acb(0xFF),
	.lcd_conn               = LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
	.pxafb_lcd_power        = titan_lcd_power,
	.pxafb_backlight_power  = titan_backlight_power,
};

/*
 * USB Device Controller
 */
static void titan_udc_command(int cmd)
{
	switch (cmd) {
	case PXA2XX_UDC_CMD_DISCONNECT:
		printk(KERN_INFO "titan: disconnecting USB client\n");
		UP2OCR = UP2OCR_HXOE | UP2OCR_HXS | UP2OCR_DMPDE | UP2OCR_DPPDE;
		break;

	case PXA2XX_UDC_CMD_CONNECT:
		printk(KERN_INFO "titan: connecting USB client\n");
		UP2OCR = UP2OCR_HXOE | UP2OCR_DPPUE;
		break;
	}
}

static struct pxa2xx_udc_mach_info titan_udc_info = {
	.udc_command = titan_udc_command,
};

#ifdef CONFIG_PM
static void titan_power_off(void)
{
	local_irq_disable();
	PWER = 0;	/* prevent DS_WAKEUP */
	cpu_suspend(PWRMODE_DEEPSLEEP,pxa27x_finish_suspend);
}
#else
#define titan_power_off		NULL
#endif

static struct pca953x_platform_data titan_pca953x_pdata[] = {
	[0] = { .gpio_base = TITAN_USER_GPIO_BASE, },
};

static struct i2c_board_info __initdata titan_i2c_devices[] = {
	{
		I2C_BOARD_INFO("pca9535",       0x20),
		.platform_data  = &titan_pca953x_pdata[0],
		.irq            = PXA_GPIO_TO_IRQ(TITAN_GPIO_IRQ),
	},
	{ I2C_BOARD_INFO("lm75a",       0x48) },
	{ I2C_BOARD_INFO("24c01",       0x50) },
	{ I2C_BOARD_INFO("isl1208",     0x6f) },
};

static mfp_cfg_t common_pin_config[] __initdata = {
        /* only chip select 1,4 and 5 on a titan */
	GPIO15_nCS_1,
	GPIO80_nCS_4,
	GPIO33_nCS_5,

	/* setup GPIO for PXA27x MMC controller */
        GPIO32_MMC_CLK,
	GPIO92_MMC_DAT_0,
	GPIO109_MMC_DAT_1,
	GPIO110_MMC_DAT_2,
	GPIO111_MMC_DAT_3,
	GPIO112_MMC_CMD,

	GPIO88_USBH1_PWR,
	GPIO89_USBH1_PEN,
        /* also see titan_ohci_init() */
        MFP_CFG_IN(GPIO114, AF0),
        MFP_CFG_OUT(GPIO22, AF0, DRIVE_LOW),

	GPIO86_LCD_LDD_16,
	GPIO87_LCD_LDD_17,

	GPIO104_CIF_DD_2,
	GPIO105_CIF_DD_1,

	GPIO48_nPOE,
	GPIO49_nPWE,
	GPIO50_nPIOR,
	GPIO51_nPIOW,
	GPIO85_nPCE_1,
	GPIO54_nPCE_2,
	GPIO79_PSKTSEL,
	GPIO56_nPWAIT,
	GPIO57_nIOIS16,

        /* GPIO lines for setting clock on DUART (ttyS3 and ttyS4)
         * and 422/485 options on ttyS4 */
        MFP_CFG_OUT(GPIO55, AF0, DRIVE_HIGH),   /* DUART, 0=8x clock, 1=16x clock */
        MFP_CFG_OUT(GPIO84, AF0, DRIVE_HIGH),   /* DUART, clock pre-scaler, 0=/4, 1=/1 */
        MFP_CFG_OUT(GPIO78, AF0, DRIVE_HIGH),   /* ttyS4, 0=RTS half duplex cntl, 1=norm */
        MFP_CFG_OUT(GPIO81, AF0, DRIVE_HIGH),   /* 0=RS485, 1=RS422 */
        MFP_CFG_OUT(GPIO115, AF0, DRIVE_HIGH),   /* 0=no term, 1=120 Ohm term */
 
        MFP_CFG_OUT(GPIO102, AF0, DRIVE_HIGH),  /* Watchdog toggle line */
 
        /* I2C */
        GPIO117_I2C_SCL,
        GPIO118_I2C_SDA,
};

#if (CONFIG_ARCOM_TITAN_VER & 1)
static mfp_cfg_t titan_v1_pin_config[] __initdata = {
       MFP_CFG_OUT(GPIO83, AF0, DRIVE_HIGH),   /* ttyS4, 0=reduced slew, 1=normal */
       MFP_CFG_OUT(GPIO96, AF0, DRIVE_HIGH),   /* Watchdog timeout, 96=1,97=1,99=0 disabled */
       MFP_CFG_OUT(GPIO97, AF0, DRIVE_HIGH),
       MFP_CFG_OUT(GPIO99, AF0, DRIVE_LOW),
};
#endif

#if (CONFIG_ARCOM_TITAN_VER & 2)
static mfp_cfg_t titan_v2_pin_config[] __initdata = {
	MFP_CFG_OUT(GPIO99, AF0, DRIVE_HIGH),   /* SD Power enable, 0=on, 1=off */

};
#endif

static void __init titan_init(void)
{
        uint32_t u32val, u32val2;
        extern struct resource ioport_resource;


	system_rev = ((TITAN_BOARD_VERSION & 0xFF) << 8) | \
	             (TITAN_CPLD_VERSION & 0xFF);

	printk(KERN_INFO "Titan board version : V%dI%d, CPLD version : V%dI%d\n", \
	                (system_rev >> 12), ((system_rev>>8) & 0xf),    \
	                ((system_rev >> 4) & 0xf), system_rev & 0xf    );

	pm_power_off = titan_power_off;

        /* report ioport range */
	printk(KERN_DEBUG "ioport_resource, start=%#zx, end=%#zx\n",
                ioport_resource.start, ioport_resource.end);

	pxa2xx_mfp_config(ARRAY_AND_SIZE(common_pin_config));
#if ((CONFIG_ARCOM_TITAN_VER & 3) == 3)
	if ((system_rev >> 12) < 2)
		pxa2xx_mfp_config(ARRAY_AND_SIZE(titan_v1_pin_config));
	else
		pxa2xx_mfp_config(ARRAY_AND_SIZE(titan_v2_pin_config));
#elif (CONFIG_ARCOM_TITAN_VER & 1)
	pxa2xx_mfp_config(ARRAY_AND_SIZE(titan_v1_pin_config));
#else
	pxa2xx_mfp_config(ARRAY_AND_SIZE(titan_v2_pin_config));
#endif

	platform_add_devices(titan_devices, ARRAY_SIZE(titan_devices));

#if ((CONFIG_ARCOM_TITAN_VER & 3) == 3)
	platform_device_register((system_rev>>12) < 2 ? &titan_dm9000_device :
	                                                &smsc911x_device    );
#elif (CONFIG_ARCOM_TITAN_VER & 1)
	platform_device_register(&titan_dm9000_device);
#else
	platform_device_register(&smsc911x_device);
#endif

	pxa_set_ohci_info(&titan_ohci_platform_data);
#if ((CONFIG_ARCOM_TITAN_VER & 3) == 3)
	pxa_set_mci_info((system_rev>>12) < 2 ? &titan_v1_mci_platform_data :
	                                        &titan_v2_mci_platform_data);
#elif (CONFIG_ARCOM_TITAN_VER & 1)
	pxa_set_mci_info(&titan_v1_mci_platform_data);
#else
	pxa_set_mci_info(&titan_v2_mci_platform_data);
#endif

	pxa_set_udc_info(&titan_udc_info);
	pxa_set_ac97_info(&titan_ac97_info);

	if (titan_setup_fb_gpios())
		pr_err("Failed to setup framebuffer GPIOs\n");
	else
		pxa_set_fb_info(NULL, &titan_fb_info);

	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(titan_i2c_devices));

#if (CONFIG_ARCOM_TITAN_VER & 1)
        /* Adjust timing to DM9000 ethernet chip, which is on chip select 1,
         * by tweaking high order bits in MSC0.
         * Default values result in "status check fail: 210" errors from the dm9000 driver.
         *
         * See "Intel PXA27x Processor Family Developer's Manual"
         *
         * Default values (apparently set in RedBoot) were (as shown by pxaregs program):
         * MSC0=0xe25924e8
         * MSC0_RT1                          1  nCS[1] ROM Type, bits 16-18
         * MSC0_RBW1                         1  nCS[1] ROM Bus Width (1=16bit), bit 19
         * MSC0_RDF1                         5  nCS[1] ROM Delay First Access, bits 20-23
         * MSC0_RDN1                         2  nCS[1] ROM Delay Next Access, bits 24-27
         * MSC0_RRR1                         6  nCS[1] ROM/SRAM Recovery Time, bits 28-30
         * MSC0_RBUFF1                       1  nCS[1] Return Buffer Behavior (1=streaming), bit 31
         *
         * Experimenting indicates that increasing RDF1 to 7 from 5 fixes the problem.
         * pxaregs program is very handy for testing these values at runtime.
         *
         * In testing, saw some "status check fail", on about 10% of titans with
         * RDF1=7. So we'll up it to 9.
         *
         * These values were successfully used on a titan on the CU Ameriflux
         * tower. RDN1=7, RDF1=c. So we'll use those.
         */
        u32val = *(unsigned int*) MSC0;
        *(unsigned int*) MSC0 = (u32val & 0xf00fffff) | 0x07c00000;
        u32val2 = *(unsigned int*) MSC0;
        printk(KERN_INFO "*MSC0 changed from %#x to %#x, CS1 RDFx=%#x RDNx=%#x, RRRx=%#x\n",
                u32val, u32val2, (u32val2 >> 20) & 0xf, (u32val2 >> 24) & 0xf,
                (u32val2 >> 28) & 0x7);
#endif
        /*
         * Adjust PC104 bus timing.  See section 6.5.5 of the
         * "Intel PXA27x Processor Family Developer's Manual", concerning
         * PXA Expansion Memory Timing Configuration Registers.
         *
         * To read/write the values of register MCIO1 on a running system, use the
         * pxaregs command:
         *    pxaregs MCIO1
         *    paxregs MCIO1 0x14891
         *
         * Default values, 2.6.35:
         *  MCIO(1)=0x14511 (HOLD=5,ASST=10,SET=17)
         *
         * With the above values a Titan could not communicate fully with a Diamond
         * Emerald 8P PC104 serial card. Increasing values to 25 is necessary:
         *  MCIO(1)=0x64c99 (HOLD=25,ASST=25,SET=25)
         */
        u32val = (25 << 14) + (25 << 7) + 25;
        *(unsigned int*) MCIO1 = u32val;
        u32val2 = *(unsigned int*) MCIO1;
        printk(KERN_INFO "*MCIO1 set to %#x (HOLD=%d,ASST=%d,SET=%d): PC104 bus timing\n",
                u32val,(u32val % 0xfc000) >> 14, (u32val & 0xf80) >> 7,
                (u32val & 0x7f));
}

static struct map_desc titan_io_desc[] __initdata = {
	{
		.virtual = (unsigned long)TITAN_CPLD_P2V(_TITAN_BOARD_VERSION),
		.pfn     = __phys_to_pfn(_TITAN_BOARD_VERSION),
		.length  = TITAN_CPLD_REG_VIRT_SIZE,
		.type    = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)TITAN_CPLD_P2V(_TITAN_HI_IRQ_STATUS),
		.pfn     = __phys_to_pfn(_TITAN_HI_IRQ_STATUS),
		.length  = TITAN_CPLD_REG_VIRT_SIZE,
		.type    = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)TITAN_CPLD_P2V(_TITAN_CPLD_VERSION),
		.pfn     = __phys_to_pfn(_TITAN_CPLD_VERSION),
		.length  = TITAN_CPLD_REG_VIRT_SIZE,
		.type    = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)TITAN_CPLD_P2V(_TITAN_LO_IRQ_STATUS),
		.pfn     = __phys_to_pfn(_TITAN_LO_IRQ_STATUS),
		.length  = TITAN_CPLD_REG_VIRT_SIZE,
		.type    = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)TITAN_CPLD_P2V(_TITAN_MISC),
		.pfn     = __phys_to_pfn(_TITAN_MISC),
		.length  = TITAN_CPLD_REG_VIRT_SIZE,
		.type    = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)TITAN_PC104IO_BASE,
		.pfn     = __phys_to_pfn(TITAN_PC104IO_PHYS),
		.length  = 0x800000,
		.type    = MT_DEVICE,
	},
};

static void __init titan_map_io(void)
{
	pxa27x_map_io();

	iotable_init(titan_io_desc, ARRAY_SIZE(titan_io_desc));

	/* Clear PSPR to ensure a full restart on wake-up. */
	PMCR = PSPR = 0;

	/* enable internal 32.768Khz oscillator (ignore OSCC_OOK) */
	OSCC |= OSCC_OON;

	/* setup default sleep mode values */
	PWER = PFER = 0x2;	/* enable DS_WAKEUP on -ve edge */
	PRER = 0;
	/* set GPIO sleep states */
	PGSR0 = 0x0800800C;
	PGSR1 = 0x00CF0002;
	PGSR2 = 0x903BC000;
	PGSR3 = 0x00100043;

	/* When entering deep-sleep on the TITAN the 3.3V supply is switched
	 * off which causes the power manager IC to assert nVDD_FAULT.  If
	 * nVDD_FAULT is asserted and not ignored the CPU will ignore wake-up
	 * events from the RTC. */
	PSLR = (0xC << 28) | (0xC << 24) | (1 << 22);	/* PSLR_IVF is (1 << 22) */

	/* Some clock cycles later (from OSCC_ON), programme PCFR (OPDE...).
	 * float chip selects */
	PCFR = PCFR_OPDE | PCFR_DC_EN | PCFR_FS;
}

MACHINE_START(ARCOM_TITAN, "Eurotech TITAN")
	/* Maintainer: Eurotech Ltd. */
	.map_io		= titan_map_io,
        .nr_irqs        = TITAN_NR_IRQS,
	.init_irq	= titan_init_irq,
        .handle_irq     = pxa27x_handle_irq,
	.init_time	= pxa_timer_init,
	.init_machine	= titan_init,
        .restart        = pxa_restart,
MACHINE_END
