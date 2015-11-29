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
#include <linux/serial_8250.h>

#include <linux/i2c/pca953x.h>
#include <linux/mmc/host.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/spi.h>
#include <linux/kernel_stat.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/i2c.h>

#include <mach/audio.h>
#include <mach/irqs.h>
#include <mach/mfp-pxa27x.h>
#include <mach/mmc.h>
#include <mach/pxa27x-udc.h>
#include <mach/pxa2xx_spi.h>
#include <mach/pxafb.h>
#include <mach/pm.h>
#include <mach/ohci.h>
#include <mach/pxa27x.h>
#include <mach/pxa2xx-regs.h>
#include <mach/regs-uart.h>
#include <mach/udc.h>

#include <mach/titan.h>

#include "generic.h"

/*
 * Interrupt handling
 */

#define MAX_PC104_IRQ_HANDLE_EVENTS 10

static unsigned int titan_irq_enabled_mask;
static const int titan_isa_irqs[] = { 3, 4, 5, 6, 7, 10, 11, 12, 9, 14, 15 };
static const int titan_isa_irq_map[] = {
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
};

static inline int titan_irq_to_bitmask(unsigned int irq)
{
	return titan_isa_irq_map[irq - PXA_ISA_IRQ(0)];
}

static inline int titan_bit_to_irq(int bit)
{
	return titan_isa_irqs[bit] + PXA_ISA_IRQ(0);
}

/* According to the Titan Technical Manual:
 *     Once the PXA270 microprocessor has serviced a PC/104 interrupt,
 *     the corresponding add-on-board clears the interrupt by driving
 *     the IRQ signal low. When the TITAN hardware sees the interrupt
 *     go low the corresponding bit is automatically cleared from the
 *     I1_REG or I2_REG register.
 * Therefore, setting this bit in the TITAN_CPLD_ISA_IRQ register
 * is unnecessary, hence NEED_TO_ACK_TITAN_PC104_IRQ is not defined.
 */
#ifdef NEED_TO_ACK_TITAN_PC104_IRQ
static void titan_ack_irq(unsigned int irq)
{
	int titan_irq = titan_irq_to_bitmask(irq);

	if (titan_irq & 0xff)
		TITAN_LO_IRQ_STATUS = titan_irq;
	else
		TITAN_HI_IRQ_STATUS = (titan_irq >> 8);
}
#endif

static void titan_mask_irq(unsigned int irq)
{
	titan_irq_enabled_mask &= ~(titan_irq_to_bitmask(irq));
}

static void titan_unmask_irq(unsigned int irq)
{
	titan_irq_enabled_mask |= titan_irq_to_bitmask(irq);
}

static inline unsigned int titan_irq_pending(void)
{
	return (TITAN_HI_IRQ_STATUS << 8 | TITAN_LO_IRQ_STATUS) &
			titan_irq_enabled_mask;
}

/*
 * Top level hander for the TITAN_ISA_IRQ GPIO interrupt.
 * Checks the bits in the interrupt status registers to see what 
 * PC104/ISA needs servicing and calls those handlers, until
 * the bits are clear..
 */
static irqreturn_t
titan_gpio_pc104_handler(int irq, void *dev_id)
{
        unsigned int pending = titan_irq_pending();
#ifdef DEBUG_SPURIOUS
        static unsigned int spurious;
#endif

        /*
         * Hack which seems to help with a possible bug in the PC104 CPLD.
         * Sometimes a PC104 IRQ is high, and this hander is called,
         * but the corresponding bit is not set in the TITAN_*_IRQ_STATUS
         * registers. In this case, set pending to the unmasked interrupts.
         * registers, and go ahead and service the enabled PC104 interrupts
         * anyway, otherwise this GPIO handler may never be called again.
         * It didn't help to just return IRQ_NONE in this case.
         */
        if (!pending) {
#ifdef DEBUG_SPURIOUS
                if (!(spurious++ % 1000))
                    printk(KERN_WARNING "%u spurious interrupts\n",spurious);
#endif
                pending = titan_irq_enabled_mask;
        }

        do {
                while (likely(pending)) {
                        int bit = __ffs(pending);
                        unsigned int titan_irq = titan_bit_to_irq(bit);
                        struct irq_desc *titan_desc = irq_to_desc(titan_irq);
                        titan_desc->handle_irq(titan_irq, titan_desc);
                        pending &= ~(1 << bit);
                }
                pending = titan_irq_pending();
        } while (pending);
        return IRQ_HANDLED;
}

/*
 *	handle_pc104_irq - Low level handler for PC104/ISA IRQS on a Titan.
 *	This is identical to handle_simple_irq in kernel/irq/chip.c, except
 *	that the action for the irq is called up to MAX_PC104_IRQ_HANDLE_EVENTS
 *	or until the return value is not IRQ_HANDLED.
 *	The intent is to make sure that the PC104 interrupt line actually is deasserted
 *	so that the Titan PC104 CPLD can do its thing.  Otherwise, if the action list for
 *	the PC104 irq is just called once, the IRQ line may not go low, and we may
 *	not get another GPIO interrupt. This is especially the case for multi-port
 *	serial cards, where doing one pass to service the UARTS may not cause the
 *	IRQ to be desasserted, if a lower numbered port interrupts while handling
 *	a higher numbered port.
 *
 *	Note that the mask/unmask functions in titan_pc104_irq_chip are not called.
 *	Only one instance of this hander will be active (on a CPU), since
 *	it is called by an edge handler, which has spinlocks around checks
 *	of whether handers for the GPIO interrupt are IRQ_INPROGRESS.
 *
 *	Because of the above, one probably doesn't have to check for
 *	IRQ_INPROGRESS here.
 */
static void
handle_pc104_irq(unsigned int irq, struct irq_desc *desc)
{
	struct irqaction *action;
	irqreturn_t action_ret;
        extern int noirqdebug;
        int ncall;

	raw_spin_lock(&desc->lock);

	if (unlikely(desc->status & IRQ_INPROGRESS))
		goto out_unlock;
	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);
	kstat_incr_irqs_this_cpu(irq, desc);

	action = desc->action;
	if (unlikely(!action || (desc->status & IRQ_DISABLED)))
		goto out_unlock;

#ifdef NEED_TO_ACK_TITAN_PC104_IRQ
        if (likely(desc->chip->ack)) desc->chip->ack(irq);
#endif

	desc->status |= IRQ_INPROGRESS;

	raw_spin_unlock(&desc->lock);

        for (ncall = 0; ncall < MAX_PC104_IRQ_HANDLE_EVENTS; ncall++) {
                action_ret = handle_IRQ_event(irq, action);
                if (!noirqdebug)
                        note_interrupt(irq, desc, action_ret);
                if (action_ret != IRQ_HANDLED) break;
        }

        /* if !noirqdebug, then interrupt lockups are being detected */
        if (unlikely(noirqdebug && ncall == MAX_PC104_IRQ_HANDLE_EVENTS))
		printk(KERN_WARNING "Possible lockup on PC104 IRQ %d\n",irq);
        else if (unlikely(ncall > 3))
 		printk(KERN_INFO "handle_IRQ_event called %d times for IRQ %d\n",ncall,irq);

	raw_spin_lock(&desc->lock);
	desc->status &= ~IRQ_INPROGRESS;
out_unlock:
	raw_spin_unlock(&desc->lock);
}

static struct irqaction titan_gpio_pc104_irq = {
        .name           = "GPIO_17-PC104",
        .flags          = IRQF_VALID | IRQF_TRIGGER_RISING,
        .handler        = titan_gpio_pc104_handler,
};

static struct irq_chip titan_pc104_irq_chip = {
	.name   = "PC104",
#ifdef NEED_TO_ACK_TITAN_PC104_IRQ
	.ack	= titan_ack_irq,
#endif
	.mask	= titan_mask_irq,
	.unmask	= titan_unmask_irq,
};

static void __init titan_init_irq(void)
{
	int level;
	int isa_irq;

	pxa27x_init_irq();

	/* Peripheral IRQs */
	set_irq_type(gpio_to_irq(TITAN_AC97_GPIO),	IRQ_TYPE_EDGE_RISING);
	set_irq_type(gpio_to_irq(TITAN_WAKEUP_GPIO),	IRQ_TYPE_EDGE_FALLING);
	set_irq_type(gpio_to_irq(TITAN_GPIO_IRQ),	IRQ_TYPE_EDGE_FALLING);
	set_irq_type(gpio_to_irq(TITAN_UARTA_GPIO),	IRQ_TYPE_EDGE_RISING);
	set_irq_type(gpio_to_irq(TITAN_UARTB_GPIO),	IRQ_TYPE_EDGE_RISING);
	set_irq_type(gpio_to_irq(TITAN_ETH0_GPIO),	IRQ_TYPE_EDGE_FALLING);

	/* Setup ISA IRQs */
	for (level = 0; level < ARRAY_SIZE(titan_isa_irqs); level++) {
		isa_irq = titan_bit_to_irq(level);
		printk(KERN_INFO "Map PC104 IRQ %d to IRQ %d, with looping handler\n", titan_isa_irqs[level], isa_irq);
		set_irq_chip(isa_irq, &titan_pc104_irq_chip);
		set_irq_handler(isa_irq, handle_pc104_irq);
		set_irq_flags(isa_irq, IRQF_VALID | IRQF_PROBE);
	}
        printk(KERN_INFO "Titan PC104 GPIO=%d, irq=%d\n", TITAN_ISA_IRQ,gpio_to_irq(TITAN_ISA_IRQ));
	set_irq_type(gpio_to_irq(TITAN_ISA_IRQ), IRQ_TYPE_EDGE_RISING);
	set_irq_chained_handler(gpio_to_irq(TITAN_ISA_IRQ), handle_edge_irq);
        setup_irq(gpio_to_irq(TITAN_ISA_IRQ), &titan_gpio_pc104_irq);
}

/*
 * Platform devices
 */

/* Flash */
static struct platform_device titan_flash_device = {
        .name           = "flash",
        .id             = -1,
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
		.irq		= gpio_to_irq(TITAN_UARTA_GPIO),
                .irqflags       = IRQF_TRIGGER_RISING,
		.uartclk	= 14745600,
		.regshift	= 1,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
	},
	{ /* COM5 */
		.mapbase	= 0x10800000,
		.irq		= gpio_to_irq(TITAN_UARTB_GPIO),
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
		.start = gpio_to_irq(TITAN_ETH0_GPIO),
		.end   = gpio_to_irq(TITAN_ETH0_GPIO),
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
		.end            = TITAN_ETH0_PHYS + 0xff,
		.flags          = IORESOURCE_MEM,
	}, {
		.start          = gpio_to_irq(TITAN_ETH0_GPIO),
		.end            = gpio_to_irq(TITAN_ETH0_GPIO),
		.flags          = IORESOURCE_IRQ,
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
	.id		= 0,
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
	&titan_flash_device,
	&titan_serial_device,
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
	if ((err = gpio_request(TITAN_USB2_PWR_EN_GPIO, "USB2_PWREN"))) {
		dev_err(dev, "Can't request USB2_PWREN\n");
		return err;
	}

	if ((err = gpio_direction_output(TITAN_USB2_PWR_EN_GPIO, 1))) {
		gpio_free(TITAN_USB2_PWR_EN_GPIO);
		dev_err(dev, "Can't enable USB2_PWREN\n");
		return err;
	}

	/* Switch on port 2. */
	gpio_set_value(TITAN_USB2_PWR_EN_GPIO, 1);

	/* Port 2 is shared between the host and client interface. */
	UP2OCR = UP2OCR_HXOE | UP2OCR_HXS | UP2OCR_DMPDE | UP2OCR_DPPDE;

	return 0;
}

static void titan_ohci_exit(struct device *dev)
{
	/* Power-off port 2 */
	gpio_set_value(TITAN_USB2_PWR_EN_GPIO, 0);
	gpio_direction_output(TITAN_USB2_PWR_EN_GPIO, 0);
	gpio_free(TITAN_USB2_PWR_EN_GPIO);
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
	pxa27x_cpu_suspend(PWRMODE_DEEPSLEEP);
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
		.irq            = gpio_to_irq(TITAN_GPIO_IRQ),
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

        /* also see titan_ohci_init() */
	MFP_CFG_OUT(GPIO22, AF0, DRIVE_HIGH),   /* USB channel 2 power enable */
	MFP_CFG_IN(GPIO114, AF0),               /* USB channel 2 over-current detect */

	/* setup GPIO for PXA27x MMC controller */
	GPIO32_MMC_CLK,
	GPIO92_MMC_DAT_0,
	GPIO109_MMC_DAT_1,
	GPIO110_MMC_DAT_2,
	GPIO111_MMC_DAT_3,
	GPIO112_MMC_CMD,

	GPIO88_USBH1_PWR,
	GPIO89_USBH1_PEN,
	GPIO119_USBH2_PWR,
	GPIO120_USBH2_PEN,

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
        uint32_t mcioval;

	system_rev = ((TITAN_BOARD_VERSION & 0xFF) << 8) | \
	             (TITAN_CPLD_VERSION & 0xFF);

	printk("Titan board version : V%dI%d, CPLD version : V%dI%d\n", \
	                (system_rev >> 12), ((system_rev>>8) & 0xf),    \
	                ((system_rev >> 4) & 0xf), system_rev & 0xf    );

	pm_power_off = titan_power_off;
	
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
		set_pxa_fb_info(&titan_fb_info);

	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(titan_i2c_devices));

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
        mcioval = (25 << 14) + (25 << 7) + 25;
        MCIO(1) = mcioval;
        printk(KERN_INFO "MCIO(1) set to %#x (HOLD=%d,ASST=%d,SET=%d): PC104 bus timing\n",
                mcioval,(mcioval % 0xfc000) >> 14, (mcioval & 0xf80) >> 7,
                (mcioval & 0x7f));
}

static struct map_desc titan_io_desc[] __initdata = {
	{
		.virtual = TITAN_CPLD_P2V(_TITAN_BOARD_VERSION),
		.pfn     = __phys_to_pfn(_TITAN_BOARD_VERSION),
		.length  = TITAN_CPLD_REG_VIRT_SIZE,
		.type    = MT_DEVICE,
	},
	{
		.virtual = TITAN_CPLD_P2V(_TITAN_HI_IRQ_STATUS),
		.pfn     = __phys_to_pfn(_TITAN_HI_IRQ_STATUS),
		.length  = TITAN_CPLD_REG_VIRT_SIZE,
		.type    = MT_DEVICE,
	},
	{
		.virtual = TITAN_CPLD_P2V(_TITAN_CPLD_VERSION),
		.pfn     = __phys_to_pfn(_TITAN_CPLD_VERSION),
		.length  = TITAN_CPLD_REG_VIRT_SIZE,
		.type    = MT_DEVICE,
	},
	{
		.virtual = TITAN_CPLD_P2V(_TITAN_LO_IRQ_STATUS),
		.pfn     = __phys_to_pfn(_TITAN_LO_IRQ_STATUS),
		.length  = TITAN_CPLD_REG_VIRT_SIZE,
		.type    = MT_DEVICE,
	},
	{
		.virtual = TITAN_CPLD_P2V(_TITAN_MISC),
		.pfn     = __phys_to_pfn(_TITAN_MISC),
		.length  = TITAN_CPLD_REG_VIRT_SIZE,
		.type    = MT_DEVICE,
	},
	{
		.virtual = TITAN_PC104IO_BASE,
		.pfn     = __phys_to_pfn(TITAN_PC104IO_PHYS),
		.length  = 0x800000,
		.type    = MT_DEVICE,
	},
};

static void __init titan_map_io(void)
{
	pxa_map_io();

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
	.phys_io	= 0x40000000,
	.io_pg_offst	= ((io_p2v(0x40000000) >> 18) & 0xfffc),
	.boot_params	= 0xa0000100,
	.map_io		= titan_map_io,
	.init_irq	= titan_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= titan_init,
MACHINE_END
