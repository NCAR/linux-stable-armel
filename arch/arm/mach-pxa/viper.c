/*
 *  linux/arch/arm/mach-pxa/viper.c
 *
 *  Support for the Arcom VIPER SBC.
 *
 *  Author:	Ian Campbell
 *  Created:    Feb 03, 2003
 *  Copyright:  Arcom Control Systems
 *
 *  Maintained by Marc Zyngier <maz@misterjones.org>
 *                             <marc.zyngier@altran.com>
 *
 * Based on lubbock.c:
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/memory.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/pxa-i2c.h>
#include <linux/serial_8250.h>
#include <linux/smc91x.h>
#include <linux/pwm_backlight.h>
#include <linux/usb/isp116x.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/syscore_ops.h>

#include <mach/pxa25x.h>
#include <mach/audio.h>
#include <linux/platform_data/video-pxafb.h>
#include <mach/regs-uart.h>
#include <mach/pxa2xx-regs.h>
#include <linux/platform_data/pcmcia-pxa2xx_viper.h>
#include <mach/viper.h>
#include <mach/smemc.h> /* MSCn */

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>
#include <asm/sizes.h>
#include <asm/system_info.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include "generic.h"
#include "devices.h"

/* Will be non-zero for Version 2 cards */
u8 viper_version;   

/*
 * Information associated with GPIO interrupt handler for PC104.
 */
static struct pc104_device
{
        unsigned long lastpend;
        unsigned int npend0;
        unsigned int nok0;
        unsigned long lastdiff;
        unsigned int ndiff;
        unsigned int nok;
} pc104_dev;

/*
 * See section titled "PC/104 Interrupts" in the Viper Technical Manual.
 * Interrupts can work in either of two modes:
 * Linux/VxWorks: leave AUTO_CLR and RETRIG bits in the
 *  interrupt control register at their default 0 value.
 *  GPIO1 then toggles on each interrupt.
 * Windows: set AUTO_CLR bit in interrupt control register, and set
 *  RETRIG bit after handling the interrupt.
 *  GPIO1 then goes low when the interrupt is serviced, and setting
 *  the RETRIG bit will cause it go to high again.
 */
#define VIPER_IRQ_AUTOCLR_RETRIG

static unsigned short pc104_icr;

static void viper_icr_set_bit(unsigned short bit)
{
	pc104_icr |= bit;
	VIPER_ICR = pc104_icr;
}

static void viper_icr_clear_bit(unsigned short bit)
{
	pc104_icr &= ~bit;
	VIPER_ICR = pc104_icr;
}

/* This function is used from the pcmcia module to reset the CF */
static void viper_cf_reset(int state)
{
	if (state)
		viper_icr_set_bit(VIPER_ICR_CF_RST);
	else
		viper_icr_clear_bit(VIPER_ICR_CF_RST);
}

static struct arcom_pcmcia_pdata viper_pcmcia_info = {
	.cd_gpio	= VIPER_CF_CD_GPIO,
	.rdy_gpio	= VIPER_CF_RDY_GPIO,
	.pwr_gpio	= VIPER_CF_POWER_GPIO,
	.reset		= viper_cf_reset,
};

static struct platform_device viper_pcmcia_device = {
	.name		= "viper-pcmcia",
	.id		= -1,
	.dev		= {
		.platform_data	= &viper_pcmcia_info,
	},
};

/*
 * The CPLD version register was not present on VIPER boards prior to
 * v2i1. On v1 boards where the version register is not present we
 * will just read back the previous value from the databus.
 *
 * Therefore we do two reads. The first time we write 0 to the
 * (read-only) register before reading and the second time we write
 * 0xff first. If the two reads do not match or they read back as 0xff
 * or 0x00 then we have version 1 hardware.
 */
static u8 viper_hw_version(void)
{
	u8 v1, v2;
	unsigned long flags;

	local_irq_save(flags);

	VIPER_VERSION = 0;
	v1 = VIPER_VERSION;
	VIPER_VERSION = 0xff;
	v2 = VIPER_VERSION;

	v1 = (v1 != v2 || v1 == 0xff) ? 0 : v1;

	local_irq_restore(flags);
	return v1;
}

/* CPU system core operations. */
static int viper_cpu_suspend(void)
{
	viper_icr_set_bit(VIPER_ICR_R_DIS);
	return 0;
}

static void viper_cpu_resume(void)
{
	viper_icr_clear_bit(VIPER_ICR_R_DIS);
}

static struct syscore_ops viper_cpu_syscore_ops = {
	.suspend	= viper_cpu_suspend,
	.resume		= viper_cpu_resume,
};

static unsigned int current_voltage_divisor;

/*
 * If force is not true then step from existing to new divisor. If
 * force is true then jump straight to the new divisor. Stepping is
 * used because if the jump in voltage is too large, the VCC can dip
 * too low and the regulator cuts out.
 *
 * force can be used to initialize the divisor to a know state by
 * setting the value for the current clock speed, since we are already
 * running at that speed we know the voltage should be pretty close so
 * the jump won't be too large
 */
static void viper_set_core_cpu_voltage(unsigned long khz, int force)
{
	int i = 0;
	unsigned int divisor = 0;
	const char *v;

	if (khz < 200000) {
		v = "1.0"; divisor = 0xfff;
	} else if (khz < 300000) {
		v = "1.1"; divisor = 0xde5;
	} else {
		v = "1.3"; divisor = 0x325;
	}

	pr_debug("viper: setting CPU core voltage to %sV at %d.%03dMHz\n",
		 v, (int)khz / 1000, (int)khz % 1000);

#define STEP 0x100
	do {
		int step;

		if (force)
			step = divisor;
		else if (current_voltage_divisor < divisor - STEP)
			step = current_voltage_divisor + STEP;
		else if (current_voltage_divisor > divisor + STEP)
			step = current_voltage_divisor - STEP;
		else
			step = divisor;
		force = 0;

		gpio_set_value(VIPER_PSU_CLK_GPIO, 0);
		gpio_set_value(VIPER_PSU_nCS_LD_GPIO, 0);

		for (i = 1 << 11 ; i > 0 ; i >>= 1) {
			udelay(1);

			gpio_set_value(VIPER_PSU_DATA_GPIO, step & i);
			udelay(1);

			gpio_set_value(VIPER_PSU_CLK_GPIO, 1);
			udelay(1);

			gpio_set_value(VIPER_PSU_CLK_GPIO, 0);
		}
		udelay(1);

		gpio_set_value(VIPER_PSU_nCS_LD_GPIO, 1);
		udelay(1);

		gpio_set_value(VIPER_PSU_nCS_LD_GPIO, 0);

		current_voltage_divisor = step;
	} while (current_voltage_divisor != divisor);
}

/* Interrupt handling */
static unsigned short viper_irq_enabled_mask;

static const unsigned int viper_isa_irqs[] = { 3, 4, 5, 6, 7, 10, 11, 12, 9, 14, 15 };

static const unsigned short viper_isa_irq_map[] = {
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

static inline unsigned short viper_irq_to_bitmask(unsigned int irq)
{
	return viper_isa_irq_map[irq - PXA_ISA_IRQ(0)];
}

static inline unsigned int viper_bit_to_irq(int bit)
{
	return viper_isa_irqs[bit] + PXA_ISA_IRQ(0);
}

/*
 * Write bit to PC104 interrupt register.
 */
static void viper_ack_pc104_irq(struct irq_data *d)
{
	unsigned short bit = viper_irq_to_bitmask(d->irq);
	if (bit & 0xff)
		VIPER_LO_IRQ_STATUS = bit;
	else
		VIPER_HI_IRQ_STATUS = (bit >> 8);
}

static void viper_mask_pc104_irq(struct irq_data *d)
{
	viper_irq_enabled_mask &= ~(viper_irq_to_bitmask(d->irq));
}

static void viper_unmask_pc104_irq(struct irq_data *d)
{
	viper_irq_enabled_mask |= viper_irq_to_bitmask(d->irq);
}

static struct irq_chip viper_pc104_irq_chip = {
	.name		= "PC104",
	.irq_ack	= viper_ack_pc104_irq,
	.irq_mask	= viper_mask_pc104_irq,
	.irq_unmask	= viper_unmask_pc104_irq
};

static inline unsigned short viper_pc104_irq_pending(void)
{
        unsigned short int ibits, mbits;
        
        ibits = (VIPER_LO_IRQ_STATUS & 0xff);
        if (viper_version) ibits |= (VIPER_HI_IRQ_STATUS & 0x7) << 8;

        /* masked bits */
        mbits = ibits & viper_irq_enabled_mask;

        // Check for spurious interrupts or unexpected CPLD behaviour.
        if (ibits != mbits) {
		unsigned long j = jiffies;
		pc104_dev.ndiff++;
		if (j - pc104_dev.lastdiff > 300 * HZ) {
                        int td = ((long)j - (long)pc104_dev.lastdiff) / HZ;
			printk(KERN_INFO "PC104 IRQ bits=%#hx, mask=%#hx, #bad=%u, %d/sec, #ok=%u, %d/sec\n",
				ibits, viper_irq_enabled_mask,
                                pc104_dev.ndiff, pc104_dev.ndiff / td,
                                pc104_dev.nok, pc104_dev.nok / td );
			pc104_dev.ndiff = 0;
			pc104_dev.nok = 0;
			pc104_dev.lastdiff = j;
		}
        }
        else if (ibits) pc104_dev.nok++;

	return mbits;
}

static void __init viper_init_irq(void)
{
	int level;
	int isa_irq;

	pxa25x_init_irq();

	/* Peripheral IRQs */
        irq_set_irq_type(PXA_GPIO_TO_IRQ(VIPER_ETH_GPIO), IRQ_TYPE_EDGE_RISING);
        irq_set_irq_type(PXA_GPIO_TO_IRQ(VIPER_USB_GPIO), IRQ_TYPE_EDGE_FALLING);
	irq_set_irq_type(PXA_GPIO_TO_IRQ(VIPER_UARTA_GPIO), IRQ_TYPE_EDGE_RISING);
	irq_set_irq_type(PXA_GPIO_TO_IRQ(VIPER_UARTB_GPIO), IRQ_TYPE_EDGE_RISING);

	/* Setup ISA IRQs */

        /* probably not necessary, but we'll do it anyway... */
        VIPER_LO_IRQ_STATUS = 0;
        if (viper_version) VIPER_HI_IRQ_STATUS = 0;

#ifdef VIPER_IRQ_AUTOCLR_RETRIG
        viper_icr_set_bit(VIPER_ICR_AUTO_CLR);
#endif
	for (level = 0; level < ARRAY_SIZE(viper_isa_irqs); level++) {
		isa_irq = viper_bit_to_irq(level);
                printk(KERN_INFO "Map PC104 IRQ %d to IRQ %d\n",
                        viper_isa_irqs[level], isa_irq);
#ifdef VIPER_IRQ_AUTOCLR_RETRIG
		irq_set_chip_and_handler(isa_irq, &viper_pc104_irq_chip,
					 handle_simple_irq);
#else
                /*
                 * Use handle_edge_irq here. handle_simple_irq does not call
                 * chip ack function.
                 */
		irq_set_chip_and_handler(isa_irq, &viper_pc104_irq_chip,
					 handle_edge_irq);
#endif
		set_irq_flags(isa_irq, IRQF_VALID | IRQF_PROBE);
	}

        printk(KERN_INFO "Viper PC104 GPIO=%d, irq=%d, nirqs=%d, pc104_icr=%#x\n",
                VIPER_CPLD_GPIO,PXA_GPIO_TO_IRQ(VIPER_CPLD_GPIO),
                VIPER_NR_IRQS, pc104_icr);

#ifdef VIPER_IRQ_AUTOCLR_RETRIG
	irq_set_irq_type(PXA_GPIO_TO_IRQ(VIPER_CPLD_GPIO), IRQ_TYPE_EDGE_RISING);
#else
	irq_set_irq_type(PXA_GPIO_TO_IRQ(VIPER_CPLD_GPIO), IRQ_TYPE_EDGE_BOTH);
#endif
}

#ifdef TRY_THREADED_HANDLER
static irqreturn_t
viper_gpio_pc104_handler(int irq, void* devid)
{
        if (!viper_irq_enabled_mask) return IRQ_NONE;
        return IRQ_WAKE_THREAD;
}
#endif

static irqreturn_t
#ifdef TRY_THREADED_HANDLER
viper_gpio_pc104_thread_handler(int irq, void* devid)
#else
viper_gpio_pc104_handler(int irq, void* devid)
#endif
{
	unsigned short pending;
#ifdef TRY_THREADED_HANDLER
        unsigned long flags;
#endif

        struct pc104_device *dev = (struct pc104_device*) devid;

	pending = viper_pc104_irq_pending();
        if (!pending) {
		unsigned long j = jiffies;
		dev->npend0++;
		if (j - dev->lastpend > 300 * HZ) {
                        int td = ((long)j - (long)pc104_dev.lastpend) / HZ;
			printk(KERN_INFO "PC104 IRQ bits=0, mask=%#hx, #zero=%u, %d/sec, #ok=%u, %d/sec\n",
                                viper_irq_enabled_mask,
                                dev->npend0, dev->npend0 / td,
                                dev->nok0, dev->nok0 / td );
			dev->npend0 = 0;
                        dev->nok0 = 0;
			dev->lastpend = j;
		}
                pending = viper_irq_enabled_mask; 
	}
        else dev->nok0++;


#ifdef TRY_THREADED_HANDLER
        local_irq_save(flags);
#endif

	do {
		while (pending) {
                        int bit = __ffs(pending);
                        unsigned int uirq = viper_bit_to_irq(bit);
			generic_handle_irq(uirq);
                        pending &= ~viper_irq_to_bitmask(uirq);
		}
		pending = viper_pc104_irq_pending();
	} while (pending);

#ifdef TRY_THREADED_HANDLER
        local_irq_restore(flags);
#endif

#ifdef VIPER_IRQ_AUTOCLR_RETRIG
        viper_icr_set_bit(VIPER_ICR_RETRIG);
#endif
        return IRQ_HANDLED;
}

/*
 * Request interrupt (113) for the the VIPER_CPLD_GPIO (1).
 */
static int viper_pc104_init(void)
{
        int err;
#ifdef VIPER_IRQ_AUTOCLR_RETRIG

#ifdef TRY_THREADED_HANDLER
        err = request_threaded_irq(PXA_GPIO_TO_IRQ(VIPER_CPLD_GPIO),
                    viper_gpio_pc104_handler,
                    viper_gpio_pc104_thread_handler,
                    IRQF_TRIGGER_RISING,
                    "GPIO1-PC104", &pc104_dev);
#else
	err = request_irq(PXA_GPIO_TO_IRQ(VIPER_CPLD_GPIO),
                viper_gpio_pc104_handler,
                IRQF_TRIGGER_RISING,
                "GPIO1-PC104", &pc104_dev);
#endif
#else
#ifdef TRY_THREADED_HANDLER
        err = request_threaded_irq(PXA_GPIO_TO_IRQ(VIPER_CPLD_GPIO),
                    viper_gpio_pc104_handler,
                    viper_gpio_pc104_thread_handler,
                    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                    "GPIO1-PC104", &pc104_dev);
#else
	err = request_irq(PXA_GPIO_TO_IRQ(VIPER_CPLD_GPIO),
                viper_gpio_pc104_handler,
                IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                "GPIO1-PC104", &pc104_dev);
#endif
#endif
        if (err)
                printk(KERN_ERR "Error %d in request_irq of IRQ %d for PC104 CPLD on GPIO %d\n",
                        err, PXA_GPIO_TO_IRQ(VIPER_CPLD_GPIO), VIPER_CPLD_GPIO);
        else
                printk(KERN_INFO "Setup IRQ %d for PC104 CPLD on GPIO %d\n",
                        PXA_GPIO_TO_IRQ(VIPER_CPLD_GPIO), VIPER_CPLD_GPIO);
        return err;
}

/*
 * Request the interrupt for the VIPER_CPLD_GPIO late in the boot sequence,
 * after the postcore_initcall(pxa_gpio_init) in drivers/gpio/gpio-pxa.c has been done.
 */
device_initcall(viper_pc104_init);

/* Flat Panel */
static struct pxafb_mode_info fb_mode_info[] = {
	{
		.pixclock	= 157500,

		.xres		= 320,
		.yres		= 240,

		.bpp		= 16,

		.hsync_len	= 63,
		.left_margin	= 7,
		.right_margin	= 13,

		.vsync_len	= 20,
		.upper_margin	= 0,
		.lower_margin	= 0,

		.sync		= 0,
	},
};

static struct pxafb_mach_info fb_info = {
	.modes			= fb_mode_info,
	.num_modes		= 1,
	.lcd_conn		= LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
};

static int viper_backlight_init(struct device *dev)
{
	int ret;

	/* GPIO9 and 10 control FB backlight. Initialise to off */
	ret = gpio_request(VIPER_BCKLIGHT_EN_GPIO, "Backlight");
	if (ret)
		goto err_request_bckl;

	ret = gpio_request(VIPER_LCD_EN_GPIO, "LCD");
	if (ret)
		goto err_request_lcd;

	ret = gpio_direction_output(VIPER_BCKLIGHT_EN_GPIO, 0);
	if (ret)
		goto err_dir;

	ret = gpio_direction_output(VIPER_LCD_EN_GPIO, 0);
	if (ret)
		goto err_dir;

	return 0;

err_dir:
	gpio_free(VIPER_LCD_EN_GPIO);
err_request_lcd:
	gpio_free(VIPER_BCKLIGHT_EN_GPIO);
err_request_bckl:
	dev_err(dev, "Failed to setup LCD GPIOs\n");

	return ret;
}

static int viper_backlight_notify(struct device *dev, int brightness)
{
	gpio_set_value(VIPER_LCD_EN_GPIO, !!brightness);
	gpio_set_value(VIPER_BCKLIGHT_EN_GPIO, !!brightness);

	return brightness;
}

static void viper_backlight_exit(struct device *dev)
{
	gpio_free(VIPER_LCD_EN_GPIO);
	gpio_free(VIPER_BCKLIGHT_EN_GPIO);
}

static struct platform_pwm_backlight_data viper_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 100,
	.dft_brightness	= 100,
	.pwm_period_ns	= 1000000,
	.enable_gpio	= -1,
	.init		= viper_backlight_init,
	.notify		= viper_backlight_notify,
	.exit		= viper_backlight_exit,
};

static struct platform_device viper_backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.parent		= &pxa25x_device_pwm0.dev,
		.platform_data	= &viper_backlight_data,
	},
};

/* Ethernet */
static struct resource smc91x_resources[] = {
	[0] = {
		.name	= "smc91x-regs",
		.start  = VIPER_ETH_PHYS + 0x300,
		.end    = VIPER_ETH_PHYS + 0x30f,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = PXA_GPIO_TO_IRQ(VIPER_ETH_GPIO),
		.end    = PXA_GPIO_TO_IRQ(VIPER_ETH_GPIO),
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
	[2] = {
		.name	= "smc91x-data32",
		.start  = VIPER_ETH_DATA_PHYS,
		.end    = VIPER_ETH_DATA_PHYS + 3,
		.flags  = IORESOURCE_MEM,
	},
};

static struct smc91x_platdata viper_smc91x_info = {
	.flags	= SMC91X_USE_16BIT | SMC91X_NOWAIT,
	.leda	= RPC_LED_100_10,
	.ledb	= RPC_LED_TX_RX,
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
	.dev		= {
		.platform_data	= &viper_smc91x_info,
	},
};

/* i2c */
static struct i2c_gpio_platform_data gpio_i2c_bus_data = {
	.sda_pin = VIPER_RTC_I2C_SDA_GPIO,
	.scl_pin = VIPER_RTC_I2C_SCL_GPIO,
        /*
         * See kernel header: include/linux/i2c-gpio.h
         * These are the settings that work best on a Viper.
         * Running "i2cdetect -y 0" scans all addresses in about 0.4 sec,
         * reporting the expected response from 0x68=ds1338.
         */
        .sda_is_open_drain = 0,
        .scl_is_open_drain = 1,
        .scl_is_output_only = 0,
	.udelay  = 5,
	.timeout = 1,
};

static struct platform_device gpio_i2c_bus_device = {
	.name		= "i2c-gpio",
	.id		= 1, /* pxa2xx-i2c is bus 0, so start at 1 */
	.dev = {
		.platform_data = &gpio_i2c_bus_data,
	}
};

static struct i2c_board_info __initdata gpio_i2c_devices[] = {
	{
		I2C_BOARD_INFO("ds1338", 0x68),
	},
};

/*
 * Serial configuration:
 * You can either have the standard PXA ports driven by the PXA driver,
 * or all the ports (PXA + 16850) driven by the 8250 driver.
 * Choose your poison.
 */

static struct resource viper_serial_resources[] = {
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
	{
		.start	= VIPER_UARTA_PHYS,
		.end	= VIPER_UARTA_PHYS + 0xf,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VIPER_UARTB_PHYS,
		.end	= VIPER_UARTB_PHYS + 0xf,
		.flags	= IORESOURCE_MEM,
	},
};

static struct plat_serial8250_port serial_platform_data[] = {
#ifndef CONFIG_SERIAL_PXA
	/* Internal UARTs */
	{
		.membase	= (void *)&FFUART,
		.mapbase	= __PREG(FFUART),
		.irq		= IRQ_FFUART,
		.uartclk	= 921600 * 16,
		.regshift	= 2,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
	},
	{
		.membase	= (void *)&BTUART,
		.mapbase	= __PREG(BTUART),
		.irq		= IRQ_BTUART,
		.uartclk	= 921600 * 16,
		.regshift	= 2,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
	},
	{
		.membase	= (void *)&STUART,
		.mapbase	= __PREG(STUART),
		.irq		= IRQ_STUART,
		.uartclk	= 921600 * 16,
		.regshift	= 2,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
	},
	/* External UARTs */
	{
		.mapbase	= VIPER_UARTA_PHYS,
		.irq		= PXA_GPIO_TO_IRQ(VIPER_UARTA_GPIO),
		.irqflags	= IRQF_TRIGGER_RISING,
		.uartclk	= 1843200,
		.regshift	= 1,
		.iotype		= UPIO_MEM,
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP |
				  UPF_SKIP_TEST,
	},
	{
		.mapbase	= VIPER_UARTB_PHYS,
		.irq		= PXA_GPIO_TO_IRQ(VIPER_UARTB_GPIO),
		.irqflags	= IRQF_TRIGGER_RISING,
		.uartclk	= 1843200,
		.regshift	= 1,
		.iotype		= UPIO_MEM,
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP |
				  UPF_SKIP_TEST,
	},
#endif
	{ },
};

static struct platform_device serial_device = {
	.name			= "serial8250",
	.id			= 0,
	.dev			= {
		.platform_data	= serial_platform_data,
	},
	.num_resources		= ARRAY_SIZE(viper_serial_resources),
	.resource		= viper_serial_resources,
};

static struct resource isp116x_resources[] = {
	[0] = { /* DATA */
		.start  = VIPER_USB_PHYS + 0,
		.end    = VIPER_USB_PHYS + 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = { /* ADDR */
		.start  = VIPER_USB_PHYS + 2,
		.end    = VIPER_USB_PHYS + 3,
		.flags  = IORESOURCE_MEM,
	},
	[2] = {
		.start  = PXA_GPIO_TO_IRQ(VIPER_USB_GPIO),
		.end    = PXA_GPIO_TO_IRQ(VIPER_USB_GPIO),
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

/* (DataBusWidth16|AnalogOCEnable|DREQOutputPolarity|DownstreamPort15KRSel ) */
static struct isp116x_platform_data isp116x_platform_data = {
	/* Enable internal resistors on downstream ports */
	.sel15Kres		= 1,
	/* On-chip overcurrent protection */
	.oc_enable		= 1,
	/* INT output polarity */
	.int_act_high		= 1,
	/* INT edge or level triggered */
	.int_edge_triggered	= 0,

	/* WAKEUP pin connected - NOT SUPPORTED  */
	/* .remote_wakeup_connected = 0, */
	/* Wakeup by devices on usb bus enabled */
	.remote_wakeup_enable	= 0,
        /* Instead of using a delay function, the timing is adjusted in MSC1.
         * PLATFORM_DELAY is not defined in drivers/usb/host/isp116x-hcd.c */
};

static struct platform_device isp116x_device = {
	.name			= "isp116x-hcd",
	.id			= -1,
	.num_resources  	= ARRAY_SIZE(isp116x_resources),
	.resource       	= isp116x_resources,
	.dev			= {
		.platform_data	= &isp116x_platform_data,
	},

};

/* MTD */
static struct resource mtd_resources[] = {
	[0] = {	/* RedBoot config + filesystem flash */
		.start	= VIPER_FLASH_PHYS,
		.end	= VIPER_FLASH_PHYS + SZ_32M - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {	/* Boot flash */
		.start	= VIPER_BOOT_PHYS,
		.end	= VIPER_BOOT_PHYS + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	[2] = { /*
		 * SRAM size is actually 256KB, 8bits, with a sparse mapping
		 * (each byte is on a 16bit boundary).
		 */
		.start	= _VIPER_SRAM_BASE,
		.end	= _VIPER_SRAM_BASE + SZ_512K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct mtd_partition viper_boot_flash_partition = {
	.name		= "RedBoot",
	.size		= SZ_1M,
	.offset		= 0,
	.mask_flags	= MTD_WRITEABLE,	/* force R/O */
};

static struct physmap_flash_data viper_flash_data[] = {
	[0] = {
		.width		= 2,
		.parts		= NULL,
		.nr_parts	= 0,
	},
	[1] = {
		.width		= 2,
		.parts		= &viper_boot_flash_partition,
		.nr_parts	= 1,
	},
};

static struct platform_device viper_mtd_devices[] = {
	[0] = {
		.name		= "physmap-flash",
		.id		= 0,
		.dev		= {
			.platform_data	= &viper_flash_data[0],
		},
		.resource	= &mtd_resources[0],
		.num_resources	= 1,
	},
	[1] = {
		.name		= "physmap-flash",
		.id		= 1,
		.dev		= {
			.platform_data	= &viper_flash_data[1],
		},
		.resource	= &mtd_resources[1],
		.num_resources	= 1,
	},
};

static struct platform_device *viper_devs[] __initdata = {
	&smc91x_device,
	&serial_device,
	&isp116x_device,
	&viper_mtd_devices[0],
	&viper_mtd_devices[1],
	/*  &viper_backlight_device, */
	&viper_pcmcia_device,
	&gpio_i2c_bus_device,
};

static mfp_cfg_t viper_pin_config[] __initdata = {
	/* Chip selects */
	GPIO15_nCS_1,
	GPIO78_nCS_2,
	GPIO79_nCS_3,
	GPIO80_nCS_4,
	GPIO33_nCS_5,

	/* AC97 */
	GPIO28_AC97_BITCLK,
	GPIO29_AC97_SDATA_IN_0,
	GPIO30_AC97_SDATA_OUT,
	GPIO31_AC97_SYNC,

	/* FP Backlight */
	GPIO9_GPIO, 				/* VIPER_BCKLIGHT_EN_GPIO */
	GPIO10_GPIO,				/* VIPER_LCD_EN_GPIO */
	GPIO16_PWM0_OUT,

	/* Ethernet PHY Ready */
	GPIO18_RDY,

	/* Serial shutdown */
	GPIO12_GPIO | MFP_LPM_DRIVE_HIGH,	/* VIPER_UART_SHDN_GPIO */

	/* Compact-Flash / PC104 */
	GPIO48_nPOE,
	GPIO49_nPWE,
	GPIO50_nPIOR,
	GPIO51_nPIOW,
	GPIO52_nPCE_1,
	GPIO53_nPCE_2,
	GPIO54_nPSKTSEL,
	GPIO55_nPREG,
	GPIO56_nPWAIT,
	GPIO57_nIOIS16,
	GPIO8_GPIO,				/* VIPER_CF_RDY_GPIO */
	GPIO32_GPIO,				/* VIPER_CF_CD_GPIO */
	MFP_CFG_OUT(GPIO82, AF0, DRIVE_LOW),    /* VIPER_CF_POWER_GPIO */

	/* Vcc regulator control */
	GPIO6_PSU_DATA,				/* VIPER_PSU_DATA_GPIO */
	GPIO11_PSU_CLK,				/* VIPER_PSU_CLK_GPIO */
	GPIO19_PSU_nCS_LD,			/* VIPER_PSU_nCS_LD_GPIO */

        /* outputs on PL9 */
        GPIO20_OUT,                             /* also used for UPS */
        GPIO21_OUT,
        GPIO22_OUT,
        GPIO23_OUT,
        GPIO24_OUT,
        GPIO25_OUT,
        GPIO26_OUT,
        GPIO27_OUT,

	/* i2c busses */
#ifdef HAS_TPM
	GPIO26_GPIO,				/* VIPER_TPM_I2C_SDA_GPIO */
	GPIO27_GPIO,				/* VIPER_TPM_I2C_SCL_GPIO */
#endif

	GPIO83_GPIO,				/* VIPER_RTC_I2C_SDA_GPIO */
	GPIO84_GPIO,				/* VIPER_RTC_I2C_SCL_GPIO */

	/* PC/104 Interrupt */
	GPIO1_GPIO | WAKEUP_ON_EDGE_RISE, 	/* VIPER_CPLD_GPIO */
};

static unsigned long viper_tpm;

static int __init viper_tpm_setup(char *str)
{
	return strict_strtoul(str, 10, &viper_tpm) >= 0;
}

__setup("tpm=", viper_tpm_setup);

static void __init viper_tpm_init(void)
{
	struct platform_device *tpm_device;
	struct i2c_gpio_platform_data i2c_tpm_data = {
		.sda_pin = VIPER_TPM_I2C_SDA_GPIO,
		.scl_pin = VIPER_TPM_I2C_SCL_GPIO,
		.udelay  = 10,
		.timeout = HZ,
	};
	char *errstr;

	/* Allocate TPM i2c bus if requested */
	if (!viper_tpm)
		return;

	tpm_device = platform_device_alloc("i2c-gpio", 2);
	if (tpm_device) {
		if (!platform_device_add_data(tpm_device,
					      &i2c_tpm_data,
					      sizeof(i2c_tpm_data))) {
			if (platform_device_add(tpm_device)) {
				errstr = "register TPM i2c bus";
				goto error_free_tpm;
			}
		} else {
			errstr = "allocate TPM i2c bus data";
			goto error_free_tpm;
		}
	} else {
		errstr = "allocate TPM i2c device";
		goto error_tpm;
	}

	return;

error_free_tpm:
	kfree(tpm_device);
error_tpm:
	pr_err("viper: Couldn't %s, giving up\n", errstr);
}

static void __init viper_init_vcore_gpios(void)
{
	if (gpio_request(VIPER_PSU_DATA_GPIO, "PSU data"))
		goto err_request_data;

	if (gpio_request(VIPER_PSU_CLK_GPIO, "PSU clock"))
		goto err_request_clk;

	if (gpio_request(VIPER_PSU_nCS_LD_GPIO, "PSU cs"))
		goto err_request_cs;

	if (gpio_direction_output(VIPER_PSU_DATA_GPIO, 0) ||
	    gpio_direction_output(VIPER_PSU_CLK_GPIO, 0) ||
	    gpio_direction_output(VIPER_PSU_nCS_LD_GPIO, 0))
		goto err_dir;

	/* c/should assume redboot set the correct level ??? */
	viper_set_core_cpu_voltage(get_clk_frequency_khz(0), 1);

	return;

err_dir:
	gpio_free(VIPER_PSU_nCS_LD_GPIO);
err_request_cs:
	gpio_free(VIPER_PSU_CLK_GPIO);
err_request_clk:
	gpio_free(VIPER_PSU_DATA_GPIO);
err_request_data:
	pr_err("viper: Failed to setup vcore control GPIOs\n");
}

static void __init viper_init_serial_gpio(void)
{
	if (gpio_request(VIPER_UART_SHDN_GPIO, "UARTs shutdown"))
		goto err_request;

	if (gpio_direction_output(VIPER_UART_SHDN_GPIO, 0))
		goto err_dir;

	return;

err_dir:
	gpio_free(VIPER_UART_SHDN_GPIO);
err_request:
	pr_err("viper: Failed to setup UART shutdown GPIO\n");
}

#ifdef CONFIG_CPU_FREQ
static int viper_cpufreq_notifier(struct notifier_block *nb,
				  unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;

	/* TODO: Adjust timings??? */

	switch (val) {
	case CPUFREQ_PRECHANGE:
		if (freq->old < freq->new) {
			/* we are getting faster so raise the voltage
			 * before we change freq */
			viper_set_core_cpu_voltage(freq->new, 0);
		}
		break;
	case CPUFREQ_POSTCHANGE:
		if (freq->old > freq->new) {
			/* we are slowing down so drop the power
			 * after we change freq */
			viper_set_core_cpu_voltage(freq->new, 0);
		}
		break;
	default:
		/* ignore */
		break;
	}

	return 0;
}

static struct notifier_block viper_cpufreq_notifier_block = {
	.notifier_call  = viper_cpufreq_notifier
};

static void __init viper_init_cpufreq(void)
{
	if (cpufreq_register_notifier(&viper_cpufreq_notifier_block,
				      CPUFREQ_TRANSITION_NOTIFIER))
		pr_err("viper: Failed to setup cpufreq notifier\n");
}
#else
static inline void viper_init_cpufreq(void) {}
#endif

static void viper_power_off(void)
{
	pr_notice("Shutting off UPS\n");
	gpio_set_value(VIPER_UPS_GPIO, 1);
	/* Spin to death... */
	while (1);
}

static void __init viper_init(void)
{
        uint32_t u32val, u32val2;

	pm_power_off = viper_power_off;

	pxa2xx_mfp_config(ARRAY_AND_SIZE(viper_pin_config));

	pxa_set_ffuart_info(NULL);
	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);

	/* Wake-up serial console */
	viper_init_serial_gpio();

#ifdef DO_FB
	pxa_set_fb_info(NULL, &fb_info);
#endif

	viper_version = viper_hw_version();

	if (viper_version) {
		pr_info("viper: hardware v%di%d detected. "
			"CPLD revision %d.\n",
			VIPER_BOARD_VERSION(viper_version),
			VIPER_BOARD_ISSUE(viper_version),
			VIPER_CPLD_REVISION(viper_version));
		system_rev = (VIPER_BOARD_VERSION(viper_version) << 8) |
			     (VIPER_BOARD_ISSUE(viper_version) << 4) |
			     VIPER_CPLD_REVISION(viper_version);
	} else {
		pr_info("viper: No version register.\n");
	}

        if (viper_version) {
            pxa_set_i2c_info(NULL);     /* PXA I2C */
        }
        else {
            gpio_i2c_bus_device.id = 0;
            /* v1 hardware cannot use the datacs line */
            smc91x_device.num_resources--;
        }

	platform_add_devices(viper_devs, ARRAY_SIZE(viper_devs));

	viper_init_vcore_gpios();
	viper_init_cpufreq();

	register_syscore_ops(&viper_cpu_syscore_ops);

        /* Adjust timing to isp116x USB interface, which is on chip select 3,
         * by tweaking high order bits in MSC1, in order to avoid using a
         * busy delay in the driver. See discussion of USE_PLATFORM_DELAY
         * and USE_NDELAY in isp116x.h and isp116x-hcd.c (drivers/usb/host).
         *
         * See "Intel PXA255 Processor Developer's Manual" for info on the MSCn registers.
         *
         * Default values (apparently set in RedBoot) were (as shown by pxaregs program):
         * MSC1=0x44fc123c
         * MSC1_RT3                          4  nCS[3] ROM Type
         * MSC1_RBW3                         1  nCS[3] ROM Bus Width (1=16bit)
         * MSC1_RDF3                        15  nCS[3] ROM Delay First Access
         * MSC1_RDN3                         4  nCS[3] ROM Delay Next Access
         * MSC1_RRR3                         4  nCS[3] ROM/SRAM Recovery Time
         * MSC1_RBUFF3                       0  nCS[3] Return Buffer Behavior (1=streaming)
         *
         * Initial test: increase to maximums: MSC1_RDN3 to 15, and MSC1_RRR3 to 7
         */
        u32val = *(unsigned int*) MSC1;
        *(unsigned int*) MSC1 = (u32val & 0x0000ffff) | 0x7ffc0000;
        u32val2 = *(unsigned int*) MSC1;
        printk(KERN_INFO "*MSC1 changed from %#x to %#x, CS3 RDFx=%#x RDNx=%#x, RRRx=%#x\n",
                u32val, u32val2, (u32val2 >> 20) & 0xf, (u32val2 >> 24) & 0xf,
                (u32val2 >> 28) & 0x7);

	i2c_register_board_info(gpio_i2c_bus_device.id, ARRAY_AND_SIZE(gpio_i2c_devices));

	viper_tpm_init();
	pxa_set_ac97_info(NULL);
}

static struct map_desc viper_io_desc[] __initdata = {
	{
		.virtual = VIPER_CPLD_BASE,
		.pfn     = __phys_to_pfn(VIPER_CPLD_PHYS),
		.length  = 0x00300000,
		.type    = MT_DEVICE,
	},
	{
		.virtual = VIPER_PC104IO_BASE,
		.pfn     = __phys_to_pfn(0x30000000),
		.length  = 0x00800000,
		.type    = MT_DEVICE,
	},
};

static void __init viper_map_io(void)
{
	pxa25x_map_io();

	iotable_init(viper_io_desc, ARRAY_SIZE(viper_io_desc));

	PCFR |= PCFR_OPDE;
}

MACHINE_START(VIPER, "Arcom/Eurotech VIPER SBC")
	/* Maintainer: Marc Zyngier <maz@misterjones.org> */
	.map_io		= viper_map_io,
	.nr_irqs	= VIPER_NR_IRQS,
	.init_irq	= viper_init_irq,
	.handle_irq	= pxa25x_handle_irq,
	.init_time	= pxa_timer_init,
	.init_machine	= viper_init,
	.restart	= pxa_restart,
MACHINE_END

