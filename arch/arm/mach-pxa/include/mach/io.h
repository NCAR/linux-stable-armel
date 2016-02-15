/*
 * arch/arm/mach-pxa/include/mach/io.h
 *
 * Copied from asm/arch/sa1100/io.h
 */
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#if defined(CONFIG_IOPORT_REGION_SIZE)
#define IO_SPACE_LIMIT (CONFIG_IOPORT_REGION_SIZE)
#else
#define IO_SPACE_LIMIT 0xffffffff
#endif

#if defined(CONFIG_IOPORT_VIRT_BASE)
#define __io(a)         __typesafe_io(CONFIG_IOPORT_VIRT_BASE + ((a) & IO_SPACE_LIMIT))
#else
/*
 * We don't actually have real ISA nor PCI buses, but there is so many
 * drivers out there that might just work if we fake them...
 */
#define __io(a)		__typesafe_io(a)

#endif

#endif
