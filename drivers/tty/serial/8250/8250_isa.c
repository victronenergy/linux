// SPDX-License-Identifier: GPL-2.0+
/*
 *  Universal/legacy driver for 8250/16550-type serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>

#include <asm/irq.h>

#include "8250.h"

/*
 * Configuration:
 *   share_irqs - whether we pass IRQF_SHARED to request_irq().  This option
 *                is unsafe when used on edge-triggered interrupts.
 */
static unsigned int share_irqs = SERIAL8250_SHARE_IRQS;

#include <asm/serial.h>
/*
 * SERIAL_PORT_DFNS tells us about built-in ports that have no
 * standard enumeration mechanism.   Platforms that can find all
 * serial ports via mechanisms like ACPI or PCI need not supply it.
 */
#ifndef SERIAL_PORT_DFNS
#define SERIAL_PORT_DFNS
#endif

static const struct old_serial_port old_serial_port[] = {
	SERIAL_PORT_DFNS /* defined in asm/serial.h */
};

#define UART_NR	CONFIG_SERIAL_8250_NR_UARTS

static void (*serial8250_isa_config)(int port, struct uart_port *up,
	u32 *capabilities);

void serial8250_set_isa_configurator(
	void (*v)(int port, struct uart_port *up, u32 *capabilities))
{
	serial8250_isa_config = v;
}
EXPORT_SYMBOL(serial8250_set_isa_configurator);

void serial8250_isa_fixup(int port, struct uart_8250_port *up)
{
	if (serial8250_isa_config)
		serial8250_isa_config(port, &up->port, &up->capabilities);
}

void __init serial8250_isa_init_ports(unsigned int nr_uarts)
{
	int irqflag = 0;
	int i;

	if (share_irqs)
		irqflag = IRQF_SHARED;

	for (i = 0; i < ARRAY_SIZE(old_serial_port) && i < nr_uarts; i++) {
		struct uart_8250_port *up = serial8250_get_port(i);
		struct uart_port *port = &up->port;

		port->iobase   = old_serial_port[i].port;
		port->irq      = irq_canonicalize(old_serial_port[i].irq);
		port->irqflags = 0;
		port->uartclk  = old_serial_port[i].baud_base * 16;
		port->flags    = old_serial_port[i].flags;
		port->hub6     = 0;
		port->membase  = old_serial_port[i].iomem_base;
		port->iotype   = old_serial_port[i].io_type;
		port->regshift = old_serial_port[i].iomem_reg_shift;

		port->irqflags |= irqflag;
		serial8250_isa_fixup(i, up);
	}
}

static void __init
serial8250_isa_register_ports(struct device *dev, unsigned int nr_uarts)
{
	int i;

	for (i = 0; i < nr_uarts; i++) {
		struct uart_8250_port *up = serial8250_get_port(i);

		if (up->port.type == PORT_8250_CIR)
			continue;

		if (up->port.dev)
			continue;

		up->port.dev = dev;

		serial8250_add_one_port(up);
	}
}

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int serial8250_isa_probe(struct platform_device *dev)
{
	struct plat_serial8250_port *p = dev_get_platdata(&dev->dev);
	struct uart_8250_port uart;
	int ret, i, irqflag = 0;

	memset(&uart, 0, sizeof(uart));

	if (share_irqs)
		irqflag = IRQF_SHARED;

	for (i = 0; p && p->flags != 0; p++, i++) {
		uart.port.iobase	= p->iobase;
		uart.port.membase	= p->membase;
		uart.port.irq		= p->irq;
		uart.port.irqflags	= p->irqflags;
		uart.port.uartclk	= p->uartclk;
		uart.port.regshift	= p->regshift;
		uart.port.iotype	= p->iotype;
		uart.port.flags		= p->flags;
		uart.port.mapbase	= p->mapbase;
		uart.port.hub6		= p->hub6;
		uart.port.has_sysrq	= p->has_sysrq;
		uart.port.private_data	= p->private_data;
		uart.port.type		= p->type;
		uart.port.serial_in	= p->serial_in;
		uart.port.serial_out	= p->serial_out;
		uart.port.handle_irq	= p->handle_irq;
		uart.port.handle_break	= p->handle_break;
		uart.port.set_termios	= p->set_termios;
		uart.port.set_ldisc	= p->set_ldisc;
		uart.port.get_mctrl	= p->get_mctrl;
		uart.port.pm		= p->pm;
		uart.port.dev		= &dev->dev;
		uart.port.irqflags	|= irqflag;
		ret = serial8250_register_8250_port(&uart);
		if (ret < 0) {
			dev_err(&dev->dev, "unable to register port at index %d "
				"(IO%lx MEM%llx IRQ%d): %d\n", i,
				p->iobase, (unsigned long long)p->mapbase,
				p->irq, ret);
		}
	}
	return 0;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int serial8250_isa_remove(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = serial8250_get_port(i);

		if (up->port.dev == &dev->dev)
			serial8250_unregister_port(i);
	}
	return 0;
}

static int serial8250_isa_suspend(struct platform_device *dev,
				  pm_message_t state)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = serial8250_get_port(i);

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			serial8250_suspend_port(i);
	}

	return 0;
}

static int serial8250_isa_resume(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = serial8250_get_port(i);

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			serial8250_resume_port(i);
	}

	return 0;
}

static struct platform_driver serial8250_isa_driver = {
	.probe		= serial8250_isa_probe,
	.remove		= serial8250_isa_remove,
	.suspend	= serial8250_isa_suspend,
	.resume		= serial8250_isa_resume,
	.driver		= {
		.name	= "serial8250",
	},
};

/*
 * This "device" covers _all_ ISA 8250-compatible serial devices listed
 * in the table in include/asm/serial.h
 */
static struct platform_device *serial8250_isa_devs;

void serial8250_isa_unregister_port(struct uart_8250_port *uart)
{
	if (!serial8250_isa_devs)
		return;

	uart->port.flags &= ~UPF_BOOT_AUTOCONF;
	uart->port.type = PORT_UNKNOWN;
	uart->port.dev = &serial8250_isa_devs->dev;
	uart->capabilities = 0;
	serial8250_add_one_port(uart);
}

int __init serial8250_isa_init(unsigned int nr_uarts)
{
	int ret;

	pr_info("Serial: 8250/16550 driver, ISA IRQ sharing %sabled\n",
		share_irqs ? "en" : "dis");

	serial8250_isa_devs = platform_device_alloc("serial8250",
						    PLAT8250_DEV_LEGACY);
	if (!serial8250_isa_devs)
		return -ENOMEM;

	ret = platform_device_add(serial8250_isa_devs);
	if (ret)
		goto put_dev;

	serial8250_isa_register_ports(&serial8250_isa_devs->dev, nr_uarts);

	ret = platform_driver_register(&serial8250_isa_driver);
	if (ret == 0)
		goto out;

	platform_device_del(serial8250_isa_devs);
put_dev:
	platform_device_put(serial8250_isa_devs);
out:
	return ret;
}

void __exit serial8250_isa_exit(void)
{
	struct platform_device *isa_dev = serial8250_isa_devs;

	/*
	 * This tells serial8250_unregister_port() not to re-register
	 * the ports (thereby making serial8250_isa_driver permanently
	 * in use.)
	 */
	serial8250_isa_devs = NULL;

	platform_driver_unregister(&serial8250_isa_driver);
	platform_device_unregister(isa_dev);
}

module_param_hw(share_irqs, uint, other, 0644);
MODULE_PARM_DESC(share_irqs, "Share IRQs with other non-8250/16x50 devices (unsafe)");
