/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2008 Schweitzer Engineering Laboratories, Pullman, Washington
 *
 * Main header file for the driver
 */
#ifndef _SELSERIAL_H_INCLUDED
#define _SELSERIAL_H_INCLUDED

#include <linux/init.h>
#include <linux/list.h>
#include <linux/serial_core.h>
#include <linux/types.h>

#include "selserial_reg.h"
#include "linux/selserial_ioctl.h"

/* Maximum number of ports this driver is ever allowed service */
#define SELSERIAL_NR                   64

/* TODO: Set this in include/uapi/linux/serial_core.h */
#define PORT_SELSERIAL                 123

/* Default value for level interrupt triggers. Picked it out of the blue
 * to give a reasonable response time. */
#define SELSERIAL_DEFAULT_COUNT_LEVEL  32

/* Default value for the end-of-* timers */
#define SELSERIAL_DEFAULT_EOXT_TIMER   40

/* Port type enumeration */
enum port_type
{
	/* Port not opened yet */
	SELSERIAL_PORT_TYPE_UNOPENED = 0,
	/* TTY port */
	SELSERIAL_PORT_TYPE_TTY,
	/* RAW port */
	SELSERIAL_PORT_TYPE_RAW,
	/* MirroredBits Port */
	SELSERIAL_PORT_TYPE_MB
};

/* Macro to increment the head/tail pointers of the buffers */
#define CIRC_BUF_INC_CNTR( cntr, inc ) \
	do { \
		(cntr) = ((cntr) + (inc)) & (SELSERIAL_CIRC_BUF_SIZE - 1); \
	} while(0)

/* Default End-of-recieve timer (EORT) is 4 characters of (bits size) +
 * 12 bits for overhead from 8250 data sheet */
#define GET_DEFAULT_EORT(bits) (4 * (bits) + 12)


/* selserial_uart_port data structure */
struct selserial_uart_port
{
	/* base uart_port structure */
	struct uart_port port;
	/* pci device this uart port is controlled by -- same pointer as in
	 * the private data, but I need it here or I can't get to the private
	 * data */
	struct pci_dev *pdev;
	/* irq list head structure */
	struct list_head list;
	/* interrupt settings */
	struct selserial_fifo_int_settings fifo_settings;
	/* enumeration to determine which type of port is opened */
	enum port_type port_type;
	/* mbok status byte */
	u8 mbok;
	/* local copy of the MCR */
	u8 mcr;
	/* local copy of the GCR */
	u8 gcr;
	/* local copy of the TCR */
	u8 tcr;
	/* local copy of the LCR */
	u32 lcr;
	/* local copy of the IER */
	u16 ier;
	/* local copy of the MBR */
	u16 mbr;
	/* timed character settings */
	u16 timedchar;
	/* flag indicating if we are in auto485 mode */
	u8 auto485 : 1;
	/* flag indicating if we had an eott/eort event */
	u8 eoxt_exp : 1;
	/* flag indicating if we are running half duplex flow control mode */
	u8 half_flow : 1;
	/* number of expirations of eott */
	u16 eott_exp_cnt;
	/* number of expirations of eort */
	u16 eort_exp_cnt;
};

/* struct to hold our IRQ list */
struct irq_info
{
	/* lock to protect the IRQ list */
	spinlock_t lock;
	/* head of the list */
	struct list_head *head;
};

/* private data for each instance of the pci device */
struct selserial_private
{
	/* device structure for the pci device -- same pointer as in the
	 * selserial_uart_port structure, but I need it here so I don't
	 * have to do something unintuitive like sport.port[0].pdev */
	struct pci_dev *dev;
	/* pointer to the beginning of the remapped iomem space */
	void __iomem *remapped_bar;
	/* Number of ports provided by this device */
	unsigned int nr;
	/* active port irq list */
	struct irq_info irq_list;
	/* Port structures for each of the ports. The port[0] allows the
	 * array of ports to be part of this structure. If we just said
	 * *port, a separate block of memory would have to be allocated
	 * (2 to keep track of now per device). This way a single big
	 * block can be allocated (sizeof(struct selserial_private) +
	 * sizeof(struct selserial_uart_port)*num_ports) and treat it like a
	 * single structure. */
	struct selserial_uart_port  ports[0];
};

/* TTY port interface routines */
int selserial_tty_init(void) __init;
void selserial_tty_exit(void);
int selserial_tty_initport(struct selserial_private *priv);
void selserial_tty_cleanupport(struct selserial_private *priv);
void selserial_tty_handle_port(struct selserial_uart_port *sport, u16 isr);

/* Special functions of the SELserial UARTS to manipulate rs485 mode,
 * Port Power, and Loopback */
int selserial_special_init(struct device *device);
void selserial_special_cleanup(struct device *device);
void __selserial_special_power_store(struct selserial_uart_port *sport, char mode);
void __selserial_special_485_store(struct selserial_uart_port *sport, char mode);

/* Interrupt handling functionality */
int selserial_register_irq_list(int irq, struct irq_info *i, struct list_head *list);
void selserial_unregister_irq_list(int irq, struct irq_info *i, struct list_head *list);

#endif
