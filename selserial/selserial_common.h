/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2008 Schweitzer Engineering Laboratories, Pullman, Washington
 *
 * Common routines used by the serial port driver
 */
#ifndef _SELSERIAL_COMMON_H_INCLUDED
#define _SELSERIAL_COMMON_H_INCLUDED

#include <linux/io.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/termios.h>

#include "selserial.h"

long selserial_common_ioctl(struct selserial_uart_port *sport,
			    unsigned int cmd,
			    unsigned long arg);

void selserial_common_set_term_settings(struct selserial_uart_port *sport,
					struct ktermios *new,
					struct ktermios *old);

unsigned int selserial_common_get_mctrl(struct selserial_uart_port *sport);

void selserial_common_set_mctrl(struct selserial_uart_port *sport, unsigned int mctrl);

void selserial_common_setup_fifo_interrupt(struct selserial_uart_port *sport,
					   u16 rxlvl,
					   u16 txlvl,
					   u16 eort,
					   u16 eott);

int selserial_common_register_and_clear_irq(struct selserial_uart_port *sport);

void selserial_common_disable_and_unregister_irq(struct selserial_uart_port *sport);

void selserial_common_txfifo_write(struct selserial_uart_port *sport, u8 data);

int selserial_common_reserve_type(struct selserial_uart_port *sport, int type);

void selserial_common_unreserve_type(struct selserial_uart_port *sport, int type);

#endif
