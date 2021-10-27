// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2008 Schweitzer Engineering Laboratories, Pullman, Washington
 *
 * TTY functionality for the port
 */

#include <linux/serial_core.h>
#include <linux/tty_flip.h>
#include <linux/version.h>

#include "selserial.h"
#include "selserial_common.h"

static struct uart_driver selserial_uart_driver =
{
   .owner         = THIS_MODULE,
   .driver_name   = "selserial",
   .dev_name      = "ttySEL", /// for when someone installs udev
   .major         = 11,       /// same as MUX, but MUX is only for parisc
   .minor         = 0,
   .nr            = SELSERIAL_NR,
   .cons          = NULL,
};

/**
 * selserial_tx_empty() - Query the driver to see if the tx FIFO is empty
 *
 * @port: Port to check transmit FIFO level.
 *
 * Return: 0 if the FIFO is not empty, TIOCSER_TEMT if it is empty.
 */
static unsigned int selserial_tx_empty(struct uart_port *port)
{
	unsigned long flags = 0;
	unsigned int ret = TIOCSER_TEMT;
	u16 tx_fifo_count = 0;

	/* Lock and disable interrupts so we don't misreport */
	spin_lock_irqsave(&port->lock, flags);

	/* See if there are any bytes in the transmit FIFO */
	tx_fifo_count = ioread16(port->membase + SELSERIAL_TX_COUNT);
	if (tx_fifo_count) {
		ret = 0;
	}

	spin_unlock_irqrestore(&port->lock, flags);

	return ret;
}

/**
 * selserial_set_mctrl() - Set the modem control lines
 *
 * @port: Port to set the modem control lines on
 * @mctrl: Bit field describing modem control lines to set
 */
static void selserial_set_mctrl(struct uart_port * port, unsigned int mctrl)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;
	selserial_common_set_mctrl(sport, mctrl);
}

/**
 * selserial_get_mctrl() - Get the state of the modem control pins
 *
 * @port: Port to retrieve pin state from
 *
 * Return: Bit field containing the state of the pins
 */
static unsigned int selserial_get_mctrl(struct uart_port *port)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;
	return selserial_common_get_mctrl(sport);
}

/**
 * selserial_stop_tx() - Stop transmitting characters as soon as possible
 *
 * @port: Port to stop transmitting on
 *
 * This gets called by the upper layer, generally on CTS deasserting or
 * reception of an XOFF when using software flow control.
 */
static void selserial_stop_tx(struct uart_port *port)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;

	/* Disable the transmit interrupts */
	sport->ier &= ~(SELSERIAL_IER_EOTT | SELSERIAL_IER_TX_LEVEL);

	/* Disable the transmitter */
	sport->lcr &= ~(SELSERIAL_LCR_TX_EN);

	/* Write them out */
	iowrite16(sport->ier, sport->port.membase + SELSERIAL_IER);
	iowrite32(sport->lcr, sport->port.membase + SELSERIAL_LCR);
}

/**
 * selserial_start_tx() - Start the transmitter
 *
 * @port: Port to start transmission on
 */
static void selserial_start_tx(struct uart_port *port)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;

	/* Enable the transmitter interrupts */
	sport->ier |= SELSERIAL_IER_TX_LEVEL;
	if (sport->fifo_settings.eott) {
		sport->ier |= SELSERIAL_IER_EOTT;
	}

	/* Enable the transmitter */
	sport->lcr |= (SELSERIAL_LCR_TX_EN);

	/* Write them out */
	iowrite16(sport->ier, sport->port.membase + SELSERIAL_IER);
	iowrite32(sport->lcr, sport->port.membase + SELSERIAL_LCR);
}

/**
 * selserial_stop_rx() - Stop all reception
 *
 * @port: Port to stop reception on
 *
 * This is generally called when the port is being shutdown.
 */
static void selserial_stop_rx(struct uart_port *port)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;

	/* Disable receive interrupt */
	sport->ier &= ~(SELSERIAL_IER_EORT | SELSERIAL_IER_RX_LEVEL);

	/* Disable receiver */
	sport->lcr &= ~(SELSERIAL_LCR_RX_EN);

	/* Write them out */
	iowrite16(sport->ier, sport->port.membase + SELSERIAL_IER);
	iowrite32(sport->lcr, sport->port.membase + SELSERIAL_LCR);
}

/**
 * selserial_enable_ms() - Enable modem status interrupts
 *
 * @port: Port to enable modem status interrupts on
 */
static void selserial_enable_ms(struct uart_port *port)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;

	sport->ier |= (SELSERIAL_IER_DCTS | SELSERIAL_IER_DDSR |
		       SELSERIAL_IER_DRING | SELSERIAL_IER_DDCD);

	/* Write it out */
	iowrite16(sport->ier, sport->port.membase + SELSERIAL_IER);
}

/**
 * selserial_break_ctl() - Set or clear a break condition on the line
 *
 * @port: Port to set or clear the break condition on
 * @ctl: If set, then start a break. If clear, then clear the break.
 */
static void selserial_break_ctl(struct uart_port *port, int ctl)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;
	unsigned long flags = 0;

	/* Save our IRQ state while we are setting the break condition */
	spin_lock_irqsave(&sport->port.lock, flags);

	if (ctl) {
		sport->mcr |= SELSERIAL_MCR_BRK;
	} else {
		sport->mcr &= ~(SELSERIAL_MCR_BRK);
	}

	iowrite8(sport->mcr, sport->port.membase + SELSERIAL_MCR);

	spin_unlock_irqrestore( &sport->port.lock, flags );
}

/**
 * selserial_startup() - Setup a port
 *
 * @port: The port being opened
 *
 * Return: 0 on success, non-zero on failure
 */
static int selserial_startup(struct uart_port *port)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;
	int rc = 0;

	/* Try to reserve the port type */
	rc = selserial_common_reserve_type(sport, SELSERIAL_PORT_TYPE_TTY);
	if (rc) {
		goto out;
	}

	/* Setup the default FIFO interrupt settings */
	selserial_common_setup_fifo_interrupt(sport,
					      SELSERIAL_DEFAULT_COUNT_LEVEL,
					      SELSERIAL_DEFAULT_COUNT_LEVEL,
					      GET_DEFAULT_EORT(8),
					      0);

	rc = selserial_common_register_and_clear_irq(sport);
	if (rc) {
		goto err_unreserve;
	}

	/* Enable the receiver. The TTY layer expects to control the
	 * transmitter. */
	sport->ier |= SELSERIAL_IER_EORT | SELSERIAL_IER_RX_LEVEL;
	sport->lcr |= SELSERIAL_LCR_RX_EN;
	iowrite16(sport->ier, sport->port.membase + SELSERIAL_IER);
	iowrite32(sport->lcr | SELSERIAL_LCR_RX_FLUSH |
		  SELSERIAL_LCR_TX_FLUSH,
		  sport->port.membase + SELSERIAL_LCR );

	return 0;

err_unreserve:
	selserial_common_unreserve_type(sport, SELSERIAL_PORT_TYPE_TTY);

out:
	return rc;
}

/**
 * selserial_shutdown() - Shutdown a port
 *
 * @port: Port being shutdown
 *
 * The port is in process of being closed, so release all resources and make
 * sure the device is in a sane state.
 */
static void selserial_shutdown(struct uart_port *port)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;

	/* Disable the receiver and transmitter and free any break */
	sport->lcr &= ~(SELSERIAL_LCR_RX_EN | SELSERIAL_LCR_TX_EN);
	sport->mcr &= ~(SELSERIAL_MCR_BRK | SELSERIAL_MCR_AUTO_RTS);
	if (sport->auto485) {
		sport->mcr |= SELSERIAL_MCR_AUTO_RTS;
	}

	iowrite32(sport->lcr | SELSERIAL_LCR_TX_FLUSH | SELSERIAL_LCR_RX_FLUSH,
		  sport->port.membase + SELSERIAL_LCR);
	iowrite8(sport->mcr | SELSERIAL_MCR_RTS,
		 sport->port.membase + SELSERIAL_MCR);

	/* Disable the interrupts */
	selserial_common_disable_and_unregister_irq(sport);

	/* Unreserve the port */
	selserial_common_unreserve_type(sport, SELSERIAL_PORT_TYPE_TTY);
}

/**
 * selserial_set_termios() - Set the terminal settings for a UART
 *
 * @port: Port to change the settings on
 * @new: New settings to use
 * @old: Old settings that were in place
 */
static void selserial_set_termios(struct uart_port *port,
				  struct ktermios *new,
				  struct ktermios *old)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;
	unsigned long flags = 0;

	/* Set the new settings */
	selserial_common_set_term_settings(sport, new, old);

	/* Disable interrupts because the rest of this affects how received
	 * characters are processed */
	spin_lock_irqsave(&sport->port.lock, flags);

	/* Set event mask so we acknowledge the appropriate incoming errors */
	sport->port.read_status_mask = SELSERIAL_DATA_READY |
				       SELSERIAL_RX_OVERFLOW_ERR;
	if (new->c_iflag & INPCK) {
		sport->port.read_status_mask |= SELSERIAL_RX_PARITY_ERR |
						SELSERIAL_RX_FRAME_ERR;
	}
	if (new->c_iflag & (BRKINT | PARMRK)) {
		sport->port.read_status_mask |= SELSERIAL_RX_BREAK_ERR;
	}

	/* Set the ignore mask to mask characters that we are going to ignore */
	sport->port.ignore_status_mask = 0;
	if (new->c_iflag & IGNPAR) {
		sport->port.ignore_status_mask |= SELSERIAL_RX_PARITY_ERR |
						  SELSERIAL_RX_FRAME_ERR;
	}
	if (new->c_iflag & IGNBRK) {
		sport->port.ignore_status_mask |= SELSERIAL_RX_BREAK_ERR;
		if (new->c_iflag & IGNPAR) {
			sport->port.ignore_status_mask |= SELSERIAL_RX_OVERFLOW_ERR;
		}
	}

	/* Ignore data if CREAD is not set */
	if ((new->c_cflag & CREAD) == 0) {
		sport->port.ignore_status_mask |= SELSERIAL_DATA_READY;
	}

	/* Enable modem status interrupts if we need to */
	sport->ier &= ~(SELSERIAL_IER_DCTS | SELSERIAL_IER_DDSR |
			SELSERIAL_IER_DRING | SELSERIAL_IER_DDCD);
	if (UART_ENABLE_MS(&sport->port, new->c_cflag)) {
		sport->ier |= SELSERIAL_IER_DCTS | SELSERIAL_IER_DDSR |
			      SELSERIAL_IER_DRING | SELSERIAL_IER_DDCD;
	}

	/* Write everything out to registers */
	iowrite16(sport->ier, sport->port.membase + SELSERIAL_IER);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

/**
 * selserial_type() - Provides a string describing the port type
 *
 * @port: Pointer to the port to describe
 *
 * Return: Always returns "SELSerial UART" for every port managed by this
 *         driver.
 */
static const char *selserial_type(struct uart_port *port)
{
	return "SELSerial UART";
}

/**
 * selserial_request_port() - Request resources for a port
 *
 * @port: Pointer to the port structure to request resources for.
 *
 * This function, along with verify_port, are used when someone changes
 * hardware level settings of the driver. This is the kind of stuff
 * setserial does to initialize the port. We should never have to use this.
 *
 * Return: Always returns -EBUSY.
 */
static int selserial_request_port(struct uart_port *port)
{
	/* We should never get to this function as far as I can tell, but I
	 * could very easily be wrong. For now, we'll print something
	 * annoying out to the log and return failure to request port. */
	dev_err(&((struct selserial_uart_port *)port)->pdev->dev,
		"TTY: request_port() called");
	return -EBUSY;
}

/**
 * selserial_release_port() - Release any resources allocated for a port
 *
 * @port: Pointer to the port to release resources for.
 */
static void selserial_release_port( struct uart_port *port )
{
	/* Nothing to do, just required to provide the function. */
}

/**
 * selserial_config_port() - Sets the port type
 *
 * @port: Pointer to the port structure.
 * @type: Bitmap containing UART_CONFIG_TYPE if we should set the port type.
 *        We should never get a UART_CONFIG_IRQ because we don't register with
 *        UPF_AUTO_IRQ.
 */
static void selserial_config_port(struct uart_port *port, int type)
{
	if (type & UART_CONFIG_TYPE) {
		port->type = PORT_SELSERIAL;
	}
}

/**
 * selserial_verify_port() - Verify provided port settings
 *
 * @port: Pointer to the port information structure.
 * @serinfo: New settings for the port.
 *
 * This function, along with request_port, are used when someone changes
 * hardware level settings of the driver. This is the kind of stuff
 * setserial does to initialize the port. We should never have to use this.
 * 
 * Return: Always returns -EINVAL.
 */
static int selserial_verify_port(struct uart_port *port,
				 struct serial_struct *serinfo)
{
	return -EINVAL;
}

/**
 * selserial_ioctl() - ioctl handler
 *
 * @port: Port to act on
 * @cmd: The requested ioctl
 * @arg: Argument to the ioctl
 */
static int selserial_ioctl(struct uart_port *port,
			   unsigned int cmd,
			   unsigned long arg)
{
	return selserial_common_ioctl((struct selserial_uart_port *)port,
				      cmd,
				      arg);
}

/**
 * selserial_transmit_chars() - Transmit characters from tty layer to hardware
 *
 * @port: Port to transmit characters on.
 *
 * Context: port->lock is already taken.
 */
static void selserial_transmit_chars(struct uart_port *port)
{
	struct selserial_uart_port *sport = (struct selserial_uart_port *)port;
	struct circ_buf *xmit = &port->state->xmit;
	u16 count;

	/* If there is an x_char to send, then send it and return */
	if (port->x_char) {
		selserial_common_txfifo_write(sport, port->x_char);

		/* Increment the statistics */
		sport->port.icount.tx++;

		/* Clear the x_char */
		port->x_char = 0;

		return;
	}

	/* If the tty layer has stopped us, then call stop_tx */
	if (uart_tx_stopped(port)) {
		selserial_stop_tx(port);
		return;
	}

	/* If the transmit buffer is empty then disable interrupts */
	if (uart_circ_empty(xmit)) {
		sport->ier &= ~(SELSERIAL_IER_EOTT | SELSERIAL_IER_TX_LEVEL);
		iowrite16(sport->ier, sport->port.membase + SELSERIAL_IER);
		return;
	}

	/* Initialize the count to what is remaining in the buffer */
	count = sport->port.fifosize -
		ioread16(sport->port.membase + SELSERIAL_TX_COUNT);
	do {
		/* Write data */
		selserial_common_txfifo_write(sport, xmit->buf[xmit->tail]);

		/* Increment the tail pointer */
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

		/* Increment the statistics */
		sport->port.icount.tx++;

		/* If we ran out of bytes to send before we fill the
		 * transmit buffer then break out of the loop */
		if (uart_circ_empty(xmit)) {
			break;
		}
	} while (--count > 0);

	/* We are required to signal the upper layer if we don't have
	 * many bytes left in circular buffer. */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(port);
	}

	/* If we are out of bytes to send then turn off the transmit
	 * interrupts. */
	if (uart_circ_empty(xmit)) {
		sport->ier &= ~(SELSERIAL_IER_EOTT | SELSERIAL_IER_TX_LEVEL);
		iowrite16(sport->ier, sport->port.membase + SELSERIAL_IER);
	}
}

/**
 * selserial_receive_chars() - Receive characters from the UART
 *
 * @sport: Port to receive characters on
 *
 * Context: port->lock is already taken
 */
static void selserial_receive_chars(struct selserial_uart_port *sport)
{
	u32 receive_reg;
	u8 data;
	char flag;
	int max_size = SELSERIAL_HW_FIFO_SIZE;

	do {
		/* Read in the receive register from FIFO */
		receive_reg = ioread32(sport->port.membase + SELSERIAL_RX);

		/* Check to see if this has data */
		if ((receive_reg & SELSERIAL_DATA_READY) == 0) {
			continue;
		}

		/* Data ready so initialize variables and increment counters */
		data = (u8)(receive_reg & SELSERIAL_DATA_BYTE);
		flag = TTY_NORMAL;
		sport->port.icount.rx++;

		/* Check for error conditions on the byte */
		if (unlikely(receive_reg & SELSERIAL_CHARACTER_ERR)) {
			/* Handle the statistics */
			if (receive_reg & SELSERIAL_RX_BREAK_ERR) {
				/* Ignore frame and parity errors if there
				 * is a break */
				receive_reg &= ~(SELSERIAL_RX_FRAME_ERR |
						 SELSERIAL_RX_PARITY_ERR);

				/* Add to statistics and handle the break */
				sport->port.icount.brk++;
				if (uart_handle_break(&sport->port)) {
					continue;
				}
			} else if (receive_reg & SELSERIAL_RX_PARITY_ERR) {
				/* Parity errors */
				sport->port.icount.parity++;
				continue;
			} else if (receive_reg & SELSERIAL_RX_FRAME_ERR) {
				/* Frame errors */
				sport->port.icount.frame++;
				continue;
			}

			/* Overflow errors - not an else because it is
			 * stacked with other errors */
			if (receive_reg & SELSERIAL_RX_OVERFLOW_ERR) {
				sport->port.icount.overrun++;
			}

			/* Mask the register to get rid of errors we don't
			 * care about. */
			receive_reg &= sport->port.read_status_mask;

			/* Handle the flags for the byte */
			if (receive_reg & SELSERIAL_RX_BREAK_ERR) {
				flag = TTY_BREAK;
			} else if (receive_reg & SELSERIAL_RX_PARITY_ERR) {
				flag = TTY_PARITY;
			} else if (receive_reg & SELSERIAL_RX_FRAME_ERR) {
				flag = TTY_FRAME;
			}
		}

		/* Handle sysrq stuff */
		if (!uart_handle_sysrq_char(&sport->port, data)) {
			/* save the char */
			uart_insert_char(&sport->port, receive_reg,
					 SELSERIAL_RX_OVERFLOW_ERR, data, flag);
		}
	} while ((receive_reg & SELSERIAL_DATA_READY) && (max_size-- > 0));

	/* Push the data up to the tty layer by unlocking the lock and calling
	 * tty_flip_buffer_push. Relock afterwards */
	spin_unlock(&sport->port.lock);
	tty_flip_buffer_push(&sport->port.state->port);
	spin_lock(&sport->port.lock);
}

/**
 * selserial_tty_handle_port() - Handle interrupts for this port
 *
 * @sport: Port to handle interrupts on
 * @isr: Interrupt status for the port
 *
 * Context: port->lock is already taken
 */
void selserial_tty_handle_port(struct selserial_uart_port *sport, u16 isr)
{
	u8 msr;

	/* Service data receipt interrupts first so we can clear room for
	 * more bytes */
	if (isr & (SELSERIAL_ISR_EORT | SELSERIAL_ISR_RX_LEVEL)) {
		selserial_receive_chars(sport);
	}

	/* Service modem control lines */
	msr = ioread8(sport->port.membase + SELSERIAL_MSR);
	if (msr & (SELSERIAL_MSR_DDCD | SELSERIAL_MSR_DCTS |
		   SELSERIAL_MSR_DRING | SELSERIAL_MSR_DDSR)) {
		if (msr & SELSERIAL_MSR_DDCD) {
			uart_handle_dcd_change(&sport->port,
					       !(msr & SELSERIAL_MSR_DCD));
		}
		if (msr & SELSERIAL_MSR_DCTS) {
			uart_handle_cts_change(&sport->port,
					       !(msr & SELSERIAL_MSR_CTS));
		}
		if (msr & SELSERIAL_MSR_DRING) {
			sport->port.icount.rng++;
		}
		if (msr & SELSERIAL_MSR_DDSR) {
			sport->port.icount.dsr++;
		}
		wake_up_interruptible(&sport->port.state->port.delta_msr_wait);
	}

	/* Finally, service the data transmit interrupts */
	if (isr & (SELSERIAL_ISR_EOTT | SELSERIAL_ISR_TX_LEVEL)) {
		selserial_transmit_chars(&sport->port);
	}
}

static struct uart_ops selserial_ops =
{
   .tx_empty      = selserial_tx_empty,
   .set_mctrl     = selserial_set_mctrl,
   .get_mctrl     = selserial_get_mctrl,
   .stop_tx       = selserial_stop_tx,
   .start_tx      = selserial_start_tx,
   .stop_rx       = selserial_stop_rx,
   .enable_ms     = selserial_enable_ms,
   .break_ctl     = selserial_break_ctl,
   .startup       = selserial_startup,
   .shutdown      = selserial_shutdown,
   .set_termios   = selserial_set_termios,
   .type          = selserial_type,
   .release_port  = selserial_release_port,
   .request_port  = selserial_request_port,
   .config_port   = selserial_config_port,
   .verify_port   = selserial_verify_port,
   .ioctl         = selserial_ioctl,
};

/**
 * selserial_tty_initport() - Initialize a selserial PCI device
 *
 * @priv: Structure describing the PCI device
 *
 * Return: 0 for success, <0 for error
 */
int selserial_tty_initport(struct selserial_private *priv)
{
	int rc = 0;
	int i = 0;

	dev_info(&priv->dev->dev, "TTY: Registering Port Devices");

	for (i = 0; i < priv->nr; ++i) {
		/* Add the file operations */
		priv->ports[i].port.ops = &selserial_ops;

		/* Try and add it */
		rc = uart_add_one_port(&selserial_uart_driver,
				       &priv->ports[i].port);
		if (rc) {
			dev_err(&priv->dev->dev, "TTY: Error adding tty port");
			break;
		}
	}

	/* Error somewhere, so unregister what we have already done */
	if (rc) {
		for (i = i-1; i >= 0; --i) {
			uart_remove_one_port(&selserial_uart_driver,
					     &priv->ports[i].port);
		}
	}

	return rc;
}

/**
 * selserial_tty_cleanupport() - Deinitialize the device
 *
 * @priv: Structure describing the PCI device
 */
void selserial_tty_cleanupport(struct selserial_private *priv)
{
	int i;
	dev_info(&priv->dev->dev, "TTY: Unregistering Port Devices");
	for (i = 0; i < priv->nr; ++i) {
		uart_remove_one_port(&selserial_uart_driver,
				     &priv->ports[i].port);
	}
}

/**
 * selserial_tty_init() - Initialize the module
 *
 * Return: 0 for success, <0 for failure
 */
int __init selserial_tty_init(void)
{
	return uart_register_driver(&selserial_uart_driver);
}

/**
 * selserial_tty_exit() - Deinitialize the module
 */
void selserial_tty_exit(void)
{
	uart_unregister_driver(&selserial_uart_driver);
}
