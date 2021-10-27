// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2008 Schweitzer Engineering Laboratories, Pullman, Washington
 *
 * Common routines used by the serial port driver
 */

#include <linux/delay.h>

#include "selserial.h"
#include "selserial_common.h"


/**
 * selserial_common_ioctl() - Common IOCTLs for the serial port
 *
 * @sport: Serial Port to handle the IOCTL on
 * @cmd: IOCTL command to manage
 * @arg: Argument to the IOCTL command
 *
 * Return: 0 for successful handle, -ENOIOCTLCMD if we don't support it,
 *         another error code if there was an error.
 */
long selserial_common_ioctl(struct selserial_uart_port *sport,
                            unsigned int cmd,
                            unsigned long arg )
{
	long rc = 0;
	unsigned long flags = 0;
	struct selserial_timed_tx_settings timetxset;
	struct selserial_fifo_int_settings fifoset;
	u32 autorts;
	u32 powerst;
	u32 mode;

	switch(cmd) {
	case SELSERIAL_IOCSTIMETX:
		/* Set the timed transmit delay counter and next character
		 * bits. We don't need to lock here becuase the FPGA takes
		 * care of synchronization at this point */

		/* read in from user space */
		if (copy_from_user(&timetxset, (void __user *)arg,
		                   _IOC_SIZE(cmd))) {
			rc = -EFAULT;
			break;
		}

		/* set the counter */
		sport->tcr = timetxset.count;
		iowrite8(sport->tcr, sport->port.membase + SELSERIAL_TM_COUNT_REG);

		/* Set the flags. This has to be done after the iowrite for
		 * the counter to avoid needing a lock. */
		sport->timedchar = timetxset.flags;
		break;

	case SELSERIAL_IOCGTIMETX:
		/* Get the current delay value in the timed transmit
		 * register and the next character bits. The reads are
		 * atomic so we don't need to lock. */

		/* Get the values */
		timetxset.flags = sport->timedchar;
		timetxset.count = sport->tcr;

		/* Copy to userland */
		if (copy_to_user((void __user *)arg, &timetxset,
		                 _IOC_SIZE(cmd))) {
			rc = -EFAULT;
		}
		break;

	case SELSERIAL_IOCSFIFOINT:
		/* Set the interrupt values for the FIFOs. These are port
		 * settings so we need to do the work under the port.lock.
		 * It affects interrupts so we need to use the _irq* variant
		 * to protect against that. */

		if (copy_from_user(&fifoset,
                             (void __user *)arg,
                             _IOC_SIZE(cmd))) {
			rc = -EFAULT;
			break;
		}

		/* Write the values out to the UART */
		spin_lock_irqsave(&sport->port.lock, flags);

		selserial_common_setup_fifo_interrupt(sport,
						      fifoset.rxcount,
						      fifoset.txcount,
						      fifoset.eort,
						      fifoset.eott );

		/* If we set an eott then enable the interrupt */
		if (sport->fifo_settings.eott != 0) {
			sport->ier |= SELSERIAL_IER_EOTT;
		} else {
			sport->ier &= ~(SELSERIAL_IER_EOTT);
		}

		/* Mainboard will probably move to a traditional interrupt
		 * service routine. When it does then just remove this if
		 * statement to get this working again. This COULD be done
		 * by moving this case out of COMMON, but it's more work to
		 * move it back in than just removing a line or two. */
		if (sport->port_type == SELSERIAL_PORT_TYPE_MB) {
			sport->ier = 0;
		}

		/* Set the ier */
		iowrite16(sport->ier, sport->port.membase + SELSERIAL_IER);
		spin_unlock_irqrestore(&sport->port.lock, flags);
		break;

	case SELSERIAL_IOCGFIFOINT:
		/* Get the current FIFO interrupt settings. This needs to
		 * be done under the port lock to make it atomic, but
		 * since it is a read we don't need the irq* varients */
		spin_lock(&sport->port.lock);
		if (copy_to_user((void __user *)arg,
				 &sport->fifo_settings,
				 _IOC_SIZE(cmd))) {
			rc = -EFAULT;
		}
		spin_unlock( &sport->port.lock );
		break;

	case SELSERIAL_IOCSAUTORTS:
		/* Change the AUTO_RTS mode for the port. This affects
		 * settings and how the interrupts work, so it needs to be
		 * done under the port lock with disabled interrupts. */

		/* Get flag from user space */
		if (copy_from_user(&autorts,
				   (void __user *)arg,
				   _IOC_SIZE(cmd))) {
			rc = -EFAULT;
			break;
		}

		/* Save the mode */
		if (autorts) {
			sport->mcr |= SELSERIAL_MCR_AUTO_RTS;
		} else {
			sport->mcr &= ~(SELSERIAL_MCR_AUTO_RTS);
		}

		/* Write the values out to the UART */
		spin_lock_irqsave(&sport->port.lock, flags);
		iowrite8(sport->mcr, sport->port.membase + SELSERIAL_MCR);
		spin_unlock_irqrestore(&sport->port.lock, flags);
		break;

	case SELSERIAL_IOCGPOWERSTATE:
		/* Get the current port power state */
		powerst = !!(sport->gcr & SELSERIAL_GCR_PORT_POWER);
		if (copy_to_user((void __user *)arg,
				 &powerst,
				 _IOC_SIZE(cmd))) {
			rc = -EFAULT;
		}
		break;

	case SELSERIAL_IOCSPOWERSTATE:
		/* Set the port power state for this port */

		/* Get flag from user space */
		if (copy_from_user(&powerst,
				   (void __user *)arg,
				   _IOC_SIZE(cmd))) {
			rc = -EFAULT;
			break;
		}

		if ((powerst != '0') && (powerst != '1')) {
			rc = -EINVAL;
			break;
		}

		/* Set the mode */
		__selserial_special_power_store(sport, (char)powerst);
		break;

	case SELSERIAL_IOCG485STATE:
		/* Get the current 485 state */
		if (sport->gcr & SELSERIAL_GCR_485_MODE) {
			if (sport->auto485) {
				mode = (u32)('a');
			} else {
				mode = (u32)('1');
			}
		} else {
			mode = (u32)('0');
		}

		if (copy_to_user((void __user *)arg, &mode, _IOC_SIZE(cmd))) {
			rc = -EFAULT;
		}
		break;

	case SELSERIAL_IOCS485STATE:
		/* Set the 485 state for this port */

		/* Get flag from user space */
		if (copy_from_user(&mode, (void __user *)arg, _IOC_SIZE(cmd))) {
			rc = -EFAULT;
			break;
		}

		/* Check modes */
		if ((mode != '0') && (mode != '1') && (mode != 'a')) {
			rc = -EINVAL;
			break;
		}

		/* Set the mode */
		spin_lock(&sport->port.lock);
		__selserial_special_485_store(sport, (char)mode);
		spin_unlock( &sport->port.lock );
		break;

	case TCFLSH:
		/* Support for flushing the hardware buffers. */
		spin_lock(&sport->port.lock);
		switch(arg) {
		case TCIFLUSH:
			iowrite32(sport->lcr |
				  SELSERIAL_LCR_RX_FLUSH,
				  sport->port.membase + SELSERIAL_LCR);
			break;

		case TCOFLUSH:
			iowrite32(sport->lcr |
				  SELSERIAL_LCR_TX_FLUSH,
				  sport->port.membase + SELSERIAL_LCR);
			break;

		case TCIOFLUSH:
			iowrite32(sport->lcr |
				  SELSERIAL_LCR_TX_FLUSH |
				  SELSERIAL_LCR_RX_FLUSH,
				  sport->port.membase + SELSERIAL_LCR);
			break;

		default:
			rc = -EINVAL;
			break;
		}
		rc = -ENOIOCTLCMD;
		spin_unlock(&sport->port.lock);
		break;

	default:
		/* Not a recognized command */
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}

/**
 * selserial_common_set_term_settings() - Set the term settings for lcr and mcr
 *
 * @sport: Pointer to the port structure to change
 * @new: New terminal settings to apply
 * @old: Terminal settings that are in place before this call
 *
 * Context: port.lock should already be taken
*/
void selserial_common_set_term_settings(struct selserial_uart_port *sport,
					struct ktermios *new,
					struct ktermios *old)
{
	u16 baud_div;
	u8 baud_frac;
	unsigned int baud;

	/* Get the baud rate */
	baud = uart_get_baud_rate(&sport->port, new, old, 0, 115200);

	/* Clear the TERMIOS settings */
	sport->lcr &= SELSERIAL_LCR_SET_TERMIOS_RESET;

	/* Set the number of data bits */
	switch(new->c_cflag & CSIZE) {
	case CS5:
		sport->lcr |= SELSERIAL_LCR_5DB;
		break;
	case CS6:
		sport->lcr |= SELSERIAL_LCR_6DB;
		break;
	case CS7:
		sport->lcr |= SELSERIAL_LCR_7DB;
		break;
	case CS8:
	default:
		sport->lcr |= SELSERIAL_LCR_8DB;
		break;
	}

	/* Check for stop bits */
	if (new->c_cflag & CSTOPB) {
		sport->lcr |= SELSERIAL_LCR_TWO_STOP;
	}

	/* Parity enable */
	if (new->c_cflag & PARENB) {
		sport->lcr |= SELSERIAL_LCR_PARITY_EN;
	}

	/* Parity even/odd */
	if (!(new->c_cflag & PARODD)) {
		sport->lcr |= SELSERIAL_LCR_EVEN_PARITY;
	}

	/* Sticky parity */
	if (new->c_cflag & CMSPAR) {
		sport->lcr |= SELSERIAL_LCR_STICKY_PARITY;
	}

	/* Update the port timeout */
	uart_update_timeout(&sport->port, new->c_cflag, baud);

	/* The math for this is taken from the CIS for the serial ports.
	 * See that for explanation. */
	baud_div = (sport->port.uartclk / (16 * baud)) - 1;
	baud_frac = (((2 * (sport->port.uartclk % (16 * baud))) / baud) + 1) / 2;
	sport->lcr = SELSERIAL_SET_BAUD_DIV(sport->lcr, baud_div);
	sport->lcr = SELSERIAL_SET_FRAC_BAUD(sport->lcr, baud_frac);

	/* Check for auto HW flow control */
	if (new->c_cflag & CRTSCTS) {
		sport->mcr |= SELSERIAL_MCR_AUTO_HS_RTS | SELSERIAL_MCR_AUTO_HS_CTS;
	} else {
		sport->mcr &= ~(SELSERIAL_MCR_AUTO_HS_RTS | SELSERIAL_MCR_AUTO_HS_CTS);
	}

	/* Check for auto HW flow control */
	if (new->c_cflag & CREAD) {
		sport->lcr |= SELSERIAL_LCR_RX_EN;
	} else {
		sport->lcr &= ~(SELSERIAL_LCR_RX_EN);
	}

	/* Write the new MCR and LCR values */
	iowrite8(sport->mcr, sport->port.membase + SELSERIAL_MCR);
	iowrite32(sport->lcr, sport->port.membase + SELSERIAL_LCR);
}

/**
 * selserial_commont_get_mctrl() - Provides the state of the modem control pins
 *
 * @sport: Retrieve pin state from this port
 *
 * Context: No locking needed for this function.
 * Return: Bitfield containing the state of the pins.
 */
unsigned int selserial_common_get_mctrl(struct selserial_uart_port *sport)
{
	unsigned int ret = 0;
	u8 msr = 0;

	/* Get the modem status register from the UART */
	msr = ioread8(sport->port.membase + SELSERIAL_MSR);

	if (!(msr & SELSERIAL_MSR_CTS)) {
		ret |= TIOCM_CTS;
	}
	if (!(msr & SELSERIAL_MSR_DSR)) {
		ret |= TIOCM_DSR;
	}
	if (!(msr & SELSERIAL_MSR_RING)) {
		ret |= TIOCM_RI;
	}
	if (!(msr & SELSERIAL_MSR_DCD)) {
		ret |= TIOCM_CAR;
	}
	return ret;
}

/**
 * selserial_common_set_mctrl() - Set the modem control lines
 *
 * @sport: Port to set the modem control lines
 * @mctrl: Bitfield of the modem control lines to set
 *
 * Context: TIOCM_OUT1 and TIOCM_OUT2 are ignored because the hardware
 *          does not implement them.
 *
 *          No locking is needed for this function.
 */
void selserial_common_set_mctrl(struct selserial_uart_port *sport,
				unsigned int mctrl)
{
	/* Prevent the application from controlling RTS in auto485 mode */
	if (sport->auto485) {
		return;
	}

	/* Clear the RTS/DTR bits in the mcr locally to start with */
	sport->mcr &= ~(SELSERIAL_MCR_DTR | SELSERIAL_MCR_RTS);
	/* Now set the desired flags */
	if (!(mctrl & TIOCM_RTS)) {
		sport->mcr |= SELSERIAL_MCR_RTS;
	}
	if (!(mctrl & TIOCM_DTR)) {
		sport->mcr |= SELSERIAL_MCR_DTR;
	}
	iowrite8(sport->mcr, sport->port.membase + SELSERIAL_MCR);
}

/**
 * selserial_common_setup_fifo_interrupt() - Set the FIFO interrupt settings
 *
 * @sport: Pointer to the port structure to use to set the settings
 * @rxlvl: Number of bytes to use as the high water mark for the receive FIFO
 * @txlvl: Number of bytes to use as the low water mark for the transmit FIFO
 * @eort: End-of-receive timer count in bit-times
 * @eott: End-of-transmit timer count in bit-times
 *
 * If rxlvl, txlvl, or eort are passed in as zero, they are set to the
 * default level. This is done so we can't configure the UART to be 
 * unresponsive. If eott is passed in as zero, it is set to the default.
 * This is done for autorts (ie auto485) and other FPGA needs per Jerry.
 */
void selserial_common_setup_fifo_interrupt(struct selserial_uart_port *sport,
					   u16 rxlvl,
					   u16 txlvl,
					   u16 eort,
					   u16 eott)
{
	sport->fifo_settings.eort = (eort ? eort : GET_DEFAULT_EORT(8));
	sport->fifo_settings.eott = (eott ? eott : SELSERIAL_DEFAULT_EOXT_TIMER);
	sport->fifo_settings.rxcount = (rxlvl ? rxlvl :
						SELSERIAL_DEFAULT_COUNT_LEVEL);
	sport->fifo_settings.txcount = (txlvl ? txlvl :
						SELSERIAL_DEFAULT_COUNT_LEVEL);
	iowrite32((sport->fifo_settings.eott << 16) |
		  sport->fifo_settings.eort,
		  sport->port.membase + SELSERIAL_EORT_REG);
	iowrite32((sport->fifo_settings.txcount << 16) |
		  sport->fifo_settings.rxcount,
		  sport->port.membase + SELSERIAL_RX_FIFO_LEVEL_REG);
}

/**
 * selserial_common_register_and_clear_irq() - Register and clear IRQ
 *
 * @sport: Pointer to the serial port structure to register
 *
 * Return: 0 for success or an error code if there was an error registering
 */
int selserial_common_register_and_clear_irq(struct selserial_uart_port *sport)
{
	int rc;
	struct irq_info *i =
		&(((struct selserial_private *)(pci_get_drvdata(sport->pdev)))->
		irq_list);

	/* Register */
	rc = selserial_register_irq_list(sport->port.irq, i, &sport->list);

	/* Clear */
	if (!rc) {
		ioread16(sport->port.membase + SELSERIAL_ISR);
	}
	return rc;
}

/**
 * selserial_common_disable_and_unregister_irq() - Disable and unregister port
 *
 * @sport: Pointer to the serial port to unregister
 *
 * Disable inturrupts, unregister from the handler, and clear any pending
 * interrupts.
 */
void selserial_common_disable_and_unregister_irq(struct selserial_uart_port *sport)
{
	struct irq_info *i =
		&(((struct selserial_private *)(pci_get_drvdata(sport->pdev)))->
		irq_list);
	/* Disable the interrupt and clear pending interrupts */
	sport->ier = 0;
	iowrite16(0, sport->port.membase + SELSERIAL_IER);
	ioread16(sport->port.membase + SELSERIAL_ISR);
	/* Unregister IRQ */
	selserial_unregister_irq_list(sport->port.irq, i, &sport->list);
}

/**
 * void selserial_common_txfifo_write() - Helper to write the transmit FIFO
 *
 * @sport: Port structure of the port to write
 * @data: Byte to write
 *
 * Context: port.lock should already be taken
 */
void selserial_common_txfifo_write(struct selserial_uart_port *sport, u8 data)
{
	/* Write the RTS bit if auto485 is set */
	if (sport->auto485) {
		iowrite8(sport->mcr & ~(SELSERIAL_MCR_RTS),
			 sport->port.membase + SELSERIAL_MCR);
	}

	/* Write the byte with the present timedchar bits */
	iowrite16(((sport->timedchar) | data),
		  sport->port.membase + SELSERIAL_TX);
	
	/* Clear the timedchar bits */
	sport->timedchar = 0;
}

/**
 * selserial_common_reserve_type() - Reserve the specified port
 *
 * @sport: Pointer to the serial port struct to reserve
 * @type: Port type that is reserving
 *
 * Return: -EBUSY if the port can't be reserved, 0 on success
 */
int selserial_common_reserve_type(struct selserial_uart_port *sport, int type )
{
	int rc = -EBUSY;

	spin_lock( &sport->port.lock );
	if (sport->port_type == SELSERIAL_PORT_TYPE_UNOPENED) {
		sport->port_type = type;
		rc = 0;
	}
	spin_unlock( &sport->port.lock );

	return rc;
}

/**
 * selserial_common_unreserve_type() - Unreserve the specified port
 *
 * @sport: Pointer to the serial port struct to unreserve
 * @type: Port type that is unreserving
 */
void selserial_common_unreserve_type(struct selserial_uart_port *sport, int type)
{
	spin_lock(&sport->port.lock);
	BUG_ON(sport->port_type != type);
	sport->port_type = SELSERIAL_PORT_TYPE_UNOPENED;
	spin_unlock(&sport->port.lock);
}
