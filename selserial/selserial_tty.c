//////////////////////////////////////////////////////////////////////////////
// COPYRIGHT (c) 2008 Schweitzer Engineering Laboratories, Pullman, Washington
///
/// @file selserial_tty.c
///
/// @brief TTY functionality for the port
//////////////////////////////////////////////////////////////////////////////

#include "selserial.h"
#include "selserial_common.h"

#include <linux/tty_flip.h>
#include <linux/version.h>

// serial interface routines
static unsigned int selserial_tx_empty( struct uart_port * port );
static void selserial_set_mctrl( struct uart_port * port, unsigned int mctrl );
static unsigned int selserial_get_mctrl( struct uart_port * port );
static void selserial_stop_tx( struct uart_port * port );
static void selserial_start_tx( struct uart_port * port );
static void selserial_stop_rx( struct uart_port * port );
static void selserial_enable_ms( struct uart_port * port );
static void selserial_break_ctl( struct uart_port *port, int ctl );
static int selserial_startup( struct uart_port *port );
static void selserial_shutdown( struct uart_port *port );
static void selserial_set_termios( struct uart_port *port,
                                   struct ktermios *new,
                                   struct ktermios *old );
static const char * selserial_type( struct uart_port *port );
static void selserial_release_port( struct uart_port *port );
static int selserial_request_port( struct uart_port *port );
static void selserial_config_port( struct uart_port *port, int type );
static int selserial_verify_port( struct uart_port *port,
                                  struct serial_struct * serinfo );
static int selserial_ioctl( struct uart_port * port,
                            unsigned int cmd,
                            unsigned long arg );

// interrupt routines
static void selserial_transmit_chars( struct uart_port *port );
static void selserial_receive_chars( struct selserial_uart_port *sport );
void selserial_tty_handle_port( struct selserial_uart_port *sport, u16 isr );

//////////////////////////////////////////////////////////////////////////////
/// Static variable declaration

/// operations I need to provide to the serial core layer
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

/// UART driver info struct
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

//////////////////////////////////////////////////////////////////////////////
/// Serial Interface Routines
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// @brief Queries the driver to see if the transmit fifo is empty
///
/// @param port Port to check transmit fifo level.
///
/// @return 0 if the fifo is not empty, TIOCSER_TEMT if it is empty.
//////////////////////////////////////////////////////////////////////////////
static unsigned int selserial_tx_empty( struct uart_port * port )
{
   unsigned long flags = 0;
   unsigned int ret = TIOCSER_TEMT;
   u16 tx_fifo_count = 0;

   // Lock and disable interrupts so we don't misreport
   spin_lock_irqsave( &port->lock, flags );

   // See if there are any bytes in the transmit fifo
   tx_fifo_count = ioread16( port->membase + SELSERIAL_TX_COUNT );
   if( tx_fifo_count )
   {
      ret = 0;
   }

   // Unlock
   spin_unlock_irqrestore( &port->lock, flags );

   return ret;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the modem control lines.
///
/// @param port Port to set the modem control lines on
/// @param mctrl Bit field describing modem control lines to set
//////////////////////////////////////////////////////////////////////////////
static void selserial_set_mctrl( struct uart_port * port, unsigned int mctrl )
{
   struct selserial_uart_port * sport = (struct selserial_uart_port *)port;
   selserial_common_set_mctrl( sport, mctrl );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief return the state of the modem control pins
///
/// @param port Port to retrieve pin state from
///
/// @return Bit field containing state of the pins
//////////////////////////////////////////////////////////////////////////////
static unsigned int selserial_get_mctrl( struct uart_port * port )
{
   struct selserial_uart_port * sport = (struct selserial_uart_port *)port;
   return selserial_common_get_mctrl( sport );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Stop transmitting characters as soon as possible
///
/// @param port Port to stop transmitting on
///
/// @remarks This gets called by the upper layer generally on CTS deasserting
///          or reception of an XOFF for when software is controlling flow
///          control.
//////////////////////////////////////////////////////////////////////////////
static void selserial_stop_tx( struct uart_port * port )
{
   struct selserial_uart_port * sport = (struct selserial_uart_port *)port;

   // disable the transmit interrupts
   sport->ier &= ~(SELSERIAL_IER_EOTT | SELSERIAL_IER_TX_LEVEL);

   // disable the transmitter
   sport->lcr &= ~(SELSERIAL_LCR_TX_EN);

   // write them out
   iowrite16( sport->ier, sport->port.membase + SELSERIAL_IER );
   iowrite32( sport->lcr, sport->port.membase + SELSERIAL_LCR );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief start up the transmitter
///
/// @param port Port to start transmission on.
//////////////////////////////////////////////////////////////////////////////
static void selserial_start_tx( struct uart_port * port )
{
   struct selserial_uart_port * sport = (struct selserial_uart_port *)port;

   // Enable the transmitter interrupts
   sport->ier |= SELSERIAL_IER_TX_LEVEL;
   if( sport->fifo_settings.eott )
   {
      sport->ier |= SELSERIAL_IER_EOTT;
   }

   // Enable the transmitter
   sport->lcr |= (SELSERIAL_LCR_TX_EN);

   // write them out
   iowrite16( sport->ier, sport->port.membase + SELSERIAL_IER );
   iowrite32( sport->lcr, sport->port.membase + SELSERIAL_LCR );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Stop all reception.
///
/// @param port Port to stop reception on
///
/// @remarks This is generally called when the port is being shutdown
//////////////////////////////////////////////////////////////////////////////
static void selserial_stop_rx( struct uart_port * port )
{
   struct selserial_uart_port * sport = (struct selserial_uart_port *)port;

   // disable receive interrupt
   sport->ier &= ~(SELSERIAL_IER_EORT | SELSERIAL_IER_RX_LEVEL);

   // disable receiver
   sport->lcr &= ~(SELSERIAL_LCR_RX_EN);

   // write them out
   iowrite16( sport->ier, sport->port.membase + SELSERIAL_IER );
   iowrite32( sport->lcr, sport->port.membase + SELSERIAL_LCR );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Enable modem status interrupts
///
/// @param port Port to enable modem status interrupts on
//////////////////////////////////////////////////////////////////////////////
static void selserial_enable_ms( struct uart_port * port )
{
   struct selserial_uart_port * sport = (struct selserial_uart_port *)port;

   sport->ier |= (SELSERIAL_IER_DCTS | SELSERIAL_IER_DDSR |
                  SELSERIAL_IER_DRING | SELSERIAL_IER_DDCD );

   // write it out
   iowrite16( sport->ier, sport->port.membase + SELSERIAL_IER );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set or clear a break condition on the line.
///
/// @param port Port to set or clear the break condition on
/// @param ctl if set then start a break, if clear then clear the break.
//////////////////////////////////////////////////////////////////////////////
static void selserial_break_ctl( struct uart_port *port, int ctl )
{
   struct selserial_uart_port * sport = (struct selserial_uart_port *)port;
   unsigned long flags = 0;

   // save our IRQ state while we are setting the break condition
   spin_lock_irqsave( &sport->port.lock, flags );

   if( ctl )
   {
      sport->mcr |= SELSERIAL_MCR_BRK;
   }
   else
   {
      sport->mcr &= ~(SELSERIAL_MCR_BRK);
   }

   // write it out
   iowrite8( sport->mcr, sport->port.membase + SELSERIAL_MCR );

   // unlock
   spin_unlock_irqrestore( &sport->port.lock, flags );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief port is being opened so do whatever setup is necessary
///
/// @param port Port being opened
///
/// @return 0 for successful startup, non-zero for failure.
//////////////////////////////////////////////////////////////////////////////
static int selserial_startup( struct uart_port *port )
{
   struct selserial_uart_port *sport = (struct selserial_uart_port *)port;
   int rc = 0;

   // try to reserve the port type
   rc = selserial_common_reserve_type( sport, SELSERIAL_PORT_TYPE_TTY );
   if( rc )
   {
      goto out;
   }

   // setup the default fifo interrupt settings
   selserial_common_setup_fifo_interrupt( sport,
                                          SELSERIAL_DEFAULT_COUNT_LEVEL,
                                          SELSERIAL_DEFAULT_COUNT_LEVEL,
                                          GET_DEFAULT_EORT(8),
                                          0 );

   rc = selserial_common_register_and_clear_irq( sport );
   if( rc )
   {
      goto err_unreserve;
   }

   // enable the receiver.  The TTY layer expects to control the transmitter.
   sport->ier |= SELSERIAL_IER_EORT | SELSERIAL_IER_RX_LEVEL;
   sport->lcr |= SELSERIAL_LCR_RX_EN;
   iowrite16( sport->ier, sport->port.membase + SELSERIAL_IER );
   iowrite32( sport->lcr | SELSERIAL_LCR_RX_FLUSH |
                           SELSERIAL_LCR_TX_FLUSH,
              sport->port.membase + SELSERIAL_LCR );

   return 0;

err_unreserve:
   selserial_common_unreserve_type( sport, SELSERIAL_PORT_TYPE_TTY );

out:
   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief The port is in process of being closed so release all resources
///        and make sure the device is in a sane state
///
/// @param port Port being shutdown
/////////////////////////////////////////////////////////////////////////////
static void selserial_shutdown( struct uart_port *port )
{
   struct selserial_uart_port * sport = (struct selserial_uart_port *)port;

   // disable the receiver and transmitter and free any break
   sport->lcr &= ~(SELSERIAL_LCR_RX_EN | SELSERIAL_LCR_TX_EN);
   sport->mcr &= ~(SELSERIAL_MCR_BRK | SELSERIAL_MCR_AUTO_RTS);
   if( sport->auto485 )
   {
      sport->mcr |= SELSERIAL_MCR_AUTO_RTS;
   }

   iowrite32( sport->lcr | SELSERIAL_LCR_TX_FLUSH | SELSERIAL_LCR_RX_FLUSH,
              sport->port.membase + SELSERIAL_LCR );
   iowrite8( (sport->mcr | SELSERIAL_MCR_RTS),
              sport->port.membase + SELSERIAL_MCR );

   // disable the interrupts
   selserial_common_disable_and_unregister_irq( sport );

   // unreserve the port
   selserial_common_unreserve_type( sport, SELSERIAL_PORT_TYPE_TTY );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the terminal settings pertaining to this UART
///
/// @param port Port to change settings on.
/// @param new New settings to use
/// @param old Old settings that were in place
//////////////////////////////////////////////////////////////////////////////
static void selserial_set_termios( struct uart_port *port,
                                   struct ktermios *new,
                                   struct ktermios *old )
{
   struct selserial_uart_port * sport = (struct selserial_uart_port *)port;
   unsigned long flags = 0;

   // set the new settings
   selserial_common_set_term_settings( sport, new, old );

   // disable interrupts because the rest of this affects how received
   // characters are processed
   spin_lock_irqsave( &sport->port.lock, flags );

   // set event mask so we acknowledge the appropriate incoming errors
   sport->port.read_status_mask = SELSERIAL_DATA_READY |
                                  SELSERIAL_RX_OVERFLOW_ERR;
   if (new->c_iflag & INPCK)
   {
      sport->port.read_status_mask |= SELSERIAL_RX_PARITY_ERR |
                                      SELSERIAL_RX_FRAME_ERR;
   }
   if (new->c_iflag & (BRKINT | PARMRK))
   {
      sport->port.read_status_mask |= SELSERIAL_RX_BREAK_ERR;
   }

   // set the ignore mask to mask characters that we are going to ignore.
   sport->port.ignore_status_mask = 0;
   if (new->c_iflag & IGNPAR)
   {
      sport->port.ignore_status_mask |= SELSERIAL_RX_PARITY_ERR |
                                        SELSERIAL_RX_FRAME_ERR;
   }
   if (new->c_iflag & IGNBRK)
   {
      sport->port.ignore_status_mask |= SELSERIAL_RX_BREAK_ERR;
      if (new->c_iflag & IGNPAR)
      {
         sport->port.ignore_status_mask |= SELSERIAL_RX_OVERFLOW_ERR;
      }
   }

   // ignore data if CREAD is not set
   if((new->c_cflag & CREAD) == 0)
   {
      sport->port.ignore_status_mask |= SELSERIAL_DATA_READY;
   }

   // Enable modem status interrupts if we need to
   sport->ier &= ~(SELSERIAL_IER_DCTS | SELSERIAL_IER_DDSR |
                   SELSERIAL_IER_DRING | SELSERIAL_IER_DDCD);
   if( UART_ENABLE_MS( &sport->port, new->c_cflag ) )
   {
      sport->ier |= SELSERIAL_IER_DCTS | SELSERIAL_IER_DDSR |
                    SELSERIAL_IER_DRING | SELSERIAL_IER_DDCD;
   }

   // write everything out to registers
   iowrite16( sport->ier, sport->port.membase + SELSERIAL_IER );

   spin_unlock_irqrestore( &sport->port.lock, flags );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Returns a string describing the port type
///
/// @param port pointer to the port to describe
///
/// @return Always "SELSerial Uart" for every port managed by this driver
//////////////////////////////////////////////////////////////////////////////
static const char * selserial_type( struct uart_port *port )
{
   return "SELSerial Uart";
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Request resources for a port if any of the resources or port setup
///        has changed.
///
/// @param port pointer to the port structure to request resources for
///
/// @return always returns -EBUSY
///
/// @remarks this function along with verify_port are used when someone
///          changes HARDWARE level settings of the driver.  This is the kind
///          of stuff like setserial does to initialize the port.  We should
///          NEVER have to use this
//////////////////////////////////////////////////////////////////////////////
static int selserial_request_port( struct uart_port *port )
{
   // We should never get to this function as far as I can tell, but I
   // could very easily be wrong.  For now, we'll print something annoying
   // out to the log and return failure to request port
   dev_err( &((struct selserial_uart_port *)port)->pdev->dev,
            "TTY: request_port() called\n" );
   return -EBUSY;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Release any resources that may be allocated for this port
///
/// @param port pointer to the port to release resources for
//////////////////////////////////////////////////////////////////////////////
static void selserial_release_port( struct uart_port *port )
{
   // nothing to do, just required to provide the function.
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Sets the port type
///
/// @param port pointer to the port structure
/// @type bitmap containing UART_CONFIG_TYPE if we should set the port type.
///       We should never get a UART_CONFIG_IRQ becuase we don't register with
///       UPF_AUTO_IRQ
//////////////////////////////////////////////////////////////////////////////
static void selserial_config_port( struct uart_port *port, int type )
{
   if( type & UART_CONFIG_TYPE )
   {
      port->type = PORT_SELSERIAL;
   }
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Check port settings passed in by serinfo to see if we can set this
///        port to them.
///
/// @param port pointer to a the port information structure.
/// @param serinfo new settings for the port
///
/// @return always returns -EINVAL
///
/// @remarks this function along with request_port are used when someone
///          changes HARDWARE level settings of the driver.  This is the kind
///          of stuff like setserial does to initialize the port.  We should
///          NEVER have to use this.
//////////////////////////////////////////////////////////////////////////////
static int selserial_verify_port( struct uart_port *port,
                                  struct serial_struct * serinfo )
{
   return -EINVAL;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief custom ioctl handler for the port
///
/// @param port Port to act on
/// @cmd the ioctl requested
/// @arg argument to the ioctl
//////////////////////////////////////////////////////////////////////////////
static int selserial_ioctl( struct uart_port * port,
                            unsigned int cmd,
                            unsigned long arg )
{
   return selserial_common_ioctl( (struct selserial_uart_port *)port,
                                  cmd,
                                  arg );
}

//////////////////////////////////////////////////////////////////////////////
/// Interrupt routines
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// @brief Transmit characters from the tty layer to the hardware
///
/// @port Port to transmit characters on.
///
/// @remarks PORT->LOCK ALREADY TAKEN.  The multiple return value thing is
///          done in the 8250 driver and the order I have here is mirrored in
///          that driver.
//////////////////////////////////////////////////////////////////////////////
static void selserial_transmit_chars( struct uart_port *port )
{
   struct selserial_uart_port * sport = (struct selserial_uart_port *)port;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
   struct circ_buf *xmit = &port->info->xmit;
#else
   struct circ_buf *xmit = &port->state->xmit;
#endif
   u16 count;

   // if there is an x_char to send, then send it and return
   if( port->x_char )
   {
      selserial_common_txfifo_write( sport, port->x_char );

      // increment the statistics
      sport->port.icount.tx++;

      // clear the x_char
      port->x_char = 0;

      return;
   }

   // if the tty layer has stopped us, then call stop_tx
   if( uart_tx_stopped( port ) )
   {
      selserial_stop_tx( port );
      return;
   }

   // if the transmit buffer is empty then disable interrupts
   if( uart_circ_empty( xmit ) )
   {
      sport->ier &= ~(SELSERIAL_IER_EOTT | SELSERIAL_IER_TX_LEVEL);
      iowrite16( sport->ier, sport->port.membase + SELSERIAL_IER );
      return;
   }

   // initialize the count to what is remaining in the buffer
   count = sport->port.fifosize -
           ioread16( sport->port.membase + SELSERIAL_TX_COUNT );
   do
   {
      // write data
      selserial_common_txfifo_write( sport, xmit->buf[xmit->tail] );

      // increment the tail pointer
      xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

      // increment the statistics
      sport->port.icount.tx++;

      // if we ran out of bytes to send before we fill the transmit buffer
      // then break out of the loop
      if (uart_circ_empty(xmit))
      {
         break;
      }
   } while (--count > 0);

   // we are required to signal the upper layer if we don't have many bytes
   // left in circular buffer
   if( uart_circ_chars_pending( xmit ) < WAKEUP_CHARS )
   {
      uart_write_wakeup( port );
   }

   // if we are out of bytes to send then turn off the transmit interrupts.
   if( uart_circ_empty( xmit ) )
   {
      sport->ier &= ~(SELSERIAL_IER_EOTT | SELSERIAL_IER_TX_LEVEL);
      iowrite16( sport->ier, sport->port.membase + SELSERIAL_IER );
   }

   // Kick the stale data timer to keep from flushing since we are sending data
   selserial_common_kick_stale_data_timer( sport );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Receive characters from the uart
///
/// @param sport Port to receive characters on
///
/// @remarks PORT->LOCK ALREADY TAKEN
//////////////////////////////////////////////////////////////////////////////
static void selserial_receive_chars( struct selserial_uart_port *sport )
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
   struct tty_struct *tty = sport->port.info->tty;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
   struct tty_struct *tty = sport->port.info->port.tty;
#endif
   u32 receive_reg;
   u8  data;
   char flag;
   int max_size = SELSERIAL_HW_FIFO_SIZE;

   do
   {
      // read in the receive register from fifo
      receive_reg = ioread32( sport->port.membase + SELSERIAL_RX );

      // check to see if this has data
      if( (receive_reg & SELSERIAL_DATA_READY) == 0 )
      {
         continue;
      }

      // data ready so initialize variables and increment counters
      data = (u8)(receive_reg & SELSERIAL_DATA_BYTE);
      flag = TTY_NORMAL;
      sport->port.icount.rx++;

      // check for error conditions on the byte
      if( unlikely( receive_reg & SELSERIAL_CHARACTER_ERR ) )
      {
         // handle the statistics
         if( receive_reg & SELSERIAL_RX_BREAK_ERR )
         {
            // ignore frame and parity errors if there is a break
            receive_reg &= ~(SELSERIAL_RX_FRAME_ERR | SELSERIAL_RX_PARITY_ERR);

            // add to statistics and then handle the break;
            sport->port.icount.brk++;
            if( uart_handle_break( &sport->port ) )
            {
               continue;
            }
         }

         // parity errors
         else if( receive_reg & SELSERIAL_RX_PARITY_ERR )
         {
            sport->port.icount.parity++;
            continue;
         }

         // frame errors
         else if( receive_reg & SELSERIAL_RX_FRAME_ERR )
         {
            sport->port.icount.frame++;
            continue;
         }

         // overflow errors - not an else because it is stacked with other
         // errors
         if( receive_reg & SELSERIAL_RX_OVERFLOW_ERR )
         {
            sport->port.icount.overrun++;
         }

         // Mask the register to get rid of errors we don't care about.
         receive_reg &= sport->port.read_status_mask;

         // handle the flags for the byte
         if( receive_reg & SELSERIAL_RX_BREAK_ERR )
         {
            flag = TTY_BREAK;
         }
         else if( receive_reg & SELSERIAL_RX_PARITY_ERR )
         {
            flag = TTY_PARITY;
         }
         else if( receive_reg & SELSERIAL_RX_FRAME_ERR )
         {
            flag = TTY_FRAME;
         }
      }

      // handle sysrq stuff
      if( !uart_handle_sysrq_char( &sport->port, data ) )
      {
         // save the char
         uart_insert_char( &sport->port, receive_reg,
                           SELSERIAL_RX_OVERFLOW_ERR, data, flag );
      }
   } while( (receive_reg & SELSERIAL_DATA_READY) && (max_size-- > 0) );

   // push the data up to the tty layer by unlocking the lock and calling
   // tty_flip_buffer_push.  Relock afterwards
   spin_unlock( &sport->port.lock );
   tty_flip_buffer_push( &sport->port.state->port );
   spin_lock( &sport->port.lock );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Handle the interrupts for this port
///
/// @param sport Port to handle interrupts on
/// @param isr Interrupt status for the port
///
/// @remarks PORT->LOCK ALREADY TAKEN
//////////////////////////////////////////////////////////////////////////////
void selserial_tty_handle_port( struct selserial_uart_port *sport, u16 isr )
{
   u8 msr;

   // service data receipt interrupts first so we can clear room for more bytes
   if( isr & (SELSERIAL_ISR_EORT |
              SELSERIAL_ISR_RX_LEVEL) )
   {
      selserial_receive_chars( sport );
   }

   // service modem control lines
   msr = ioread8( sport->port.membase + SELSERIAL_MSR );
   if( msr & (SELSERIAL_MSR_DDCD | SELSERIAL_MSR_DCTS |
              SELSERIAL_MSR_DRING | SELSERIAL_MSR_DDSR ) )
   {
      if( msr & SELSERIAL_MSR_DDCD )
      {
         uart_handle_dcd_change( &sport->port, !(msr & SELSERIAL_MSR_DCD) );
      }
      if( msr & SELSERIAL_MSR_DCTS )
      {
         uart_handle_cts_change( &sport->port, !(msr & SELSERIAL_MSR_CTS) );
      }
      if( msr & SELSERIAL_MSR_DRING )
      {
         sport->port.icount.rng++;
      }
      if( msr & SELSERIAL_MSR_DDSR )
      {
         sport->port.icount.dsr++;
      }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
      wake_up_interruptible( &sport->port.info->delta_msr_wait );
#else
      wake_up_interruptible( &sport->port.state->port.delta_msr_wait );
#endif
   }

   // finally, service the data transmit interrupts
   if( isr & (SELSERIAL_ISR_EOTT | SELSERIAL_ISR_TX_LEVEL) )
   {
      selserial_transmit_chars( &sport->port );
   }
}

//////////////////////////////////////////////////////////////////////////////
/// PCI init/exit functionality
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// @brief Initialize a selserial PCI device
///
/// @param priv Structure describing the PCI device
///
/// @return 0 for success, <0 for error
//////////////////////////////////////////////////////////////////////////////
int selserial_tty_initport( struct selserial_private *priv )
{
   int rc = 0;
   int i = 0;

   // let the user know what we are doing
   dev_info( &priv->dev->dev, "TTY: Registering Port Devices\n" );

   for( i = 0; i < priv->nr; ++i )
   {
      // add the file operations
      priv->ports[i].port.ops = &selserial_ops;

      // try and add it
      rc = uart_add_one_port(&selserial_uart_driver, &priv->ports[i].port);
      if(rc)
      {
         dev_err( &priv->dev->dev, "TTY: Error adding tty port\n");
         break;
      }
   }

   // error somewhere, so unregister what we have already done
   if( rc )
   {
      for( i = i-1; i >= 0; --i )
      {
         uart_remove_one_port( &selserial_uart_driver, &priv->ports[i].port );
      }
   }

   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief deinitializes the device
///
/// @param priv Structure describing the PCI device
//////////////////////////////////////////////////////////////////////////////
void selserial_tty_cleanupport( struct selserial_private *priv )
{
   int i;
   dev_info( &priv->dev->dev,"TTY: Unregistering Port Devices\n" );
   for( i = 0; i < priv->nr; ++i )
   {
      uart_remove_one_port( &selserial_uart_driver, &priv->ports[i].port );
   }
}

//////////////////////////////////////////////////////////////////////////////
/// Module init/exit functionality
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// @brief initialize the module by registering the devices
///
/// @return 0 for success, <0 for failure
//////////////////////////////////////////////////////////////////////////////
int __init selserial_tty_init( void )
{
   return uart_register_driver( &selserial_uart_driver );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Unregister the drivers.
//////////////////////////////////////////////////////////////////////////////
void selserial_tty_exit( void )
{
   // unregister drivers
   uart_unregister_driver( &selserial_uart_driver );
}

// vim:sm:et:sw=3:sts=3

