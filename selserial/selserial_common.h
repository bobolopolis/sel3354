//////////////////////////////////////////////////////////////////////////////
// COPYRIGHT (c) 2008 Schweitzer Engineering Laboratories, Pullman, Washington
///
/// @file selserial_common.h
///
/// @brief common routines used by the serial port driver
//////////////////////////////////////////////////////////////////////////////
#ifndef _SELSERIAL_COMMON_H_INCLUDED
#define _SELSERIAL_COMMON_H_INCLUDED

#include "selserial.h"

#include <linux/io.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/termios.h>

//////////////////////////////////////////////////////////////////////////////
// Common functions defined in selserial_common.c -- documented there

extern void selserial_common_set_term_settings (
      struct selserial_uart_port *sport,
      struct ktermios *new,
      struct ktermios *old );

extern long selserial_common_ioctl(
      struct selserial_uart_port *sport,
      unsigned int cmd,
      unsigned long arg );

extern long selserial_common_terminal_ioctl (
      struct selserial_uart_port *sport,
      unsigned int cmd,
      unsigned long arg );

extern void selserial_common_wait_until_sent (
      struct selserial_uart_port *sport );

//////////////////////////////////////////////////////////////////////////////
/// @brief return the state of the modem control pins
///
/// @param sport Port to retrieve pin state from
///
/// @return Bit field containing state of the pins
///
/// @remarks No locking needed for this function, and lock should not be taken
//////////////////////////////////////////////////////////////////////////////
static inline unsigned int selserial_common_get_mctrl(
      struct selserial_uart_port *sport )
{
   unsigned int ret = 0;
   u8 msr = 0;

   // get the modem status register from the uart
   msr = ioread8( sport->port.membase + SELSERIAL_MSR );

   // set ret to the appropriate state
   if( !(msr & SELSERIAL_MSR_CTS) )
   {
      ret |= TIOCM_CTS;
   }
   if( !(msr & SELSERIAL_MSR_DSR) )
   {
      ret |= TIOCM_DSR;
   }
   if( !(msr & SELSERIAL_MSR_RING) )
   {
      ret |= TIOCM_RI;
   }
   if( !(msr & SELSERIAL_MSR_DCD) )
   {
      ret |= TIOCM_CAR;
   }

   return ret;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the modem control lines.
///
/// @param sport Port to set the modem control lines on
/// @param mctrl Bit field describing modem control lines to set
///
/// @remarks TIOCM_OUT1 and TIOCM_OUT2 are ignored becuase the HW doesn't
///          implement them.
///
///          No locking needed for this function, and lock should not be taken
//////////////////////////////////////////////////////////////////////////////
static inline void selserial_common_set_mctrl(
      struct selserial_uart_port *sport,
      unsigned int mctrl )
{
   // prevent the application from controlling RTS in auto485 mode
   if( sport->auto485 )
   {
      // do nothing
      return;
   }

   // clear the RTS/DTR bits in the mcr locally to start with
   sport->mcr &= ~(SELSERIAL_MCR_DTR | SELSERIAL_MCR_RTS);

   // now set them if we are supposed to
   if( !(mctrl & TIOCM_RTS) )
   {
      sport->mcr |= SELSERIAL_MCR_RTS;
   }
   if( !(mctrl & TIOCM_DTR) )
   {
      sport->mcr |= SELSERIAL_MCR_DTR;
   }

   // write it out
   iowrite8( sport->mcr, sport->port.membase + SELSERIAL_MCR );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the fifo interrupt settings.
///
/// @param sport Pointer to the port structure to use to set the settings
/// @param rxlvl Number of bytes to use as the high-water mark for the
///              receive fifo
/// @param txlvl Number of bytes to use as the low-water mark for the
///              transmit fifo
/// @param eort  End-of-receive timer count in bit-times.
/// @param eott  End-of-transmit timer count in bit-times.
///
/// @remarks If rxlvl, txlvl, or eort are passed in as zero, they are set to
///          the default level.  This is done so we can't configure the
///          UART to be unresponsive.
///          If eott is passed in as zero, it is set to the default.  This is
///          done for autorts (ie auto485) and other fpga needs per Jerry.
//////////////////////////////////////////////////////////////////////////////
static inline void selserial_common_setup_fifo_interrupt(
      struct selserial_uart_port *sport,
      u16 rxlvl,
      u16 txlvl,
      u16 eort,
      u16 eott )
{
   sport->fifo_settings.eort = (eort ? eort : GET_DEFAULT_EORT(8));
   sport->fifo_settings.eott = (eott ? eott : SELSERIAL_DEFAULT_EOXT_TIMER);
   sport->fifo_settings.rxcount = (rxlvl ? rxlvl :
                                           SELSERIAL_DEFAULT_COUNT_LEVEL);
   sport->fifo_settings.txcount = (txlvl ? txlvl :
                                           SELSERIAL_DEFAULT_COUNT_LEVEL);
   iowrite32( (sport->fifo_settings.eott << 16) |
               sport->fifo_settings.eort,
              sport->port.membase + SELSERIAL_EORT_REG );
   iowrite32( (sport->fifo_settings.txcount << 16) |
               sport->fifo_settings.rxcount,
              sport->port.membase + SELSERIAL_RX_FIFO_LEVEL_REG );
}

//////////////////////////////////////////////////////////////////////////////
/// @remarks Register for the IRQ handler and clear any pending interrupts
///
/// @param sport Pointer to the serial port structure to register
///
/// @return 0 for success, or an error code if there was an error registering
//////////////////////////////////////////////////////////////////////////////
static inline int selserial_common_register_and_clear_irq(
      struct selserial_uart_port *sport )
{
   int rc;
   struct irq_info *i =
      &(((struct selserial_private *)(pci_get_drvdata( sport->pdev )))->
         irq_list);

   // register
   rc = selserial_register_irq_list( sport->port.irq, i, &sport->list );

   // clear
   if( !rc )
      ioread16( sport->port.membase + SELSERIAL_ISR );

   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Disable interrupts for the port, unregister from the handler, and
///        clear any pending interrupts
///
/// @param sport Pointer to the serial port to unregister
//////////////////////////////////////////////////////////////////////////////
static inline void selserial_common_disable_and_unregister_irq(
      struct selserial_uart_port *sport )
{
   struct irq_info *i =
      &(((struct selserial_private *)(pci_get_drvdata( sport->pdev )))->
         irq_list);

   // disable the interrupt and clear the pending interrupts
   sport->ier = 0;
   iowrite16( 0, sport->port.membase + SELSERIAL_IER );
   ioread16( sport->port.membase + SELSERIAL_ISR );

   // unregister the IRQ
   selserial_unregister_irq_list( sport->port.irq, i, &sport->list );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief helper function to handle writing to the transmit fifo
///
/// @param sport selserial port structure of the port to write to.
/// @param data byte to write
///
/// @remarks port.lock should already be taken at this point so don't take
///          it again.
//////////////////////////////////////////////////////////////////////////////
static inline void selserial_common_txfifo_write(
      struct selserial_uart_port *sport, u8 data )
{
   // write the RTS bit if auto485 is set
   if( sport->auto485 )
   {
      iowrite8( sport->mcr & ~(SELSERIAL_MCR_RTS),
                sport->port.membase + SELSERIAL_MCR );
   }

   // write the byte with the current timedchar bits
   iowrite16( ((sport->timedchar) | data),
              sport->port.membase + SELSERIAL_TX );

   // clear the timedchar bits
   sport->timedchar = 0;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Reserve the port in the name of the port passed in.
///
/// @param sport Pointer to the serial port structure to reserve for.
/// @param type Port Type that is reserving
///
/// @return -EBUSY if we can't reserve the port, 0 if we successfully
///         reserved it.
//////////////////////////////////////////////////////////////////////////////
static inline int selserial_common_reserve_type(
      struct selserial_uart_port *sport, int type )
{
   int rc = -EBUSY;

   spin_lock( &sport->port.lock );
   if( sport->port_type == SELSERIAL_PORT_TYPE_UNOPENED )
   {
      sport->port_type = type;
      rc = 0;
   }
   spin_unlock( &sport->port.lock );

   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Unreserve the port
///
/// @param sport Pointer to the serial port structure to unreserve for.
/// @param type Port Type that is unreserving
//////////////////////////////////////////////////////////////////////////////
static inline void selserial_common_unreserve_type(
      struct selserial_uart_port *sport, int type )
{
   spin_lock( &sport->port.lock );
   BUG_ON( sport->port_type != type );
   sport->port_type = SELSERIAL_PORT_TYPE_UNOPENED;
   spin_unlock( &sport->port.lock );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Read function that is used after someone calls close on the port
///
/// @param file File structure of the open file
/// @param buf Buffer to read into
/// @param cnt Number of bytes to read
/// @param offset Offset to apply to the read data.
///
/// @return always 0 for the HUP call
//////////////////////////////////////////////////////////////////////////////
static inline ssize_t selserial_common_read_hup( struct file *file,
                                                 char __user *buf,
                                                 size_t cnt,
                                                 loff_t *offset )
{
   return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Write function that is used after someone calls close on the port
///
/// @param file File structure of the open file
/// @param buf Buffer to write from
/// @param cnt Number of bytes to write
/// @param offset Offset to apply to the write data.
///
/// @return always -EIO for the HUP call
//////////////////////////////////////////////////////////////////////////////
static inline ssize_t selserial_common_write_hup( struct file *file,
                                                  const char __user *buf,
                                                  size_t cnt,
                                                  loff_t *offset )
{
   return -EIO;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Poll function that is used after someone calls close on the port
///
/// @param file File structure of the open file
/// @param pts Poll table structure passed to poll_wait
///
/// @return HUP poll returns all events
//////////////////////////////////////////////////////////////////////////////
static inline unsigned int selserial_common_poll_hup(
      struct file *file,
      struct poll_table_struct *pts )
{
   return POLLIN | POLLOUT | POLLERR |
          POLLHUP | POLLRDNORM | POLLWRNORM | POLLMSG;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Poll function that is used after someone calls close on the port
///
/// @param file File structure of the open file
/// @param cmd IOCTL command to execute
/// @param arg Argument to the IOCTL command
///
/// @return HUP ioctl always returns -EIO
//////////////////////////////////////////////////////////////////////////////
static inline long selserial_common_ioctl_hup( struct file *file,
                                               unsigned int cmd,
                                               unsigned long arg )
{
   return -EIO;
}


#define selserial_common_init_stale_data_timer(priv)
#define selserial_common_kick_stale_data_timer(sport)

#endif

// vim:sm:et:sw=3:sts=3

