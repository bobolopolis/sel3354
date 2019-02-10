//////////////////////////////////////////////////////////////////////////////
// COPYRIGHT (c) 2008 Schweitzer Engineering Laboratories, Pullman, Washington
///
/// @file selserial_common.c
///
/// @brief common routines used by the serial port driver
//////////////////////////////////////////////////////////////////////////////
#include "selserial.h"
#include "selserial_common.h"

#include <linux/delay.h>


//////////////////////////////////////////////////////////////////////////////
/// @brief Common IOCTLs for the serial port
///
/// @param sport Serial Port to handle the IOCTL on
/// @param cmd IOCTL command to manage
/// @param arg Argument to the IOCTL command
///
/// @return 0 for successful handle, -ENOIOCTLCMD if we don't support it,
///         another error code if there was an error.
//////////////////////////////////////////////////////////////////////////////
long selserial_common_ioctl( struct selserial_uart_port *sport,
                             unsigned int cmd,
                             unsigned long arg )
{
   long rc = 0;
   unsigned long flags = 0;

   switch( cmd )
   {
      // set the timed transmit delay counter and next character bits.
      // We don't need to lock here becuase the FPGA takes care of
      // synchronization at this point
      case SELSERIAL_IOCSTIMETX:
      {
         struct selserial_timed_tx_settings timetxset;

         // read in from user space
         if( copy_from_user( &timetxset,
                             (void __user *)arg,
                             _IOC_SIZE(cmd) ) )
         {
            rc = -EFAULT;
            break;
         }

         // set the counter
         sport->tcr = timetxset.count;
         iowrite8( sport->tcr, sport->port.membase + SELSERIAL_TM_COUNT_REG );

         // set the flags -- this has to be done after the iowrite for
         // the counter to avoid needing a lock
         sport->timedchar = timetxset.flags;

         break;
      }

      // get the current delay value in the timed transmit register and the
      // next character bits. The reads are atomic so we don't need to lock.
      case SELSERIAL_IOCGTIMETX:
      {
         struct selserial_timed_tx_settings timetxset;

         // get the values
         timetxset.flags = sport->timedchar;
         timetxset.count = sport->tcr;

         // copy to userland
         if( copy_to_user( (void __user *)arg,
                           &timetxset,
                           _IOC_SIZE(cmd) ) )
         {
            rc = -EFAULT;
         }
         break;
      }

      // Set the interrupt values for the fifos.  This is port settings so we
      // need to do the work under the port.lock and it affects interrupts
      // so we need to use the _irq* variant to protect against that.
      case SELSERIAL_IOCSFIFOINT:
      {
         struct selserial_fifo_int_settings fifoset;

         if( copy_from_user( &fifoset,
                             (void __user *)arg,
                             _IOC_SIZE(cmd) ) )
         {
            rc = -EFAULT;
            break;
         }

         // write the values out to the uart
         spin_lock_irqsave( &sport->port.lock, flags );

         selserial_common_setup_fifo_interrupt( sport,
                                                fifoset.rxcount,
                                                fifoset.txcount,
                                                fifoset.eort,
                                                fifoset.eott );

         // if we set an eott then enable the interrupt
         if( sport->fifo_settings.eott != 0 )
         {
            sport->ier |= SELSERIAL_IER_EOTT;
         }
         else
         {
            sport->ier &= ~(SELSERIAL_IER_EOTT);
         }

         // XXX: This is temporary.  MB will eventually probably move to a
         //      traditional interrupt service routine.  When it does then
         //      just remove this if statement to get this working again.
         //      This COULD be done by moving this case out of COMMON, but
         //      it's more work to move it back in than just removing a line
         //      or two
         if( sport->port_type == SELSERIAL_PORT_TYPE_MB ) sport->ier = 0;

         // set the ier
         iowrite16( sport->ier, sport->port.membase + SELSERIAL_IER );

         spin_unlock_irqrestore( &sport->port.lock, flags );

         break;
      }

      // get the current fifo interrupt settings.  This needs to be done under
      // the port lock to make it atomic, but since it is a read we don't need
      // the irq* varients
      case SELSERIAL_IOCGFIFOINT:
      {
         spin_lock( &sport->port.lock );
         if( copy_to_user( (void __user *)arg,
                           &sport->fifo_settings,
                           _IOC_SIZE(cmd) ) )
         {
            rc = -EFAULT;
         }
         spin_unlock( &sport->port.lock );
         break;
      }

      // change the AUTO_RTS mode for the port.  This affects settings and how
      // the interrupts work, so it needs to be done under the port lock with
      // disabled interrupts
      case SELSERIAL_IOCSAUTORTS:
      {
         u32 autorts;

         // get flag from user space
         if( copy_from_user( &autorts,
                             (void __user *)arg,
                             _IOC_SIZE(cmd) ) )
         {
            rc = -EFAULT;
            break;
         }

         // save the mode
         if( autorts )
         {
            sport->mcr |= SELSERIAL_MCR_AUTO_RTS;
         }
         else
         {
            sport->mcr &= ~(SELSERIAL_MCR_AUTO_RTS);
         }

         // write the values out to the uart
         spin_lock_irqsave( &sport->port.lock, flags );
         iowrite8( sport->mcr, sport->port.membase + SELSERIAL_MCR );
         spin_unlock_irqrestore( &sport->port.lock, flags );

         break;
      }

      // Get the current port power state
      case SELSERIAL_IOCGPOWERSTATE:
      {
         u32 powerst = !!(sport->gcr & SELSERIAL_GCR_PORT_POWER);
         if( copy_to_user( (void __user *)arg,
                           &powerst,
                           _IOC_SIZE(cmd) ) )
         {
            rc = -EFAULT;
         }
         break;
      }

      // set the port power state for this port
      case SELSERIAL_IOCSPOWERSTATE:
      {
         u32 powerst;

         // get flag from user space
         if( copy_from_user( &powerst,
                             (void __user *)arg,
                             _IOC_SIZE(cmd) ) )
         {
            rc = -EFAULT;
            break;
         }

         if( (powerst != '0') && (powerst != '1') )
         {
            rc = -EINVAL;
            break;
         }

         // set the mode
         __selserial_special_power_store( sport, (char)powerst );

         break;
      }

      // get the current 485 state
      case SELSERIAL_IOCG485STATE:
      {
         u32 mode = (sport->gcr & SELSERIAL_GCR_485_MODE) ?
                        ((sport->auto485) ? (u32)('a') : (u32)('1')) :
                        (u32)('0');
         if( copy_to_user( (void __user *)arg,
                           &mode,
                           _IOC_SIZE(cmd) ) )
         {
            rc = -EFAULT;
         }
         break;
      }

      // set the 485 state for this port
      case SELSERIAL_IOCS485STATE:
      {
         u32 mode;

         // get flag from user space
         if( copy_from_user( &mode,
                             (void __user *)arg,
                             _IOC_SIZE(cmd) ) )
         {
            rc = -EFAULT;
            break;
         }

         // check modes
         if( (mode != '0') && (mode != '1') && (mode != 'a') )
         {
            rc = -EINVAL;
            break;
         }

         // lock the port
         spin_lock( &sport->port.lock );

         // set the mode
         __selserial_special_485_store( sport, (char)mode );

         // unlock the port
         spin_unlock( &sport->port.lock );

         break;
      }

      // Support for flushing the hardware buffers.
      case TCFLSH:
      {
         spin_lock( &sport->port.lock );
         switch( arg )
         {
            case TCIFLUSH:
               iowrite32( sport->lcr |
                            SELSERIAL_LCR_RX_FLUSH,
                          sport->port.membase + SELSERIAL_LCR );
               break;

            case TCOFLUSH:
               iowrite32( sport->lcr |
                            SELSERIAL_LCR_TX_FLUSH,
                          sport->port.membase + SELSERIAL_LCR );
               break;

            case TCIOFLUSH:
               iowrite32( sport->lcr |
                            SELSERIAL_LCR_TX_FLUSH |
                            SELSERIAL_LCR_RX_FLUSH,
                          sport->port.membase + SELSERIAL_LCR );
               break;

            default:
               rc = -EINVAL;
               break;
         }
         rc = -ENOIOCTLCMD;
         spin_unlock( &sport->port.lock );
         break;
      }

      // not a recognized command
      default:
         rc = -ENOIOCTLCMD;
         break;
   }

   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the terminal settings in the lcr and mcr
///
/// @param sport Pointer to the port structure to change
/// @param new New terminal settings to apply
/// @param old Terminal settings that are in place before this call
///
/// @remarks port.lock should already be taken
//////////////////////////////////////////////////////////////////////////////
void selserial_common_set_term_settings( struct selserial_uart_port *sport,
                                         struct ktermios *new,
                                         struct ktermios *old )
{
   u16 baud_div;
   u8 baud_frac;
   unsigned int baud;

   // get the baud rate
   baud = uart_get_baud_rate( &sport->port, new, old, 0, 115200 );

   // clear the TERMIOS settings
   sport->lcr &= SELSERIAL_LCR_SET_TERMIOS_RESET;

   // Set the number of data bits
   switch( new->c_cflag & CSIZE )
   {
      case CS5:
      {
         sport->lcr |= SELSERIAL_LCR_5DB;
         break;
      }
      case CS6:
      {
         sport->lcr |= SELSERIAL_LCR_6DB;
         break;
      }
      case CS7:
      {
         sport->lcr |= SELSERIAL_LCR_7DB;
         break;
      }
      case CS8:
      default:
      {
         sport->lcr |= SELSERIAL_LCR_8DB;
         break;
      }
   }

   // check for stop bits
   if( new->c_cflag & CSTOPB )
   {
      sport->lcr |= SELSERIAL_LCR_TWO_STOP;
   }

   // parity enable
   if( new->c_cflag & PARENB )
   {
      sport->lcr |= SELSERIAL_LCR_PARITY_EN;
   }

   // parity even/odd
   if(!(new->c_cflag & PARODD))
   {
      sport->lcr |= SELSERIAL_LCR_EVEN_PARITY;
   }

#ifdef CMSPAR
   // sticky parity
   if( new->c_cflag & CMSPAR )
   {
      sport->lcr |= SELSERIAL_LCR_STICKY_PARITY;
   }
#endif

   // update the port timeout
   uart_update_timeout( &sport->port, new->c_cflag, baud );

   // the math for this is taken from the CIS for the serial ports.  See that
   // for explanation
   baud_div = ( sport->port.uartclk / ( 16 * baud )) - 1;
   baud_frac = (((2 * (sport->port.uartclk % ( 16 * baud ))) / baud ) + 1) / 2;
   sport->lcr = SELSERIAL_SET_BAUD_DIV( sport->lcr, baud_div );
   sport->lcr = SELSERIAL_SET_FRAC_BAUD( sport->lcr, baud_frac );

   // check for auto HW flow control
   if( new->c_cflag & CRTSCTS )
   {
      sport->mcr |= SELSERIAL_MCR_AUTO_HS_RTS | SELSERIAL_MCR_AUTO_HS_CTS;
   }
   else
   {
      sport->mcr &= ~(SELSERIAL_MCR_AUTO_HS_RTS | SELSERIAL_MCR_AUTO_HS_CTS);
   }

   // check for auto HW flow control
   if( new->c_cflag & CREAD )
   {
      sport->lcr |= SELSERIAL_LCR_RX_EN;
   }
   else
   {
      sport->lcr &= ~(SELSERIAL_LCR_RX_EN);
   }

   // write the new MCR and LCR values
   iowrite8(  sport->mcr, sport->port.membase + SELSERIAL_MCR );
   iowrite32( sport->lcr, sport->port.membase + SELSERIAL_LCR );
}



// vim:sm:et:sw=3:sts=3

