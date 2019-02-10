//////////////////////////////////////////////////////////////////////////////
// COPYRIGHT (c) 2008 Schweitzer Engineering Laboratories, Pullman, Washington
///
/// @file selserial_irq.c
///
/// @brief interrupt routine for the selserial driver
//////////////////////////////////////////////////////////////////////////////
#include "selserial.h"
#include "selserial_reg.h"

#include <linux/pci.h>
#include <linux/device.h>
#include <asm/io.h>

//////////////////////////////////////////////////////////////////////////////
/// @brief Main interrupt routine for the selserial driver
///
/// @param irq IRQ number that fired
/// @param dev_id Private data for the handler
///
/// @return 1 for handled or 0 for not handled
//////////////////////////////////////////////////////////////////////////////
static irqreturn_t selserial_interrupt( int irq, void *dev_id )
{
   struct irq_info *i = dev_id;
   struct list_head *listhead;
   struct list_head *end = NULL;
   int handled = 0;

   // lock the list so it doesn't get changed under us
   spin_lock( &i->lock );

   // loop through each port registered to check for interrupts.
   listhead = i->head;
   do
   {
      struct selserial_uart_port *sport;
      u16 tmp_isr;

      // get the list item.
      sport = list_entry( listhead, struct selserial_uart_port, list );

      // check for Interrupts on this port
      tmp_isr = ioread16( sport->port.membase + SELSERIAL_ISR );
      tmp_isr &= sport->ier;

      if( tmp_isr != 0 )
      {
         // lock the port lock to prevent settings change
         spin_lock( &sport->port.lock );
	 switch( sport->port_type )
         {
            case SELSERIAL_PORT_TYPE_TTY:
            {
               selserial_tty_handle_port( sport, tmp_isr );
               break;
            }
            default:
            {
               break;
            }
	 };
         spin_unlock( &sport->port.lock );

         handled = 1;
         end = NULL;
      }
      else if (end == NULL) {
         end = listhead;
      }

      // get the next item
      listhead = listhead->next;

   } while( listhead != end );

   // unlock the list
   spin_unlock( &i->lock );

   return IRQ_RETVAL( handled );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Add the port to the irq service list and request the IRQ if
///        necessary
///
/// @param irq IRQ number to register
/// @param i   pointer to the IRQ info structure to update
/// @param list pointer to the port list to add
///
/// @return 0 on success, non-zero on failure
//////////////////////////////////////////////////////////////////////////////
int selserial_register_irq_list( int irq,
                                 struct irq_info *i,
                                 struct list_head *list )
{
   int rc = 0;

   // Lock the IRQ list spin lock to avoid races
   spin_lock_irq( &i->lock );

   // If this isn't the first port to enable then just add it to the list
   if( i->head )
   {
      list_add( list, i->head );
   }
   // Otherwise, initialize the list head and request the irq
   else
   {
      INIT_LIST_HEAD( list );
      i->head = list;

      rc = request_irq( irq, selserial_interrupt,
                        IRQF_SHARED, "selserial", i );
      if( rc < 0 )
      {
         i->head = NULL;
      }
   }

   // all done
   spin_unlock_irq( &i->lock );

   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Remove the port from the irq service list and release the IRQ if
///        necessary
///
/// @param irq IRQ number to unregister
/// @param i   pointer to the IRQ info structure to update
/// @param list pointer to the port list to remove
///
/// @return 0 on success, non-zero on failure
//////////////////////////////////////////////////////////////////////////////
void selserial_unregister_irq_list( int irq,
                                    struct irq_info *i,
                                    struct list_head *list )
{
   // lock the list to prevent races
   spin_lock_irq( &i->lock );

   // delete the node
   if( !list_empty( i->head ) )
   {
      if( i->head == list )
      {
         i->head = i->head->next;
      }
      list_del( list );
   }
   else
   {
      free_irq(irq, i);
      i->head = NULL;
   }

   // unlock the list
   spin_unlock_irq( &i->lock );
}

// vim:sm:et:sw=3:sts=3

