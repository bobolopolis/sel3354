// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2008 Schweitzer Engineering Laboratories, Pullman, Washington
 * 
 * Interrupt routine for the selserial driver
 */

#include <linux/pci.h>
#include <linux/device.h>
#include <asm/io.h>

#include "selserial.h"
#include "selserial_reg.h"


/**
 * selserial_interrupt() - Main interrupt service routine
 *
 * @irq: IRQ number that fired
 * @dev_id: Private data for the handler
 *
 * Return: 1 for handled, 0 for not handled
 */
static irqreturn_t selserial_interrupt(int irq, void *dev_id)
{
	struct irq_info *i = dev_id;
	struct list_head *listhead;
	struct list_head *end = NULL;
	int handled = 0;

	/* Lock the list so it doesn't get changed under us */
	spin_lock( &i->lock );

	/* Loop through each port registered to check for interrupts */
	listhead = i->head;
	do {
		struct selserial_uart_port *sport;
		u16 tmp_isr;

		/* Get the list item */
		sport = list_entry(listhead, struct selserial_uart_port, list);

		/* Check for Interrupts on this port */
		tmp_isr = ioread16(sport->port.membase + SELSERIAL_ISR);
		tmp_isr &= sport->ier;

		if (tmp_isr != 0) {
			/* Lock the port lock to prevent settings change */
			spin_lock(&sport->port.lock);
			switch(sport->port_type) {
			case SELSERIAL_PORT_TYPE_TTY:
				selserial_tty_handle_port(sport, tmp_isr);
				break;
			default:
				break;
			}
			spin_unlock(&sport->port.lock);

			handled = 1;
			end = NULL;
		} else if (end == NULL) {
			end = listhead;
		}

		/* Get the next item */
		listhead = listhead->next;
	} while(listhead != end);

	/* Unlock the list */
	spin_unlock(&i->lock);

	return IRQ_RETVAL(handled);
}

/**
 * selserial_register_irq_list() - Add a port to the IRQ service list
 *
 * @irq: IRQ number to register
 * @info: Pointer to the IRQ info struct to update
 * @list: Pointer to the port list to add
 *
 * Return: 0 on success, non-zero on failure
 */
int selserial_register_irq_list(int irq,
				struct irq_info *info,
				struct list_head *list)
{
	int rc = 0;

	/* Lock the IRQ list spin lock to avoid races */
	spin_lock_irq(&info->lock);

	/* If this isn't the first port to enable then just add it to the
	 * list */
	if (info->head) {
		list_add(list, info->head);
	} else {
		/* Otherwise, initialize the list head and request the irq */
		INIT_LIST_HEAD(list);
		info->head = list;

		rc = request_irq(irq, selserial_interrupt, IRQF_SHARED,
				 "selserial", info);
		if (rc < 0) {
			info->head = NULL;
		}
	}

	/* All done */
	spin_unlock_irq(&info->lock);

	return rc;
}

/**
 * selserial_unregister_irq_list() - Remove a port from the IRQ service list
 *
 * @irq: IRQ number to unregister
 * @info: Pointer to the IRQ info struct to update
 * @list: Pointer to the port list to remove
 *
 * Return: 0 on success, non-zero on failure
 */
void selserial_unregister_irq_list(int irq,
				   struct irq_info *info,
				   struct list_head *list)
{
	/* Lock the list to prevent races */
	spin_lock_irq(&info->lock);

	/* Delete the node */
	if (!list_empty(info->head)) {
		if (info->head == list) {
			info->head = info->head->next;
		}
		list_del(list);
	} else {
		free_irq(irq, info);
		info->head = NULL;
	}

	/* Unlock the list */
	spin_unlock_irq(&info->lock);
}
