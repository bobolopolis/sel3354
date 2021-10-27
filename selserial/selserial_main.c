// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2008 Schweitzer Engineering Laboratories, Pullman, Washington
 *
 * Module entry point for the selserial driver
 */

#include <linux/module.h>
#include <linux/pci.h>

#include "selserial.h"
#include "selserial_common.h"
#include "linux/pci_ids_sel.h"

/* PCI functions */
static void selserial_remove_board(struct pci_dev *pdev);
static int selserial_init_board(struct pci_dev *pdev,
				const struct pci_device_id *ent);

/* PCI device table for the serial ports */
/*static struct pci_device_id selserial_pci_tbl[] =
{
	{ PCI_DEVICE(PCI_VENDOR_ID_SEL, PCI_DEVICE_ID_SEL_SERIAL), 0, 0, 0 },
	{ 0, }
};*/
static struct pci_device_id selserial_pci_tbl[] =
{
	{PCI_DEVICE(PCI_VENDOR_ID_SEL, PCI_DEVICE_ID_SEL_SERIAL)},
	{ }
};

/* structure to hold the driver information */
static struct pci_driver selserial_driver =
{
	.name     = "selserial",
	.id_table = selserial_pci_tbl,
	.probe    = selserial_init_board,
	.remove   = selserial_remove_board,
};
//	.remove   = __exit_p(selserial_remove_board),

/* Variable to handle multiple function enumeration */
static int selserial_port_count = 0;

/**
 * selserial_init_board() - Initialize a selserial PCI device
 *
 * @pdev: Pointer to the pci_device struct containing information about
 *        the device being initialized.
 * @ent: Entry in the pci_tbl that describes the device found
 *
 * Return: 0 for success, <0 for error
 */
static int selserial_init_board(struct pci_dev *pdev,
				const struct pci_device_id *ent)
{
	int i;
	int rc = 0;
	struct selserial_private *priv;
	struct uart_port *port;

	/* Status Message */
	dev_info(&pdev->dev, "Initializing PCI device");

	/* Enable the PCI device */
	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "Cannot enable PCI device");
		goto err_out;
	}

	/* Register the memory regions for the device */
	rc = pci_request_regions(pdev, "selserial");
	if (rc) {
		dev_err(&pdev->dev,
			"Cannot request memory regions from PCI device");
		goto err_out_disable;
	}

	/* Allocate the structure to hold the pointer to our ioremapped
	 * memory pci subdevice ID contains the number of serial ports
	 * provided by the function. */
	priv = kzalloc(sizeof(struct selserial_private) +
		       sizeof(struct selserial_uart_port) *
		       (pdev->subsystem_device & 0xff), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "Cannot allocate space for private data");
		rc = -ENOMEM;
		goto err_out_release;
	}

	/* Fill in said structure */
	priv->dev = pdev;
	priv->nr = pdev->subsystem_device & 0xff;
	priv->irq_list.head = NULL;
	spin_lock_init(&priv->irq_list.lock);
	// TODO
	//priv->remapped_bar = ioremap(pci_resource_start(pdev, 0),
	//                              pci_resource_len(pdev, 0));
	priv->remapped_bar = pci_iomap(pdev, 0, pci_resource_len(pdev, 0));
	if(!priv->remapped_bar) {
		dev_err(&pdev->dev, "Cannot remap memory regions");
		rc = -ENOMEM;
		goto err_out_mem;
	}

	/* Allocate all the ports */
	for(i = 0; i < priv->nr; ++i) {
		priv->ports[i].pdev = pdev;
		priv->ports[i].port_type = SELSERIAL_PORT_TYPE_UNOPENED;
		port = &priv->ports[i].port;

		/* Initialize the spin lock */
		spin_lock_init(&port->lock);

		/* fill in all the goodies */
		port->type = PORT_SELSERIAL;
		port->line = i + selserial_port_count;
		port->iotype = UPIO_MEM;
		port->iobase = 0;
		port->mapbase = pci_resource_start(pdev, 0) +
				i * SELSERIAL_PORT_REG_SIZE;
		port->membase = priv->remapped_bar +
				i * SELSERIAL_PORT_REG_SIZE;
		port->regshift = 0;
		port->fifosize = SELSERIAL_HW_FIFO_SIZE;
		port->uartclk = SELSERIAL_UART_CLOCK_FREQ;
		port->irq = pdev->irq;
		port->flags = UPF_LOW_LATENCY | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
		port->cons = NULL;

		/* prime the GCR */
		priv->ports[i].gcr = ioread8(port->membase + SELSERIAL_GCR);

	}

	/* Initialize the tty devices */
	rc = selserial_tty_initport(priv);
	if (rc) {
		goto err_out_add_ttyport;
	}

	/* Special feature initialization (485/232, loopback, power sysfs) */
	rc = selserial_special_init(&pdev->dev);
	if (rc) {
		goto err_out_all;
	}

	/* Count the ports as good */
	selserial_port_count += priv->nr;

	/* Save the pointer to our private data */
	pci_set_drvdata(pdev, priv);

	/* Status Message */
	dev_info(&pdev->dev, "Initialized PCI device");

	return 0;

err_out_all:
	selserial_tty_cleanupport(priv);
err_out_add_ttyport:
	iounmap(priv->remapped_bar);
err_out_mem:
	kfree(priv);
err_out_release:
	pci_release_regions(pdev);
err_out_disable:
	pci_disable_device(pdev);
err_out:
	return rc;
}

/**
 * selserial_remove_board() - Deinitializes the device
 *
 * @pdev: Pointer to the device struct describing the port
 */
static void selserial_remove_board(struct pci_dev *pdev)
{
	struct selserial_private *priv;

	/* Status to user */
	dev_info(&pdev->dev, "Releasing PCI device");

	/* Get the private data and clear the pointer */
	priv = pci_get_drvdata(pdev);
	pci_set_drvdata(pdev, NULL);

	/* special feature cleanup */
	selserial_special_cleanup(&pdev->dev);

	/* Cleanup the tty stuff */
	selserial_tty_cleanupport(priv);

	/* Unmap the bar */
	iounmap(priv->remapped_bar);

	/* Free the private memory */
	kfree(priv);

	/* Release the memory/io regions */
	pci_release_regions(pdev);

	/* Disable the device */
	pci_disable_device(pdev);

	/* Status to user */
	dev_info(&pdev->dev, "Released PCI device");
}

/**
 * selserial_init() - Initialize the module
 *
 * Return: 0 for success, <0 for failure
 */
static int __init selserial_init(void)
{
	int rc;

	selserial_port_count = 0;

	// TODO: unify printk/dev_*/pr_* calls
	printk(KERN_INFO "SELserial: Module Initializing");

	/* Initialize the TTY interface to the ports. Have to do this here
	 * as opposed to initport becuase the TTY driver must register all
	 * possible TTY devices that could ever be added, whereas the RAW
	 * and MB ports can just reserve a subset of the minor numbers */
	rc = selserial_tty_init();
	if (rc) {
		printk(KERN_INFO "selserial: selserial_tty_init error: %d", rc);
		goto init_err_tty;
	}

	/* Initialize the PCI device. This MUST be after the tty init
	 * becuase this will call the probe() function before returning. */
	rc = pci_register_driver(&selserial_driver);
	if (rc) {
		printk(KERN_INFO "selserial: pci_register_driver error: %d", rc);
		goto init_err_pci;
	}

	printk(KERN_INFO "SELserial: Successfully Initialized");

	return rc;

init_err_pci:
	selserial_tty_exit();
init_err_tty:
	return rc;
}

/**
 * selserial_exit() - Unregister the module
 */
static void __exit selserial_exit(void)
{
	/* Status Message */
	printk(KERN_INFO "SELserial: Unloading driver");

	/* Unregister the pci driver */
	pci_unregister_driver(&selserial_driver);

	/* Unregister the tty */
	selserial_tty_exit();

	printk(KERN_INFO "SELserial: Driver Unloaded");
}

module_init(selserial_init);
module_exit(selserial_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Schweitzer Engineering Laboratories, Inc.");
MODULE_DESCRIPTION("Driver to handle communication to the SEL UART");
MODULE_VERSION("1.0.0");
MODULE_DEVICE_TABLE(pci, selserial_pci_tbl);
