//////////////////////////////////////////////////////////////////////////////
// COPYRIGHT (c) 2008 Schweitzer Engineering Laboratories, Pullman, Washington
///
/// @file selserial_main.c
///
/// @brief Module entry point for the selserial driver
//////////////////////////////////////////////////////////////////////////////

#include "selserial.h"
#include "selserial_common.h"
#include "linux/pci_ids_sel.h"

#include <linux/module.h>
#include <linux/pci.h>

// PCI functions
static void selserial_remove_board( struct pci_dev *pdev );
static int selserial_init_board( struct pci_dev *pdev,
                                 const struct pci_device_id *ent );

//////////////////////////////////////////////////////////////////////////////
/// Static variable declaration

/// PCI device table for the serial ports
static struct pci_device_id selserial_pci_tbl[] =
{
   { PCI_DEVICE(PCI_VENDOR_ID_SEL, PCI_DEVICE_ID_SEL_SERIAL), 0, 0, 0 },
   { 0, }
};

/// structure to hold the driver information
static struct pci_driver selserial_driver =
{
   .name           = "selserial",
   .id_table       = selserial_pci_tbl,
   .probe          = selserial_init_board,
   .remove         = __exit_p(selserial_remove_board),
};

/// Variable to handle multiple function enumeration
static int selserial_port_count = 0;

//////////////////////////////////////////////////////////////////////////////
/// PCI init/exit functionality
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// @brief Initialize a selserial PCI device
///
/// @param pdev pointer to the pci_device structure containing information
///             about the device being initialized
/// @param ent  entry in the pci_tbl that describes the device found
///
/// @return 0 for success, <0 for error
//////////////////////////////////////////////////////////////////////////////
static int selserial_init_board( struct pci_dev *pdev,
                                 const struct pci_device_id *ent )
{
   int i;
   int rc = 0;
   struct selserial_private *priv;
   struct uart_port *port;

   // Status Message
   dev_info( &pdev->dev, "Initializing PCI device\n" );

   // Enable the PCI device
   rc = pci_enable_device(pdev);
   if(rc)
   {
      dev_err(&pdev->dev, "Cannot enable PCI device\n");
      goto err_out;
   }

   // register the memory regions for the device
   rc = pci_request_regions(pdev, "selserial");
   if(rc)
   {
      dev_err(&pdev->dev, "Cannot request memory regions from PCI device\n");
      goto err_out_disable;
   }

   // Allocate the structure to hold the pointer to our ioremapped memory
   // pci subdevice ID contains the number of serial ports provided by the
   // function.
   priv = kzalloc( sizeof(struct selserial_private) +
                   sizeof(struct selserial_uart_port) * (pdev->subsystem_device & 0xff),
                   GFP_KERNEL );
   if( !priv )
   {
      dev_err(&pdev->dev, "Cannot allocate space for private data\n" );
      rc = -ENOMEM;
      goto err_out_release;
   }

   // fill in said structure
   priv->dev = pdev;
   priv->nr = (pdev->subsystem_device & 0xff);
   priv->irq_list.head = NULL;
   spin_lock_init( &priv->irq_list.lock );
   priv->remapped_bar = ioremap_nocache( pci_resource_start(pdev, 0),
                                         pci_resource_len(pdev, 0) );
   if( !priv->remapped_bar )
   {
      dev_err(&pdev->dev, "Cannot remap memory regions\n" );
      rc = -ENOMEM;
      goto err_out_mem;
   }

   // allocate all the ports
   for(i = 0; i < priv->nr; ++i)
   {
      priv->ports[i].pdev = pdev;
      priv->ports[i].port_type = SELSERIAL_PORT_TYPE_UNOPENED;
      selserial_common_init_stale_data_timer( &priv->ports[i] );
      port = &priv->ports[i].port;

      // initialize the spin lock
      spin_lock_init(&port->lock);

      // fill in all the goodies
      port->type = PORT_SELSERIAL;
      port->line = i + selserial_port_count;
      port->iotype = UPIO_MEM;
      port->iobase = 0;
      port->mapbase = pci_resource_start(pdev, 0) +
                      i * SELSERIAL_PORT_REG_SIZE;
      port->membase = priv->remapped_bar + i * SELSERIAL_PORT_REG_SIZE;
      port->regshift = 0;
      port->fifosize = SELSERIAL_HW_FIFO_SIZE;
      port->uartclk = SELSERIAL_UART_CLOCK_FREQ;
      port->irq = pdev->irq;
      port->flags = UPF_LOW_LATENCY | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
      port->cons = NULL;

      // prime the GCR
      priv->ports[i].gcr = ioread8( port->membase + SELSERIAL_GCR );

   }

   // initialize the tty devices
   rc = selserial_tty_initport( priv );
   if(rc)
   {
      goto err_out_add_ttyport;
   }



   // special feature initialization (485/232, loopback, power sysfs)
   rc = selserial_special_init( &pdev->dev );
   if( rc )
   {
      goto err_out_all;
   }

   // count the ports as good
   selserial_port_count += priv->nr;

   // save the pointer to our private data
   pci_set_drvdata( pdev, priv );

   // Status Message
   dev_info( &pdev->dev, "Initialized PCI device\n" );

   return 0;

err_out_all:



   selserial_tty_cleanupport( priv );
err_out_add_ttyport:
   iounmap( priv->remapped_bar );
err_out_mem:
   kfree( priv );
err_out_release:
   pci_release_regions(pdev);
err_out_disable:
   pci_disable_device(pdev);
err_out:
   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief deinitializes the device
///
/// @param pdev pointer to the device structure describing the port
//////////////////////////////////////////////////////////////////////////////
static void selserial_remove_board( struct pci_dev *pdev )
{
   struct selserial_private *priv;

   // status to user
   dev_info( &pdev->dev, "Releasing PCI device\n" );

   // get the private data and clear the pointer
   priv = pci_get_drvdata( pdev );
   pci_set_drvdata( pdev, NULL );

   // special feature cleanup
   selserial_special_cleanup( &pdev->dev );



   // cleanup the tty stuff
   selserial_tty_cleanupport( priv );

   // unmap the bar
   iounmap( priv->remapped_bar );

   // free the private memory
   kfree( priv );

   // release the memory/io regions
   pci_release_regions( pdev );

   // disable the device
   pci_disable_device( pdev );

   // status to user
   dev_info( &pdev->dev, "Released PCI device\n" );
}

//////////////////////////////////////////////////////////////////////////////
/// Module init/exit functionality
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// @brief initialize the module by registering the devices
///
/// @return 0 for success, <0 for failure
//////////////////////////////////////////////////////////////////////////////
static int __init selserial_init( void )
{
   int rc;

   selserial_port_count = 0;

   printk( KERN_INFO "SELserial: Module Initializing\n" );

   // initialize the TTY interface to the ports
   // Have to do this here as opposed to initport becuase the TTY driver must
   // register all possible TTY devices that could ever be added, whereas the
   // RAW and MB ports can just reserve a subset of the minor numbers
   rc = selserial_tty_init();
   if( rc )
   {
      goto init_err_tty;
   }

   // initialize the PCI device
   // NOTE:  This MUST be after the tty init becuase this will call the probe()
   //        function before returning
   rc = pci_register_driver(&selserial_driver);
   if( rc )
   {
      goto init_err_pci;
   }

   printk( KERN_INFO "SELserial: Successfully Initialized\n" );

   return rc;

init_err_pci:
   selserial_tty_exit();

init_err_tty:
   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Unregister the drivers.
//////////////////////////////////////////////////////////////////////////////
static void __exit selserial_exit( void )
{
   // Status Message
   printk( KERN_INFO "SELserial: Unloading driver\n" );

   // unregister the pci driver
   pci_unregister_driver( &selserial_driver );

   // unregister the tty
   selserial_tty_exit();

   printk( KERN_INFO "SELserial: Driver Unloaded\n" );
}

//////////////////////////////////////////////////////////////////////////////
/// Module definitions
//////////////////////////////////////////////////////////////////////////////

module_init( selserial_init );
module_exit( selserial_exit );

MODULE_LICENSE( "GPL" );
MODULE_AUTHOR( "Schweitzer Engineering Laboratories, Inc." );
MODULE_DESCRIPTION( "Driver to handle communication to the SEL Uart." );
MODULE_VERSION( "1.0.0" );
MODULE_DEVICE_TABLE(pci, selserial_pci_tbl);

// vim:sm:et:sw=3:sts=3

