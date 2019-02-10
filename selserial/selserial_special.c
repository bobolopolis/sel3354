//////////////////////////////////////////////////////////////////////////////
// COPYRIGHT (c) 2008 Schweitzer Engineering Laboratories, Pullman, Washington
///
/// @file selserial_special.c
///
/// @brief Routines to control the 485, port power, and loopback modes of the
///        selserial port
//////////////////////////////////////////////////////////////////////////////
#include "selserial.h"

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/types.h>

//////////////////////////////////////////////////////////////////////////////
/// function prototypes

// sysfs routines
static ssize_t selserial_special_485_show( struct device *dev,
                                           struct device_attribute *attr,
                                           char *buffer );
static ssize_t selserial_special_485_store( struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buffer,
                                            size_t size );
static ssize_t selserial_special_power_show( struct device *dev,
                                             struct device_attribute *attr,
                                             char *buffer );
static ssize_t selserial_special_power_store( struct device *dev,
                                              struct device_attribute *attr,
                                              const char *buffer,
                                              size_t size );
static ssize_t selserial_special_loopback_show( struct device *dev,
                                                struct device_attribute *attr,
                                                char *buffer );
static ssize_t selserial_special_loopback_store( struct device *dev,
                                                 struct device_attribute *attr,
                                                 const char *buffer,
                                                 size_t size );
static ssize_t selserial_special_half_flow_show( struct device *dev,
                                                 struct device_attribute *attr,
                                                 char *buffer );
static ssize_t selserial_special_half_flow_store( struct device *dev,
                                                  struct device_attribute *attr,
                                                  const char *buffer,
                                                  size_t size );
static ssize_t selserial_special_half_duplex_show( struct device *dev,
                                                   struct device_attribute *attr,
                                                   char *buffer );
static ssize_t selserial_special_half_duplex_store( struct device *dev,
                                                    struct device_attribute *attr,
                                                    const char *buffer,
                                                    size_t size );

//////////////////////////////////////////////////////////////////////////////
/// Static variable declaration

/// Declare a device attribute for the port_power sysfs file
DEVICE_ATTR( port_power,
             S_IRUGO | S_IWUSR,
             selserial_special_power_show,
             selserial_special_power_store );

/// Declare a device attribute for the loopback sysfs file
DEVICE_ATTR( loopback,
             S_IRUGO | S_IWUSR,
             selserial_special_loopback_show,
             selserial_special_loopback_store );

/// Declare a device attribute for the rs485 sysfs file
DEVICE_ATTR( rs485,
             S_IRUGO | S_IWUSR,
             selserial_special_485_show,
             selserial_special_485_store );

/// Declare a device attribute for the half_flow sysfs file
DEVICE_ATTR( half_flow,
             S_IRUGO | S_IWUSR,
             selserial_special_half_flow_show,
             selserial_special_half_flow_store );

/// Declare a device attribute for the half_duplex sysfs file
DEVICE_ATTR( half_duplex,
             S_IRUGO | S_IWUSR,
             selserial_special_half_duplex_show,
             selserial_special_half_duplex_store );

//////////////////////////////////////////////////////////////////////////////
/// SYSFS functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// @brief Parse the incoming data stream from a write to the rs485 file or
///        the port_power file and check the syntax
///
/// @param str Incoming stream
/// @param sz  Number of bytes in str
/// @param port Pointer to storage to place the port number in.
/// @param modechr Pointer to storage to place a pointer to the mode character
///                from the imput stream into.
///
/// @return 0 for success, -EINVAL if the syntax is incorrect
//////////////////////////////////////////////////////////////////////////////
static int selserial_special_parse_store_string( const char *str,
                                                 size_t sz,
                                                 int *port,
                                                 const char **modechr )
{
   int rc = -EINVAL;

   // initialize the port variable
   *port = 0;

   // get the port number
   while( sz && isdigit(*str) )
   {
      *port = (10 * (*port)) + ((*str) - '0');
      --sz;
      ++str;
   }

   // if we have at least 2 characters left and the next is a space
   if( (sz >= 2) && (*str == ' ') )
   {
      *modechr = str + 1;
      rc = 0;
   }

   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief helper routine to show either the 485 or port power state
///
/// @param priv Structure describing the PCI device
/// @param mask Mask showing which attribute to show
/// @param buffer buffer to insert data into...buffer is always page_size big
///
/// @return number of bytes actually inserted into the buffer
//////////////////////////////////////////////////////////////////////////////
static ssize_t __selserial_special_common_show(
                          struct selserial_private *priv,
                          u8 mask,
                          char *buffer )
{
   ssize_t i;

   // get state for each port
   for( i = 0; i < priv->nr; ++i )
   {
      // check the state
      buffer[i] = (priv->ports[i].gcr & mask) ? '1' : '0';
   }

   // newline and terminate
   buffer[i++] = '\n';
   buffer[i] = '\0';

   return i;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Shows the state of the port power mode for each of the ports
///
/// @param dev device structure this should operate on
/// @param attr attribute that is being queried
/// @param buffer buffer to insert data into...buffer is always page_size big
///
/// @return number of bytes actually inserted into the buffer
//////////////////////////////////////////////////////////////////////////////
static ssize_t selserial_special_power_show( struct device *dev,
                                             struct device_attribute *attr,
                                             char *buffer )
{
   struct pci_dev *pdev = container_of( dev, struct pci_dev, dev );
   struct selserial_private *priv = pci_get_drvdata( pdev );

   return __selserial_special_common_show( priv,
                                           SELSERIAL_GCR_PORT_POWER,
                                           buffer );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Shows the state of the rs485 mode for each of the ports
///
/// @param dev device structure this should operate on
/// @param attr attribute that is being queried
/// @param buffer buffer to insert data into...buffer is always page_size big
///
/// @return number of bytes actually inserted into the buffer
//////////////////////////////////////////////////////////////////////////////
static ssize_t selserial_special_485_show( struct device *dev,
                                             struct device_attribute *attr,
                                             char *buffer )
{
   struct pci_dev *pdev = container_of( dev, struct pci_dev, dev );
   struct selserial_private *priv = pci_get_drvdata( pdev );

   return __selserial_special_common_show( priv,
                                           SELSERIAL_GCR_485_MODE,
                                           buffer );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the 485 mode for a particular port
///
/// @param sport Port to set the mode for
/// @param mode 485 mode to set
//////////////////////////////////////////////////////////////////////////////
void __selserial_special_485_store( struct selserial_uart_port *sport,
                                    char mode )
{
   // if we were in auto485 mode, then assume we are going to clear
   // the auto_RTS bit
   if( sport->auto485 )
   {
      sport->auto485 = 0;
      sport->mcr &= ~(SELSERIAL_MCR_AUTO_RTS);
   }

   // what we do next depends on the mode that is selected
   switch( mode )
   {
      // turn off 485
      case '0':
      {
         sport->gcr &= ~(SELSERIAL_GCR_485_MODE);
         if( sport->half_flow )
         {
            sport->auto485 = 1;
            sport->mcr |= SELSERIAL_MCR_AUTO_RTS;
            sport->mcr &= ~(SELSERIAL_MCR_AUTO_HS_RTS |
                            SELSERIAL_MCR_AUTO_HS_CTS);
         }
         break;
      }

      // 485 with autoRTS
      case 'a':
      {
         sport->auto485 = 1;
         sport->mcr |= SELSERIAL_MCR_AUTO_RTS;
         sport->mcr &= ~(SELSERIAL_MCR_AUTO_HS_RTS |
                         SELSERIAL_MCR_AUTO_HS_CTS);
         if( !sport->fifo_settings.eott )
         {
            sport->fifo_settings.eott = SELSERIAL_DEFAULT_EOXT_TIMER;
         }
         // INTENTIONAL FALLTHROUGH
      }
      // normal 485
      default:
      {
         sport->gcr |= SELSERIAL_GCR_485_MODE;
         break;
      }
   }

   // set the registers
   iowrite16( sport->fifo_settings.eott,
              sport->port.membase + SELSERIAL_EOTT_REG );
   iowrite8( sport->mcr, sport->port.membase + SELSERIAL_MCR );
   iowrite8( sport->gcr,
             sport->port.membase + SELSERIAL_GCR );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the 485 mode of a port
///
/// @param dev device structure this should operate on
/// @param attr attribute that is being queried
/// @param buffer buffer to read commands from
/// @param size size of buffer
///
/// @return size for success, or < 0 for failure
//////////////////////////////////////////////////////////////////////////////
static ssize_t selserial_special_485_store( struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buffer,
                                            size_t size )
{
   int port = 0;
   const char *mode;
   int rc;

   // get the private data for the port
   struct pci_dev *pdev = container_of( dev, struct pci_dev, dev );
   struct selserial_private *priv = pci_get_drvdata( pdev );

   // parse the input string
   rc = selserial_special_parse_store_string( buffer, size, &port, &mode );
   if( !rc &&
       (port < priv->nr) &&
       ((*mode == '0') || (*mode == '1') || (*mode == 'a')) )
   {
      // lock the port
      spin_lock( &priv->ports[port].port.lock );

      // set it up according to the mode
      __selserial_special_485_store( &priv->ports[port], *mode );

      // unlock the port
      spin_unlock( &priv->ports[port].port.lock );

      // report success
      rc = size;
   }
   else
   {
      rc = -EINVAL;
   }

   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the port power mode for a particular port
///
/// @param sport Port to set the mode for
/// @param mode port power mode to set
//////////////////////////////////////////////////////////////////////////////
void __selserial_special_power_store( struct selserial_uart_port *sport,
                                      char mode )
{
   // set the GCR based on the mode
   if( mode == '0' )
   {
      sport->gcr &= ~(SELSERIAL_GCR_PORT_POWER);
   }
   else
   {
      sport->gcr |= SELSERIAL_GCR_PORT_POWER;
   }

   // set the registers
   iowrite8( sport->gcr, sport->port.membase + SELSERIAL_GCR );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the port power mode of a port
///
/// @param dev device structure this should operate on
/// @param attr attribute that is being queried
/// @param buffer buffer to read commands from
/// @param size size of buffer
///
/// @return size for success, or < 0 for failure
//////////////////////////////////////////////////////////////////////////////
static ssize_t selserial_special_power_store( struct device *dev,
                                              struct device_attribute *attr,
                                              const char *buffer,
                                              size_t size )
{
   int port = 0;
   const char *mode;
   int rc;

   struct pci_dev *pdev = container_of( dev, struct pci_dev, dev );
   struct selserial_private *priv = pci_get_drvdata( pdev );

   // parse the input string
   rc = selserial_special_parse_store_string( buffer, size, &port, &mode );
   if( !rc &&
       (port < priv->nr) &&
       ((*mode == '0') || (*mode == '1')) )
   {
      // set it up according to the mode
      __selserial_special_power_store( &priv->ports[port], *mode );

      // report success
      rc = size;
   }
   else
   {
      rc = -EINVAL;
   }

   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Shows the state of the loopback mode for the port
///
/// @param dev device structure this should operate on
/// @param attr attribute that is being queried
/// @param buffer buffer to insert data into...buffer is always page_size big
///
/// @return number of bytes actually inserted into the buffer
//////////////////////////////////////////////////////////////////////////////
static ssize_t selserial_special_loopback_show( struct device *dev,
                                                struct device_attribute *attr,
                                                char *buffer )
{
   struct pci_dev *pdev = container_of( dev, struct pci_dev, dev );
   struct selserial_private *priv = pci_get_drvdata( pdev );

   // check the loopback state
   buffer[0] = (priv->ports[0].gcr & SELSERIAL_GCR_LOOP) ? '1' : '0';

   // newline and terminate
   buffer[1] = '\n';
   buffer[2] = '\0';

   return 2;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief set the state of the loopback mode
///
/// @param dev device to operate against
/// @param attr attribute being modified
/// @param buffer data written to sysfs file
/// @param size size of data passed in
///
/// @return size if successful, <0 for failure
///
/// @remarks '0' passed in turns it off, '1' turns it on
//////////////////////////////////////////////////////////////////////////////
static ssize_t selserial_special_loopback_store( struct device *dev,
                                                 struct device_attribute *attr,
                                                 const char *buffer,
                                                 size_t size )
{
   struct pci_dev *pdev = container_of( dev, struct pci_dev, dev );
   struct selserial_private *priv = pci_get_drvdata( pdev );

   // make sure there is at least one byte in the buffer and the
   // passed in byte is a 0 or a 1
   if( (size < 1) || ((*buffer != '0') && (*buffer != '1')) )
   {
      return -EINVAL;
   }

   // a '1' turns on loopback, '0' turns it off
   if( *buffer == '1' )
   {
      priv->ports[0].gcr |= SELSERIAL_GCR_LOOP;
   }
   else
   {
      priv->ports[0].gcr &= ~(SELSERIAL_GCR_LOOP);
   }

   // write it out
   iowrite8( priv->ports[0].gcr,
             priv->ports[0].port.membase + SELSERIAL_GCR );

   return size;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Shows the state of the half duplex mode for each of the ports
///
/// @param dev device structure this should operate on
/// @param attr attribute that is being queried
/// @param buffer buffer to insert data into...buffer is always page_size big
///
/// @return number of bytes actually inserted into the buffer
//////////////////////////////////////////////////////////////////////////////
static ssize_t selserial_special_half_duplex_show( struct device *dev,
                                                   struct device_attribute *attr,
                                                   char *buffer )
{
   struct pci_dev *pdev = container_of( dev, struct pci_dev, dev );
   struct selserial_private *priv = pci_get_drvdata( pdev );

   return __selserial_special_common_show( priv,
                                           SELSERIAL_GCR_HALF_DUPLEX,
                                           buffer );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the half duplex mode for a particular port
///
/// @param sport Port to set the mode for
/// @param mode half duplex mode to set
//////////////////////////////////////////////////////////////////////////////
void __selserial_special_half_duplex_store( struct selserial_uart_port *sport,
                                            char mode )
{
   // what we do next depends on the mode that is selected
   switch( mode )
   {
      // turn off half duplex
      case '0':
      {
         sport->gcr &= ~(SELSERIAL_GCR_HALF_DUPLEX);
         break;
      }

      // turn on half duplex
      default:
      {
         sport->gcr |= SELSERIAL_GCR_HALF_DUPLEX;
         break;
      }
   }

   // set the registers
   iowrite8( sport->gcr, sport->port.membase + SELSERIAL_GCR );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the Half Duplex mode for the port
///
/// @param dev device structure this should operate on
/// @param attr attribute that is being queried
/// @param buffer buffer to read commands from
/// @param size size of buffer
///
/// @return size for success, or < 0 for failure
//////////////////////////////////////////////////////////////////////////////
static ssize_t selserial_special_half_duplex_store( struct device *dev,
                                                    struct device_attribute *attr,
                                                    const char *buffer,
                                                    size_t size )
{
   int port = 0;
   const char *mode;
   int rc;

   // get the private data for the port
   struct pci_dev *pdev = container_of( dev, struct pci_dev, dev );
   struct selserial_private *priv = pci_get_drvdata( pdev );

   // parse the input string
   rc = selserial_special_parse_store_string( buffer, size, &port, &mode );
   if( !rc &&
       (port < priv->nr) &&
       ((*mode == '0') || (*mode == '1')) )
   {
      // lock the port
      spin_lock( &priv->ports[port].port.lock );

      // set it up according to the mode
      __selserial_special_half_duplex_store( &priv->ports[port], *mode );

      // unlock the port
      spin_unlock( &priv->ports[port].port.lock );

      // report success
      rc = size;
   }
   else
   {
      rc = -EINVAL;
   }

   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Shows the state of the half flow mode for each of the ports
///
/// @param dev device structure this should operate on
/// @param attr attribute that is being queried
/// @param buffer buffer to insert data into...buffer is always page_size big
///
/// @return number of bytes actually inserted into the buffer
//////////////////////////////////////////////////////////////////////////////
static ssize_t selserial_special_half_flow_show( struct device *dev,
                                                 struct device_attribute *attr,
                                                 char *buffer )
{
   ssize_t i;
   struct pci_dev *pdev = container_of( dev, struct pci_dev, dev );
   struct selserial_private *priv = pci_get_drvdata( pdev );

   // get state for each port
   for( i = 0; i < priv->nr; ++i )
   {
      // check the state
      buffer[i] = (priv->ports[i].half_flow) ? '1' : '0';
   }

   // newline and terminate
   buffer[i++] = '\n';
   buffer[i] = '\0';

   return i;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the half flow mode for a particular port
///
/// @param sport Port to set the mode for
/// @param mode half flow mode to set
//////////////////////////////////////////////////////////////////////////////
void __selserial_special_half_flow_store( struct selserial_uart_port *sport,
                                          char mode )
{
   // what we do next depends on the mode that is selected
   switch( mode )
   {
      // turn off half duplex
      case '0':
      {
         sport->half_flow = 0;
         if( !(sport->gcr & SELSERIAL_GCR_485_MODE) )
         {
            sport->mcr &= ~(SELSERIAL_MCR_AUTO_RTS);
            sport->auto485 = 0;
         }
         break;
      }

      // turn on half duplex
      default:
      {
         sport->half_flow = 1;
         if( !(sport->gcr & SELSERIAL_GCR_485_MODE) )
         {
            sport->auto485 = 1;
            sport->mcr |= SELSERIAL_MCR_AUTO_RTS;
            sport->mcr &= ~(SELSERIAL_MCR_AUTO_HS_RTS |
                            SELSERIAL_MCR_AUTO_HS_CTS);
            if( !sport->fifo_settings.eott )
            {
               sport->fifo_settings.eott = SELSERIAL_DEFAULT_EOXT_TIMER;
            }
         }
         break;
      }
   }

   iowrite8( sport->mcr, sport->port.membase + SELSERIAL_MCR );
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Set the Half Flow mode for the port
///
/// @param dev device structure this should operate on
/// @param attr attribute that is being queried
/// @param buffer buffer to read commands from
/// @param size size of buffer
///
/// @return size for success, or < 0 for failure
//////////////////////////////////////////////////////////////////////////////
static ssize_t selserial_special_half_flow_store( struct device *dev,
                                                  struct device_attribute *attr,
                                                  const char *buffer,
                                                  size_t size )
{
   int port = 0;
   const char *mode;
   int rc;

   // get the private data for the port
   struct pci_dev *pdev = container_of( dev, struct pci_dev, dev );
   struct selserial_private *priv = pci_get_drvdata( pdev );

   // parse the input string
   rc = selserial_special_parse_store_string( buffer, size, &port, &mode );
   if( !rc &&
       (port < priv->nr) &&
       ((*mode == '0') || (*mode == '1')) )
   {
      // lock the port
      spin_lock( &priv->ports[port].port.lock );

      // set it up according to the mode
      __selserial_special_half_flow_store( &priv->ports[port], *mode );

      // unlock the port
      spin_unlock( &priv->ports[port].port.lock );

      // report success
      rc = size;
   }
   else
   {
      rc = -EINVAL;
   }

   return rc;
}
//////////////////////////////////////////////////////////////////////////////
/// Initialization routines
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// @brief Create the device files in the sysfs filesystem
///
/// @param device Device to attach the files to
///
/// @return 0 for success, an error code for failure
//////////////////////////////////////////////////////////////////////////////
int selserial_special_init( struct device *device )
{
   int rc;

   if( (rc = device_create_file( device, &dev_attr_port_power) ) )
      goto out1;
   if( (rc = device_create_file( device, &dev_attr_loopback) ) )
      goto out2;
   if( (rc = device_create_file( device, &dev_attr_rs485) ) )
      goto out3;
   if( (rc = device_create_file( device, &dev_attr_half_flow) ) )
      goto out4;
   if( (rc = device_create_file( device, &dev_attr_half_duplex) ) )
      goto out5;

   goto out1;

out5:
   device_remove_file( device, &dev_attr_half_flow );
out4:
   device_remove_file( device, &dev_attr_rs485 );
out3:
   device_remove_file( device, &dev_attr_loopback );
out2:
   device_remove_file( device, &dev_attr_port_power );
out1:
   return rc;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Remove the sysfs files from the device
///
/// @param device Device to remove the files from.
//////////////////////////////////////////////////////////////////////////////
void selserial_special_cleanup( struct device *device )
{
   device_remove_file( device, &dev_attr_half_duplex );
   device_remove_file( device, &dev_attr_half_flow );
   device_remove_file( device, &dev_attr_rs485 );
   device_remove_file( device, &dev_attr_loopback );
   device_remove_file( device, &dev_attr_port_power );
}

// vim:sm:et:sw=3:sts=3

