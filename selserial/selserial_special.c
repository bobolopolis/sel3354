// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2008 Schweitzer Engineering Laboratories, Pullman, Washington
 *
 * Routines to control the 485, port power, and loopback modes of the
 * selserial port
 */

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/types.h>

#include "selserial.h"

/**
 * selserial_special_parse_store_string() - Parse incoming sysfs file data
 *
 * @str: Incoming data stream
 * @sz: Number of bytes in @str
 * @port: Pointer to storage to place the port number in
 * @modechr: Pointer to storage to place a pointer to the mode character
 *           from the input stream into.
 *
 * Parse the incoming data stream from a write to the rs485 file or the
 * port_power file and check the syntax.
 *
 * Return: 0 for success, -EINVAL if the syntax is incorrect
 */
static int selserial_special_parse_store_string(const char *str,
						size_t sz,
						int *port,
						const char **modechr)
{
	int rc = -EINVAL;

	/* Initialize the port variable */
	*port = 0;

	/* Get the port number */
	while (sz && isdigit(*str)) {
		*port = (10 * (*port)) + ((*str) - '0');
		--sz;
		++str;
	}

	/* If we have at least 2 characters left and the next is a space */
	if ((sz >= 2) && (*str == ' ')) {
		*modechr = str + 1;
		rc = 0;
	}

	return rc;
}

/**
 * __selserial_special_common_show() - Helper for showing 485 or port_power
 *
 * @priv: Struct describing the PCI device.
 * @mask: Mask showing which attribute to show.
 * @buffer: Buffer to insert data into. Is always PAGE_SIZE big.
 *
 * Return: Number of bytes actually inserted into the buffer.
 */
static ssize_t __selserial_special_common_show(struct selserial_private *priv,
					       u8 mask,
					       char *buffer)
{
	ssize_t i;

	/* Get state for each port */
	for(i = 0; i < priv->nr; ++i) {
		/* Check the state */
		buffer[i] = (priv->ports[i].gcr & mask) ? '1' : '0';
	}

	/* newline and terminate */
	buffer[i++] = '\n';
	buffer[i] = '\0';

	return i;
}

/**
 * selserial_special_485_show() - Show rs485 state for each of the ports
 *
 * @dev: Device struct to operate on.
 * @attr: Attribute being queried.
 * @buffer: Buffer to insert data into. Always PAGE_SIZE big.
 *
 * Return: Number of bytes actually inserted into the buffer.
 */
static ssize_t selserial_special_485_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buffer)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct selserial_private *priv = pci_get_drvdata(pdev);

	return __selserial_special_common_show(priv,
					       SELSERIAL_GCR_485_MODE,
					       buffer);
}

/**
 * __selserial_special_485_store() - Helper to set the 485 mode for a port
 *
 * @sport: Port to set the mode for
 * @mode: 485 mode to set
 */
void __selserial_special_485_store(struct selserial_uart_port *sport,
				   char mode)
{
	/* If we were in auto485 mode, then assume we are going to clear
	 * the auto_RTS bit */
	if (sport->auto485) {
		sport->auto485 = 0;
		sport->mcr &= ~(SELSERIAL_MCR_AUTO_RTS);
	}

	/* What we do next depends on the mode that is selected */
	switch(mode) {
	case '0':
		/* turn off 485 */
		sport->gcr &= ~(SELSERIAL_GCR_485_MODE);
		if (sport->half_flow) {
			sport->auto485 = 1;
			sport->mcr |= SELSERIAL_MCR_AUTO_RTS;
			sport->mcr &= ~(SELSERIAL_MCR_AUTO_HS_RTS |
					SELSERIAL_MCR_AUTO_HS_CTS);
		}
		break;
	case 'a':
		/* 485 with autoRTS */
		sport->auto485 = 1;
		sport->mcr |= SELSERIAL_MCR_AUTO_RTS;
		sport->mcr &= ~(SELSERIAL_MCR_AUTO_HS_RTS |
				SELSERIAL_MCR_AUTO_HS_CTS);
		if (!sport->fifo_settings.eott) {
			sport->fifo_settings.eott = SELSERIAL_DEFAULT_EOXT_TIMER;
		}
		/* INTENTIONAL FALLTHROUGH */
	default:
		/* normal 485 */
		sport->gcr |= SELSERIAL_GCR_485_MODE;
		break;
	}

	/* Set the registers */
	iowrite16(sport->fifo_settings.eott,
		  sport->port.membase + SELSERIAL_EOTT_REG);
	iowrite8(sport->mcr, sport->port.membase + SELSERIAL_MCR);
	iowrite8(sport->gcr, sport->port.membase + SELSERIAL_GCR);
}

/**
 * selserial_special_485_store() - Set the 485 mode of a port
 *
 * @dev: Device struct to operate on
 * @attr: Attribute to query
 * @buffer: Buffer to read commands from
 * @size: Size of the buffer
 *
 * Return: @size for success, < 0 for failure
 */
static ssize_t selserial_special_485_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buffer,
					   size_t size)
{
	int port = 0;
	const char *mode;
	int rc;

	/* Get the private data for the port */
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct selserial_private *priv = pci_get_drvdata(pdev);

	/* Parse the input string */
	rc = selserial_special_parse_store_string(buffer, size, &port, &mode);
	if (!rc &&
	    (port < priv->nr) &&
	    ((*mode == '0') || (*mode == '1') || (*mode == 'a'))) {
		/* Lock the port */
		spin_lock(&priv->ports[port].port.lock);

		/* Set it up according to the mode */
		__selserial_special_485_store(&priv->ports[port], *mode);

		/* Unlock the port */
		spin_unlock(&priv->ports[port].port.lock);

		/* Report success */
		rc = size;
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Declare a device attribute for the rs485 sysfs file */
DEVICE_ATTR(rs485,
	    S_IRUGO | S_IWUSR,
	    selserial_special_485_show,
	    selserial_special_485_store);

/**
 * __selserial_special_power_store() - Helper to set port power
 *
 * @sport: Port to set the mode for
 * @mode: Port power mode to set
 */
void __selserial_special_power_store(struct selserial_uart_port *sport,
				     char mode)
{
	/* Set the GCR based on the mode */
	if (mode == '0') {
		sport->gcr &= ~(SELSERIAL_GCR_PORT_POWER);
	} else {
		sport->gcr |= SELSERIAL_GCR_PORT_POWER;
	}

	/* set the registers */
	iowrite8(sport->gcr, sport->port.membase + SELSERIAL_GCR);
}

/**
 * selserial_special_power_show() - Show the state of the port power mode
 *
 * @dev: Device struct to operate on
 * @attr: Attribute to query
 * @buffer: Buffer to insert data into. Always PAGE_SIZE big.
 *
 * Return: Number of bytes actually inserted into the buffer.
 */
static ssize_t selserial_special_power_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buffer)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct selserial_private *priv = pci_get_drvdata(pdev);

	return __selserial_special_common_show(priv,
					       SELSERIAL_GCR_PORT_POWER,
					       buffer);
}

/**
 * selserial_special_power_store() - Set the port power mode of a port
 *
 * @dev: Device struct to operate on
 * @attr: Attribute to query
 * @buffer: Buffer to read commands from
 * @size: Size of the buffer
 *
 * Return: @size for success, < 0 for failure
 */
static ssize_t selserial_special_power_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buffer,
					     size_t size)
{
	int port = 0;
	const char *mode;
	int rc;

	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct selserial_private *priv = pci_get_drvdata(pdev);

	/* Parse the input string */
	rc = selserial_special_parse_store_string(buffer, size, &port, &mode);
	if (!rc &&
	    (port < priv->nr) &&
	    ((*mode == '0') || (*mode == '1'))) {
		/* Set it up according to the mode */
		__selserial_special_power_store(&priv->ports[port], *mode);
		rc = size;
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Declare a device attribute for the port_power sysfs file */
DEVICE_ATTR(port_power,
            S_IRUGO | S_IWUSR,
            selserial_special_power_show,
            selserial_special_power_store);

/**
 * selserial_special_loopback_show() - Show the state of the loopback mode
 *
 * @dev: Device struct to operate on
 * @attr: Attribute to query
 * @buffer: Buffer to insert data into. Always PAGE_SIZE big.
 *
 * Return: Number of bytes actually inserted into the buffer.
 */
static ssize_t selserial_special_loopback_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buffer)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct selserial_private *priv = pci_get_drvdata(pdev);

	/* Check the loopback state */
	buffer[0] = (priv->ports[0].gcr & SELSERIAL_GCR_LOOP) ? '1' : '0';

	/* newline and terminate */
	buffer[1] = '\n';
	buffer[2] = '\0';

	return 2;
}

/**
 * selserial_special_loopback_store() - Set the state of the loopback mode
 *
 * @dev: Device struct to operate on
 * @attr: Attribute to modify
 * @buffer: Data written to sysfs file
 * @size: Size of data passed in
 *
 * '0' passid in turns it off, '1' turns it on.
 *
 * Return: @size if successful, < 0 on failure
 */
static ssize_t selserial_special_loopback_store(struct device *dev,
						struct device_attribute *attr,
						const char *buffer,
						size_t size)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct selserial_private *priv = pci_get_drvdata(pdev);

	/* Make sure there is at least one byte in the buffer and the
	 * passed in byte is a 0 or a 1 */
	if ((size < 1) || ((*buffer != '0') && (*buffer != '1'))) {
		return -EINVAL;
	}

	/* '1' turns on loopback, '0' turns it off */
	if (*buffer == '1') {
		priv->ports[0].gcr |= SELSERIAL_GCR_LOOP;
	} else {
		priv->ports[0].gcr &= ~(SELSERIAL_GCR_LOOP);
	}

	/* write it out */
	iowrite8(priv->ports[0].gcr,
		 priv->ports[0].port.membase + SELSERIAL_GCR);

	return size;
}

/* Declare a device attribute for the loopback sysfs file */
DEVICE_ATTR(loopback,
	    S_IRUGO | S_IWUSR,
	    selserial_special_loopback_show,
	    selserial_special_loopback_store);

/**
 * selserial_special_half_duplex_show() - Show the state of half duplex mode
 *
 * @dev: Device struct to operate on
 * @attr: Attribute to query
 * @buffer: Buffer to insert data into. Always PAGE_SIZE big.
 *
 * Return: Number of bytes actually inserted into the buffer.
 */
static ssize_t selserial_special_half_duplex_show(struct device *dev,
						  struct device_attribute *attr,
						  char *buffer)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct selserial_private *priv = pci_get_drvdata(pdev);

	return __selserial_special_common_show(priv,
					       SELSERIAL_GCR_HALF_DUPLEX,
					       buffer);
}

/**
 * __selserial_special_half_duplex_store() - Helper to set the half duplex mode
 *
 * @sport: Port to set the mode for
 * @mode: Half duplex mode to set
 */
void __selserial_special_half_duplex_store(struct selserial_uart_port *sport,
					   char mode)
{
	/* What we do next depends on the mode that is selected */
	switch(mode) {
	case '0':
		/* turn off half duplex */
		sport->gcr &= ~(SELSERIAL_GCR_HALF_DUPLEX);
		break;
	default:
		/* turn on half duplex */
		sport->gcr |= SELSERIAL_GCR_HALF_DUPLEX;
		break;
	}

	/* set the registers */
	iowrite8(sport->gcr, sport->port.membase + SELSERIAL_GCR);
}

/**
 * selserial_special_half_duplex_store() - Set the half duplex mode
 *
 * @dev: Device struct to operate on
 * @attr: Attribute to query
 * @buffer: Buffer to read commands from
 * @size: Size of the buffer
 *
 * Return: @size for success, < 0 for failure
 */
static ssize_t selserial_special_half_duplex_store(struct device *dev,
						   struct device_attribute *attr,
						   const char *buffer,
						   size_t size)
{
	int port = 0;
	const char *mode;
	int rc;

	/* Get the private data for the port */
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct selserial_private *priv = pci_get_drvdata(pdev);

	/* Parse the input string */
	rc = selserial_special_parse_store_string(buffer, size, &port, &mode);
	if (!rc && (port < priv->nr) && ((*mode == '0') || (*mode == '1'))) {
		/* Lock the port */
		spin_lock(&priv->ports[port].port.lock);

		/* Set it up according to the mode */
		__selserial_special_half_duplex_store(&priv->ports[port], *mode);

		/* Unlock the port */
		spin_unlock(&priv->ports[port].port.lock);

		/* Report success */
		rc = size;
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Declare a device attribute for the half_duplex sysfs file */
DEVICE_ATTR(half_duplex,
	    S_IRUGO | S_IWUSR,
	    selserial_special_half_duplex_show,
	    selserial_special_half_duplex_store);

/**
 * selserial_special_half_flow_show() - Show the state ofhalf flow mode
 *
 * @dev: Device struct to operate on
 * @attr: Attribute to query
 * @buffer: Buffer to insert data into. Always PAGE_SIZE big.
 *
 * Return: Number of bytes actually inserted into the buffer.
 */
static ssize_t selserial_special_half_flow_show(struct device *dev,
						struct device_attribute *attr,
						char *buffer)
{
	ssize_t i;
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct selserial_private *priv = pci_get_drvdata(pdev);

	/* Get state for each port */
	for (i = 0; i < priv->nr; ++i) {
		/* Check the state */
		buffer[i] = (priv->ports[i].half_flow) ? '1' : '0';
	}

	/* newline and terminate */
	buffer[i++] = '\n';
	buffer[i] = '\0';

	return i;
}

/**
 * __selserial_special_half_flow_store() - Helper to set the half flow mode
 *
 * @sport: Port to set the mode for
 * @mode: Half flow mode to set
 */
void __selserial_special_half_flow_store(struct selserial_uart_port *sport,
					 char mode)
{
	/* What we do next depends on the mode that is selected */
	switch(mode) {
	case '0':
		/* Turn off half duplex */
		sport->half_flow = 0;
		if (!(sport->gcr & SELSERIAL_GCR_485_MODE)) {
			sport->mcr &= ~(SELSERIAL_MCR_AUTO_RTS);
			sport->auto485 = 0;
		}
		break;
	default:
		/* Turn on half duplex */
		sport->half_flow = 1;
		if (!(sport->gcr & SELSERIAL_GCR_485_MODE)) {
			sport->auto485 = 1;
			sport->mcr |= SELSERIAL_MCR_AUTO_RTS;
			sport->mcr &= ~(SELSERIAL_MCR_AUTO_HS_RTS |
					SELSERIAL_MCR_AUTO_HS_CTS);
			if (!sport->fifo_settings.eott) {
				sport->fifo_settings.eott = SELSERIAL_DEFAULT_EOXT_TIMER;
			}
		}
		break;
	}

	iowrite8(sport->mcr, sport->port.membase + SELSERIAL_MCR);
}

/**
 * selserial_special_half_flow_store() - Set the half flow mode for the port
 *
 * @dev: Device struct to operate on
 * @attr: Attribute to query
 * @buffer: Buffer to read commands from
 * @size: Size of the buffer
 *
 * Return: @size for success, < 0 on failure
 */
static ssize_t selserial_special_half_flow_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buffer,
						 size_t size)
{
	int port = 0;
	const char *mode;
	int rc;

	/* Get the private data for the port */
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct selserial_private *priv = pci_get_drvdata(pdev);

	/* Parse the input string */
	rc = selserial_special_parse_store_string(buffer, size, &port, &mode);
	if (!rc && (port < priv->nr) && ((*mode == '0') || (*mode == '1'))) {
		/* Lock the port */
		spin_lock(&priv->ports[port].port.lock);

		/* Set it up according to the mode */
		__selserial_special_half_flow_store(&priv->ports[port], *mode);

		/* unlock the port */
		spin_unlock(&priv->ports[port].port.lock);

		rc = size;
	} else {
		rc = -EINVAL;
	}

	return rc;
}

/* Declare a device attribute for the half_flow sysfs file */
DEVICE_ATTR(half_flow,
	    S_IRUGO | S_IWUSR,
	    selserial_special_half_flow_show,
	    selserial_special_half_flow_store);

/**
 * selserial_special_init() - Create the files for sysfs
 *
 * @device: Device to attach the files to
 *
 * Return: 0 for success, < 0 on failure
 */
int selserial_special_init(struct device *device)
{
	int rc;

	if ((rc = device_create_file(device, &dev_attr_port_power)))
		goto out1;
	if ((rc = device_create_file(device, &dev_attr_loopback)))
		goto out2;
	if ((rc = device_create_file(device, &dev_attr_rs485)))
		goto out3;
	if ((rc = device_create_file(device, &dev_attr_half_flow)))
		goto out4;
	if ((rc = device_create_file(device, &dev_attr_half_duplex)))
		goto out5;

	goto out1;

out5:
	device_remove_file(device, &dev_attr_half_flow);
out4:
	device_remove_file(device, &dev_attr_rs485);
out3:
	device_remove_file(device, &dev_attr_loopback);
out2:
	device_remove_file(device, &dev_attr_port_power);
out1:
	return rc;
}

/**
 * selserial_special_cleanup() - Remove the files for sysfs
 *
 * @device: Device to remove the files from
 */
void selserial_special_cleanup(struct device *device)
{
	device_remove_file(device, &dev_attr_half_duplex);
	device_remove_file(device, &dev_attr_half_flow);
	device_remove_file(device, &dev_attr_rs485);
	device_remove_file(device, &dev_attr_loopback);
	device_remove_file(device, &dev_attr_port_power);
}
