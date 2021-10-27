/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2008 Schweitzer Engineering Laboratories, Pullman, Washington
 *
 * Register and bit definitions for the SEL UART
 */

#ifndef _SELSERIAL_UART_REG_H_INCLUDED
#define _SELSERIAL_UART_REG_H_INCLUDED

/* Register set size for a single port */
#define SELSERIAL_PORT_REG_SIZE 0x20

/* FIFO size for the serial ports */
#define SELSERIAL_HW_FIFO_SIZE 256

/* Clock for the UARTs */
#define SELSERIAL_UART_CLOCK_FREQ 33000000

/* Data ready mask to provide some 8250-like functionality */
#define SELSERIAL_DATA_READY 0xFFFF0000

/* Mask for the data byte */
#define SELSERIAL_DATA_BYTE 0x000000FF

/* Mask to tell when there are errors */
#define SELSERIAL_CHARACTER_ERR 0x0000F000

/*
 * Receive FIFO Register. 16 bit register (data loss if accessed as 8 bit)
 */
#define SELSERIAL_RX                0
/* MirroredBits Time-out Error */
#define SELSERIAL_RX_MB_TIMEOUT_ERR BIT(8)
/* MirroredBits Attention Error */
#define SELSERIAL_RX_MB_ATTN_ERR    BIT(9)
/* MirroredBits Attention Message */
#define SELSERIAL_RX_MB_ATTN_MSG    BIT(10)
/* MirroredBits OK Status */
#define SELSERIAL_RX_MBOK           BIT(11)
/* Break Error */
#define SELSERIAL_RX_BREAK_ERR      BIT(12)
/* Overflow Error */
#define SELSERIAL_RX_OVERFLOW_ERR   BIT(13)
/* Parity Error */
#define SELSERIAL_RX_PARITY_ERR     BIT(14)
/* Frame Error */
#define SELSERIAL_RX_FRAME_ERR      BIT(15)

/*
 * Receive FIFO Byte count register (16 bits)
 */
#define SELSERIAL_RX_COUNT          2

/*
 * Transmit FIFO Register (16 bits)
 */
#define SELSERIAL_TX                4

/* Timed Transmitter PRE-mode */
#define SELSERIAL_TM_MODE_PRE       BIT(14)
/* Enable Timed Transmitter */
#define SELSERIAL_TM_EN             BIT(15)

/*
 * Transmit FIFO Byte count register (16 bits)
 */
#define SELSERIAL_TX_COUNT          6
/*
 * Modem Status Register (8 bits)
 */
#define SELSERIAL_MSR       8
/* CTS changed */
#define SELSERIAL_MSR_DCTS  BIT(0)
/* DSR changed */
#define SELSERIAL_MSR_DDSR  BIT(1)
/* RING changed */
#define SELSERIAL_MSR_DRING BIT(2)
/* DCD changed */
#define SELSERIAL_MSR_DDCD  BIT(3)
/* CTS state */
#define SELSERIAL_MSR_CTS   BIT(4)
/* DSR state */
#define SELSERIAL_MSR_DSR   BIT(5)
/* RING state */
#define SELSERIAL_MSR_RING  BIT(6)
/* DCD state */
#define SELSERIAL_MSR_DCD   BIT(7)

/*
 * Modem Control Register (8 bits)
 */
#define SELSERIAL_MCR             9
/* Set DTR high */
#define SELSERIAL_MCR_DTR         BIT(0)
/* Set RTS high */
#define SELSERIAL_MCR_RTS         BIT(1)
/* Set Break */
#define SELSERIAL_MCR_BRK         BIT(2)
/* Enable Auto half-duplex control */
#define SELSERIAL_MCR_AUTO_RTS    BIT(3)
/* Enable Auto Hardware flow-control RTS pin */
#define SELSERIAL_MCR_AUTO_HS_RTS BIT(4)
/* Enable Auto Hardware flow-control CTS pin */
#define SELSERIAL_MCR_AUTO_HS_CTS BIT(5)

/*
 * General Control register (8 bits)
 */
#define SELSERIAL_GCR             10
/* Set 485 mode */
#define SELSERIAL_GCR_485_MODE    BIT(0)
/* Port Power enable */
#define SELSERIAL_GCR_PORT_POWER  BIT(1)
/* Enable loopback mode */
#define SELSERIAL_GCR_LOOP        BIT(2)
/* Enable Half Duplex TX/RX mode */
#define SELSERIAL_GCR_HALF_DUPLEX BIT(3)

/*
 * Timed Transmit count register (8 bits)
 */
#define SELSERIAL_TM_COUNT_REG    11

/*
 * Line Configuration register (32 bits)
 */
#define SELSERIAL_LCR                   12
/* Baudrate Divisor Mask bit mask */
#define SELSERIAL_LCR_BAUD_DIV_MASK     0x0000FFFF
/* Sets the Baud Rate divisor Bits
 * @reg: LCR contents
 * @bauddiv: New baud rate divisor
 */
#define SELSERIAL_SET_BAUD_DIV( reg, bauddiv ) \
	((reg) | ((bauddiv) & (SELSERIAL_LCR_BAUD_DIV_MASK)))
/* Fractional baud rate divisor bit mask */
#define SELSERIAL_LCR_FRAC_BAUD_MASK    0x000F0000
/* Sets the Fractional Baud Rate divisor
 * @reg: LCR contents
 * @fracbaud: new fractional baud rate divisor
 */
#define SELSERIAL_SET_FRAC_BAUD( reg, fracbaud ) \
	((reg) | (((fracbaud) << 16) & (SELSERIAL_LCR_FRAC_BAUD_MASK)))
/* Number of data bits mask */
#define SELSERIAL_LCR_DATA_BITS_MASK    0x00F00000
/* 5 Data bits */
#define SELSERIAL_LCR_5DB               0x00500000
/* 6 Data bits */
#define SELSERIAL_LCR_6DB               0x00600000
/* 7 Data bits */
#define SELSERIAL_LCR_7DB               0x00700000
/* 8 Data bits */
#define SELSERIAL_LCR_8DB               0x00800000
/* Enable Parity */
#define SELSERIAL_LCR_PARITY_EN         BIT(24)
/* Enable Even parity */
#define SELSERIAL_LCR_EVEN_PARITY       BIT(25)
/* Enable Sticky Parity */
#define SELSERIAL_LCR_STICKY_PARITY     BIT(26)
/* Enable two stop bits */
#define SELSERIAL_LCR_TWO_STOP          BIT(27)
/* Enable the receiver */
#define SELSERIAL_LCR_RX_EN             BIT(28)
/* Enable the transmitter */
#define SELSERIAL_LCR_TX_EN             BIT(29)
/* Flush the receive buffer */
#define SELSERIAL_LCR_RX_FLUSH          BIT(30)
/* Flush the transmit buffer */
#define SELSERIAL_LCR_TX_FLUSH          BIT(31)
/* Mask to use to clear all the set_termios type settings */
#define SELSERIAL_LCR_SET_TERMIOS_RESET 0xF0000000

/*
 * End of receive timer register (16 bits)
 */
#define SELSERIAL_EORT_REG                    16

/*
 * End of Transmit timer Register (16 bits)
 */
#define SELSERIAL_EOTT_REG                    18

/*
 * Receive FIFO over-level threshold (16 bits)
 */
#define SELSERIAL_RX_FIFO_LEVEL_REG           20

/*
 * Transmit FIFO under-level threshold (16-bits)
 */
#define SELSERIAL_TX_FIFO_LEVEL_REG           22

/*
 * Interrupt enable register (16 bits)
 */
#define SELSERIAL_IER                         24
/* EORT Interrupt Status */
#define SELSERIAL_IER_EORT                    BIT(0)
/* EOTT Interrupt Status */
#define SELSERIAL_IER_EOTT                    BIT(1)
/* RX Level Interrupt Status */
#define SELSERIAL_IER_RX_LEVEL                BIT(2)
/* TX Level Interrupt Status */
#define SELSERIAL_IER_TX_LEVEL                BIT(3)
/* Break Interrupt Status */
#define SELSERIAL_IER_BREAK                   BIT(4)
/* Overflow Error Interrupt Status */
#define SELSERIAL_IER_OVERFLOW                BIT(5)
/* Parity Error Interrupt Status */
#define SELSERIAL_IER_PARITY                  BIT(6)
/* Frame Error Interrupt Status */
#define SELSERIAL_IER_FRAME                   BIT(7)
/* Change in CTS Interrupt Status */
#define SELSERIAL_IER_DCTS                    BIT(8)
/* Change in DSR Interrupt Status */
#define SELSERIAL_IER_DDSR                    BIT(9)
/* Change in Ring Interrupt Status */
#define SELSERIAL_IER_DRING                   BIT(10)
/* Change in DCD Interrupt Status */
#define SELSERIAL_IER_DDCD                    BIT(11)
/* MirroredBits OK Interrupt Status */
#define SELSERIAL_IER_MBOK                    BIT(12)
/* MirroredBits Data Ready Interrupt Status */
#define SELSERIAL_IER_MBDATA                  BIT(13)

/*
 * Interrupt status register
 */
#define SELSERIAL_ISR                         26
/* EORT Interrupt Status */
#define SELSERIAL_ISR_EORT                    BIT(0)
/* EOTT Interrupt Status */
#define SELSERIAL_ISR_EOTT                    BIT(1)
/* RX Level Interrupt Status */
#define SELSERIAL_ISR_RX_LEVEL                BIT(2)
/* TX Level Interrupt Status */
#define SELSERIAL_ISR_TX_LEVEL                BIT(3)
/* Break Interrupt Status */
#define SELSERIAL_ISR_BREAK                   BIT(4)
/* Overflow Error Interrupt Status */
#define SELSERIAL_ISR_OVERFLOW                BIT(5)
/* Parity Error Interrupt Status */
#define SELSERIAL_ISR_PARITY                  BIT(6)
/* Frame Error Interrupt Status */
#define SELSERIAL_ISR_FRAME                   BIT(7)
/* Change in CTS Interrupt Status */
#define SELSERIAL_ISR_DCTS                    BIT(8)
/* Change in DSR Interrupt Status */
#define SELSERIAL_ISR_DDSR                    BIT(9)
/* Change in Ring Interrupt Status */
#define SELSERIAL_ISR_DRING                   BIT(10)
/* Change in DCD Interrupt Status */
#define SELSERIAL_ISR_DDCD                    BIT(11)
/* MirroredBits OK Interrupt Status */
#define SELSERIAL_ISR_MBOK                    BIT(12)
/* MirroredBits Data Ready Interrupt Status */
#define SELSERIAL_ISR_MBDATA                  BIT(13)

#endif
