//////////////////////////////////////////////////////////////////////////////
// COPYRIGHT (c) 2008 Schweitzer Engineering Laboratories, Pullman, Washington
///
/// @file selserial_uart_reg.h
///
/// @brief Register and bit definitions for the SEL UART
//////////////////////////////////////////////////////////////////////////////

#ifndef _SELSERIAL_UART_REG_H_INCLUDED
#define _SELSERIAL_UART_REG_H_INCLUDED

/// Register set size for a single port
#define SELSERIAL_PORT_REG_SIZE                  0x20

/// Fifo size for the serial ports
#define SELSERIAL_HW_FIFO_SIZE                   256

/// Clock for the UARTS
#define SELSERIAL_UART_CLOCK_FREQ                33000000

/// Data ready mask to provide some 8250-like functionality
#define SELSERIAL_DATA_READY                     0xFFFF0000

/// Mask for the data byte
#define SELSERIAL_DATA_BYTE                      0x000000FF

/// Mask to tell when there are errors
#define SELSERIAL_CHARACTER_ERR                  0x0000F000

//////////////////////////////////////////////////////////////////////////////
/// Receive Fifo Register
/// @remarks 16-bit register (data loss if accessed via 8-bits)
#define SELSERIAL_RX                             0
   /// MirroredBits Time-out Error
   #define SELSERIAL_RX_MB_TIMEOUT_ERR           0x0100
   /// MirroredBits Attention Error
   #define SELSERIAL_RX_MB_ATTN_ERR              0x0200
   /// MirroredBits Attention Message
   #define SELSERIAL_RX_MB_ATTN_MSG              0x0400
   /// MirroredBits OK Status
   #define SELSERIAL_RX_MBOK                     0x0800
   /// Break Error
   #define SELSERIAL_RX_BREAK_ERR                0x1000
   /// Overflow Error
   #define SELSERIAL_RX_OVERFLOW_ERR             0x2000
   /// Parity Error
   #define SELSERIAL_RX_PARITY_ERR               0x4000
   /// Frame Error
   #define SELSERIAL_RX_FRAME_ERR                0x8000

//////////////////////////////////////////////////////////////////////////////
/// Receive Fifo Byte count register
/// @remarks 16-bit register
#define SELSERIAL_RX_COUNT                       2

//////////////////////////////////////////////////////////////////////////////
/// Transmit Fifo Register
/// @remarks 16-bit register (may change in future)
#define SELSERIAL_TX                             4

/// Timed Transmitter PRE-mode
#define SELSERIAL_TM_MODE_PRE             0x4000
/// Enable Timed Transmitter
#define SELSERIAL_TM_EN                   0x8000

//////////////////////////////////////////////////////////////////////////////
/// Transmit Fifo Byte count register
/// @remarks 16-bit register
#define SELSERIAL_TX_COUNT                       6

//////////////////////////////////////////////////////////////////////////////
/// Modem Status Register
/// @remarks 8-bits
#define SELSERIAL_MSR                            8
   /// CTS changed
   #define SELSERIAL_MSR_DCTS                    0x01
   /// DSR changed
   #define SELSERIAL_MSR_DDSR                    0x02
   /// RING changed
   #define SELSERIAL_MSR_DRING                   0x04
   /// DCD changed
   #define SELSERIAL_MSR_DDCD                    0x08
   /// CTS state
   #define SELSERIAL_MSR_CTS                     0x10
   /// DSR state
   #define SELSERIAL_MSR_DSR                     0x20
   /// RING state
   #define SELSERIAL_MSR_RING                    0x40
   /// DCD state
   #define SELSERIAL_MSR_DCD                     0x80

//////////////////////////////////////////////////////////////////////////////
/// Modem Control Register
/// @remarks 8-bits
#define SELSERIAL_MCR                            9
   /// Set DTR high
   #define SELSERIAL_MCR_DTR                     0x01
   /// Set RTS high
   #define SELSERIAL_MCR_RTS                     0x02
   /// Set Break
   #define SELSERIAL_MCR_BRK                     0x04
   /// Enable Auto half-duplex control
   #define SELSERIAL_MCR_AUTO_RTS                0x08
   /// Enable Auto Hardware flow-control RTS pin
   #define SELSERIAL_MCR_AUTO_HS_RTS             0x10
   /// Enable Auto Hardware flow-control CTS pin
   #define SELSERIAL_MCR_AUTO_HS_CTS             0x20

//////////////////////////////////////////////////////////////////////////////
/// General Control register
/// @remarks 8-bits
#define SELSERIAL_GCR                            10
   /// set 485 mode
   #define SELSERIAL_GCR_485_MODE                0x01
   /// Port Power enable
   #define SELSERIAL_GCR_PORT_POWER              0x02
   /// Enable loopback mode
   #define SELSERIAL_GCR_LOOP                    0x04
   /// Enable Half Duplex Tx/Rx mode
   #define SELSERIAL_GCR_HALF_DUPLEX             0x08

//////////////////////////////////////////////////////////////////////////////
/// Timed Transmit count register
/// @remarks 8-bits
#define SELSERIAL_TM_COUNT_REG                   11

//////////////////////////////////////////////////////////////////////////////
/// Line Configuration register
/// @remarks 32-bit register
#define SELSERIAL_LCR                            12
   /// Baudrate Divisor Mask bit mask
   #define SELSERIAL_LCR_BAUD_DIV_MASK           0x0000FFFF
      /// Sets the Baud Rate divisor Bits
      /// @param reg LCR contents
      /// @param bauddiv New baud rate divisor
      #define SELSERIAL_SET_BAUD_DIV( reg, bauddiv ) \
         ((reg) | ((bauddiv) & (SELSERIAL_LCR_BAUD_DIV_MASK)))
   /// Fractional baud rate divisor bit mask
   #define SELSERIAL_LCR_FRAC_BAUD_MASK          0x000F0000
      /// Sets the Fractional Baud Rate divisor
      /// @param reg LCR contents
      /// @param fracbaud new fractional baud rate divisor
      #define SELSERIAL_SET_FRAC_BAUD( reg, fracbaud ) \
         ((reg) | (((fracbaud) << 16) & (SELSERIAL_LCR_FRAC_BAUD_MASK)))
   /// Number of data bits mask
   #define SELSERIAL_LCR_DATA_BITS_MASK          0x00F00000
      /// 5 Data bits
      #define SELSERIAL_LCR_5DB                  0x00500000
      /// 6 Data bits
      #define SELSERIAL_LCR_6DB                  0x00600000
      /// 7 Data bits
      #define SELSERIAL_LCR_7DB                  0x00700000
      /// 8 Data bits
      #define SELSERIAL_LCR_8DB                  0x00800000
   /// Enable Parity
   #define SELSERIAL_LCR_PARITY_EN               0x01000000
   /// Enable Even parity
   #define SELSERIAL_LCR_EVEN_PARITY             0x02000000
   /// Enable Sticky Parity
   #define SELSERIAL_LCR_STICKY_PARITY           0x04000000
   /// Enable two stop bits
   #define SELSERIAL_LCR_TWO_STOP                0x08000000
   /// Enable the receiver
   #define SELSERIAL_LCR_RX_EN                   0x10000000
   /// Enable the transmitter
   #define SELSERIAL_LCR_TX_EN                   0x20000000
   /// Flush the receive buffer
   #define SELSERIAL_LCR_RX_FLUSH                0x40000000
   /// Flush the transmit buffer
   #define SELSERIAL_LCR_TX_FLUSH                0x80000000
   /// Mask to use to clear all the set_termios type settings
   #define SELSERIAL_LCR_SET_TERMIOS_RESET       0xF0000000

//////////////////////////////////////////////////////////////////////////////
/// End of receive timer register
/// @remarks 16-bit wide
#define SELSERIAL_EORT_REG                       16

//////////////////////////////////////////////////////////////////////////////
/// End of Transmit timer Register
/// @remarks 16-bit wide
#define SELSERIAL_EOTT_REG                       18

//////////////////////////////////////////////////////////////////////////////
/// Receive fifo over-level threshold
/// @remarks 16-bit register
#define SELSERIAL_RX_FIFO_LEVEL_REG              20

//////////////////////////////////////////////////////////////////////////////
/// Transmit fifo under-level threshold
/// @remarks 16-bit register
#define SELSERIAL_TX_FIFO_LEVEL_REG              22

//////////////////////////////////////////////////////////////////////////////
/// Interrupt enable register
/// @remarks 16-bit register
#define SELSERIAL_IER                            24
   /// EORT Interrupt Status
   #define SELSERIAL_IER_EORT                    0x0001
   /// EOTT Interrupt Status
   #define SELSERIAL_IER_EOTT                    0x0002
   /// RX Level Interrupt Status
   #define SELSERIAL_IER_RX_LEVEL                0x0004
   /// TX Level Interrupt Status
   #define SELSERIAL_IER_TX_LEVEL                0x0008
   /// Break Interrupt Status
   #define SELSERIAL_IER_BREAK                   0x0010
   /// Overflow Error Interrupt Status
   #define SELSERIAL_IER_OVERFLOW                0x0020
   /// Parity Error Interrupt Status
   #define SELSERIAL_IER_PARITY                  0x0040
   /// Frame Error Interrupt Status
   #define SELSERIAL_IER_FRAME                   0x0080
   /// Change in CTS Interrupt Status
   #define SELSERIAL_IER_DCTS                    0x0100
   /// Change in DSR Interrupt Status
   #define SELSERIAL_IER_DDSR                    0x0200
   /// Change in Ring Interrupt Status
   #define SELSERIAL_IER_DRING                   0x0400
   /// Change in DCD Interrupt Status
   #define SELSERIAL_IER_DDCD                    0x0800
   /// MirroredBits OK Interrupt status
   #define SELSERIAL_IER_MBOK                    0x1000
   /// MirroredBits Data Ready Interrupt Status
   #define SELSERIAL_IER_MBDATA                  0x2000

//////////////////////////////////////////////////////////////////////////////
/// Interrupt status register
#define SELSERIAL_ISR                            26
   /// EORT Interrupt Status
   #define SELSERIAL_ISR_EORT                    0x0001
   /// EOTT Interrupt Status
   #define SELSERIAL_ISR_EOTT                    0x0002
   /// RX Level Interrupt Status
   #define SELSERIAL_ISR_RX_LEVEL                0x0004
   /// TX Level Interrupt Status
   #define SELSERIAL_ISR_TX_LEVEL                0x0008
   /// Break Interrupt Status
   #define SELSERIAL_ISR_BREAK                   0x0010
   /// Overflow Error Interrupt Status
   #define SELSERIAL_ISR_OVERFLOW                0x0020
   /// Parity Error Interrupt Status
   #define SELSERIAL_ISR_PARITY                  0x0040
   /// Frame Error Interrupt Status
   #define SELSERIAL_ISR_FRAME                   0x0080
   /// Change in CTS Interrupt Status
   #define SELSERIAL_ISR_DCTS                    0x0100
   /// Change in DSR Interrupt Status
   #define SELSERIAL_ISR_DDSR                    0x0200
   /// Change in Ring Interrupt Status
   #define SELSERIAL_ISR_DRING                   0x0400
   /// Change in DCD Interrupt Status
   #define SELSERIAL_ISR_DDCD                    0x0800
   /// MirroredBits OK Interrupt status
   #define SELSERIAL_ISR_MBOK                    0x1000
   /// MirroredBits Data Ready Interrupt Status
   #define SELSERIAL_ISR_MBDATA                  0x2000


#endif

// vim:sm:et:sw=3:sts=3

