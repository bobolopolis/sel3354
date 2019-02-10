//////////////////////////////////////////////////////////////////////////////
// COPYRIGHT (c) 2008 Schweitzer Engineering Laboratories, Pullman, Washington
///
/// @file selserial_ioctl.h
///
/// @brief IOCTLs to be used on the SELSerial driver
//////////////////////////////////////////////////////////////////////////////

#ifndef _SELSERIAL_UART_IOCTL_H_INCLUDED
#define _SELSERIAL_UART_IOCTL_H_INCLUDED

#include <linux/ioctl.h>
#include <linux/types.h>

/// Structure to set the fifo interrupt settings
struct selserial_fifo_int_settings {
   /// End of transmit timer
   __u16 eott;
   /// End of receive timer...if 0 is passed in, then defaults are set
   __u16 eort;
   /// tx level interrupt...if 0 is passed in, then defaults are set
   __u16 txcount;
   /// rx level interrupt...if 0 is passed in, then defaults are set
   __u16 rxcount;
};

/// enable timed transmitter for the next byte
#define SELSERIAL_TIMED_TX_EN             0x8000
/// use a predelay
#define SELSERIAL_TIMED_TX_PRE            0x4000

/// Timed transmitter settings for the next byte
struct selserial_timed_tx_settings {
   /// delay flags
   __u16 flags;
   /// length of dealy
   __u16 count;
};

/// enable mirrored bits
#define SELSERIAL_MB_EN                   0x0001
/// enable rx fifo bypass
#define SELSERIAL_MB_RX_BYPASS            0x0002
/// enable tx fifo bypass
#define SELSERIAL_MB_TX_BYPASS            0x0004
/// enable tx repeater mode
#define SELSERIAL_MB_TX_REPEAT            0x0008
/// enable loop mode
#define SELSERIAL_MB_LOOP_ENABLE          0x4000
/// enable character pacing
#define SELSERIAL_MB_PACE                 0x8000

/// Mirrored-Bits option structure
struct selserial_mb_options {
   /// option flags
   __u16 flags;
   /// RX id of the Mirrored-Bits device
   __u8  rx_id;
   /// TX id of the Mirrored-Bits device
   __u8  tx_id;
};

/// Mirrored-Bits timeout
#define SELSERIAL_MB_TIMEOUT              0x01
/// Mirrored-Bits attention error
#define SELSERIAL_MB_ATTN_ERR             0x02
/// Mirrored-Bits attention message
#define SELSERIAL_MB_ATTN_MSG             0x04
/// Mirrored-Bits status ok
#define SELSERIAL_MB_MBOK                 0x08
/// Mirrored-Bits break error
#define SELSERIAL_MB_BREAK                0x10
/// Mirrored-Bits overflow error ok
#define SELSERIAL_MB_OVERFLOW             0x20
/// Mirrored-Bits parity error
#define SELSERIAL_MB_PARITY               0x40
/// Mirrored-Bits framing error
#define SELSERIAL_MB_FRAME                0x80

/// Mirrored-Bits status ok
#define SELSERIAL_MBSTATE_MBOK             0x0800

/// Mirrored-Bits status structure
struct selserial_mb_status {
   /// status flags
   __u16 flags;
   /// Mirrored-Bits state
   __u16 mbstate;
};

/// Transmitter state structure
struct selserial_eoxt_state {
   /// eott expired
   __u16 eott_exp_cnt;
   /// bytes in UART tx fifo
   __u16 eort_exp_cnt;
};

/// Magic number to identify the SELSERIAL ioctls
#define SELSERIAL_IOCTL_MAGIC_NUMBER         0xF0

/// set timed transmitter settings
#define SELSERIAL_IOCSTIMETX       _IOW( SELSERIAL_IOCTL_MAGIC_NUMBER, 0, \
                                         struct selserial_timed_tx_settings )

/// get timed transmitter settings
#define SELSERIAL_IOCGTIMETX       _IOR( SELSERIAL_IOCTL_MAGIC_NUMBER, 1, \
                                         struct selserial_timed_tx_settings )

/// Set the FIFO interrupt handler settings
#define SELSERIAL_IOCSFIFOINT      _IOW( SELSERIAL_IOCTL_MAGIC_NUMBER, 2, \
                                         struct selserial_fifo_int_settings )

/// Get the FIFO interrupt handler settings
#define SELSERIAL_IOCGFIFOINT      _IOR( SELSERIAL_IOCTL_MAGIC_NUMBER, 3, \
                                         struct selserial_fifo_int_settings )

/// Enable/Disable Auto RTS
#define SELSERIAL_IOCSAUTORTS      _IOW( SELSERIAL_IOCTL_MAGIC_NUMBER, 4, int )

/// Set mirrored-Bits settings
#define SELSERIAL_IOCSMBOPTIONS    _IOW( SELSERIAL_IOCTL_MAGIC_NUMBER, 5, \
                                         struct selserial_mb_options )

/// Get mirrored-Bits settings
#define SELSERIAL_IOCGMBOPTIONS    _IOR( SELSERIAL_IOCTL_MAGIC_NUMBER, 6, \
                                         struct selserial_mb_options )

/// Get Mirrored-Bits State
#define SELSERIAL_IOCGMBSTATE      _IOR( SELSERIAL_IOCTL_MAGIC_NUMBER, 7, \
                                         struct selserial_mb_status )

/// Get Transmit Fifo State
#define SELSERIAL_IOCGEOXTFIFOSTATE _IOR( SELSERIAL_IOCTL_MAGIC_NUMBER, 8, \
                                          struct selserial_eoxt_state )

/// Get the port power state for the port
#define SELSERIAL_IOCGPOWERSTATE   _IOR( SELSERIAL_IOCTL_MAGIC_NUMBER, 9, int )

/// Set the port power state for the port
#define SELSERIAL_IOCSPOWERSTATE   _IOW( SELSERIAL_IOCTL_MAGIC_NUMBER, 10, int )

/// Get the rs485 state for the port
#define SELSERIAL_IOCG485STATE     _IOR( SELSERIAL_IOCTL_MAGIC_NUMBER, 11, int )

/// Set the rs485 state for the port
#define SELSERIAL_IOCS485STATE     _IOW( SELSERIAL_IOCTL_MAGIC_NUMBER, 12, int )

#endif

// vim:et:sm:sw=3:sts=3

