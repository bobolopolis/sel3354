///////////////////////////////////////////////////////////////////////////////
/// COPYRIGHT (C) 2010 Schweitzer Engineering Laboratories, Pullman, WA
///////////////////////////////////////////////////////////////////////////////
///  @file
///  irig.c
///
///  @brief
///  IRIG driver code
///
///  Requirements Spec: N/A
///
///  Design Description:
///     This driver follows your standard PCI linux driver layout with
///     init/cleanup, open/close, poll/read/write functions.
///
///     Reading incoming IRIG data:
///     The incoming IRIG signal generates a 1 pulse per second (pps) signal,
///     the rising edge of which indicates when a second has rolled over.  The
///     timestamp of this rising edge is stored in the ts_irig_1pps register in
///     the FPGA and (if it's enabled) an interrupt is generated.  The timestamp
///     is also stored in a per-thread data store.  When either the poll or read
///     functions start to execute they compare the current 1pps timestamp with
///     the last one stored.  If they're different then there's new data to be
///     read, otherwise it's the same data as was read last time and the
///     poll/read function will suspend the thread on the Irig_Read_Q wait
///     queue.  When the interrupt comes in (indicating new data is available)
///     irig_isr() clears the interrupt and wakes the suspended thread.
///
///     Writing outgoing IRIG data:
///     Writing data to the IRIG encode works in a similar way.  New data
///     shouldn't be written until the old data has been encoded and sent which
///     is triggered by the rising edge of the 1pps signal from the PLL.  The
///     rising edge of the 1pps signal also generates an interrupt, and the ISR
///     sets a flag called Ready_To_Send.  When set Ready_To_Send indicates the
///     old data has beed sent and it's safe to write more data to the encoder.
///     When clear it indicates the encoder isn't ready for new data yet.  So,
///     if the poll/write function finds Ready_To_Send is clear it knows the
///     encoder isn't ready for new data yet and blocks the thread.  The ISR
///     runs, sets Ready_To_Send, and wakes the thread.  The write completes,
///     clears Ready_To_Send, and the cycle repeats.
///
///     NOTE: It's inherently dangerous for multiple threads to be writing data
///           to the encoder because they may put conflicting data out there
///           many IRIG slaves get very un-happy if subsequent messages contain
///           conflicting data (i.e., two messages with the same time or two
///           messages with more than a second between them).  Therefore, this
///           driver can only be opened for writing by one thread at a time.
///           There is no limit on the number of readers.  This implies that if
///           there is a seperate thread for writing data, all the threads that
///           are reading data have to open /dev/irig with O_RDONLY.
///
///     See /vobs/platform/33xx_35xx_platform/rtl/irig_controller/doc/irig_controller_cis.doc
///     for details on the HW design.
///
///     Calibrating FRC ticks per second value:
///
///     The FRC ticks per second value (field frc_tick in the irig_time struct)
///     is computed in the ISR on IRIG_PPS_INT interrupts (incoming IRIG PPS
///     pulse received interrupt) and stored in irig_time struct returned by
///     irig_read.  The ISR "filters" the value as follows:
///     * Values are considered "bad" and thrown out if they are more than 1%
///       above or below the value of NOMINAL_FRC_FREQ.  The value that gets
///       used (put in frc_tick) when a bad value occurs is:
///       * If the previous value was good, the previous value is used.
///       * If the previous value was bad (i.e. we have seen two bad
///         values in a row), the NOMINAL_FRC_FREQ value is used.
///     * Frc_Ticks_Value_Good_Shift is a bitmask indicating which of the last
///       32 samples were good or bad.  The least significant bit corresponds
///       to the most recent sample, the most significant bit is the oldest
///       sample.  A bit with value 1 indicates the sample was good, 0
///       indicates the sample was bad.
///       * Frc_Ticks_Value_Good_Shift is initialized to indicate that all
///         (32) previous samples were bad.
///       + Note that the first value will be bad because it is computed as
///         a difference to the previous sample (so, since there is no
///         previous sample the computation is bogus.) Since
///         Frc_Ticks_Value_good_Shift is initialized to indicate all previous
///         samples are bad, the NOMINAL_FRC_FREQ value will be returned.
///       + Note that it would be easy to have the most recent "good" value
///         used when up to N ( 1 <= N < 32 ) bad values in a row have
///         occurred by changing the value used to mask
///         Frc_Ticks_Value_Good_Shift in the ISR.
///
///    Read Return Value:
///
///    The read operation now returns EIO ("Input/Ouput Error") if the IRIG
///    input to the box is not good.
///
///////////////////////////////////////////////////////////////////////////////

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#include <asm/semaphore.h>
#else
#include <linux/semaphore.h>
#endif

#include "linux/irig.h"
#include "linux/sel_class.h"
#include "linux/pci_ids_sel.h"

#define IRIG_DATA_SIZE 4
#define GOOD_PARITY_TESTS   5          /// The number of consecutive correct parity checks
                                       /// that are required to determine whether or not
                                       /// the IRIG source is C37.118 compliant.
#define NOMINAL_FRC_FREQ    33000000   /// The FRC should be updated at a 33 MHz rate.

// Values for the 1% margin on acceptable FRC TicksPerSecond values...
// If NOMINAL is 33,000,000, then MIN is 32,670,000 and MAX is 33,330,000
#define FRC_TPS_MARGIN (NOMINAL_FRC_FREQ / 100)
#define FRC_TPS_MIN    (NOMINAL_FRC_FREQ - FRC_TPS_MARGIN)
#define FRC_TPS_MAX    (NOMINAL_FRC_FREQ + FRC_TPS_MARGIN)

// IRIG register offsets in the FPGA, refer to the IRIG controller design
// document for a detailed description of the registers.
#define CONFIG_REG            0
#define PLL_CONTROL_REG       1
#define PLL_CPP_REG           2
#define PLL_ERROR_REG         3
#define PLL_DELAY_REG         4
#define FRC_REG               5
#define TS_IRIG_PPS           6
#define TS_EXT_PPS            7
#define TS_DIFF_PPS           8
#define INCOMING_IRIG_DATA    9
#define OUTGOING_IRIG_DATA   13
#define IRIG_IRQ_MASK_REG    17
#define IRIG_IRQ_STATUS_REG  18
#define PPS_TIMER_REG        19
#define TS_PPS_PLL_REG       20

// IRQ mask/status registers
#define IRIG_PPS_INT        0x1
#define EXTERNAL_PPS_INT    0x2
#define PLL_PPS_INT         0x4
#define IRIG_INT_MASK (IRIG_PPS_INT | EXTERNAL_PPS_INT | PLL_PPS_INT)

#define INTERNAL_PPS_TIMER_MASK 0x03ffffff // Can only write 26 bits to
                                           // PPS timer register
static __u32 *Data_Ptr;           // Pointer to the IRIG mem region of the FPGA
static __u32 Irig_1pps_Ts = 0;    // Current timestamp of decoder 1pps signal
static __u8 Ready_To_Send = 0;    // 0 = encoder has old data
                                  // 1 = encoder is ready for new data
static __u8 Open_For_Write = 0;   // 0 = not opend for writing yet
                                  // 1 = opened for writing
static __s32 Time_Adjustment = 0; // The number of minutes to shift the incoming
                                  // IRIG time

// Ticks per second value computed in the ISR
static __u32 Frc_Ticks_Per_Second = NOMINAL_FRC_FREQ;

// Shift register: bits indicate if the previous Frc Ticks values were good.
// Initialize to 0x0 to indicate all previous samples were bad.
static __u32 Frc_Ticks_Value_Good_Shift = 0x0;

static __u8  Irig_Input_Good = 0; // 0 = Irig input is not good

#ifdef DECLARE_MUTEX
DECLARE_MUTEX(IRIG_Write_Mutex); // Controls access to the Open_For_Write flag
#else
DEFINE_SEMAPHORE(IRIG_Write_Mutex);
#endif

DECLARE_WAIT_QUEUE_HEAD(Irig_Read_Q);
DECLARE_WAIT_QUEUE_HEAD(Irig_Write_Q);

// function protos
static int irig_open(struct inode *inode, struct file *filp);
static int irig_release(struct inode *inode, struct file *filp);
static ssize_t irig_read(struct file *filp, char __user *buf, size_t count,
                         loff_t *loc);
static ssize_t irig_write(struct file *filp, const char __user *buf, size_t count,
                          loff_t *loc);
static int irig_ioctl(struct inode *inode, struct file *filp,
                      unsigned int cmd, unsigned long arg);
static unsigned int irig_poll(struct file *filp, poll_table *wait);
static int __devinit irig_probe(struct pci_dev *dev,
                                const struct pci_device_id *id);
static void __devexit irig_remove(struct pci_dev *dev);

static struct file_operations Irig_Fops =
{
   .owner   = THIS_MODULE,
   .read    = irig_read,
   .write   = irig_write,
   .ioctl   = irig_ioctl,
   .open    = irig_open,
   .release = irig_release,
   .poll    = irig_poll,
   .llseek  = no_llseek,
};

static struct __devinitdata pci_device_id Irig_Pci_Tbl[] =
{
   {PCI_DEVICE(PCI_VENDOR_ID_SEL, PCI_DEVICE_ID_SEL_IRIG), 0, 0, 0},
   {0,} // terminate list
};
MODULE_DEVICE_TABLE(pci, Irig_Pci_Tbl);

static struct pci_driver Irig_Pci_driver =
{
   .name = "irig_pci",
   .id_table = Irig_Pci_Tbl,
   .probe = irig_probe,
   .remove = __devexit_p(irig_remove),
};

// Character driver stuff
static struct cdev *Irig_Cdev;
static dev_t Dev_No;

/////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_isr
///
///  @brief
///  Wake anyone waiting for new IRIG data to come in or waiting to send
///     IRIG data out
///
///  @param
///      INPUTS:    irq - unused, required by API
///              dev_id - unused, required by API
///
///  @remarks
///     Note that the first time this function is called, Irig_1pps_Ts will
///     have been initialized to 0, so the calculated Frc_Ticks_Per_Second
///     will probably be bogus.  Since Frc_Ticks_Value_Good_Shift has been
///     initialized to 0, Frc_Ticks_Per_Second will be set to NOMINAL_FRC_FREQ.
///
///  @param
///     OUTPUTS: ret - either the interrupt was handled or it wasn't ours
///
/////////////////////////////////////////////////////////////////////////////
static irqreturn_t irig_isr(int irq, void *dev_id)
{
   irqreturn_t ret;
   __u32 irig_irq;
   __u32 last_ts;   // Temporarily store the last IRIG_1pps_Ts value
   __u32 last_tps;  // Temporarily store the last Frc_Ticks_Per_Second

   // Because we're using a shared interrupt it's possible that it's not for us
   // and we're going to assume that is the case
   ret = IRQ_NONE;

   // Get the IRQ status from the FPGA
   irig_irq = ioread32(Data_Ptr + IRIG_IRQ_STATUS_REG);

   //
   // Handle interrupt from incoming IRIG PPS
   //
   if((irig_irq & IRIG_PPS_INT) == IRIG_PPS_INT)
   {
      // Clear the incoming interrupt
      iowrite32((irig_irq & IRIG_PPS_INT), Data_Ptr + IRIG_IRQ_STATUS_REG);

      // If we got an IRIG interrupt the IRIG input must be good...
      Irig_Input_Good = 1;

      // Store last Irig_1pps_Ts to compute TPS
      last_ts = Irig_1pps_Ts;

      // Get the new timestamp
      Irig_1pps_Ts = ioread32(Data_Ptr + TS_IRIG_PPS);

      // Store off the last TPS value so we can re-use it if the new one is
      // out of range
      last_tps = Frc_Ticks_Per_Second;
      Frc_Ticks_Per_Second = Irig_1pps_Ts - last_ts; // Compute TPS

      // Sanity check on the value of FRC_Ticks_Per_Second...
      if( Frc_Ticks_Per_Second < FRC_TPS_MIN || FRC_TPS_MAX < Frc_Ticks_Per_Second )
      {
         //
         // Bad value: outside acceptable margin range
         //

         // The 0x1 here could be changed to require more than 2 bad samples
         // in a row before back-tracking to use the NOMINAL value.
         //   0x3 - require 3 bad samples in a row
         //   0x7 - require 4 bad samples in a row
         //   0xF - require 5 bad samples in a row
         //   etc...
         if( (Frc_Ticks_Value_Good_Shift & 0x1) == 0 )
         {
            // Last sample was bad - so two (or more) samples in a row have
            // been bad: set the NOMINAL value...
            Frc_Ticks_Per_Second = NOMINAL_FRC_FREQ;

         }
         else
         {
            //  This sample is bad, but the last sample was good - send it out
            //  again
            Frc_Ticks_Per_Second = last_tps;
         }

         // Shift a 0 into the shift register to indicate this value was bad
         Frc_Ticks_Value_Good_Shift <<= 1;
      }
      else
      {
         //
         // Good value
         //

         // Frc_Ticks_Per_Second value was within the good margin, shift
         // a 1 onto the least-significant bit of the Shift value.
         Frc_Ticks_Value_Good_Shift <<= 1;
         Frc_Ticks_Value_Good_Shift |= 0x1;
      }

      wake_up_interruptible(&Irig_Read_Q);
      ret = IRQ_HANDLED;
   }

   //
   // Handle interrupt from Internal PPS counter
   //
   if((irig_irq & PLL_PPS_INT) == PLL_PPS_INT)
   {
      // Clear the outgoing interrupt
      iowrite32((irig_irq & PLL_PPS_INT), Data_Ptr + IRIG_IRQ_STATUS_REG);

      // Set the flag indicating we just sent an IRIG message and
      // wake the thread
      Ready_To_Send = 1;
      wake_up_interruptible(&Irig_Write_Q);

      // If IRIG is bad, wake up any reads so that we don't block forever
      // waiting for an IRIG_PPS_INT to happen...
      if( (ioread32( Data_Ptr + CONFIG_REG ) & ( 1 << IRIG_CONFIG_IRIG_GOOD )) == 0 )
      {
         Irig_Input_Good = 0;  // Indicate that IRIG is not good
         wake_up_interruptible(&Irig_Read_Q);
      }

      ret = IRQ_HANDLED;
   }

   return ret;
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_open
///
///  @brief
///  Called when a user space app opens /dev/irig
///
///  @param
///      INPUTS: inode - the /dev/irig inode structure
///               filp - the /dev/irig file structure for this thread
///
///  @param
///     OUTPUTS: ret -            0 = successfully opened the device
///                          ENOMEM = kmalloc failed
///                     ERESTARTSYS = couldn't get the mutex
///                           EBUSY = /dev/irig is already open for writing
///
////////////////////////////////////////////////////////////////////////////////
static int irig_open(struct inode *inode, struct file *filp)
{
   int ret;

   ret = 0; // Assume success

   if(down_interruptible(&IRIG_Write_Mutex) != 0)
      ret = -ERESTARTSYS;
   else if(Open_For_Write == 1 && (filp->f_mode & FMODE_WRITE) == FMODE_WRITE)
   {
      // Somebody already has the driver open for writing
      ret = -EBUSY;
      up(&IRIG_Write_Mutex);
   }
   else
   {
      if((filp->f_mode & FMODE_WRITE) == FMODE_WRITE)
      {
         Open_For_Write = 1; // Indicate we're now open for writing
      }

      up(&IRIG_Write_Mutex);

      // Get the current PPS timestamp and save it away for future use.
      // NOTE: About the cast here, private_data is of type void *, and I can't
      // change that because is core kernel code.  I don't want to use
      // private_data as a pointer I want to use it as a uint32, but the gnu
      // compiler won't let me cast it that way, so I have to cast what I'm
      // putting into it as a void * and make sure that I extract the data
      // later as a uint32
      // NOTE2 (eugebo): The cast to void* is not 64-bit safe (32-bit integer
      //                 cast to a 64-bit pointer), so an intermediate cast
      //                 to increase the size of the integer is needed.
      (filp->private_data) = (void *)(unsigned long)(ioread32(Data_Ptr + TS_IRIG_PPS));

      // Always read and write from the begining of the IRIG device
      nonseekable_open(inode, filp);
   }

   return ret;
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_release
///
///  @brief
///  Called when a user space app closes /dev/irig
///
///  @param
///      INPUTS: inode - the /dev/irig inode structure
///               filp - the /dev/irig file structure for this thread
///
///  @param
///     OUTPUTS: always return success
///
////////////////////////////////////////////////////////////////////////////////
static int irig_release(struct inode *inode, struct file *filp)
{
   // Let other writers in
   if((filp->f_mode & FMODE_WRITE) == FMODE_WRITE)
      Open_For_Write = 0;

   return 0;
}

/////////////////////////////////////////////////////////////////////////////////
///   Determine whether or not the given year is a leap year
///
///   @param year - IN:  The year to be tested
///
///   @return TRUE if it is a leap year, FALSE if not.
///
/////////////////////////////////////////////////////////////////////////////////
static __u8 is_leapyear(unsigned int year)
{
   return ((year % 4) == 0 && (((year % 100) != 0 || (year % 400) == 0)));
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  convert_to_utc
///
///  @brief
///  Convert the given IRIG time from local time to UTC time.
///
///  @param
///      INPUTS:  itime - The IRIG time to be modified
///
///  @param
///     OUTPUTS: itime - The modified IRIG time.
///
///  @notes
///  IRIG provides local time.  NTPD wants UTC time.  If the IRIG data contained
///  a time offset then we know how to convert to UTC.
///  Local Time = UTC Time + Time Offset thus UTC Time = Local Time - Time Offset
///
////////////////////////////////////////////////////////////////////////////////
static void convert_to_utc(struct irig_time *itime)
{
   __s32 time_offset;
   __s32 ytd_minutes, tot_minutes;

   /// The provided time offset is scaled by 10 to allow it to be an integer and
   /// still represent the possibility of a 1/2 hour adjustment.

   /// Convert the given offset to minutes (remove the x10 scaling).
   time_offset = itime->time_offset * 6;
   ytd_minutes = itime->day * 24 * 60 + itime->hour * 60 + itime->minute;

   /// Subtract the offset to get UTC time
   ytd_minutes -= time_offset;

   /// Now determine whether or not we crossed a year boundary
   tot_minutes = (365 + is_leapyear(itime->year)) * 24 * 60;

   if(ytd_minutes < 0)
   {
      /// Need to roll back a year, but how many minutes were in last year?
      if(is_leapyear(itime->year - 1 + 1900) == 1)
         ytd_minutes += 366 * 24 * 60;
      else
         ytd_minutes += 365 * 24 * 60;

      itime->year--;
   }
   else if(ytd_minutes > tot_minutes)
   {
      /// Move to next year
      itime->year++;
      ytd_minutes -= tot_minutes;
   }

   /// Now we should have a reasonable value for ytd_minutes.  Need to convert it
   /// back to the broken-down form.
   itime->day = ytd_minutes / (24 * 60);
   ytd_minutes -= (itime->day * 24 * 60);
   itime->hour = ytd_minutes / 60;
   ytd_minutes -= (itime->hour * 60);
   itime->minute = ytd_minutes;

}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  apply_offsets
///
///  @brief
///  Shift the given time by 1 second and time_offset minutes forward.
///
///  @param
///      INPUTS:  itime - The IRIG time to be shifted
///
///  @param
///     OUTPUTS: itime - The shifted IRIG time.
///
///  @notes
///  The incoming IRIG time packet takes almost a full second to arrive and be
///  decoded.  In fact, the flag that tells us when to process that IRIG packet
///  is when the packet_frc value changes which is actually the FRC for the next
///  packet.  In order to keep the FRC accuracy the best way to use the received
///  IRIG time is to move it forward by 1 second.
///
///  It is also possible that a user has specified an offset (in minutes) to shift
///  the incoming IRIG time.  This is primarily for situations like one where you
///  are receiving IRIG from a timezone other than the one you are in.  In this case
///  you can shift that incoming IRIG time by the appropriate amount before it is
///  used.
///
///  This function asssumes that itime is provided in UTC in which case DST is
///  not an issue.
///
////////////////////////////////////////////////////////////////////////////////
static void apply_offsets(struct irig_time *itime)
{
   __s16 new_days, new_hours, new_minutes, tot_days;
   __s32 tmp_adjust;

   itime->second++;

   /// The offset is provided in minutes, but can be either positive or negative.
   new_days = Time_Adjustment / (60*24);

   /// Remove the days from the total
   tmp_adjust = Time_Adjustment - new_days * 60 * 24;
   new_hours = tmp_adjust / 60;

   /// Remove the hours from the total
   tmp_adjust = tmp_adjust - new_hours * 60;
   new_minutes = tmp_adjust;

   itime->day += new_days;
   itime->hour += new_hours;
   itime->minute += new_minutes;

   if(itime->second >= 60)
   {
      /// Overran into the next minute
      itime->minute++;
      itime->second -= 60;
   }

   if(itime->minute >= 60)
   {
      /// Overran into the next hour
      itime->hour++;
      itime->minute -= 60;
   }
   else if(itime->minute < 0)
   {
      /// Moved into the previous hour
      itime->hour--;
      itime->minute += 60;
   }

   if(itime->hour >= 24)
   {
      /// Overran into the next day
      itime->day++;
      itime->hour -= 24;
   }
   else if(itime->hour < 0)
   {
      /// Moved into the previous day
      itime->day--;
      itime->hour += 24;
   }

   tot_days = 365 + is_leapyear(itime->year);

   if(itime->day >= tot_days)
   {
      /// Moved into the next year
      itime->year++;
      itime->day -= tot_days;
   }
   else if(itime->day < 0)
   {
      /// Moved back into last year
      itime->year--;

      tot_days = 365 + is_leapyear(itime->year);

      itime->day += tot_days;
   }

}

////////////////////////////////////////////////////////////////////////////////////
///   Take the given 32-bit quantity and add one to the given parity total for every
///   bit that is set.  This will be used to calculate a parity bit for outgoing IRIG.
///
///   @param reg        - IN:  The 32-bit quantity to test
///   @param parity - IN/OUT:  The parity value
///
////////////////////////////////////////////////////////////////////////////////////
static void calc_parity(__u32 reg, __u16 *parity)
{
   __s8 i;

   for(i=0; i < 32; i++)
   {
      *parity += (reg >> i) & 1;
   }
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_read
///
///  @brief
///  Read incoming IRIG time data from the FPGA
///
///  @param
///      INPUTS:  filp - the /dev/irig file structure for this thread
///                buf - buffer for sending data to user space
///              count - amount of data caller is asking for, must be
///                      sizeof(struct irig_time)
///                loc - unused, required by the API
///
///  @param
///     OUTPUTS: return - On success: the number of bytes read in, else:
///                       EINVAL - if count is smaller than sizeof( struct irig_time )
///                       ERESTARTSYS - if wait_event_interruptible returns non-zero
///                       EIO - if the IRIG input is bad
///
////////////////////////////////////////////////////////////////////////////////
static ssize_t irig_read(struct file *filp, char __user *buf, size_t count,
                         loff_t *loc)
{
   __u8 loop;
   __u32 data[IRIG_DATA_SIZE];
   __u32 old_ts;
   __u16 parity = 0;
   ssize_t ret;
   struct irig_time itime;       // Data to be sent to the IRIG driver
   // The number of consecutive parity checks that have been correct - held at
   // GOOD_PARITY_TESTS once it reaches that value.
   static __u8 consecutive_good_parity_count = 0;
   // The number of parity checks that have been performed - held at
   // GOOD_PARITY_TESTS once it reaches that value.
   static __u8 parity_checks_performed = 0;

   ret = 0;

   // Throw an error if they're asking for any amount of data other than what
   // we read from the IRIG controller
   if( count < sizeof(struct irig_time) )
      ret = -EINVAL;
   else
   {
      // NOTE: The cast from void* to __u32 is 64-bit unsafe.  We need an
      //       intermediate cast to a native integer type first.
      old_ts = (__u32)(unsigned long)(filp->private_data);

      // Remove the following if clause to do non-blocking always
      if( (Irig_1pps_Ts == old_ts) && ((filp->f_flags & O_NONBLOCK) != O_NONBLOCK) )
      {
         // We're open in blocking mode and the PPS timestamp has not changed,
         // so block...

         if(wait_event_interruptible(Irig_Read_Q, Irig_1pps_Ts != old_ts) != 0)
         {
            ret = -ERESTARTSYS; // Catch signals other than our own
         }

         // Force the if statement to enter the code to read a completely new
         // sample from the IRIG device
         old_ts = Irig_1pps_Ts - 1;
      }

      // If Irig input is not good, return an EIO
      // Note: doing this after wait_event_interruptible is intended.
      if( (ret == 0) && (Irig_Input_Good == 0) )
      {
         ret = -EIO;
      }

      if( ret == 0 )
      {
         // Using old_ts here to ensure that we don't get a new PPS while
         // we're doing the following work.  This initial value just ensures
         // that we enter the while loop to do the work the first time...
         old_ts = Irig_1pps_Ts - 1;

         // This while loop checks to make sure that we don't start with data
         // from one PPS, then get the FRC value (at the end) from another
         // PPS.  We want to get the FRC as late as possible so it is accurate.
         //
         // This while loop should never be executed more than twice, and only
         // when the first entry starts right before a PPS comes in (so the
         // PPS comes in while we're doing the work here...)
         //
         // Note that Irig_1pps_Ts is updated in the ISR, we don't update it
         // here.
         while( ret == 0 && Irig_1pps_Ts != old_ts )
         {
            // Cause the next while loop entry to fail unless Irig_1pps_Ts
            // gets updated...
            old_ts = Irig_1pps_Ts;

            // Read the incoming IRIG data from the FPGA
            for(loop = 0; loop < IRIG_DATA_SIZE; loop++)
               data[loop] = ioread32(Data_Ptr + INCOMING_IRIG_DATA + loop);

            /// Need to determine if the source is C37.118 compliant.  The
            /// only way I know to do this is to check the parity bit.
            calc_parity(data[0], &parity);
            calc_parity(data[1], &parity);
            /// Parity only goes through bit 23
            calc_parity(data[2] & 0x7FFFFF, &parity);

            /// The IRIG parity calculation seems to be the opposite of what I
            /// was expecting.  If our parity calculation results in an odd
            /// value then the parity bit must not be set.
            if((parity & 1) == ((data[2] >> 23) & 1))
               consecutive_good_parity_count = 0;
            else
               consecutive_good_parity_count++;

            /// Require GOOD_PARITY_TESTS consecutive packets where the parity
            /// value is correct
            if(consecutive_good_parity_count >= GOOD_PARITY_TESTS)
            {
               itime.c37_comp = 1;
               // Hold count at GOOD_PARITY_TESTS
               consecutive_good_parity_count = GOOD_PARITY_TESTS;
            }
            else
            {
               itime.c37_comp = 0;
            }

            // Keep a count of the number of parity checks performed that stops
            // incrementing at GOOD_PARITY_TESTS.
            if( parity_checks_performed < GOOD_PARITY_TESTS )
            {
               ++parity_checks_performed;
            }
            else
            {
               // Prevent multi-threading from causing parity_checks_performed
               // from getting larger than GOOD_PARITY_TESTS
               parity_checks_performed = GOOD_PARITY_TESTS;
            }

            ///
            /// Now we have all the registers, time to convert them to the
            /// appropriate data structure
            ///

            itime.second = (data[0] & 0xF) + ((data[0] & 0xE0) >> 5) * 10;
            itime.minute = ((data[0] & 0xF00) >> 8) + ((data[0] & 0xE000) >> 13) * 10;
            itime.hour = ((data[0] & 0x1E0000) >> 17) + ((data[0] & 0xC00000) >> 22) * 10;

            itime.day = (data[1] & 0xF) + ((data[1] & 0x1E0) >> 5) * 10 +
               ((data[1] & 0x600) >> 9) * 100;

            //
            // Fill in data fields that come from the FPGA...
            //

            // Read the FRC of this time packet
            itime.packet_frc = ioread32(Data_Ptr + TS_IRIG_PPS);

            itime.ts_diff = ioread32(Data_Ptr + TS_DIFF_PPS);

            itime.frc_tick = Frc_Ticks_Per_Second;

            //
            // Cases at this point:
            //  1) consecutive_good_parity_count == GOOD_PARITY_TESTS
            //     -> Have C37 incoming time
            //  2) parity_checks_performed == consecutive_good_parity_count
            //     -> Don't know - assume C37 time - possibly wrong!
            //  3) otherwise (parity_checks_performed != consecutive_good_parity_count)
            //     -> Have BCD incoming time
            //
            //  Note: I'm handling #3 in the "if", then handling #1 and #2 in
            //  the "else"...
            //

            if( parity_checks_performed != consecutive_good_parity_count )
            {
               // Have BCD incoming: zero out DST, DSP and offset fields
               itime.dsp = 0;
               itime.dst = 0;
               itime.time_offset = 0;

               // Set time quality to unknown
               itime.time_quality = 15;  // 15 is "Clock Failed"

               // Fill in year 2000. This preserves previous functionality.
               // Note: I tried to get the year from the system clock.  I can
               //       get a timeval using do_gettimeofday(), but there's 
               //       no method to decode it and get the year.
               itime.year = 2000;
            }
            else
            {
               // Have C37 or think we have C37 - so extract the C37 data
               // fields...

               // The potential "error" behavior is if we have BCD coming in and
               // the first few samples happen to match C37 parity, then their
               // dst, dsp, time_offset, year and time_quality fields will be
               // whatever was in those data fields, and we'll add the
               // time_offset to the time values.

               itime.year = (data[2] & 0xF) + ((data[2] & 0x1E0) >> 5) * 10;

               /// IRIG returns a 2-digit year.  For now assume we know the
               /// century.  At some point it might be nice to clean this up.
               itime.year += 2000;

               /// Is DST pending?
               itime.dsp = (data[2] & 0x800) >> 11;

               /// Are we observing DST?
               itime.dst = (data[2] & 0x1000) >> 12;

               // What is the UTC offset (scaled by 10)?
               itime.time_offset = ((data[2] & 0x3C000) >> 14) * 10;

               /// Is there an additional 1/2 hour on the offset?
               if((data[2] & 0x40000) > 0)
                  itime.time_offset += 5;

               /// Is the UTC offset positive or negative?
               if((data[2] & 0x2000) > 0)
                  itime.time_offset = 0 - itime.time_offset;

               /// Apply the offset, yielding UTC
               if(itime.time_offset != 0)
                  convert_to_utc(&itime);

               /// How about time quality?
               itime.time_quality = (data[2] & 0x780000) >> 19;
            }

            apply_offsets(&itime);
         }  // End of the while loop
      }  //  End if( ret == 0 )

      // else we're not blocking and got a second call in the same PPS, so we
      // didn't need to decode anything - just copy the data (with the new frc
      // value retrieved above) and return.

      // If ret is not zero we're in the blocking case and
      // wait_event_interruptible() returned a non-zero value so skip the copy
      // and return...
      if( ret == 0 )
      {
         // Read the current FRC
         itime.frc = ioread32(Data_Ptr + FRC_REG);

         // Send data to user space, if possible
         copy_to_user(buf, &itime, sizeof(itime));
         ret = sizeof(itime);

         // Save the current IRIG PPS timestamp (see my note in irig_open about
         // the use of private_data).
         //
         // Use old_ts here instead of Irig_1pps_Ts because Irig_1pps_Ts is
         // potentially being updated by another thread/process.
         //
         // NOTE: The cast from __u32 to void* is 64-bit unsafe.  We need an
         //       intermediate cast to a native integer type first.
         (filp->private_data) = (void *)(unsigned long)old_ts;
      }

   } // End if( count != sizeof( struct irig_time ) )

   return ret;
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_write
///
///  @brief
///  Write outgoing IRIG time data to the FPGA
///
///  @param
///      INPUTS:  filp - the /dev/irig file structure for this thread
///                buf - buffer with the data sent from user space
///              count - amount of data caller is providing, must be
///                      sizeof(struct irig_time)
///                loc - unused, required by the API
///
///  @param
///     OUTPUTS: return - the number of bytes written out
///
////////////////////////////////////////////////////////////////////////////////
static ssize_t irig_write(struct file *filp, const char __user *buf,
                          size_t count, loff_t *loc)
{
   __u8 i;
   __u32 data[IRIG_DATA_SIZE];
   ssize_t ret;

   ret = 0;

   // Throw an error if they're sending any amount of data other than what
   // we write to the IRIG controller
   if(count != IRIG_DATA_SIZE * 4)
      ret = -EINVAL;
   else if(Ready_To_Send == 0)
   {
      // Can't send new data so if we're in non-blocking mode we need to return
      if((filp->f_flags & O_NONBLOCK) == O_NONBLOCK)
         ret = -EAGAIN;
      // Wait for the interrupt to tell us we're ready to send
      if(wait_event_interruptible(Irig_Write_Q, Ready_To_Send == 1) != 0)
         ret = -ERESTARTSYS; // Catch signals other than our own
   }

   if(ret == 0)
   {
      // Get the data from user space
      copy_from_user(data, buf, IRIG_DATA_SIZE * 4);

      // And send it to the FPGA
      for(i = 0; i < IRIG_DATA_SIZE; i++)
         iowrite32(data[i], Data_Ptr + OUTGOING_IRIG_DATA + i);

      Ready_To_Send = 0;
      ret = IRIG_DATA_SIZE * 4;
   }

   return ret;
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_ioctl
///
///  @brief
///  Control the IRIG module in the FPGA
///
///  @param
///      INPUTS: inode - the /dev/irig inode structure
///               filp - the /dev/irig file structure for this thread
///                cmd - the IOCTL command to be processed
///                arg - optional argument to cmd
///
///  @param
///     OUTPUTS: ret -      0 = successfull completion of the command
///                    ENOTTY = the IOCTL commadn wasn't recognized
///
////////////////////////////////////////////////////////////////////////////////
static int irig_ioctl(struct inode *inode, struct file *filp,
                      unsigned int cmd, unsigned long arg)
{
   int ret = 0;
   __u32 value = 0;

   // We shouldn't modify anything if we are RDONLY
   if( (_IOC_DIR(cmd) == _IOC_WRITE) &&
       !(filp->f_mode & FMODE_WRITE))
      return -EPERM;

   // Write command so save the value into the temporary variable
   // and cast appropriately to maintain 64-bit safeness
   if( _IOC_DIR(cmd) == _IOC_WRITE ) {
      value = (__u32)arg;
   }

   switch(cmd) {
      case IRIG_IOC_W_CONFIG:
         // Set the config register
         iowrite32(value, Data_Ptr + CONFIG_REG);
         break;
      case IRIG_IOC_R_CONFIG:
         // Read the config register
         value = ioread32(Data_Ptr + CONFIG_REG);
         ret = put_user(value, (__u32 __user *)arg);
         break;
      case IRIG_IOC_W_PLL_CONTROL:
         // Set the PLL control register
         iowrite32(value, Data_Ptr + PLL_CONTROL_REG);
         break;
      case IRIG_IOC_R_PLL_CONTROL:
         // Read the PLL control register
         value = ioread32(Data_Ptr + PLL_CONTROL_REG);
         ret = put_user(value, (__u32 __user *)arg);
         break;
      case IRIG_IOC_W_PLL_CPP:
         // Set the PLL CPP register
         iowrite32(value, Data_Ptr + PLL_CPP_REG);
         break;
      case IRIG_IOC_R_PLL_CPP:
         // Read the PLL CPP register
         value = ioread32(Data_Ptr + PLL_CPP_REG);
         ret = put_user(value, (__u32 __user *)arg);
         break;
      case IRIG_IOC_R_PLL_ERROR:
         // Read the PLL error register
         value = ioread32(Data_Ptr + PLL_ERROR_REG);
         ret = put_user((__s32)value, (__s32 __user *)arg);
         break;
      case IRIG_IOC_W_PLL_DELAY:
         // Set the PLL delay register
         iowrite32(value, Data_Ptr + PLL_DELAY_REG);
         break;
      case IRIG_IOC_R_PLL_DELAY:
         // Read the PLL delay register
         value = ioread32(Data_Ptr + PLL_DELAY_REG);
         ret = put_user(value, (__u32 __user *)arg);
         break;
      case IRIG_IOC_R_FRC:
         // Read the FRC register
         value = ioread32(Data_Ptr + FRC_REG);
         ret = put_user(value, (__u32 __user *)arg);
         break;
      case IRIG_IOC_R_TS_IRIG_1PPS:
         // Read the IRIG PPS timestamp register
         value = ioread32(Data_Ptr + TS_IRIG_PPS);
         ret = put_user(value, (__u32 __user *)arg);
         break;
      case IRIG_IOC_R_TS_PPS:
         // Read the external PPS timestamp register
         value = ioread32(Data_Ptr + TS_EXT_PPS);
         ret = put_user(value, (__u32 __user *)arg);
         break;
      case IRIG_IOC_R_TS_DIFF:
         // Read the PPS timestamp difference register
         value = ioread32(Data_Ptr + TS_DIFF_PPS);
         ret = put_user((__s32)value, (__s32 __user *)arg);
         break;
      case IRIG_IOC_W_IRQ_MASK:
         // Set the IRQ mask register
         iowrite32(value, Data_Ptr + IRIG_IRQ_MASK_REG);
         break;
      case IRIG_IOC_R_IRQ_MASK:
         // Read the IRQ mask register
         value = ioread32(Data_Ptr + IRIG_IRQ_MASK_REG);
         ret = put_user(value, (__u32 __user *)arg);
         break;
      case IRIG_IOC_W_PPS_TIMER:
         // Set the PPS Timer register
         iowrite32(value, Data_Ptr + PPS_TIMER_REG);
         break;
      case IRIG_IOC_R_PPS_TIMER:
         // Read the PPS Timer register
         value = ioread32(Data_Ptr + PPS_TIMER_REG);
         ret = put_user(value, (__u32 __user *)arg);
         break;
      case IRIG_IOC_W_TIME_OFF:
         Time_Adjustment = (__s32)value;
         break;
      case IRIG_IOC_R_TIME_OFF:
         ret = put_user(Time_Adjustment, (__s32 __user *)arg);
         break;
      default:
         ret = -ENOIOCTLCMD;
   }

   return ret;
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_poll
///
///  @brief
///  IRIG device poll function
///
///  @param
///      INPUTS: filp - the /dev/irig file structure for this thread
///              wait - the table of wait queues this thread can wait on
///
///  @param
///     OUTPUTS: mask - bitmask of the operations that can be performed
///                     immediately w/out blocking
///
////////////////////////////////////////////////////////////////////////////////
static unsigned int irig_poll(struct file *filp, poll_table *wait)
{
   __u32 mask = 0;
   __u32 local_irig_1pps_ts;
   __u32 old_ts;

   local_irig_1pps_ts = ioread32(Data_Ptr + TS_IRIG_PPS);
   // NOTE: The cast from void* to __u32 is 64-bit unsafe.  We need an
   //       intermediate cast to a native integer type first.
   old_ts = (__u32)(unsigned long)(filp->private_data);

   // Tell the thread it may have to wait on our queues
   poll_wait(filp, &Irig_Read_Q, wait);
   poll_wait(filp, &Irig_Write_Q, wait);

   if( Irig_Input_Good == 0 )
      mask = POLLIN | POLLERR;
   else if(local_irig_1pps_ts != old_ts)
      mask = POLLIN | POLLRDNORM; // New data available so device is readable

   if(Ready_To_Send == 1)
      mask |= POLLOUT | POLLWRNORM; // Last outgoing IRIG data sent so device
                                    // is writeable

   return mask;
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_probe
///
///  @brief
///  Do the actual work of installing the IRIG driver into the kernel
///
///  @param
///      INPUTS: dev - the PCI device structure allocated by the kernl
///               id - unused, required by the API
///
///  @param
///     OUTPUTS: ret -         0 = successfull installed the driver
///                    Otherwise = failed to get all resources or a kernel call
///                                failed
///
////////////////////////////////////////////////////////////////////////////////
static int __devinit irig_probe(struct pci_dev *dev,
                                const struct pci_device_id *id)
{
   int ret = 0;

   // Try and turn on the PCI device
   if(pci_enable_device(dev) != 0)
   {
      printk(KERN_INFO "Can't enable device\n");
      ret = -ENODEV;
   }
   else
   {
      // Try to get the FPGA memory region assigned to the IRIG device
      if(pci_request_region(dev, 0, "irig") != 0)
      {
         printk(KERN_INFO "Couldn't get mem address\n");
         pci_disable_device(dev);
         ret = -ENOMEM;
      }
      else
      {
         // And remap it into kernel virtual address space
         Data_Ptr = ioremap_nocache(pci_resource_start(dev, 0),
                            pci_resource_len(dev, 0));

         // Finish up by doing all the boilerplate CHAR device init stuff
         if(alloc_chrdev_region(&Dev_No, 0, 1, "irig") != 0)
         {
            printk(KERN_INFO "Couldn't register char driver\n");
            pci_release_region(dev, 0);
            pci_disable_device(dev);
            ret = -ENODEV;
         }
         else
         {
            Irig_Cdev = cdev_alloc();
            Irig_Cdev->ops = &Irig_Fops;
            Irig_Cdev->owner = THIS_MODULE;
            if(cdev_add(Irig_Cdev, Dev_No, 1) != 0)
            {
               printk(KERN_INFO "Couldn't add cdev\n");
               unregister_chrdev_region(Dev_No, 1);
               pci_release_region(dev, 0);
               pci_disable_device(dev);
               ret = -ENODEV;
            }

            // Get the interrupt
            ret = request_irq(dev->irq, irig_isr, IRQF_SHARED, "irig", irig_isr);
            if(ret == 0)
            {
               // Enable irig pps interrupt
               iowrite32((IRIG_PPS_INT | PLL_PPS_INT),
                          Data_Ptr + IRIG_IRQ_MASK_REG);

               // Initialize the timer register so that the TS_PPS_PLL_REG
               // value goes to NOMINAL_FRC_FREQ if there is no incoming IRIG
               // and nothing is training the PPS_TIMER_REG.
               iowrite32( NOMINAL_FRC_FREQ, Data_Ptr + PPS_TIMER_REG );
            }
            else
            {
               printk(KERN_INFO "Couldn't get irq\n");
               unregister_chrdev_region(Dev_No, 1);
               pci_release_region(dev, 0);
               pci_disable_device(dev);
               ret = -ENODEV;
            }

            sel_register( Dev_No, "irig" );
         } // register the char device
      } // get the PCI region
   } // enable the device

   return ret;
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_remove
///
///  @brief
///  Do the actual work of removing the IRIG driver from the kernel
///
///  @param
///      INPUTS: dev - the PCI device structure allocated by the kernl
///
///  @param
///     OUTPUTS: none
///
////////////////////////////////////////////////////////////////////////////////
static void __devexit irig_remove(struct pci_dev *dev)
{
   // Release all the stuff we got in the probe func

   // Disable, clear and release the irig pps interrupt
   iowrite32(0, Data_Ptr + IRIG_IRQ_MASK_REG);
   iowrite32(IRIG_INT_MASK, Data_Ptr + IRIG_IRQ_STATUS_REG);
   free_irq(dev->irq, irig_isr);

   sel_deregister(Dev_No);
   cdev_del(Irig_Cdev);
   unregister_chrdev_region(Dev_No, 1);
   iounmap(Data_Ptr);
   pci_release_region(dev, 0);
   pci_disable_device(dev);
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_init
///
///  @brief
///  Install the IRIG driver into the kernel
///
///  @param
///      INPUTS: none
///
///  @param
///     OUTPUTS: ret -         0 = successfully installed the driver
///                    Otherwise = failed
///
////////////////////////////////////////////////////////////////////////////////
static int __init irig_init(void)
{
   int ret;

   ret = pci_register_driver(&Irig_Pci_driver);

   return ret;
}

////////////////////////////////////////////////////////////////////////////////
///
///  @fn
///  irig_cleanup
///
///  @brief
///  Remove the IRIG driver from the kernel
///
///  @param
///      INPUTS: none
///
///  @param
///     OUTPUTS: none
///
////////////////////////////////////////////////////////////////////////////////
static void __exit irig_cleanup(void)
{
   pci_unregister_driver(&Irig_Pci_driver);
}

module_init(irig_init);
module_exit(irig_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Schweitzer Engeering Laboratories");
MODULE_DESCRIPTION("Driver for the SEL IRIG interface");
