//****************************************************************************
// Copyright (C) 2001-2007  PEAK System-Technik GmbH
//
// linux@peak-system.com
// www.peak-system.com 
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// Maintainer(s): Klaus Hitschler (klaus.hitschler@gmx.de)
//
// Major contributions by:
//                Edouard Tisserant (edouard.tisserant@lolitech.fr) XENOMAI
//                Laurent Bessard   (laurent.bessard@lolitech.fr)   XENOMAI
//                Oliver Hartkopp   (oliver.hartkopp@volkswagen.de) socketCAN
//
// Contributions: Arnaud Westenberg (arnaud@wanadoo.nl)
//                Matt Waters (Matt.Waters@dynetics.com)
//                Benjamin Kolb (Benjamin.Kolb@bigfoot.de)
//****************************************************************************

//****************************************************************************
//
// pcan_sja1000.c - all about sja1000 init and data handling
//
// $Id: pcan_sja1000.c 527 2007-11-04 15:02:32Z khitschler $
//
//****************************************************************************

//****************************************************************************
// INCLUDES
#include <src/pcan_common.h>
#include <linux/sched.h>
#include <asm/errno.h>
#include <asm/byteorder.h>  // because of little / big endian
#include <linux/delay.h>
#include <src/pcan_main.h>
#include <src/pcan_fifo.h>
#include <src/pcan_sja1000.h>

#ifdef NETDEV_SUPPORT
#include <src/pcan_netdev.h>
#endif

//****************************************************************************
// DEFINES

// sja1000 registers, only PELICAN mode - TUX like it
#define MODE                   0      // mode register
#define COMMAND                1
#define CHIPSTATUS             2
#define INTERRUPT_STATUS       3
#define INTERRUPT_ENABLE       4      // acceptance code
#define TIMING0                6      // bus timing 0
#define TIMING1                7      // bus timing 1
#define OUTPUT_CONTROL         8      // output control
#define TESTREG                9

#define ARBIT_LOST_CAPTURE    11      // transmit buffer: Identifier
#define ERROR_CODE_CAPTURE    12      // RTR bit und data length code
#define ERROR_WARNING_LIMIT   13      // start byte of data field
#define RX_ERROR_COUNTER      14
#define TX_ERROR_COUNTER      15

#define ACCEPTANCE_CODE_BASE  16
#define RECEIVE_FRAME_BASE    16
#define TRANSMIT_FRAME_BASE   16

#define ACCEPTANCE_MASK_BASE  20

#define RECEIVE_MSG_COUNTER   29
#define RECEIVE_START_ADDRESS 30

#define CLKDIVIDER            31      // set bit rate and pelican mode

// important sja1000 register contents, MODE register
#define SLEEP_MODE             0x10
#define ACCEPT_FILTER_MODE     0x08
#define SELF_TEST_MODE         0x04
#define LISTEN_ONLY_MODE       0x02
#define RESET_MODE             0x01
#define NORMAL_MODE            0x00

// COMMAND register
#define CLEAR_DATA_OVERRUN     0x08
#define RELEASE_RECEIVE_BUFFER 0x04
#define ABORT_TRANSMISSION     0x02
#define TRANSMISSION_REQUEST   0x01

// CHIPSTATUS register
#define BUS_STATUS             0x80
#define ERROR_STATUS           0x40
#define TRANSMIT_STATUS        0x20
#define RECEIVE_STATUS         0x10
#define TRANS_COMPLETE_STATUS  0x08
#define TRANS_BUFFER_STATUS    0x04
#define DATA_OVERRUN_STATUS    0x02
#define RECEIVE_BUFFER_STATUS  0x01

// INTERRUPT STATUS register
#define BUS_ERROR_INTERRUPT    0x80
#define ARBIT_LOST_INTERRUPT   0x40
#define ERROR_PASSIV_INTERRUPT 0x20
#define WAKE_UP_INTERRUPT      0x10
#define DATA_OVERRUN_INTERRUPT 0x08
#define ERROR_WARN_INTERRUPT   0x04
#define TRANSMIT_INTERRUPT     0x02
#define RECEIVE_INTERRUPT      0x01

// INTERRUPT ENABLE register
#define BUS_ERROR_INTERRUPT_ENABLE    0x80
#define ARBIT_LOST_INTERRUPT_ENABLE   0x40
#define ERROR_PASSIV_INTERRUPT_ENABLE 0x20
#define WAKE_UP_INTERRUPT_ENABLE      0x10
#define DATA_OVERRUN_INTERRUPT_ENABLE 0x08
#define ERROR_WARN_INTERRUPT_ENABLE   0x04
#define TRANSMIT_INTERRUPT_ENABLE     0x02
#define RECEIVE_INTERRUPT_ENABLE      0x01

// OUTPUT CONTROL register
#define OUTPUT_CONTROL_TRANSISTOR_P1  0x80
#define OUTPUT_CONTROL_TRANSISTOR_N1  0x40
#define OUTPUT_CONTROL_POLARITY_1     0x20
#define OUTPUT_CONTROL_TRANSISTOR_P0  0x10
#define OUTPUT_CONTROL_TRANSISTOR_N0  0x08
#define OUTPUT_CONTROL_POLARITY_0     0x04
#define OUTPUT_CONTROL_MODE_1         0x02
#define OUTPUT_CONTROL_MODE_0         0x01

// TRANSMIT or RECEIVE BUFFER
#define BUFFER_EFF                    0x80 // set for 29 bit identifier
#define BUFFER_RTR                    0x40 // set for RTR request
#define BUFFER_DLC_MASK               0x0f

// CLKDIVIDER register
#define CAN_MODE                      0x80
#define CAN_BYPASS                    0x40
#define RXINT_OUTPUT_ENABLE           0x20
#define CLOCK_OFF                     0x08
#define CLOCK_DIVIDER_MASK            0x07

// additional informations
#define CLOCK_HZ                  16000000 // crystal frequency

// time for mode register to change mode
#define MODE_REGISTER_SWITCH_TIME 100 // msec 

// some CLKDIVIDER register contents, hardware architecture dependend 
#define PELICAN_SINGLE  (CAN_MODE | CAN_BYPASS | 0x07 | CLOCK_OFF)
#define PELICAN_MASTER  (CAN_MODE | CAN_BYPASS | 0x07            )
#define PELICAN_DEFAULT (CAN_MODE                                )
#define CHIP_RESET      PELICAN_SINGLE 

// hardware depended setup for OUTPUT_CONTROL register
#define OUTPUT_CONTROL_SETUP (OUTPUT_CONTROL_TRANSISTOR_P0 | OUTPUT_CONTROL_TRANSISTOR_N0 | OUTPUT_CONTROL_MODE_1)

// the interrupt enables
#define INTERRUPT_ENABLE_SETUP (RECEIVE_INTERRUPT_ENABLE | TRANSMIT_INTERRUPT_ENABLE | DATA_OVERRUN_INTERRUPT_ENABLE | BUS_ERROR_INTERRUPT_ENABLE | ERROR_PASSIV_INTERRUPT_ENABLE | ERROR_WARN_INTERRUPT_ENABLE)

// the maximum number of handled messages in one interrupt 
#define MAX_MESSAGES_PER_INTERRUPT 8

// the maximum number of handled sja1000 interrupts in 1 handler entry
#define MAX_INTERRUPTS_PER_ENTRY   4

// constants from Arnaud Westenberg email:arnaud@wanadoo.nl
#define MAX_TSEG1  15
#define MAX_TSEG2  7
#define BTR1_SAM   (1<<1)

//****************************************************************************
// GLOBALS

//****************************************************************************
// LOCALS

//****************************************************************************
// CODE

#ifdef NO_RT
  #include "pcan_sja1000_linux.c"
#else
  #include "pcan_sja1000_rt.c"
#endif

//----------------------------------------------------------------------------
// switches the chip into reset mode
static int set_reset_mode(struct pcandev *dev)
{
  u32 dwStart = get_mtime();
  u8  tmp;

  tmp = dev->readreg(dev, MODE);
  while (!(tmp & RESET_MODE) && ((get_mtime() - dwStart) < MODE_REGISTER_SWITCH_TIME))
  {
    dev->writereg(dev, MODE, RESET_MODE); // force into reset mode
    wmb();
    udelay(1);
    //schedule();
    tmp = dev->readreg(dev, MODE);
  }

  if (!(tmp & RESET_MODE))
    return -EIO;
  else
    return 0;
}

//----------------------------------------------------------------------------
// switches the chip back from reset mode
static int set_normal_mode(struct pcandev *dev, u8 ucModifier)
{
  u32 dwStart = get_mtime();
  u8  tmp;

  tmp = dev->readreg(dev, MODE);
  while ((tmp != ucModifier) && ((get_mtime() - dwStart) < MODE_REGISTER_SWITCH_TIME))
  {
    dev->writereg(dev, MODE, ucModifier); // force into normal mode
    wmb();
    udelay(1);
    //schedule();
    tmp = dev->readreg(dev, MODE);
  }

  if (tmp != ucModifier)
    return -EIO;
  else
    return 0;
} 

//----------------------------------------------------------------------------
// interrupt enable and disable
static inline void sja1000_irq_enable(struct pcandev *dev)
{
  dev->writereg(dev, INTERRUPT_ENABLE, INTERRUPT_ENABLE_SETUP);
}

static inline void sja1000_irq_disable(struct pcandev *dev)
{
  dev->writereg(dev, INTERRUPT_ENABLE, 0);
}

//----------------------------------------------------------------------------
// find the proper clock divider
static inline u8 clkdivider(struct pcandev *dev)
{
  if (!dev->props.ucExternalClock)  // crystal based
    return PELICAN_DEFAULT;

  // configure clock divider register, switch into pelican mode, depended of of type
  switch (dev->props.ucMasterDevice)
  {
    case CHANNEL_SLAVE:
    case CHANNEL_SINGLE:
      return PELICAN_SINGLE;  // neither a slave nor a single device distribute the clock ...

    default:
      return PELICAN_MASTER;  // ... but a master does
  }
}

//----------------------------------------------------------------------------
// init CAN-chip
int sja1000_open(struct pcandev *dev, u16 btr0btr1, u8 bExtended, u8 bListenOnly)
{
  int result      = 0;
  u8  _clkdivider = clkdivider(dev);
  u8  ucModifier  = (bListenOnly) ? LISTEN_ONLY_MODE : NORMAL_MODE;

  DPRINTK(KERN_DEBUG "%s: sja1000_open(), minor = %d.\n", DEVICE_NAME, dev->nMinor);

  // switch to reset 
  result = set_reset_mode(dev);
  if (result)
    goto fail;

  // take a fresh status
  dev->wCANStatus = 0;

  // store extended mode (standard still accepted)
  dev->bExtended = bExtended;

  // configure clock divider register, switch into pelican mode, depended of of type
  dev->writereg(dev, CLKDIVIDER, _clkdivider);

  // configure acceptance code registers
  dev->writereg(dev, ACCEPTANCE_CODE_BASE,     0);
  dev->writereg(dev, ACCEPTANCE_CODE_BASE + 1, 0);
  dev->writereg(dev, ACCEPTANCE_CODE_BASE + 2, 0);
  dev->writereg(dev, ACCEPTANCE_CODE_BASE + 3, 0);

  // configure all acceptance mask registers to don't care
  dev->writereg(dev, ACCEPTANCE_MASK_BASE,     0xff);
  dev->writereg(dev, ACCEPTANCE_MASK_BASE + 1, 0xff);
  dev->writereg(dev, ACCEPTANCE_MASK_BASE + 2, 0xff);
  dev->writereg(dev, ACCEPTANCE_MASK_BASE + 3, 0xff);

  // configure bus timing registers
  dev->writereg(dev, TIMING0, (u8)((btr0btr1 >> 8) & 0xff));
  dev->writereg(dev, TIMING1, (u8)((btr0btr1     ) & 0xff));

  // configure output control registers
  dev->writereg(dev, OUTPUT_CONTROL, OUTPUT_CONTROL_SETUP);

  // clear any pending interrupt
  dev->readreg(dev, INTERRUPT_STATUS);

  // enter normal operating mode
  result = set_normal_mode(dev, ucModifier);
  if (result)
    goto fail;

  // enable CAN interrupts
  sja1000_irq_enable(dev);

  fail:
  return result;
}

//----------------------------------------------------------------------------
// release CAN-chip
void sja1000_release(struct pcandev *dev)
{
  DPRINTK(KERN_DEBUG "%s: sja1000_release()\n", DEVICE_NAME);

  // abort pending transmissions
  dev->writereg(dev, COMMAND, ABORT_TRANSMISSION);

  // disable CAN interrupts and set chip in reset mode
  sja1000_irq_disable(dev);
  set_reset_mode(dev);
}

//----------------------------------------------------------------------------
// read CAN-data from chip, supposed a message is available
static int sja1000_read_frames(SJA1000_METHOD_ARGS)
{
  int msgs   = MAX_MESSAGES_PER_INTERRUPT;
  u8 fi;
  u8 dreg;
  u8 dlc;
  ULCONV localID;
  struct can_frame frame;
  struct timeval tv;

  int i;
  int result = 0;

  // DPRINTK(KERN_DEBUG "%s: sja1000_read_frames()\n", DEVICE_NAME);

  do
  {
    DO_GETTIMEOFDAY(tv); /* create timestamp */

    fi  = dev->readreg(dev, RECEIVE_FRAME_BASE);
    dlc = fi & BUFFER_DLC_MASK;

    if (dlc > 8)
      dlc = 8;

    if (fi & BUFFER_EFF) {
      /* extended frame format (EFF) */
      dreg = RECEIVE_FRAME_BASE + 5;

      #if defined(__LITTLE_ENDIAN)
      localID.uc[3] = dev->readreg(dev, RECEIVE_FRAME_BASE + 1);
      localID.uc[2] = dev->readreg(dev, RECEIVE_FRAME_BASE + 2);
      localID.uc[1] = dev->readreg(dev, RECEIVE_FRAME_BASE + 3);
      localID.uc[0] = dev->readreg(dev, RECEIVE_FRAME_BASE + 4);
      #elif defined(__BIG_ENDIAN)
      localID.uc[0] = dev->readreg(dev, RECEIVE_FRAME_BASE + 1);
      localID.uc[1] = dev->readreg(dev, RECEIVE_FRAME_BASE + 2);
      localID.uc[2] = dev->readreg(dev, RECEIVE_FRAME_BASE + 3);
      localID.uc[3] = dev->readreg(dev, RECEIVE_FRAME_BASE + 4);
      #else
        #error  "Please fix the endianness defines in <asm/byteorder.h>"
      #endif

      // frame.can_id = (dev->readreg(dev, RECEIVE_FRAME_BASE + 1) << (5+16))
      //              | (dev->readreg(dev, RECEIVE_FRAME_BASE + 2) << (5+8))
      //              | (dev->readreg(dev, RECEIVE_FRAME_BASE + 3) << 5)
      //              | (dev->readreg(dev, RECEIVE_FRAME_BASE + 4) >> 3);

      frame.can_id = (localID.ul >> 3) | CAN_EFF_FLAG;

    } else {
      /* standard frame format (EFF) */
      dreg = RECEIVE_FRAME_BASE + 3;
      
      localID.ul    = 0;
      #if defined(__LITTLE_ENDIAN)
      localID.uc[3] = dev->readreg(dev, RECEIVE_FRAME_BASE + 1);
      localID.uc[2] = dev->readreg(dev, RECEIVE_FRAME_BASE + 2);
      #elif defined(__BIG_ENDIAN)
      localID.uc[0] = dev->readreg(dev, RECEIVE_FRAME_BASE + 1);
      localID.uc[1] = dev->readreg(dev, RECEIVE_FRAME_BASE + 2);
      #else
        #error  "Please fix the endianness defines in <asm/byteorder.h>"
      #endif

      frame.can_id = (localID.ul >> 21);

      //frame.can_id = (dev->readreg(dev, RECEIVE_FRAME_BASE + 1) << 3)
      //             | (dev->readreg(dev, RECEIVE_FRAME_BASE + 2) >> 5);
    }

    if (fi & BUFFER_RTR)
      frame.can_id |= CAN_RTR_FLAG;

    *(__u64 *) &frame.data[0] = (__u64) 0; /* clear aligned data section */

    for (i = 0; i < dlc; i++)
      frame.data[i] = dev->readreg(dev, dreg++);

    frame.can_dlc = dlc;

    SJA1000_LOCK_IRQSAVE(in_lock);

    if ((i = pcan_xxxdev_rx(dev, &frame, &tv))) // put into specific data sink
      result = i; // save the last result

    SJA1000_UNLOCK_IRQRESTORE(in_lock);

    // Any error processing on result =! 0 here?
    // Indeed we have to read from the controller as long as we receive data to
    // unblock the controller. If we have problems to fill the CAN frames into
    // the receive queues, this cannot be handled inside the interrupt.

    // release the receive buffer
    dev->writereg(dev, COMMAND, RELEASE_RECEIVE_BUFFER);
    wmb();

    // give time to settle
    udelay(1);

  } while (dev->readreg(dev, CHIPSTATUS) & RECEIVE_BUFFER_STATUS && (msgs--));

  return result;
}

//----------------------------------------------------------------------------
// write CAN-data to chip, 
int sja1000_write_frame(struct pcandev *dev, struct can_frame *cf)
{
  u8 fi;
  u8 dlc;
  canid_t id;
  ULCONV localID;
  u8 dreg;
  int i;

  fi = dlc = cf->can_dlc;
  id = localID.ul = cf->can_id;

  if (id & CAN_RTR_FLAG)
    fi |= BUFFER_RTR;

  if (id & CAN_EFF_FLAG) {
    dreg  = TRANSMIT_FRAME_BASE + 5;
    fi   |= BUFFER_EFF;

    localID.ul   <<= 3;

    #if defined(__LITTLE_ENDIAN)
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 1, localID.uc[3]);
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 2, localID.uc[2]);
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 3, localID.uc[1]);
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 4, localID.uc[0]);
    #elif defined(__BIG_ENDIAN)
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 1, localID.uc[0]);
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 2, localID.uc[1]);
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 3, localID.uc[2]);
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 4, localID.uc[3]);
    #else
      #error  "Please fix the endianness defines in <asm/byteorder.h>"
    #endif

    // dev->writereg(dev, TRANSMIT_FRAME_BASE + 1, (id & 0x1fe00000) >> (5 + 16));
    // dev->writereg(dev, TRANSMIT_FRAME_BASE + 2, (id & 0x001fe000) >> (5 + 8));
    // dev->writereg(dev, TRANSMIT_FRAME_BASE + 3, (id & 0x00001fe0) >> 5);
    // dev->writereg(dev, TRANSMIT_FRAME_BASE + 4, (id & 0x0000001f) << 3);

  } else {
    dreg = TRANSMIT_FRAME_BASE + 3;

    localID.ul   <<= 21;

    #if defined(__LITTLE_ENDIAN)
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 1, localID.uc[3]);
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 2, localID.uc[2]);
    #elif defined(__BIG_ENDIAN)
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 1, localID.uc[0]);
    dev->writereg(dev, TRANSMIT_FRAME_BASE + 2, localID.uc[1]);
    #else
      #error  "Please fix the endianness defines in <asm/byteorder.h>"
    #endif

    // dev->writereg(dev, TRANSMIT_FRAME_BASE + 1, (id & 0x000007f8) >> 3);
    // dev->writereg(dev, TRANSMIT_FRAME_BASE + 2, (id & 0x00000007) << 5);
  }

  for (i = 0; i < dlc; i++) {
    dev->writereg(dev, dreg++, cf->data[i]);
  }

  dev->writereg(dev, TRANSMIT_FRAME_BASE, fi);

  // request a transmission
  dev->writereg(dev, COMMAND, TRANSMISSION_REQUEST);
  wmb();

  // strange hack to get write stall managed
  if ((dev->readreg(dev, CHIPSTATUS) & TRANS_COMPLETE_STATUS))
  {
    DPRINTK(KERN_ERR "%s: Transmission finished early..., minor=%d\n", DEVICE_NAME, dev->nMinor);
  }

  return 0;
}

//----------------------------------------------------------------------------
// write CAN-data from FIFO to chip 
int sja1000_write(SJA1000_METHOD_ARGS)
{
  int        result;
  TPCANMsg   msg;
  struct can_frame cf;

  // DPRINTK(KERN_DEBUG "%s: sja1000_write() %d\n", DEVICE_NAME, dev->writeFifo.nStored);

  SJA1000_LOCK_IRQSAVE(out_lock);

  // get a fifo element and step forward
  result = pcan_fifo_get(&dev->writeFifo, &msg);

  SJA1000_WAKEUP_EMPTY();

  SJA1000_UNLOCK_IRQRESTORE(out_lock);

  if (result)
    return result;

  /* convert from old style FIFO message until FIFO supports new */
  /* struct can_frame and error frames */
  msg2frame(&cf, &msg);

  sja1000_write_frame(dev, &cf);

  return 0;
}

//----------------------------------------------------------------------------
// handle a interrupt request
int sja1000_irqhandler_common(SJA1000_METHOD_ARGS)
{
  int ret = SJA1000_IRQ_NONE;
  u8 irqstatus;
  int err;
  u16 rwakeup = 0;
  u16 wwakeup = 0;
  int j = MAX_INTERRUPTS_PER_ENTRY;
  struct can_frame ef;

  memset(&ef, 0, sizeof(ef));

  // DPRINTK(KERN_DEBUG "%s: sja1000_base_irqhandler|sja1000_irqhandler_rt()\n", DEVICE_NAME);

  SJA1000_LOCK_IRQSAVE(sja_lock);

  while ((j--) && (irqstatus = dev->readreg(dev, INTERRUPT_STATUS)))
  {
    // DPRINTK(KERN_DEBUG "%s: sja1000_[base_irqhandler|irqhandler_rt](0x%02x)\n", DEVICE_NAME, irqstatus);

    dev->dwInterruptCounter++;
    err = 0;

    // quick hack to badly workaround write stall
    // if ((irqstatus & TRANSMIT_INTERRUPT) || (!atomic_read(&dev->DataSendReady) && !pcan_fifo_empty(&dev->writeFifo) && (dev->readreg(dev, CHIPSTATUS) & TRANS_BUFFER_STATUS)))
    if (irqstatus & TRANSMIT_INTERRUPT) 
    {
      // handle transmission
      if ((err = SJA1000_FUNCTION_CALL(sja1000_write)))
      {
        if (err == -ENODATA)
          wwakeup++;
        else
        {
          dev->nLastError = err;
          dev->dwErrorCounter++;
          dev->wCANStatus |= CAN_ERR_QXMTFULL; // fatal error!
        }
      }
      dev->ucActivityState = ACTIVITY_XMIT; // reset to ACTIVITY_IDLE by cyclic timer
    }

    if (irqstatus & DATA_OVERRUN_INTERRUPT)
    {
      // handle data overrun
      dev->wCANStatus |= CAN_ERR_OVERRUN;
      ef.can_id  |= CAN_ERR_CRTL;
      ef.data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
      rwakeup++;
      dev->dwErrorCounter++;

      dev->writereg(dev, COMMAND, CLEAR_DATA_OVERRUN);
      wmb();
      dev->ucActivityState = ACTIVITY_XMIT; // reset to ACTIVITY_IDLE by cyclic timer
    }

    if (irqstatus & RECEIVE_INTERRUPT)
    {
      // handle receiption
      if ((err = SJA1000_FUNCTION_CALL(sja1000_read_frames)) < 0) /* put to input queues */
      {
        dev->nLastError = err;
        dev->dwErrorCounter++;
        dev->wCANStatus |=  CAN_ERR_QOVERRUN;

        // throw away last message which was refused by fifo
        dev->writereg(dev, COMMAND, RELEASE_RECEIVE_BUFFER);
        wmb();
      }

      if (err > 0) // successfully enqueued into chardev FIFO
        rwakeup++;

      dev->ucActivityState = ACTIVITY_XMIT; // reset to ACTIVITY_IDLE by cyclic timer
    }

    if (irqstatus & (BUS_ERROR_INTERRUPT | ERROR_PASSIV_INTERRUPT | ERROR_WARN_INTERRUPT))
    {
      // DPRINTK(KERN_DEBUG "%s: sja1000_irqhandler(), irqstatus=0x%02x\n", DEVICE_NAME, irqstatus); 

      if (irqstatus & (ERROR_PASSIV_INTERRUPT | ERROR_WARN_INTERRUPT))
      {
        u8 chipstatus = dev->readreg(dev, CHIPSTATUS);
        
		// DPRINTK(KERN_DEBUG "%s: sja1000_irqhandler(), chipstatus=0x%02x\n", DEVICE_NAME, chipstatus);
        switch (chipstatus & (BUS_STATUS | ERROR_STATUS))
        {
          case 0x00: 
                    // error active, clear only local status
                    dev->busStatus = CAN_ERROR_ACTIVE;
                    DPRINTK(KERN_DEBUG "%s: sja1000_irqhandler(), busStatus=CAN_ERROR_ACTIVE (1)\n", DEVICE_NAME);                   
                    dev->wCANStatus &=  ~(CAN_ERR_BUSOFF | CAN_ERR_BUSHEAVY | CAN_ERR_BUSLIGHT); 
                    break;
          case BUS_STATUS:
          case BUS_STATUS | ERROR_STATUS:
                    // bus-off
                    dev->busStatus = CAN_BUS_OFF;                    
                    dev->wCANStatus |=  CAN_ERR_BUSOFF;
                    ef.can_id |= CAN_ERR_BUSOFF_NETDEV;
                    DPRINTK(KERN_DEBUG "%s: sja1000_irqhandler(), busStatus=CAN_BUS_OFF\n", DEVICE_NAME);                   
                    break;
          case ERROR_STATUS: 
                    if (irqstatus & ERROR_PASSIV_INTERRUPT)
                    {
                      // either enter or leave error passive status
                      if (dev->busStatus == CAN_ERROR_PASSIVE)
                      {
                        // go back to error active
                        dev->busStatus = CAN_ERROR_ACTIVE;
                        dev->wCANStatus &=  ~(CAN_ERR_BUSOFF | CAN_ERR_BUSHEAVY); 
                        dev->wCANStatus |=  CAN_ERR_BUSLIGHT;
                        DPRINTK(KERN_DEBUG "%s: sja1000_irqhandler(), busStatus=CAN_ERROR_ACTIVE (2)\n", DEVICE_NAME);                   
                      }
                      else
                      {
                        // enter error passive state
                        dev->busStatus = CAN_ERROR_PASSIVE;
                        dev->wCANStatus &=  ~CAN_ERR_BUSOFF; 
                        dev->wCANStatus |=  CAN_ERR_BUSHEAVY;                     
                        ef.can_id  |= CAN_ERR_CRTL;
                        ef.data[1] |= (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE);
                        DPRINTK(KERN_DEBUG "%s: sja1000_irqhandler(), busStatus=CAN_ERROR_PASSIVE\n", DEVICE_NAME);                   
                      }
                    }
                    else
                    {
                      // it was a warning limit reached event
                      dev->busStatus = CAN_ERROR_ACTIVE;                    
                      dev->wCANStatus |=  CAN_ERR_BUSLIGHT;                     
                      ef.can_id  |= CAN_ERR_CRTL;
                      ef.data[1] |= (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING);
                      DPRINTK(KERN_DEBUG "%s: sja1000_irqhandler(), busStatus=CAN_ERROR_ACTIVE (3)\n", DEVICE_NAME);                   
                    }
                    break;
        }
      }

      // count each error signal even if it does not change any bus or error state
      dev->dwErrorCounter++;        

      // wake up pending reads or writes
      rwakeup++;
      wwakeup++;
      dev->ucActivityState = ACTIVITY_XMIT; // reset to ACTIVITY_IDLE by cyclic timer
    }

    /* if an error condition occurred, send an error frame to the userspace */
    if (ef.can_id)
    {
      struct timeval tv;

      DO_GETTIMEOFDAY(tv); /* create timestamp */

      ef.can_id  |= CAN_ERR_FLAG;
      ef.can_dlc  = CAN_ERR_DLC;

      SJA1000_LOCK_IRQSAVE(in_lock);

      if (pcan_xxxdev_rx(dev, &ef, &tv) > 0) // put into specific data sink
        rwakeup++;

      SJA1000_UNLOCK_IRQRESTORE(in_lock);

      memset(&ef, 0, sizeof(ef)); // clear for next loop
    }

    ret = SJA1000_IRQ_HANDLED;
  }

  SJA1000_UNLOCK_IRQRESTORE(sja_lock);

  if (wwakeup)
  {
    atomic_set(&dev->DataSendReady, 1); // signal to write I'm ready
    wmb();
    SJA1000_WAKEUP_WRITE();
    #ifdef NETDEV_SUPPORT
    if (dev->netdev)
      netif_wake_queue(dev->netdev);
    #endif
  }

  if (rwakeup)
    SJA1000_WAKEUP_READ();

  return ret;
}

//----------------------------------------------------------------------------
// probe for a sja1000 - use it only in reset mode!
int  sja1000_probe(struct pcandev *dev)
{
  u8  tmp;
  u8  _clkdivider= clkdivider(dev);

  DPRINTK(KERN_DEBUG "%s: sja1000_probe()\n", DEVICE_NAME);

  // trace the clockdivider register to test for sja1000 / 82c200
  tmp = dev->readreg(dev, CLKDIVIDER);
  DPRINTK(KERN_DEBUG "%s: CLKDIVIDER traced (0x%02x)\n", DEVICE_NAME, tmp);
  if (tmp & 0x10)
    goto fail;

  // until here, it's either a 82c200 or a sja1000
  if (set_reset_mode(dev))
    goto fail;

  dev->writereg(dev, CLKDIVIDER, _clkdivider); // switch to PeliCAN mode
  sja1000_irq_disable(dev);                    // precautionary disable interrupts
  wmb();
  DPRINTK(KERN_DEBUG "%s: Hopefully switched to PeliCAN mode\n", DEVICE_NAME);

  tmp = dev->readreg(dev, CHIPSTATUS);
  DPRINTK(KERN_DEBUG "%s: CHIPSTATUS traced (0x%02x)\n", DEVICE_NAME, tmp);
  if ((tmp & 0x30) != 0x30)
    goto fail;

  tmp = dev->readreg(dev, INTERRUPT_STATUS);
  DPRINTK(KERN_DEBUG "%s: INTERRUPT_STATUS traced (0x%02x)\n", DEVICE_NAME, tmp);
  if (tmp & 0xfb)
    goto fail;

  tmp = dev->readreg(dev, RECEIVE_MSG_COUNTER);
  DPRINTK(KERN_DEBUG "%s: RECEIVE_MSG_COUNTER traced (0x%02x)\n", DEVICE_NAME, tmp);
  if (tmp)
    goto fail;

  DPRINTK(KERN_DEBUG "%s: sja1000_probe() is OK\n", DEVICE_NAME);
  return 0;

  fail:
  DPRINTK(KERN_DEBUG "%s: sja1000_probe() failed\n", DEVICE_NAME);

  return -ENXIO; // no such device or address
}

//----------------------------------------------------------------------------
// calculate BTR0BTR1 for odd bitrates
//
// most parts of this code is from Arnaud Westenberg email:arnaud@wanadoo.nl
// www.home.wanadoo.nl/arnaud
//
// Set communication parameters.
// param rate baud rate in Hz
// param clock frequency of sja1000 clock in Hz
// param sjw synchronization jump width (0-3) prescaled clock cycles
// param sampl_pt sample point in % (0-100) sets (TSEG1+2)/(TSEG1+TSEG2+3) ratio
// param flags fields BTR1_SAM, OCMODE, OCPOL, OCTP, OCTN, CLK_OFF, CBP
//
static int sja1000_baud_rate(int rate, int flags)
{
  int best_error = 1000000000;
  int error;
  int best_tseg=0, best_brp=0, best_rate=0, brp=0;
  int tseg=0, tseg1=0, tseg2=0;
  int clock = CLOCK_HZ / 2;
  u16 wBTR0BTR1;
  int sjw = 0;
  int sampl_pt = 90;

  // some heuristic specials
  if (rate > ((1000000 + 500000) / 2))
    sampl_pt = 75;

  if (rate < ((12500 + 10000) / 2))
    sampl_pt = 75;

  if (rate < ((100000 + 125000) / 2))
    sjw = 1;

  // tseg even = round down, odd = round up
  for (tseg = (0 + 0 + 2) * 2; tseg <= (MAX_TSEG2 + MAX_TSEG1 + 2) * 2 + 1; tseg++) 
  {
    brp = clock / ((1 + tseg / 2) * rate) + tseg % 2;
    if ((brp == 0) || (brp > 64))
      continue;

    error = rate - clock / (brp * (1 + tseg / 2));
    if (error < 0)
      error = -error;

    if (error <= best_error) 
    {
      best_error = error;
      best_tseg = tseg/2;
      best_brp = brp-1;
      best_rate = clock/(brp*(1+tseg/2));
    }
  }

  if (best_error && (rate / best_error < 10)) 
  {
    DPRINTK(KERN_ERR "%s: bitrate %d is not possible with %d Hz clock\n", DEVICE_NAME, rate, 2 * clock);

    return 0;
  }

  tseg2 = best_tseg - (sampl_pt * (best_tseg + 1)) / 100;

  if (tseg2 < 0)
    tseg2 = 0;

  if (tseg2 > MAX_TSEG2)
    tseg2 = MAX_TSEG2;

  tseg1 = best_tseg - tseg2 - 2;

  if (tseg1 > MAX_TSEG1) 
  {
    tseg1 = MAX_TSEG1;
    tseg2 = best_tseg-tseg1-2;
  }

  wBTR0BTR1 = ((sjw<<6 | best_brp) << 8) | (((flags & BTR1_SAM) != 0)<<7 | tseg2<<4 | tseg1);

  return wBTR0BTR1;
}

//----------------------------------------------------------------------------
// get BTR0BTR1 init values
u16 sja1000_bitrate(u32 dwBitRate)
{
  u16 wBTR0BTR1 = 0;;

  // get default const values
  switch (dwBitRate)
  {
    case 1000000: wBTR0BTR1 = CAN_BAUD_1M;   break;
    case  500000: wBTR0BTR1 = CAN_BAUD_500K; break;
    case  250000: wBTR0BTR1 = CAN_BAUD_250K; break;
    case  125000: wBTR0BTR1 = CAN_BAUD_125K; break;
    case  100000: wBTR0BTR1 = CAN_BAUD_100K; break;
    case   50000: wBTR0BTR1 = CAN_BAUD_50K;  break;
    case   20000: wBTR0BTR1 = CAN_BAUD_20K;  break;
    case   10000: wBTR0BTR1 = CAN_BAUD_10K;  break;
    case    5000: wBTR0BTR1 = CAN_BAUD_5K;   break;
    case       0: wBTR0BTR1 = 0;             break;

    default:
         // calculate for exotic values
           wBTR0BTR1 = sja1000_baud_rate(dwBitRate, 0);
  }

  DPRINTK(KERN_DEBUG "%s: sja1000_bitrate(%d) = 0x%04x\n", DEVICE_NAME, dwBitRate, wBTR0BTR1);

  return wBTR0BTR1;
}

// end


