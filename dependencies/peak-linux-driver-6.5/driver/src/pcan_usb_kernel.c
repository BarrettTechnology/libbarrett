//****************************************************************************
// Copyright (C) 2003-2007  PEAK System-Technik GmbH
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
//                Oliver Hartkopp   (oliver.hartkopp@volkswagen.de) socketCAN
//                     
//****************************************************************************

//****************************************************************************
//
// pcan_usb-kernel.c - the inner parts for pcan-usb support
//
// $Id: pcan_usb_kernel.c 510 2007-05-29 13:07:10Z khitschler $
//
//****************************************************************************

//****************************************************************************
// INCLUDES
#include <src/pcan_common.h>
#include <linux/sched.h>
#include <src/pcan_main.h>
#include <src/pcan_fifo.h>
#include <src/pcan_usb_kernel.h>
#include <asm/byteorder.h>       // because of little / big endian

#ifdef NETDEV_SUPPORT
#include <src/pcan_netdev.h>     // for hotplug pcan_netdev_register()
#endif

//****************************************************************************
// DEFINES

// bit masks for status/length field in a USB message
#define STLN_WITH_TIMESTAMP 0x80
#define STLN_INTERNAL_DATA  0x40
#define STLN_EXTENDED_ID    0x20
#define STLN_RTR            0x10
#define STLN_DATA_LENGTH    0x0F         // mask for length of data bytes

// Error-Flags for PCAN-USB
#define XMT_BUFFER_FULL           0x01
#define CAN_RECEIVE_QUEUE_OVERRUN 0x02
#define BUS_LIGHT                 0x04
#define BUS_HEAVY                 0x08
#define BUS_OFF                   0x10
#define QUEUE_RECEIVE_EMPTY       0x20
#define QUEUE_OVERRUN             0x40
#define QUEUE_XMT_FULL            0x80

// timestamp calculation stuff
#define FASTSCALE                            // use this for slow architectures without native 64 bit division support
#define SCALED_MICROSECONDS_PER_TICK  44739  // error = 5,424e-4%
#define SCALE_SHIFTER                    20  // unscale, shift runs faster than divide
#define COMPARE_OFFSET                  128  // enhance wrap comparison tolerance caused by idle timestamps

#define SCALE_MULTIPLIER               1024  // multiplier for direct calculation of timestamp
#define SCALE_DIVISOR                 24000  // divisor for direct calculation of timestamp

#define TICKS(msec) ((msec * HZ) / 1000)     // to calculate ticks from milliseconds

#define COMMAND_TIMEOUT           1000       // msec timeout for control EP0 urb get set

typedef struct                               // pcan-usb parameter get an set function
{
  u8  Function;
  u8  Number;
  u8  Param[14];
} __attribute__ ((packed)) PCAN_USB_PARAM;

//****************************************************************************
// GLOBALS

//****************************************************************************
// LOCALS

//****************************************************************************
// CODE  

//****************************************************************************
// functions to handle time construction
static void pcan_reset_timestamp(struct pcandev *dev)
{
  PCAN_USB_TIME *t = dev->port.usb.pUSBtime;

  DPRINTK(KERN_DEBUG "%s: reset_timestamp()\n", DEVICE_NAME);

  // reset time stamp information
  t->ullCumulatedTicks    = 0;
  t->ullOldCumulatedTicks = 0;
  t->wLastTickValue       = 0;
  t->ucLastTickValue      = 0;
  t->wOldLastTickValue    = 0;
  t->StartTime.tv_sec     = 0;
  t->StartTime.tv_usec    = 0;  
  t->wStartTicks          = 0;
}

#if 0
// TODO: take the timestamp from ticks * (42 + 2/3) usecs of PCAN-USB
static inline void pcan_calcTimevalFromTicks(struct pcandev *dev, struct timeval *tv)
{
  register PCAN_USB_TIME *t = dev->port.usb.pUSBtime;

  u64 llx;

  llx   = t->ullCumulatedTicks - t->wStartTicks;    // subtract initial offset
  
  #ifdef FASTSCALE                                  
  llx  *= SCALED_MICROSECONDS_PER_TICK;             
  return  (u32)(llx >>= SCALE_SHIFTER);             // unscale, shift runs faster than divide
  #else
  llx  *= SCALE_MULTIPLIER; 
  return ((u32)(*(u64 *)&llx) / SCALE_DIVISOR);     // trick to circumvent missing __udivid3 entry message
  #endif
}

static inline void pcan_getTimeStamp(struct pcandev *dev, struct timeval *tv)
{
  ...
}
#endif

static void pcan_updateTimeStampFromWord(struct pcandev *dev, u16 wTimeStamp, u8 ucStep)
{
  register PCAN_USB_TIME *t = dev->port.usb.pUSBtime;

  // DPRINTK(KERN_DEBUG "%s: updateTimeStampFromWord() Tim = %d, Last = %d, Cum = %lld\n", DEVICE_NAME, wTimeStamp, t->wLastTickValue, t->ullCumulatedTicks);

  if ((!t->StartTime.tv_sec) && (!t->StartTime.tv_usec))
  {
    get_relative_time(NULL, &t->StartTime);
    t->wStartTicks          = wTimeStamp;
    t->wOldLastTickValue    = wTimeStamp;
    t->ullCumulatedTicks    = wTimeStamp;
    t->ullOldCumulatedTicks = wTimeStamp;
  }

  // correction for status timestamp in the same telegram which is more recent, restore old contents
  if (ucStep)
  {
    t->ullCumulatedTicks = t->ullOldCumulatedTicks;
    t->wLastTickValue    = t->wOldLastTickValue;
  }
    
  // store current values for old ...
  t->ullOldCumulatedTicks = t->ullCumulatedTicks;
  t->wOldLastTickValue    = t->wLastTickValue;
    
  if (wTimeStamp < t->wLastTickValue)  // handle wrap, enhance tolerance
    t->ullCumulatedTicks += 0x10000LL;

  t->ullCumulatedTicks &= ~0xFFFFLL;   // mask in new 16 bit value - do not cumulate cause of error propagation
  t->ullCumulatedTicks |= wTimeStamp;

  t->wLastTickValue   = wTimeStamp;      // store for wrap recognition
  t->ucLastTickValue  = (u8)(wTimeStamp & 0xff); // each update for 16 bit tick updates the 8 bit tick, too
}

static void pcan_updateTimeStampFromByte(struct pcandev *dev, u8 ucTimeStamp)
{
  register PCAN_USB_TIME *t = dev->port.usb.pUSBtime;

  // DPRINTK(KERN_DEBUG "%s: updateTimeStampFromByte() Tim = %d, Last = %d, Cum = %lld\n", DEVICE_NAME, ucTimeStamp, t->ucLastTickValue, t->ullCumulatedTicks);
  
  if (ucTimeStamp < t->ucLastTickValue)  // handle wrap
  {
    t->ullCumulatedTicks += 0x100;
    t->wLastTickValue    += 0x100;
  }

  t->ullCumulatedTicks &= ~0xFFLL;       // mask in new 8 bit value - do not cumulate cause of error propagation
  t->ullCumulatedTicks |= ucTimeStamp;

  t->wLastTickValue    &= ~0xFF;         // correction for word timestamp, too
  t->wLastTickValue    |= ucTimeStamp;

  t->ucLastTickValue    = ucTimeStamp;   // store for wrap recognition
}

//****************************************************************************
// get and set parameters to or from device
//
#ifdef LINUX_26
static void pcan_param_xmit_notify(struct urb *purb, struct pt_regs *regs)
#else
static void pcan_param_xmit_notify(purb_t purb)
#endif
{
  struct pcandev *dev = purb->context;

  DPRINTK(KERN_DEBUG "%s: pcan_param_xmit_notify() = %d\n", DEVICE_NAME, purb->status);

  // un-register outstanding urb
  atomic_dec(&dev->port.usb.active_urbs);

  atomic_set(&dev->port.usb.param_xmit_finished, 1);
}

static int pcan_hw_setcontrol_urb(struct pcandev *dev, u8 function, u8 number, 
                             u8 param0, u8 param1, u8 param2, u8 param3,  u8 param4,  u8 param5,  u8 param6, 
                             u8 param7, u8 param8, u8 param9, u8 param10, u8 param11, u8 param12, u8 param13)
{
  PCAN_USB_PARAM myParameter;
  int nResult = 0;
  register purb_t pt;

  // DPRINTK(KERN_DEBUG "%s: pcan_set_parameter()\n", DEVICE_NAME);

  // don't do anything with non-existent hardware
  if (!dev->ucPhysicallyInstalled)
    return -ENODEV;

  myParameter.Function = function;
  myParameter.Number   = number;
  myParameter.Param[0]   = param0;
  myParameter.Param[1]   = param1;
  myParameter.Param[2]   = param2;
  myParameter.Param[3]   = param3;
  myParameter.Param[4]   = param4;
  myParameter.Param[5]   = param5;
  myParameter.Param[6]   = param6;
  myParameter.Param[7]   = param7;
  myParameter.Param[8]   = param8;
  myParameter.Param[9]   = param9;
  myParameter.Param[10]  = param10;
  myParameter.Param[11]  = param11;
  myParameter.Param[12]  = param12;
  myParameter.Param[13]  = param13;

  pt = dev->port.usb.param_urb;

  FILL_BULK_URB(pt, dev->port.usb.usb_dev,
                usb_sndbulkpipe(dev->port.usb.usb_dev, dev->port.usb.Endpoint[1].ucNumber),
                &myParameter, sizeof(myParameter), pcan_param_xmit_notify, dev);

  #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
  pt->timeout = TICKS(COMMAND_TIMEOUT);
  #endif
  
  if (__usb_submit_urb(pt))
  {
    DPRINTK(KERN_ERR "%s: pcan_set_parameter() can't submit!\n",DEVICE_NAME);
    nResult = pt->status;
    goto fail;
  }
  else
    atomic_inc(&dev->port.usb.active_urbs);

  // wait until submit is finished, either normal or thru timeout
  while (!atomic_read(&dev->port.usb.param_xmit_finished))
    schedule();
  
  // remove urb
  nResult = pt->status;

  fail:
  atomic_set(&dev->port.usb.param_xmit_finished, 0);
  
  return nResult;
}     

static int pcan_set_function(struct pcandev *dev, u8 function, u8 number)
{
  DPRINTK(KERN_DEBUG "%s: pcan_set_function(%d, %d)\n", DEVICE_NAME, function, number); 
  
  return pcan_hw_setcontrol_urb(dev, function, number, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}  

// in opposition to WIN method this performs the complete write and read cycle!
static int pcan_hw_getcontrol_urb(struct pcandev *dev, u8 function, u8 number, 
                             u8 *param0, u8 *param1, u8 *param2, u8 *param3, 
                             u8 *param4, u8 *param5, u8 *param6, u8 *param7)
{
  PCAN_USB_PARAM myParameter;
  int nResult = 0;
  register purb_t pt;
  USB_PORT *u = &dev->port.usb;  

  DPRINTK(KERN_DEBUG "%s: pcan_hw_getcontrol_urb(%d, %d)\n", DEVICE_NAME, function, number); 

  // don't do anything with non-existent hardware
  if (!dev->ucPhysicallyInstalled)
    return -ENODEV;

  // first write function and number to device
  nResult = pcan_set_function(dev,  function, number);

  // heuristic result - wait a little bit
  mdelay(5);

  if (!nResult)
  {
    u32 startTime;
    
    pt = u->param_urb;

    FILL_BULK_URB(pt, u->usb_dev,
                  usb_rcvbulkpipe(u->usb_dev, u->Endpoint[0].ucNumber),
                  &myParameter, sizeof(myParameter), pcan_param_xmit_notify, dev);
  
    myParameter.Function = function;
    myParameter.Number   = number;

    #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
    pt->timeout = TICKS(COMMAND_TIMEOUT);
    #endif
    
    if (__usb_submit_urb (pt))
    {
      printk(KERN_ERR "%s: pcan_get_parameter() can't submit!\n",DEVICE_NAME);
      nResult = pt->status;
      goto fail;
    }
    else
      atomic_inc(&u->active_urbs);

    startTime = get_mtime();
    while (!atomic_read(&u->param_xmit_finished) && ((get_mtime() - startTime) < COMMAND_TIMEOUT))
      schedule();

    if (!atomic_read(&u->param_xmit_finished))
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
      usb_kill_urb(pt);
    #else
      usb_unlink_urb(pt); /* any better solution here for Kernel 2.4 ? */
    #endif
      
    if (!pt->status)
    {
      *param0 = myParameter.Param[0]; 
      *param1 = myParameter.Param[1];
      *param2 = myParameter.Param[2];
      *param3 = myParameter.Param[3];
      *param4 = myParameter.Param[4];
      *param5 = myParameter.Param[5];
      *param6 = myParameter.Param[6];
      *param7 = myParameter.Param[7];
    }

    nResult = pt->status;
  }

  fail:
  atomic_set(&u->param_xmit_finished, 0);
  
  return nResult;
}

//****************************************************************************
// specialized high level hardware access functions
//
static int pcan_hw_setBTR0BTR1(struct pcandev *dev, u16 wBTR0BTR1)
{
  u8  dummy  = 0;
  u8  param0 = (u8)(wBTR0BTR1 & 0xff);
  u8  param1 = (u8)((wBTR0BTR1 >> 8) & 0xff);

  DPRINTK(KERN_DEBUG "%s: pcan_hw_setBTR0BTR1(0x%04x)\n", DEVICE_NAME, wBTR0BTR1); 

  return pcan_hw_setcontrol_urb(dev, 1, 2, param0, param1, 
                           dummy, dummy, dummy, dummy, dummy, dummy,
                           dummy, dummy, dummy, dummy, dummy, dummy);
}

int pcan_hw_SetCANOn(struct pcandev *dev)
{
  u8  dummy  = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_SetCANOn()\n", DEVICE_NAME); 

  return pcan_hw_setcontrol_urb(dev, 3, 2, 1, dummy, 
                           dummy, dummy, dummy, dummy, dummy, dummy,
                           dummy, dummy, dummy, dummy, dummy, dummy);
}

int pcan_hw_SetCANOff(struct pcandev *dev)
{
  int err;
  u8  dummy  = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_SetCANOff()\n", DEVICE_NAME); 

  err = pcan_hw_setcontrol_urb(dev, 3, 2, 0, dummy, 
                           dummy, dummy, dummy, dummy, dummy, dummy,
                           dummy, dummy, dummy, dummy, dummy, dummy);
  return err;
}

static int pcan_hw_SetCANSilentOn(struct pcandev *dev)
{
  u8  dummy  = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_SetCANSilentOn()\n", DEVICE_NAME); 

  return pcan_hw_setcontrol_urb(dev, 3, 3, 1, dummy, 
                           dummy, dummy, dummy, dummy, dummy, dummy,
                           dummy, dummy, dummy, dummy, dummy, dummy);
}

static int pcan_hw_SetCANSilentOff(struct pcandev *dev)
{
  u8  dummy  = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_SetCANSilentOff()\n", DEVICE_NAME); 

  return pcan_hw_setcontrol_urb(dev, 3, 3, 0, dummy, 
                           dummy, dummy, dummy, dummy, dummy, dummy,
                           dummy, dummy, dummy, dummy, dummy, dummy);
}

int pcan_hw_getBTR0BTR1(struct pcandev *dev, u16 *pwBTR0BTR1)
{
  int err;
  u8  dummy  = 0;
  u8  param0 = 0;
  u8  param1 = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_getBTR0BTR1()\n", DEVICE_NAME); 

  err = pcan_hw_getcontrol_urb(dev, 1, 1, &param0, &param1, &dummy, &dummy, &dummy, &dummy, &dummy, &dummy);

  *pwBTR0BTR1 = param1;
  *pwBTR0BTR1 <<= 8;
  *pwBTR0BTR1 |= param0;

  DPRINTK(KERN_DEBUG "%s: BTR0BTR1 = 0x%04x\n", DEVICE_NAME, *pwBTR0BTR1); 

  return err;
}

int pcan_hw_getQuartz(struct pcandev *dev, u32 *pdwQuartzHz)
{
  int err = 0;
  u8  dummy  = 0;
  u8  param0 = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_getQuartz()\n", DEVICE_NAME); 

  err = pcan_hw_getcontrol_urb(dev, 2, 1, &param0, &dummy, &dummy, &dummy, &dummy, &dummy, &dummy, &dummy);

  *pdwQuartzHz = param0;
  *pdwQuartzHz *= 1000000L;

  DPRINTK(KERN_DEBUG "%s: Frequenz = %u\n", DEVICE_NAME, *pdwQuartzHz); 
  
  return err;
}

int pcan_hw_getAnything(struct pcandev *dev, u8 ucFunction, u8 ucNumber)
{
  int err = 0;
  u8  dummy[8];
  int i;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_getAnything()\n", DEVICE_NAME); 

  for (i = 0; i < 7; i++)
    dummy[i] = 0;

  err = pcan_hw_getcontrol_urb(dev, ucFunction, ucNumber, 
                          &dummy[0], &dummy[1], &dummy[2], &dummy[3], &dummy[4], &dummy[5], &dummy[6], &dummy[7]);

  DPRINTK(KERN_DEBUG "%s: Fun/Num:%d/%d  0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", DEVICE_NAME,
          ucFunction, ucNumber, dummy[0], dummy[1], dummy[2], dummy[3], dummy[4], dummy[5], dummy[6], dummy[7]); 

  return err;
}

int pcan_hw_getDeviceNr(struct pcandev *dev, u8 *pucDeviceNr)
{
  int err;
  u8  dummy  = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_getDeviceNr()\n", DEVICE_NAME); 

  err = pcan_hw_getcontrol_urb(dev, 4, 1, pucDeviceNr, &dummy, &dummy, &dummy, &dummy, &dummy, &dummy, &dummy);

  DPRINTK(KERN_DEBUG "%s: DeviceNr = 0x%02x\n", DEVICE_NAME, *pucDeviceNr); 

  return err;
}

int pcan_hw_SetExtVCCOn(struct pcandev *dev)
{
  u8  dummy  = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_SetExtVCCOn()\n", DEVICE_NAME); 

  return pcan_hw_setcontrol_urb(dev, 0xA, 2, 1, dummy, 
                           dummy, dummy, dummy, dummy, dummy, dummy,
                           dummy, dummy, dummy, dummy, dummy, dummy);
}

int pcan_hw_SetDeviceNr(struct pcandev *dev, u8 ucDeviceNr)
{
  u8  dummy  = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_SetDeviceNr()\n", DEVICE_NAME); 

  return pcan_hw_setcontrol_urb(dev, 4, 2, ucDeviceNr, dummy, 
                           dummy, dummy, dummy, dummy, dummy, dummy,
                           dummy, dummy, dummy, dummy, dummy, dummy);
}

int pcan_hw_SetSNR(struct pcandev *dev, u32 dwSNR)
{
  u8  dummy  = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_SetSNR()\n", DEVICE_NAME); 

  return pcan_hw_setcontrol_urb(dev, 6, 2, 
                           (u8)( dwSNR        & 0xff), 
                           (u8)((dwSNR >>  8) & 0xff), 
                           (u8)((dwSNR >> 16) & 0xff), 
                           (u8)((dwSNR >> 24) & 0xff), 
                           dummy, dummy, dummy, dummy, dummy,
                           dummy, dummy, dummy, dummy, dummy);
}


static int pcan_hw_SetExtVCCOff(struct pcandev *dev)
{
  u8  dummy  = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_SetExtVCCOff()\n", DEVICE_NAME); 

  return pcan_hw_setcontrol_urb(dev, 0xA, 2, 0, dummy, 
                           dummy, dummy, dummy, dummy, dummy, dummy,
                           dummy, dummy, dummy, dummy, dummy, dummy);
}

int pcan_hw_getSNR(struct pcandev *dev, u32 *pdwSNR)
{
  int err = 0;
  ULCONV SNR;
  u8  dummy;
  int i = 1;
  
  DPRINTK(KERN_DEBUG "%s: pcan_hw_getSNR()\n", DEVICE_NAME); 

  memset(&SNR, 0, sizeof(SNR));
  
  // sometimes the hardware won't provide the number - so try twice
  do
    err = pcan_hw_getcontrol_urb(dev, 6, 1, &SNR.uc[3], &SNR.uc[2], &SNR.uc[1], &SNR.uc[0], &dummy, &dummy, &dummy, &dummy);
  while ((i--) && (err == -2));
  
  *pdwSNR = SNR.ul;
  
  DPRINTK(KERN_DEBUG "%s: SNR = 0x%08x\n", DEVICE_NAME, *pdwSNR);

  return err;
}


//****************************************************************************
// init hardware parts
int pcan_hw_Init(struct pcandev *dev, u16 btr0btr1, u8 bListenOnly)
{
  int err = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_hw_Init()\n", DEVICE_NAME); 

  err = pcan_hw_setBTR0BTR1(dev, btr0btr1);
  if (err)
    goto fail;

  if (dev->port.usb.ucRevision > 3)
  {
    // set listen only
    if (bListenOnly)
      err = pcan_hw_SetCANSilentOn(dev);
    else
      err = pcan_hw_SetCANSilentOff(dev);
    if (err)
      goto fail;
  }
  else
  {
    // generate err if one tries to set bListenOnly
    if (bListenOnly)
    {
      err = -EINVAL;
      goto fail;
    }
  }

  // don't know how to handle - walk the save way
  err = pcan_hw_SetExtVCCOff(dev);
  
  // prepare for new start of timestamp calculation
  pcan_reset_timestamp(dev);

  fail:
  return err;
}

//****************************************************************************
// takes USB-message frames out of ucMsgPtr, decodes and packs them into readFifo
int pcan_hw_DecodeMessage(struct pcandev *dev, u8 *ucMsgPtr, int lCurrentLength)
{
  int err = 0;
  int i, j;
  u8 ucMsgPrefix;
  u8 ucMsgLen;         // number of frames in one USB packet
  u8 ucStatusLen = 0;  // storage for the status/length entry leading each data frame
  u8 ucLen;            // len in bytes of received (CAN) data
  UWCONV       wTimeStamp;
  u8           *ucMsgStart = ucMsgPtr; // store start of buffer for overflow compare
  int rwakeup = 0;
  u8  *org = ucMsgPtr;
  u8  dataPacketCounter = 0;

  //DPRINTK(KERN_DEBUG "%s: pcan_hw_DecodeMessage(%p, %d)\n", DEVICE_NAME, ucMsgPtr, lCurrentLength); 

  // sometimes is nothing to do
  if (!lCurrentLength)
    return err;
  
  // get prefix of message and step over
  ucMsgPrefix = *ucMsgPtr++;

  // get length of message and step over
  ucMsgLen    = *ucMsgPtr++;

  for (i = 0; (i < ucMsgLen); i++)
  {
    ULCONV localID;
    struct timeval tv;

    ucStatusLen = *ucMsgPtr++;

    // TODO: take timestamp from PCAN-USB
    do_gettimeofday(&tv);
      
    // normal CAN message are always with timestamp
    if (!(ucStatusLen & STLN_INTERNAL_DATA))
    {
      int nRtrFrame;
      struct can_frame frame;
            
      ucLen = ucStatusLen & STLN_DATA_LENGTH;
      if (ucLen > 8)
        ucLen = 8;
      frame.can_dlc = ucLen;
      
      nRtrFrame = ucStatusLen & STLN_RTR;
      if (nRtrFrame)
        frame.can_id = CAN_RTR_FLAG;         // re-set to RTR value
      else
        frame.can_id = 0;                    // re-set to default value
        
      if (ucStatusLen & STLN_EXTENDED_ID)
      {
        frame.can_id |= CAN_EFF_FLAG; // modify if it was extended
        
        #if defined(__LITTLE_ENDIAN)
        localID.uc[0] = *ucMsgPtr++;
        localID.uc[1] = *ucMsgPtr++;
        localID.uc[2] = *ucMsgPtr++;
        localID.uc[3] = *ucMsgPtr++;
        #elif defined(__BIG_ENDIAN)
        localID.uc[3] = *ucMsgPtr++;
        localID.uc[2] = *ucMsgPtr++;
        localID.uc[1] = *ucMsgPtr++;
        localID.uc[0] = *ucMsgPtr++;
        #else
          #error  "Please fix the endianness defines in <asm/byteorder.h>"
        #endif
  
        localID.ul   >>= 3;
      }
      else
      {
        localID.ul    = 0;

        #if defined(__LITTLE_ENDIAN)
        localID.uc[0] = *ucMsgPtr++;
        localID.uc[1] = *ucMsgPtr++;
        #elif defined(__BIG_ENDIAN)
        localID.uc[3] = *ucMsgPtr++;
        localID.uc[2] = *ucMsgPtr++;
        #else
          #error  "Please fix the endianness defines in <asm/byteorder.h>"
        #endif
  
        localID.ul   >>= 5;
      }

      frame.can_id |= localID.ul;

      // read timestamp, first timestamp in packet is 16 bit AND data, following timestamps are 8 bit in length
      if (!dataPacketCounter)
      {
        #if defined(__LITTLE_ENDIAN)
        wTimeStamp.uc[0] = *ucMsgPtr++;
        wTimeStamp.uc[1] = *ucMsgPtr++;
        #elif defined(__BIG_ENDIAN)
        wTimeStamp.uc[1] = *ucMsgPtr++;
        wTimeStamp.uc[0] = *ucMsgPtr++;
        #else
          #error  "Please fix the endianness defines in <asm/byteorder.h>"
        #endif
        
        pcan_updateTimeStampFromWord(dev, wTimeStamp.uw, i);
      }
      else
        pcan_updateTimeStampFromByte(dev, *ucMsgPtr++);

      // read data
      j = 0;
      if (!nRtrFrame)
      {
        while (ucLen--)
          frame.data[j++] = *ucMsgPtr++;
      }

      // only for beauty, replace useless telegram content with zeros
      while (j < 8)
        frame.data[j++] = 0;
      
      if ((err = pcan_xxxdev_rx(dev, &frame, &tv)) < 0) // put into data sink
        goto fail; 
        
      if (err > 0) // successfully enqueued into chardev FIFO
        rwakeup++;

      dataPacketCounter++;
    }
    // Status Daten
    else
    {
      u8 ucFunction;
      u8 ucNumber;
      u8 dummy;
      TPCANRdMsg msg;
      struct can_frame ef; /* error frame */
      
      memset(&ef, 0, sizeof(ef));
      
      // declare as status Msg
      msg.Msg.MSGTYPE = MSGTYPE_STATUS;

      // prepare length of data
      ucLen = ucStatusLen & STLN_DATA_LENGTH;
      if (ucLen > 8)
        ucLen = 8;
      msg.Msg.LEN = ucLen;

      // get function and number
      ucFunction = *ucMsgPtr++;
      ucNumber   = *ucMsgPtr++;

      if (ucStatusLen & STLN_WITH_TIMESTAMP)
      {
        // only the first packet supplies a word timestamp
        if (!i)
        {
          #if defined(__LITTLE_ENDIAN)
          wTimeStamp.uc[0] = *ucMsgPtr++;
          wTimeStamp.uc[1] = *ucMsgPtr++;
          #elif defined(__BIG_ENDIAN)
          wTimeStamp.uc[1] = *ucMsgPtr++;
          wTimeStamp.uc[0] = *ucMsgPtr++;
          #else
            #error  "Please fix the endianness defines in <asm/byteorder.h>"
          #endif

          pcan_updateTimeStampFromWord(dev, wTimeStamp.uw, i);
        }
        else
          pcan_updateTimeStampFromByte(dev, *ucMsgPtr++);
      }
      
      switch (ucFunction)
      {
        case 1: // can_error. number = flags, special decoding in PCAN-USB
          if ((ucNumber & CAN_RECEIVE_QUEUE_OVERRUN) || (ucNumber & QUEUE_OVERRUN))
          {
            dev->wCANStatus |= CAN_ERR_OVERRUN;
            ef.can_id  |= CAN_ERR_CRTL;
            ef.data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
            dev->dwErrorCounter++;
          }

          if (ucNumber & BUS_OFF)
          {
            dev->wCANStatus |=  CAN_ERR_BUSOFF;
            ef.can_id |= CAN_ERR_BUSOFF_NETDEV;
            dev->dwErrorCounter++;        
          }

          if (ucNumber & BUS_HEAVY)
          {
            dev->wCANStatus |=  CAN_ERR_BUSHEAVY;
            ef.can_id  |= CAN_ERR_CRTL;
            ef.data[1] |= CAN_ERR_CRTL_RX_WARNING;
            dev->dwErrorCounter++;        
          }

          if (ucNumber & BUS_LIGHT)
            dev->wCANStatus |= CAN_ERR_BUSLIGHT;
          
          // version 3: sometimes the telegram carries 3 additional data without note in ucStatusLen. 
          // Don't know what to do ??
          j = 0;
          while (ucLen--)
            msg.Msg.DATA[j++] = *ucMsgPtr++;
          break;
        case 2: // get_analog_value, remove bytes
          dummy = *ucMsgPtr++; 
          dummy = *ucMsgPtr++;
          break;
        case 3: // get_bus_load, remove byte
          dummy = *ucMsgPtr++; 
          break;
        case 4: // only timestamp
          #if defined(__LITTLE_ENDIAN)
          wTimeStamp.uc[0] = *ucMsgPtr++;
          wTimeStamp.uc[1] = *ucMsgPtr++;
          #elif defined(__BIG_ENDIAN)
          wTimeStamp.uc[1] = *ucMsgPtr++;
          wTimeStamp.uc[0] = *ucMsgPtr++;
          #else
            #error  "Please fix the endianness defines in <asm/byteorder.h>"
          #endif

          pcan_updateTimeStampFromWord(dev, wTimeStamp.uw, i);
          break;
        case 5: // ErrorFrame/ErrorBusEvent.
          if (ucNumber & QUEUE_XMT_FULL) 
          {
            printk(KERN_ERR "%s: QUEUE_XMT_FULL signaled, ucNumber = 0x%02x\n", DEVICE_NAME, ucNumber);
            dev->wCANStatus |= CAN_ERR_QXMTFULL; // fatal error!
            dev->dwErrorCounter++;
          }
 
          j = 0;
          while (ucLen--)
            msg.Msg.DATA[j++] = *ucMsgPtr++;
          break;
        case 10: // prepared for future
          break;
        default:
          printk(KERN_ERR "%s: unexpected function, i = %d, ucStatusLen = 0x%02x\n", DEVICE_NAME, 
                  i, ucStatusLen);
          buffer_dump(org, 4);
      }
      
      /* if an error condition occurred, send an error frame to the userspace */
      if (ef.can_id) 
      {
        ef.can_id  |= CAN_ERR_FLAG;
        ef.can_dlc  = CAN_ERR_DLC;

        if ((err = pcan_xxxdev_rx(dev, &ef, &tv)) < 0) // put into data sink
          goto fail;
        
        if (err > 0) // successfully enqueued into chardev FIFO
          rwakeup++;
      } 
    }

    // check for 'read from'-buffer overrun
    if ((ucMsgPtr - ucMsgStart) > lCurrentLength)    // must be <= dev->port.usb.Endpoint[2].wDataSz)
    {
      // sometimes version 3 overrides the buffer by 1 byte
      if ((dev->port.usb.ucRevision > 3) || 
          ((dev->port.usb.ucRevision <= 3) && ((ucMsgPtr - ucMsgStart) > (lCurrentLength + 1))))
      {
        err = -EFAULT;
        #ifdef __LP64__
        printk(KERN_ERR "%s: Internal Error = %d (%ld, %d)\n", DEVICE_NAME, err, (ucMsgPtr - ucMsgStart), lCurrentLength);
        #else
        printk(KERN_ERR "%s: Internal Error = %d (%d, %d)\n", DEVICE_NAME, err, (ucMsgPtr - ucMsgStart), lCurrentLength);
        #endif
        buffer_dump(org, 4);
        goto fail;
      }
    }
  }

  if (rwakeup)
    wake_up_interruptible(&dev->read_queue);

  fail:
  return err;
}

// gets messages out of write-fifo, encodes and puts them into USB buffer ucMsgPtr
// returns -ENODATA and *pnDataLength > 0 if I made a telegram and no more data are available
//         -ENODATA and *pnDataLength == 0 if I made no telegram and no more data are available
//         any ERROR else if something happend
//         no ERROR if I made a telegram and there are more data available
int pcan_hw_EncodeMessage(struct pcandev *dev, u8 *ucMsgPtr, int *pnDataLength)
{
  int err         = 0;
  int nMsgCounter = 0;          // counts the messages stored in this URB packet
  u8  *ptr        = ucMsgPtr;   // work pointer into write buffer
  u8  ucLen;                    // CAN data length
  ULCONV localID;               // for easy endian conversion
  u8    *pucStatusLen;          // pointer to ucStatusLen byte in URB message buffer
  u8    *pucMsgCountPtr;        // pointer to MsgCount byte in URB message buffer
  int   j;                      // working counter
  u8    bFinish = 0;
  int   nDataLength = *pnDataLength;  
  int   nBufferTop  = nDataLength - 14;  // buffer fill high water mark 

  // DPRINTK(KERN_DEBUG "%s: pcan_hw_EncodeMessage() %d %d\n", DEVICE_NAME, dev->writeFifo.nStored, pcan_fifo_empty(&dev->writeFifo));

  // indicate no packet
  *pnDataLength = 0;

  // put packet type information
  *ptr++ = 2;
  pucMsgCountPtr = ptr++;  // fill later the count of messages

  // pack packet
  while (!bFinish && ((ptr - ucMsgPtr) < nBufferTop))
  {
    int nRtrFrame;
    TPCANMsg msg;                // pointer to supplied CAN message
    
    // release fifo buffer and step forward in fifo
    if ((err = pcan_fifo_get(&dev->writeFifo, &msg)))
    {
      bFinish = 1;
      
      if (err != -ENODATA)
      {
        DPRINTK(KERN_DEBUG "%s: can't get data out of writeFifo, avail data: %d, err: %d\n", DEVICE_NAME, dev->writeFifo.nStored, err);
      }
        
      continue;
    }
      
    // get ptr to ucStatusLen byte
    pucStatusLen = ptr++;
    
    *pucStatusLen = ucLen = msg.LEN & STLN_DATA_LENGTH;
      
    nRtrFrame = msg.MSGTYPE & MSGTYPE_RTR;
    if (nRtrFrame)
      *pucStatusLen |= STLN_RTR;  // add RTR flag

    j = 0;
    localID.ul = msg.ID;
    if (msg.MSGTYPE & MSGTYPE_EXTENDED)
    {
      *pucStatusLen |= STLN_EXTENDED_ID;
      localID.ul   <<= 3;

      #if defined(__LITTLE_ENDIAN)
      *ptr++ = localID.uc[0];
      *ptr++ = localID.uc[1];
      *ptr++ = localID.uc[2];
      *ptr++ = localID.uc[3];
      #elif defined(__BIG_ENDIAN)
      *ptr++ = localID.uc[3];
      *ptr++ = localID.uc[2];
      *ptr++ = localID.uc[1];
      *ptr++ = localID.uc[0];
      #else
        #error  "Please fix the endianness defines in <asm/byteorder.h>"
      #endif
    }
    else
    {
      localID.ul   <<= 5;

      #if defined(__LITTLE_ENDIAN)
      *ptr++ = localID.uc[0];
      *ptr++ = localID.uc[1];
      #elif defined(__BIG_ENDIAN)
      *ptr++ = localID.uc[3];
      *ptr++ = localID.uc[2];
      #else
        #error  "Please fix the endianness defines in <asm/byteorder.h>"
      #endif
    }

    if (!nRtrFrame)
    {
      // put data
      j = 0;
      while (ucLen--)
        *ptr++ = msg.DATA[j++];
    }

    nMsgCounter++;
  }

  // generate external nDataLength if I carry payload
  if ((ptr - ucMsgPtr) > 2)
  {
    *pnDataLength = nDataLength;

    // set count of telegrams
    ptr = ucMsgPtr + nDataLength - 1;
    *ptr = (u8)(dev->port.usb.dwTelegramCount++ & 0xff);

    // last to do: put count of messages
    *pucMsgCountPtr = nMsgCounter;
  }
  else
  {
    *pnDataLength   = 0;
    *pucMsgCountPtr = 0;
  }

  return err;
}

#ifdef NETDEV_SUPPORT
// writes a CAN-Frame pointed to by *cf into USB buffer ucMsgPtr
// returns -ENODATA and *pnDataLength > 0 if I made a telegram and no more data are available
//         -ENODATA and *pnDataLength == 0 if I made no telegram and no more data are available
//         any ERROR else if something happend
//         no ERROR if I made a telegram and there are more data available
int pcan_hw_EncodeMessage_frame(struct pcandev *dev, struct can_frame *cf, u8 *ucMsgPtr, int *pnDataLength)
{
  int nMsgCounter = 0;          // counts the messages stored in this URB packet
  u8  *ptr        = ucMsgPtr;   // work pointer into write buffer
  u8  ucLen;                    // CAN data length
  ULCONV localID;               // for easy endian conversion
  u8    *pucStatusLen;          // pointer to ucStatusLen byte in URB message buffer
  u8    *pucMsgCountPtr;        // pointer to MsgCount byte in URB message buffer
  int   j;                      // working counter
  int   nDataLength = *pnDataLength;  

  DPRINTK(KERN_DEBUG "%s: pcan_hw_EncodeMessage_frame() %d %d\n", DEVICE_NAME, dev->writeFifo.nStored, pcan_fifo_empty(&dev->writeFifo));

  // indicate no packet
  *pnDataLength = 0;

  // put packet type information
  *ptr++ = 2;
  pucMsgCountPtr = ptr++;  // fill later the count of messages

  // pack packet

  // get ptr to ucStatusLen byte
  pucStatusLen = ptr++;
      
  *pucStatusLen = ucLen = cf->can_dlc & STLN_DATA_LENGTH;
        
  if (cf->can_id & CAN_RTR_FLAG)
    *pucStatusLen |= STLN_RTR;  // add RTR flag

  if (cf->can_id & CAN_EFF_FLAG)
  {
    localID.ul = cf->can_id & CAN_EFF_MASK;
    *pucStatusLen |= STLN_EXTENDED_ID;
    localID.ul   <<= 3;

    #if defined(__LITTLE_ENDIAN)
    *ptr++ = localID.uc[0];
    *ptr++ = localID.uc[1];
    *ptr++ = localID.uc[2];
    *ptr++ = localID.uc[3];
    #elif defined(__BIG_ENDIAN)
    *ptr++ = localID.uc[3];
    *ptr++ = localID.uc[2];
    *ptr++ = localID.uc[1];
    *ptr++ = localID.uc[0];
    #else
      #error  "Please fix the endianness defines in <asm/byteorder.h>"
    #endif
  }
  else
  {
    localID.ul = cf->can_id & CAN_SFF_MASK;
    localID.ul   <<= 21;

    #if defined(__LITTLE_ENDIAN)
    *ptr++ = localID.uc[2];
    *ptr++ = localID.uc[3];
    #elif defined(__BIG_ENDIAN)
    *ptr++ = localID.uc[3];
    *ptr++ = localID.uc[2];
    #else
      #error  "Please fix the endianness defines in <asm/byteorder.h>"
    #endif
  }

  // put data
  j = 0;
  while (ucLen--)
    *ptr++ = cf->data[j++];

  nMsgCounter++;

  // generate external nDataLength if I carry payload
  if ((ptr - ucMsgPtr) > 2)
  {
    *pnDataLength = nDataLength;

    // set count of telegrams
    ptr = ucMsgPtr + nDataLength - 1;
    *ptr = (u8)(dev->port.usb.dwTelegramCount++ & 0xff);

    // last to do: put count of messages
    *pucMsgCountPtr = nMsgCounter;
  }
  else
  {
    *pnDataLength   = 0;
    *pucMsgCountPtr = 0;
  }

  return -ENODATA; /* simulate empty fifo return value */
}

#endif
