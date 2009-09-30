#ifndef __PCAN_USB_KERNEL_H__
#define __PCAN_USB_KERNEL_H__
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
//                Oliver Hartkopp   (oliver.hartkopp@volkswagen.de) socketCAN
//                     
//****************************************************************************

//****************************************************************************
//
// pcan_usb-kernel.h - the inner usb parts header for pcan-usb support
//
// $Id: pcan_usb_kernel.h 447 2007-01-28 14:05:50Z khitschler $
//
//****************************************************************************


//****************************************************************************
// DEFINES

//****************************************************************************
// INCLUDES
#include <linux/types.h>
#include <linux/usb.h>

#include <src/pcan_main.h>

#ifdef LINUX_26
  #define __usb_submit_urb(x) usb_submit_urb(x, GFP_ATOMIC)
  #define __usb_alloc_urb(x)  usb_alloc_urb(x, GFP_ATOMIC)
  #define FILL_BULK_URB(a, b, c, d, e, f, g) usb_fill_bulk_urb(a, b, c, d, e, (usb_complete_t)f, (void *)g)
#else
  #define __usb_submit_urb(x) usb_submit_urb(x)
  #define __usb_alloc_urb(x)  usb_alloc_urb(x)
#endif

//****************************************************************************
// DEFINES
int pcan_hw_SetCANOn(struct pcandev *dev);
int pcan_hw_SetCANOff(struct pcandev *dev);

int pcan_hw_Init(struct pcandev *dev, u16 btr0btr1, u8 bListenOnly);
int pcan_hw_getSNR(struct pcandev *dev, u32 *pdwSNR);

int pcan_hw_DecodeMessage(struct pcandev *dev, u8 *ucMsgPtr, int lCurrentLength);
int pcan_hw_EncodeMessage(struct pcandev *dev, u8 *ucMsgPtr, int *pnDataLength);
#ifdef NETDEV_SUPPORT
int pcan_hw_EncodeMessage_frame(struct pcandev *dev, struct can_frame *cf, u8 *ucMsgPtr, int *pnDataLength);
#endif

#endif // __PCAN_USB_KERNEL_H__

