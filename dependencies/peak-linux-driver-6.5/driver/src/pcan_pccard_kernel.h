#ifndef __PCAN_PCCARD_KERNEL_H__
#define __PCAN_PCCARD_KERNEL_H__
//****************************************************************************
// Copyright (C) 2006-2007  PEAK System-Technik GmbH
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
//****************************************************************************

//***************************************************************************
//
// all parts to handle the interface specific parts of pcan-pccard
//
// $Id: pcan_pccard_kernel.h 517 2007-07-09 09:40:42Z edouard $
//
//****************************************************************************

//****************************************************************************
// INCLUDES
#include <src/pcan_common.h>     // must always be the 1st include
#include <asm/io.h>
#include <linux/timer.h>         // to support activity scan

#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/ds.h>

#include <src/pcan_main.h>

//****************************************************************************
// DEFINES
#define PCCARD_MINOR_BASE  40               // starting point of minors for PCCARD devices

#define PCCARD_CHANNELS    2                // maximum n channels per card

typedef struct pcan_pccard
{
  #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17)
  struct pcmcia_device *pcc_dev;             // associated system pcmcia device 
  #else
  dev_link_t link;
  #endif
  ioaddr_t   basePort;                       // base of io area for all channels
  u_int      commonIrq;                      // irq for all channels
  ioaddr_t   commonPort;                     // channels commonly used port 
  struct     pcandev *dev[PCCARD_CHANNELS];  // point to associated channels
  
  #ifdef NO_RT
  int       run_activity_timer_cyclic;       // a flag to synchronize stop conditions
  struct    timer_list activity_timer;       // to scan for activity, set the time
  #endif
  
  dev_node_t node;                           // to satisfy cardmgr needs
} PCAN_PCCARD;

int  pccard_create_all_devices(struct pcan_pccard *card);
void pccard_release_all_devices(struct pcan_pccard *card);

#endif // __PCAN_PCCARD_KERNEL_H__
