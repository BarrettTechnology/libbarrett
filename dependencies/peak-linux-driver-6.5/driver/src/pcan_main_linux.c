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
// Contributions: Marcel Offermans (marcel.offermans@luminis.nl)
//                Philipp Baer     (philipp.baer@informatik.uni-ulm.de)
//                Garth Zeglin     (garthz@ri.cmu.edu)
//                Harald Koenig    (H.Koenig@science-computing.de)
//****************************************************************************

//****************************************************************************
//
// pcan_main_linux.c - the starting point of the driver,
//               init and cleanup and proc interface
//
// $Id: pcan_main_linux.c $
//
//****************************************************************************

//****************************************************************************
// DEFINES
#define DEV_REGISTER dev_register
#define DEV_UNREGISTER() unregister_chrdev(pcan_drv.nMajor, DEVICE_NAME);\
                         dev_unregister()
#define REMOVE_DEV_LIST remove_dev_list
#define ISA_SHARED_IRQ_LISTS pcan_create_isa_shared_irq_lists

//----------------------------------------------------------------------------
// put received CAN frame into chardev receive FIFO
// maybe this goes to a new file pcan_chardev.c some day.
int pcan_chardev_rx(struct pcandev *dev, struct can_frame *cf, struct timeval *tv)
{
  int result = 0;

  // filter out extended messages in non extended mode
  if (dev->bExtended || !(cf->can_id & CAN_EFF_FLAG)) 
  {
    if (!pcan_do_filter(dev->filter, cf->can_id))
    {
      TPCANRdMsg msg;
      struct timeval tr;
      get_relative_time(tv, &tr);
      timeval2pcan(&tr, &msg.dwTime, &msg.wUsec);

      /* convert to old style FIFO message until FIFO supports new */
      /* struct can_frame and error frames */
      frame2msg(cf, &msg.Msg);
      
      // step forward in fifo
      result = pcan_fifo_put(&dev->readFifo, &msg);
      
      // flag to higher layers that a message was put into fifo or an error occurred
      result = (result) ? result : 1;
    }
  }

  return result;
}

static int dev_register(void)
{
  int result = 0;

  // register the driver by the OS
  if ((result = register_chrdev(pcan_drv.nMajor, DEVICE_NAME, &pcan_fops)) < 0)
    return result;

  #ifdef NETDEV_SUPPORT
  {
    struct list_head *ptr;
    struct pcandev   *pdev;
    
    // create all netdevice entries except those for hotplug-devices
    // USB   : is done by pcan_usb_plugin().
    // PCCARD: is done by pcan_pccard_register_devices() at driver init time 
    //         (here & now! - see above) or at plugin time.
    for (ptr = pcan_drv.devices.next; ptr != &pcan_drv.devices; ptr = ptr->next)
    {
      pdev = (struct pcandev *)ptr;
      if ((pdev->wType != HW_USB) && (pdev->wType != HW_PCCARD))
        pcan_netdev_register(pdev);
    }
  }
  #endif

  return result;
}
