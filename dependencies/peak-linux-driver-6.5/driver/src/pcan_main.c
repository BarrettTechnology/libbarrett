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
// pcan_main.c - the starting point of the driver,
//               init and cleanup and proc interface
//
// $Id: pcan_main.c 526 2007-10-29 21:42:02Z khitschler $
//
//****************************************************************************

//****************************************************************************
// INCLUDES
#include <src/pcan_common.h>     // must always be the 1st include
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
#include <linux/config.h>
#else
#include <linux/autoconf.h>
#endif

// #define KBUILD_MODNAME pcan

#include <linux/module.h>
#include <linux/kernel.h>   // DPRINTK()
#include <linux/slab.h>     // kmalloc()
#include <linux/fs.h>       // everything...
#include <linux/errno.h>    // error codes
#include <linux/types.h>    // size_t
#include <linux/proc_fs.h>  // proc
#include <linux/fcntl.h>    // O_ACCMODE
#include <linux/capability.h> // all about restrictions
#include <linux/param.h>    // because of HZ
#include <asm/system.h>     // cli(), *_flags
#include <asm/uaccess.h>    // copy_...

#include <src/pcan_main.h>

#ifdef PCI_SUPPORT
#include <src/pcan_pci.h>   // get support for PCAN-PCI
#endif
#ifdef ISA_SUPPORT
#include <src/pcan_isa.h>   // get support for PCAN-ISA and PCAN-104
#endif
#ifdef DONGLE_SUPPORT
#include <src/pcan_dongle.h> // get support for PCAN-Dongle
#endif
#ifdef USB_SUPPORT
#include <src/pcan_usb.h>   // get support for PCAN-USB
#endif
#ifdef PCCARD_SUPPORT
#include <src/pcan_pccard.h>
#endif
#ifdef NETDEV_SUPPORT
#include <src/pcan_netdev.h>
#endif

#include <src/pcan_fops.h>
#include <src/pcan_fifo.h>
#include <src/pcan_filter.h>

//****************************************************************************
// DEFINES
#define DEFAULT_BTR0BTR1    CAN_BAUD_500K  // defaults to 500 kbit/sec
#define DEFAULT_EXTENDED    1              // catch all frames
#define DEFAULT_LISTENONLY  0

//****************************************************************************
// GLOBALS

// filled by module initialisation
char *type[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
u16  io[8]    = {0, 0, 0, 0, 0, 0, 0, 0};
u8   irq[8]   = {0, 0, 0, 0, 0, 0, 0, 0};
u16  bitrate  = DEFAULT_BTR0BTR1;
char *assign  = NULL;

//----------------------------------------------------------------------------
// the global driver object, create it
struct driverobj pcan_drv = {};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
//----------------------------------------------------------------------------
// some stuff to support SysFS coming with kernel 2.6
#include <linux/device.h>

// not yet implemented

#endif

// build current driver config string for output in kernel log and procfs
const char current_config[] = " "
#ifdef DEBUG
"[dbg] "
#endif
#ifdef MODVERSIONS
"[mod] "
#endif
#ifdef ISA_SUPPORT
"[isa] "
#endif
#ifdef PCI_SUPPORT
"[pci] "
#endif
#ifdef DONGLE_SUPPORT
"[dng] "
#endif
#ifdef PARPORT_SUBSYSTEM
"[par] "
#endif
#ifdef USB_SUPPORT
"[usb] "
#endif
#ifdef PCCARD_SUPPORT
"[pcc] "
#endif
#ifdef NETDEV_SUPPORT
"[net] "
#endif
#ifndef NO_RT
"[rt] "
#endif
;

// for procfs output the current_config is copied into this string (centered!)
char config[] = "*----------------------------------------------------------------------------";

//****************************************************************************
// LOCALS

//****************************************************************************
// CODE
#ifdef NO_RT
  #include "pcan_main_linux.c"
#else
  #include "pcan_main_rt.c"
#endif

//****************************************************************************
// debug utility
void buffer_dump(u8 *pucBuffer, u16 wLineCount)
{
  #ifdef DEBUG
  int i, j;

  for (i = 0; i < wLineCount; i++)
  {
    printk(KERN_DEBUG "%s: %04x ", DEVICE_NAME, i * 16);

    for (j = 0; j < 8; j++)
      printk(" %02x", *pucBuffer++);

    printk(" ");

    for (j = 0; j < 8; j++)
      printk(" %02x", *pucBuffer++);

    printk("\n");
  }
  #endif
}

//----------------------------------------------------------------------------
// convert struct can_frame to struct TPCANMsg
// To reduce the complexity (and CPU usage) there are no checks (e.g. for dlc)
// here as it is assumed that the creator of the source struct has done this work
void frame2msg(struct can_frame *cf, TPCANMsg *msg)
{
  if (cf->can_id & CAN_ERR_FLAG)
  {
    memset(msg, 0, sizeof(*msg));
    msg->MSGTYPE = MSGTYPE_STATUS;
    msg->LEN     = 4;
    
    if (cf->can_id & CAN_ERR_CRTL)
    {
      // handle data overrun
      if (cf->data[1] & CAN_ERR_CRTL_RX_OVERFLOW)
        msg->DATA[3] |= CAN_ERR_OVERRUN;
      
      // handle CAN_ERR_BUSHEAVY
      if (cf->data[1] & CAN_ERR_CRTL_RX_WARNING)
        msg->DATA[3] |= CAN_ERR_BUSHEAVY;
    }
    
    if (cf->can_id & CAN_ERR_BUSOFF_NETDEV)
      msg->DATA[3] |= CAN_ERR_BUSOFF;
      
    return;
  }
  
  if (cf->can_id & CAN_RTR_FLAG)
    msg->MSGTYPE = MSGTYPE_RTR;
  else
    msg->MSGTYPE = MSGTYPE_STANDARD;
      
  if (cf->can_id & CAN_EFF_FLAG)
    msg->MSGTYPE |= MSGTYPE_EXTENDED;

  msg->ID  = cf->can_id & CAN_EFF_MASK; /* remove EFF/RTR/ERR flags */
  msg->LEN = cf->can_dlc; /* no need to check value range here */

  memcpy(&msg->DATA[0], &cf->data[0], 8); /* also copy trailing zeros */
}

//----------------------------------------------------------------------------
// convert struct TPCANMsg to struct can_frame
// To reduce the complexity (and CPU usage) there are no checks (e.g. for dlc)
// here as it is assumed that the creator of the source struct has done this work
void msg2frame(struct can_frame *cf, TPCANMsg *msg)
{
  cf->can_id = msg->ID;

  if (msg->MSGTYPE & MSGTYPE_RTR)
    cf->can_id |= CAN_RTR_FLAG;

  if (msg->MSGTYPE & MSGTYPE_EXTENDED)
    cf->can_id |= CAN_EFF_FLAG;

  // if (msg->MSGTYPE & MSGTYPE_??????)
  //   cf->can_id |= CAN_ERR_FLAG;

  cf->can_dlc = msg->LEN; /* no need to check value range here */

  memcpy(&cf->data[0], &msg->DATA[0], 8); /* also copy trailing zeros */
}

//----------------------------------------------------------------------------
// request time in msec, fast
u32 get_mtime(void)
{    
  return (jiffies / HZ) * 1000;
}

// x = (x >= y) ? x - y : 0;
static void subtract_timeval(struct timeval *x, struct timeval *y)
{
  if (x->tv_usec >= y->tv_usec)
    x->tv_usec -= y->tv_usec;
  else
  {
    if (x->tv_sec)
    {
      x->tv_sec--;
      x->tv_usec += (1000000 - y->tv_usec);
    }
    else
      goto fail;
  }
    
  if (x->tv_sec >= y->tv_sec)
  {
    x->tv_sec -= y->tv_sec;
    return;
  }
    
  fail:
  x->tv_sec = x->tv_usec = 0;
}
 
// get relative time to start of driver
void get_relative_time(struct timeval *tv, struct timeval *tr)
{
  if (!tv)
    DO_GETTIMEOFDAY((*tr));
  else
    memcpy(tr, tv, sizeof(*tr));
  
  subtract_timeval(tr, &pcan_drv.sInitTime);
}

// convert timeval to pcan used milliseconds / microseconds notation
void timeval2pcan(struct timeval *tv, u32 *msecs, u16 *usecs)
{
  *msecs = (u32)(tv->tv_sec * 1000 + tv->tv_usec / 1000);
  *usecs = (u16)(tv->tv_usec % 1000);  
}

//----------------------------------------------------------------------------
// is called when 'cat /proc/pcan' invoked
static int pcan_read_procmem(char *page, char **start, off_t offset, int count, int *eof, void *data)
{
  struct pcandev *dev;
  struct list_head *ptr;
  int    len = 0;
  
  DPRINTK(KERN_DEBUG "%s: pcan_read_procmem()\n", DEVICE_NAME);

  len += sprintf(page + len, "\n");
  len += sprintf(page + len, "*------------ PEAK-Systems CAN interfaces (www.peak-system.com) -------------\n");
  len += sprintf(page + len, "*--------------------------  %s  ----------------------------\n", pcan_drv.szVersionString);
  len += sprintf(page + len, "%s\n", config);
  len += sprintf(page + len, "*--------------------- %d interfaces @ major %03d found -----------------------\n", 
                     pcan_drv.wDeviceCount, pcan_drv.nMajor);
  len += sprintf(page + len, "*n -type- ndev --base-- irq --btr- --read-- --write- --irqs-- -errors- status\n");
  
  // loop trough my devices
  for (ptr = pcan_drv.devices.next; ptr != &pcan_drv.devices; ptr = ptr->next)
  {
    u32 dwPort = 0;
    u16 wIrq   = 0;
    #ifdef NETDEV_SUPPORT
    struct net_device_stats *stats; /* rx/tx statistics can be found here */
    #endif
    
    dev = (struct pcandev *)ptr;  
    switch (dev->wType)
    {
      case HW_ISA_SJA:
        dwPort = dev->port.isa.dwPort;
        wIrq   = dev->port.isa.wIrq;
        break;
      case HW_DONGLE_SJA:
      case HW_DONGLE_SJA_EPP:
        dwPort = dev->port.dng.dwPort;
        wIrq   = dev->port.dng.wIrq;
        break;
      case HW_PCI:
        dwPort = dev->port.pci.dwPort;
        wIrq   = dev->port.pci.wIrq;
        break;
      case HW_USB:
        #ifdef USB_SUPPORT
        // get serial number of device
        if (!dev->ucPhysicallyInstalled)
        {
          dev->port.usb.dwSerialNumber   = 0x00dead00;  // it is dead
          dev->port.usb.ucHardcodedDevNr = 0;
        }

        dwPort = dev->port.usb.dwSerialNumber;
        wIrq   = dev->port.usb.ucHardcodedDevNr;
        #endif
        break;
      case HW_PCCARD:
        #ifdef PCCARD_SUPPORT
        dwPort = dev->port.pccard.dwPort;
        wIrq   = dev->port.pccard.wIrq;
        #endif
        break;
    }

    #ifdef NETDEV_SUPPORT
    stats = dev->netdev->get_stats(dev->netdev);
    #endif

    len += sprintf(page + len, "%2d %6s %4s %08x %03d 0x%04x %08lx %08lx %08x %08x 0x%04x\n",
                   dev->nMinor,
                   dev->type,
                   #ifdef NETDEV_SUPPORT
                   dev->netdev->name,
                   #else
                   "-NA-",
                   #endif
                   dwPort,
                   wIrq,
                   dev->wBTR0BTR1,
                   #ifdef NETDEV_SUPPORT
                   stats->rx_packets,
                   stats->tx_packets + dev->writeFifo.dwTotal,
                   #else
                   (unsigned long)dev->readFifo.dwTotal,
                   (unsigned long)dev->writeFifo.dwTotal,
                   #endif
                   dev->dwInterruptCounter,
                   dev->dwErrorCounter,
                   dev->wCANStatus);
  }
  
  len += sprintf(page + len, "\n");
  
  *eof = 1;
  return len;
}

void dev_unregister(void)
{
#ifdef NETDEV_SUPPORT
  struct list_head *ptr;
  struct pcandev   *pdev;
  // remove all netdevice registrations except those for USB-devices
  // which is done by pcan_usb_plugout().
  for (ptr = pcan_drv.devices.next; ptr != &pcan_drv.devices; ptr = ptr->next)
  {
    pdev = (struct pcandev *)ptr;
    if (pdev->wType != HW_USB)
      pcan_netdev_unregister(pdev);
  }
#endif
}

void remove_dev_list(void)
{
  struct pcandev *dev;
  while (!list_empty(&pcan_drv.devices)) // cycle through the list of devices and remove them
  {
    dev = (struct pcandev *)pcan_drv.devices.prev; // empty in reverse order
    dev->cleanup(dev);
    list_del(&dev->list);
    // free all device allocted memory 
    kfree(dev);
  }
}
//----------------------------------------------------------------------------
// is called when the device is removed 'rmmod pcan'
void cleanup_module(void)
{

  DPRINTK(KERN_DEBUG "%s: cleanup_module()\n", DEVICE_NAME);

  switch (pcan_drv.wInitStep)
  {
    case 3: remove_proc_entry(DEVICE_NAME, NULL);
    case 2:
            DEV_UNREGISTER();
    case 1: 
    case 0: 
            #ifdef USB_SUPPORT
            pcan_usb_deinit();
            #endif

            #ifdef PCCARD_SUPPORT
            pcan_pccard_deinit();
            #endif

            REMOVE_DEV_LIST();

            pcan_drv.wInitStep = 0;
  }

  printk(KERN_INFO "%s: removed.\n", DEVICE_NAME);
}

//----------------------------------------------------------------------------
// init some equal parts of dev
void pcan_soft_init(struct pcandev *dev, char *szType, u16 wType)
{
  dev->wType            = wType;
  dev->type             = szType; 
        
  dev->nOpenPaths       = 0;
  dev->nLastError       = 0;
  dev->busStatus        = CAN_ERROR_ACTIVE;
  dev->dwErrorCounter   = 0;
  dev->dwInterruptCounter = 0;
  dev->wCANStatus       = 0;
  dev->bExtended        = 1;   // accept all frames
  dev->wBTR0BTR1        = bitrate;
  dev->ucCANMsgType     = DEFAULT_EXTENDED;
  dev->ucListenOnly     = DEFAULT_LISTENONLY;
  
  memset(&dev->props, 0, sizeof(dev->props));

  // set default access functions
  dev->device_open      = NULL;
  dev->device_release   = NULL;
  dev->device_write     = NULL;
  #ifdef NETDEV_SUPPORT
  dev->netdevice_write  = NULL;
  #endif

  dev->ucPhysicallyInstalled = 0;  // assume the device is not installed
  dev->ucActivityState       = ACTIVITY_NONE;

  atomic_set(&dev->DataSendReady, 1);

  // init fifos
  pcan_fifo_init(&dev->readFifo,   &dev->rMsg[0], &dev->rMsg[READ_MESSAGE_COUNT - 1],  READ_MESSAGE_COUNT,  sizeof(TPCANRdMsg));
  pcan_fifo_init(&dev->writeFifo,  &dev->wMsg[0], &dev->wMsg[WRITE_MESSAGE_COUNT - 1], WRITE_MESSAGE_COUNT, sizeof(TPCANMsg) );
}

//----------------------------------------------------------------------------
// create all declared Peak legacy devices
static int make_legacy_devices(void)
{
  int result = 0;
  int i;

  DPRINTK(KERN_DEBUG "%s: make_legacy_devices()\n", DEVICE_NAME);
  
  for (i = 0; ((i < 8) && (type[i] != NULL)); i++)
  {
    #ifdef ISA_SUPPORT
    if (!strncmp(type[i], "isa", 4))
      result = pcan_create_isa_devices(type[i], io[i], irq[i]);
    #endif
    
    #ifdef DONGLE_SUPPORT
    if (!strncmp(type[i], "sp", 4) || !strncmp(type[i], "epp", 4))
      result = pcan_create_dongle_devices(type[i], io[i], irq[i]);
    #endif
        
    if (result)
      break;
  }
  
  #ifdef ISA_SUPPORT
  // create lists of devices with the same irqs
  ISA_SHARED_IRQ_LISTS();
  #endif
 
  return result;
}

//----------------------------------------------------------------------------
// is called when the device is installed 'insmod pcan.o' or 'insmod pcan.ko'
int init_module(void)
{
  int result = 0;

  memset(&pcan_drv, 0, sizeof(pcan_drv));
  pcan_drv.wInitStep       = 0;
  DO_GETTIMEOFDAY(pcan_drv.sInitTime); // store time for timestamp relation, increments in usec
  pcan_drv.szVersionString = CURRENT_RELEASE; // get the release name global
  pcan_drv.nMajor          = PCAN_MAJOR;
  
  printk(KERN_INFO "%s: %s\n", DEVICE_NAME, pcan_drv.szVersionString);
  printk(KERN_INFO "%s: driver config%s\n", DEVICE_NAME, current_config);

  // Copy the centered string only one time and use sizeof() for
  // compiletime value calculation and optimisation. Also ensure
  // to have a valid current_config and that it fits into config[]
  if ((sizeof(current_config) > 3) && (sizeof(config) > sizeof(current_config)))
    strncpy(config + (sizeof(config)-sizeof(current_config))/2, current_config, sizeof(current_config)-1);

  INIT_LIST_HEAD(&pcan_drv.devices);
  pcan_drv.wDeviceCount = 0;

  #ifdef PCI_SUPPORT
  // search pci devices
  if ((result = pcan_search_and_create_pci_devices()))
    goto fail;
  #endif

  // create isa and dongle devices
  if ((result = make_legacy_devices()))
    goto fail;

  #ifdef USB_SUPPORT
  // register usb devices only
  if ((result = pcan_usb_register_devices()))
    goto fail;
  #endif
  
  #ifdef PCCARD_SUPPORT
  if ((result = pcan_pccard_register_devices()))
    goto fail;
  #endif

  #if !defined USB_SUPPORT && !defined PCCARD_SUPPORT
  // no device found, stop all
  if (!pcan_drv.wDeviceCount)
    goto fail;
  #endif
  
  pcan_drv.wInitStep = 1;

  result = DEV_REGISTER();
  if (result < 0)
    goto fail;

  if (!pcan_drv.nMajor)
    pcan_drv.nMajor = result;

  pcan_drv.wInitStep = 2;

  // create the proc entry
  if (create_proc_read_entry(DEVICE_NAME, 0, NULL, pcan_read_procmem, NULL) == NULL)
  {
    result = -ENODEV; // maybe wrong if there is no proc filesystem configured
    goto fail;
  }

  pcan_drv.wInitStep = 3;

  printk(KERN_INFO "%s: major %d.\n", DEVICE_NAME, pcan_drv.nMajor);
  return 0; // succeed

  fail:
  cleanup_module();
  return result;
}
