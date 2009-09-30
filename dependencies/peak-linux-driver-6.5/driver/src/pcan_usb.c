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
// Contributions: Philipp Baer (philipp.baer@informatik.uni-ulm.de)
//                Tom Heinrich
//                John Privitera (JohnPrivitera@dciautomation.com)
//****************************************************************************

//****************************************************************************
//
// pcan_usb.c - the outer usb parts for pcan-usb support
//
// $Id: pcan_usb.c 510 2007-05-29 13:07:10Z khitschler $
//
//****************************************************************************

//****************************************************************************
// INCLUDES
#include <src/pcan_common.h>     // must always be the 1st include
#include <linux/stddef.h>        // NULL
#include <linux/errno.h>
#include <linux/slab.h>          // kmalloc()
#include <linux/module.h>        // MODULE_DEVICE_TABLE()

#include <linux/usb.h>

#include <src/pcan_main.h>
#include <src/pcan_fops.h>
#include <src/pcan_usb.h>
#include <src/pcan_usb_kernel.h>
#include <src/pcan_filter.h>

#ifdef NETDEV_SUPPORT
#include <src/pcan_netdev.h>     // for hotplug pcan_netdev_(un)register()
#endif

//****************************************************************************
// DEFINES
#define PCAN_USB_MINOR_BASE 32           // starting point of minors for USB devices
#define PCAN_USB_VENDOR_ID  0x0c72
#define PCAN_USB_PRODUCT_ID 0x000c

#define URB_READ_BUFFER_SIZE      1024   // buffer for read URB data (IN)
#define URB_READ_BUFFER_SIZE_OLD    64   // used length for revision < 7

#define URB_WRITE_BUFFER_SIZE      128   // buffer for write URB (OUT)
#define URB_WRITE_BUFFER_SIZE_OLD   64   // used length for hardware < 7

#define MAX_CYCLES_TO_WAIT_FOR_RELEASE 100  // max no. of schedules before release

#define STARTUP_WAIT_TIME            2   // wait this time at startup to get first messages

//****************************************************************************
// GLOBALS
static struct usb_device_id pcan_usb_ids[] =
{
  { USB_DEVICE(PCAN_USB_VENDOR_ID, PCAN_USB_PRODUCT_ID) },
  { }            // Terminating entry
};

MODULE_DEVICE_TABLE (usb, pcan_usb_ids);

#ifdef LINUX_26
static struct usb_class_driver pcan_class =
{
  .name = "usb/pcan%d",
  .fops = &pcan_fops,
  #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
  .mode = S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
  #endif
  .minor_base = PCAN_USB_MINOR_BASE,
};
#endif

//****************************************************************************
// LOCALS
static u16 usb_devices = 0;        // the number of accepted usb_devices

//****************************************************************************
// CODE

//****************************************************************************
// get cyclic data from endpoint 2
#ifdef LINUX_26
static void pcan_usb_write_notify(struct urb *purb, struct pt_regs *pregs);
#else
static void pcan_usb_write_notify(purb_t purb);
#endif

static int pcan_usb_write(struct pcandev *dev)
{
  int err = 0;
  USB_PORT *u = &dev->port.usb;
  int nDataLength;
  int m_nCurrentLength;

  DPRINTK(KERN_DEBUG "%s: pcan_usb_write()\n", DEVICE_NAME);

  // don't do anything with non-existent hardware
  if (!dev->ucPhysicallyInstalled)
    return -ENODEV;

  // improvement to control 128 bytes buffers
  m_nCurrentLength    = URB_WRITE_BUFFER_SIZE_OLD;
  err = pcan_hw_EncodeMessage(dev, u->pucWriteBuffer, &m_nCurrentLength);
  if (err || (u->ucRevision < 7))
    nDataLength = m_nCurrentLength;
  else
  {
    m_nCurrentLength     = URB_WRITE_BUFFER_SIZE_OLD;
    err = pcan_hw_EncodeMessage(dev, u->pucWriteBuffer + 64, &m_nCurrentLength);
    nDataLength          = URB_WRITE_BUFFER_SIZE_OLD + m_nCurrentLength;    
  }
  
  // fill the URB and submit
  if (nDataLength)
  {
    FILL_BULK_URB(u->write_data, u->usb_dev,
                  usb_sndbulkpipe(u->usb_dev, u->Endpoint[3].ucNumber),
                  u->pucWriteBuffer, nDataLength, pcan_usb_write_notify, dev);

    // start next urb
    if ((err = __usb_submit_urb(u->write_data)))
    {
      dev->nLastError = err;
      dev->dwErrorCounter++; 
      DPRINTK(KERN_ERR "%s: pcan_usb_write() can't submit! (%d)",DEVICE_NAME, err);
    }
    else
      atomic_inc(&dev->port.usb.active_urbs);
  }

  // it's no error if I can't get more data but still a packet was sent
  if ((err == -ENODATA) && (nDataLength))
    err = 0;

  return err;
}

#ifdef NETDEV_SUPPORT
static int pcan_usb_write_frame(struct pcandev *dev, struct can_frame *cf)
{
  int err = 0;
  USB_PORT *u = &dev->port.usb;
  int nDataLength;
  int m_nCurrentLength;

  DPRINTK(KERN_DEBUG "%s: %s\n", DEVICE_NAME, __FUNCTION__);

  // don't do anything with non-existent hardware
  if (!dev->ucPhysicallyInstalled)
    return -ENODEV;

  // improvement to control 128 bytes buffers
  m_nCurrentLength    = URB_WRITE_BUFFER_SIZE_OLD;
  err = pcan_hw_EncodeMessage_frame(dev, cf, u->pucWriteBuffer, &m_nCurrentLength);
  if (err || (u->ucRevision < 7))
    nDataLength = m_nCurrentLength;
  else
  {
    m_nCurrentLength     = URB_WRITE_BUFFER_SIZE_OLD;
    err = pcan_hw_EncodeMessage_frame(dev, cf, u->pucWriteBuffer + 64, &m_nCurrentLength);
    nDataLength          = URB_WRITE_BUFFER_SIZE_OLD + m_nCurrentLength;    
  }
  
  // fill the URB and submit
  if (nDataLength)
  {
    FILL_BULK_URB(u->write_data, u->usb_dev,
                  usb_sndbulkpipe(u->usb_dev, u->Endpoint[3].ucNumber),
                  u->pucWriteBuffer, nDataLength, pcan_usb_write_notify, dev);

    // start next urb
    if ((err = __usb_submit_urb(u->write_data)))
    {
      dev->nLastError = err;
      dev->dwErrorCounter++; 
      DPRINTK(KERN_ERR "%s: pcan_usb_write_frame() can't submit! (%d)",DEVICE_NAME, err);
    }
    else
      atomic_inc(&dev->port.usb.active_urbs);
  }

  // it's no error if I can't get more data but still a packet was sent
  if ((err == -ENODATA) && (nDataLength))
    err = 0;

  return err;
}
#endif

//****************************************************************************
// notify functions for read and write data
#ifdef LINUX_26
static void pcan_usb_write_notify(struct urb *purb, struct pt_regs *pregs)
#else
static void pcan_usb_write_notify(purb_t purb)
#endif
{
  int err     = 0;
  u16 wwakeup = 0;
  struct pcandev *dev = purb->context;

  // DPRINTK(KERN_DEBUG "%s: pcan_usb_write_notify() (%d)\n", DEVICE_NAME, purb->status);

  // un-register outstanding urb
  atomic_dec(&dev->port.usb.active_urbs);

  // don't count interrupts - count packets
  dev->dwInterruptCounter++;                                

  // do write
  if (!purb->status) // stop with first error
    err = pcan_usb_write(dev);

  if (err)
  {
   if (err == -ENODATA)
     wwakeup++;
   else
   {
     DPRINTK(KERN_DEBUG "%s: unexpected error %d from pcan_usb_write()\n", DEVICE_NAME, err);
     dev->nLastError = err;
     dev->dwErrorCounter++;
     dev->wCANStatus |= CAN_ERR_QXMTFULL; // fatal error!
   }
  }

  if (wwakeup)
  {
    atomic_set(&dev->DataSendReady, 1); // signal to write I'm ready
    wake_up_interruptible(&dev->write_queue);

    #ifdef NETDEV_SUPPORT
    netif_wake_queue(dev->netdev);
    #endif
  }
}

#ifdef LINUX_26
static void pcan_usb_read_notify(struct urb *purb, struct pt_regs *pregs)
#else
static void pcan_usb_read_notify(purb_t purb)
#endif
{
  int err = 0;
  struct pcandev *dev = purb->context;
  USB_PORT *u = &dev->port.usb;

  // DPRINTK(KERN_DEBUG "%s: pcan_usb_read_notify() (%d)\n", DEVICE_NAME, purb->status);

  // un-register outstanding urb
  atomic_dec(&dev->port.usb.active_urbs);

  // don't count interrupts - count packets
  dev->dwInterruptCounter++;                                

  // do interleaving read
  if (!purb->status && dev->ucPhysicallyInstalled) // stop with first error
  {
    u8  *m_pucTransferBuffer = purb->transfer_buffer;
    int m_nCurrentLength     = purb->actual_length;

    // buffer interleave to increase speed
    if (m_pucTransferBuffer == u->pucReadBuffer[0])
    {
      FILL_BULK_URB(purb, u->usb_dev,
                    usb_rcvbulkpipe(u->usb_dev, u->Endpoint[2].ucNumber),
                    u->pucReadBuffer[1], u->wReadBufferLength, pcan_usb_read_notify, dev);
    }
    else
    {
      FILL_BULK_URB(purb, u->usb_dev,
                    usb_rcvbulkpipe(u->usb_dev, u->Endpoint[2].ucNumber),
                    u->pucReadBuffer[0], u->wReadBufferLength, pcan_usb_read_notify, dev);
    }

    // start next urb
    if ((err = __usb_submit_urb(purb)))
    {
      dev->nLastError = err; 
      dev->dwErrorCounter++;
      printk(KERN_ERR "%s: pcan_usb_read_notify() can't submit! (%d)",DEVICE_NAME, err);
    }
    else
      atomic_inc(&dev->port.usb.active_urbs);

    do
    {
      err = pcan_hw_DecodeMessage(dev, m_pucTransferBuffer, m_nCurrentLength);
      if (err < 0)
      {
        dev->nLastError = err; 
        dev->wCANStatus |=  CAN_ERR_QOVERRUN;
        dev->dwErrorCounter++;
        DPRINTK(KERN_DEBUG "%s: error %d from pcan_hw_DecodeMessage()\n", DEVICE_NAME, err);
      }
      m_pucTransferBuffer += 64;
      m_nCurrentLength    -= 64;
    } while (m_nCurrentLength > 0);
  }
  else
  {
    if (purb->status != -ENOENT)
    {
      printk(KERN_ERR "%s: read data stream torn off caused by ", DEVICE_NAME);
      if (!dev->ucPhysicallyInstalled)
        printk("device plug out!\n");
      else
        printk("err %d!\n", purb->status);
    }
  }
}

//****************************************************************************
// usb resource allocation 
//
static int pcan_usb_allocate_resources(struct pcandev *dev)
{
  int err = 0;
  USB_PORT *u = &dev->port.usb;

  DPRINTK(KERN_DEBUG "%s: pcan_usb_allocate_resources()\n", DEVICE_NAME);

  // allocate PCAN_USB_TIME data for comparison
  // TODO: integrate PCAN_USB_TIME in USB_PORT
  if (!u->pUSBtime)
  {
    if (!(u->pUSBtime = kmalloc(sizeof(PCAN_USB_TIME), GFP_ATOMIC)))
    {
      err = -ENOMEM;
      goto fail;
    }
  }
  dev->wInitStep = 4;

  // make param URB
  u->param_urb = __usb_alloc_urb(0);
  if (!u->param_urb)
    err = -ENOMEM;
  dev->wInitStep = 5;

  // allocate write buffer
  if (u->ucRevision < 7)
    u->pucWriteBuffer =  kmalloc(URB_WRITE_BUFFER_SIZE_OLD, GFP_ATOMIC);
  else
    u->pucWriteBuffer =  kmalloc(URB_WRITE_BUFFER_SIZE, GFP_ATOMIC);
  if (!u->pucWriteBuffer)
  {
    err = -ENOMEM;
    goto fail;
  }
  dev->wInitStep = 6;

  // make write urb
  u->write_data = __usb_alloc_urb(0);
  if (!u->write_data)
  {
    err = -ENOMEM;
    goto fail;
  }
  dev->wInitStep = 7;

  // reset telegram count
  u->dwTelegramCount = 0;

  // allocate both read buffers for URB
  u->pucReadBuffer[0] =  kmalloc(URB_READ_BUFFER_SIZE * 2, GFP_ATOMIC);
  if (!u->pucReadBuffer[0])
  {
    err = -ENOMEM;
    goto fail;
  }
  u->pucReadBuffer[1] = u->pucReadBuffer[0] + URB_READ_BUFFER_SIZE;
  dev->wInitStep = 8;

  // make read urb
  u->read_data = __usb_alloc_urb(0);
  if (!u->read_data)
  {
    err = -ENOMEM;
    goto fail;
  }
  dev->wInitStep = 9;

  // different revisions use different buffer sizes
  if (u->ucRevision < 7)
    u->wReadBufferLength = URB_READ_BUFFER_SIZE_OLD;
  else
    u->wReadBufferLength = URB_READ_BUFFER_SIZE;

  fail:
  return err;
}

//****************************************************************************
// usb resource de-allocation 
//
static int pcan_usb_free_resources(struct pcandev *dev)
{
  int err = 0;
  USB_PORT *u = &dev->port.usb;

  DPRINTK(KERN_DEBUG "%s: pcan_usb_free_resources()\n", DEVICE_NAME);

  // at this point no URB must be pending
  // this is forced at pcan_usb_stop
  switch (dev->wInitStep)
  {
    case 9:
      // free read URB
      usb_free_urb(u->read_data);

    case 8:
      kfree(u->pucReadBuffer[0]);

    case 7:
      // free write urb
      usb_free_urb(u->write_data);

    case 6:
      kfree(u->pucWriteBuffer);

    case 5:
      // free param urb
      usb_free_urb(u->param_urb);

    case 4:
      // remove PCAN_USB_TIME information
      if (u->pUSBtime)
      {
        kfree(u->pUSBtime);
        u->pUSBtime = NULL;
      }

      dev->wInitStep = 3;
  }

  return err;
}

//****************************************************************************
// start and stop functions for CAN data IN and OUT
static int pcan_usb_start(struct pcandev *dev)
{
  int err = 0;
  USB_PORT *u = &dev->port.usb;

  DPRINTK(KERN_DEBUG "%s: pcan_usb_start(), minor = %d.\n", DEVICE_NAME, dev->nMinor);

  FILL_BULK_URB(u->read_data, u->usb_dev,
                usb_rcvbulkpipe(u->usb_dev, u->Endpoint[2].ucNumber),
                u->pucReadBuffer[0], u->wReadBufferLength, pcan_usb_read_notify, dev);

  // submit urb
  if ((err = __usb_submit_urb(u->read_data)))
    printk(KERN_ERR "%s: pcan_usb_start() can't submit! (%d)\n",DEVICE_NAME, err);
  else
    atomic_inc(&dev->port.usb.active_urbs);

  return err;
}

static int pcan_kill_sync_urb(struct urb *urb)
{
  int err = 0;
  
  if (urb->status == -EINPROGRESS)
  {
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
    usb_kill_urb(urb);
    #else
    err = usb_unlink_urb(urb);
    #endif
    DPRINTK(KERN_DEBUG "%s: pcan_kill_sync_urb done ...\n", DEVICE_NAME);
  }
  
  return err;
}

static int pcan_usb_stop(struct pcandev *dev)
{
  int err = 0;
  USB_PORT *u = &dev->port.usb;
  int i = MAX_CYCLES_TO_WAIT_FOR_RELEASE;

  DPRINTK(KERN_DEBUG "%s: pcan_usb_stop(), minor = %d.\n", DEVICE_NAME, dev->nMinor);

  err = pcan_hw_SetCANOff(dev);

  // wait until all has settled
  mdelay(5);

  // unlink URBs
  pcan_kill_sync_urb(u->read_data);
  pcan_kill_sync_urb(u->write_data);
  pcan_kill_sync_urb(u->param_urb);

  // wait until all urbs returned to sender
  // (I hope) this should be no problem because all urb's are unlinked 
  while ((atomic_read(&u->active_urbs) > 0) && (i--))
    schedule();
  if (i <= 0)
  {
    DPRINTK(KERN_ERR "%s: have still active URBs: %d!\n", DEVICE_NAME, atomic_read(&u->active_urbs));
  }

  return err;
}

//****************************************************************************
// remove device resources 
//
static int pcan_usb_cleanup(struct pcandev *dev)
{
  DPRINTK(KERN_DEBUG "%s: pcan_usb_cleanup()\n", DEVICE_NAME);

  if (dev)
  {
    pcan_usb_free_resources(dev);

    switch(dev->wInitStep)
    {
      case 4: 
              #ifdef NETDEV_SUPPORT
              pcan_netdev_unregister(dev);
              #endif
      case 3: usb_devices--;
              pcan_drv.wDeviceCount--;
      case 2: list_del(&dev->list);
      case 1: 
      case 0: pcan_delete_filter_chain(dev->filter);
              dev->filter = NULL;
              dev->wInitStep = 0;
              kfree(dev);
    }
  }
  
  return 0;
}

// dummy entries for request and free irq
static int pcan_usb_req_irq(struct pcandev *dev)
{
  DPRINTK(KERN_DEBUG "%s: pcan_usb_req_irq()\n", DEVICE_NAME);
  return 0;
}

static void pcan_usb_free_irq(struct pcandev *dev)
{
  DPRINTK(KERN_DEBUG "%s: pcan_usb_free_irq()\n", DEVICE_NAME);

  // mis-used here for another purpose
  // pcan_usb_free_irq() calls when the last path to device just closing
  // and the device itself is already plugged out
  if ((dev) && (!dev->ucPhysicallyInstalled))
    pcan_usb_cleanup(dev);
}

// interface depended open and close
static int pcan_usb_open(struct pcandev *dev)
{
  DPRINTK(KERN_DEBUG "%s: pcan_usb_open(), minor = %d.\n", DEVICE_NAME, dev->nMinor);

  return 0;
}

static int pcan_usb_release(struct pcandev *dev)
{
  DPRINTK(KERN_DEBUG "%s: pcan_usb_release(), minor = %d.\n", DEVICE_NAME, dev->nMinor);

  return 0;
}

// emulated device access functions
// call is only possible if device exists
static int pcan_usb_device_open(struct pcandev *dev, u16 btr0btr1, u8 bExtended, u8 bListenOnly)
{
  int err = 0;

  DPRINTK(KERN_DEBUG "%s: pcan_usb_device_open(), minor = %d.\n", DEVICE_NAME, dev->nMinor);

  // in general, when second open() occures
  // remove and unlink urbs, when interface is already running
  if ((dev->nOpenPaths) && (dev->device_release))
    dev->device_release(dev);

  // first action: turn CAN off
  if ((err = pcan_hw_SetCANOff(dev)))
    goto fail;

  if ((err = pcan_usb_start(dev)))
    goto fail;

  // init hardware specific parts
  if ((err = pcan_hw_Init(dev, btr0btr1, bListenOnly)))
    goto fail;

  // store extended mode (standard still accepted)
  dev->bExtended = bExtended;

  // take a fresh status
  dev->wCANStatus = 0;

  // copy from NT driver
  mdelay(20);

  // last action: turn CAN on
  if ((err = pcan_hw_SetCANOn(dev)))
    goto fail;
    
  // delay to get first messages read
  set_current_state(TASK_INTERRUPTIBLE);
  schedule_timeout(STARTUP_WAIT_TIME * HZ);

  fail:
  return err;
}

static void pcan_usb_device_release(struct pcandev *dev)
{
  DPRINTK(KERN_DEBUG "%s: pcan_usb_device_release(), minor = %d.\n", DEVICE_NAME, dev->nMinor);

  // do not stop usb immediately, give a chance for the PCAN-USB to send this telegram
  schedule();
  mdelay(100);

  pcan_usb_stop(dev);
}

static int pcan_usb_device_write(struct pcandev *dev)
{
  return pcan_usb_write(dev);
}

#ifdef NETDEV_SUPPORT
static int pcan_usb_device_write_frame(struct pcandev *dev, struct can_frame *cf)
{
  return pcan_usb_write_frame(dev, cf);
}
#endif

#ifndef LINUX_26
//****************************************************************************
// special assignment of minors due to PuP
//
static int assignMinorNumber(struct pcandev *dev)
{
  int searchedMinor;
  u8 occupied;
  struct pcandev   *devWork = (struct pcandev *)NULL;
  struct list_head *ptr;

  DPRINTK(KERN_DEBUG "%s: assignMinorNumber()\n", DEVICE_NAME);

  for (searchedMinor = PCAN_USB_MINOR_BASE; searchedMinor < (PCAN_USB_MINOR_BASE + 8); searchedMinor++)
  {
    occupied = 0;

    // loop trough my devices
    for (ptr = pcan_drv.devices.next; ptr != &pcan_drv.devices; ptr = ptr->next)
    {
      devWork = (struct pcandev *)ptr;  

      // stop if it is occupied
      if (devWork->nMinor == searchedMinor)
      {
        occupied = 1;
        break;
      }
    }
    
    // jump out when the first available number is found
    if (!occupied)
      break;
  }

  if (!occupied)
  {
    dev->nMinor = searchedMinor;
    return 0;
  }
  else
  {
    dev->nMinor = -1;
    return -ENXIO;
  }
}
#endif

//****************************************************************************
// propagate getting of serial number to my 'small' interface
//
int  pcan_usb_getSerialNumber(struct pcandev *dev)
{
  
  if (dev && dev->ucPhysicallyInstalled)
    return pcan_hw_getSNR(dev, &dev->port.usb.dwSerialNumber);
  else
    return -ENODEV;
}

//****************************************************************************
// things todo after plugin or plugout of device (and power on too)
//
#ifdef LINUX_26
static int pcan_usb_plugin(struct usb_interface *interface, const struct usb_device_id *id)
{
  struct pcandev *dev = NULL;
  int    err = 0;
  int    i;
  USB_PORT *u;
  struct usb_host_interface *iface_desc;
  struct usb_endpoint_descriptor *endpoint;
  struct usb_device *usb_dev = interface_to_usbdev(interface);

  DPRINTK(KERN_DEBUG "%s: pcan_usb_plugin(0x%04x, 0x%04x)\n", DEVICE_NAME, 
          usb_dev->descriptor.idVendor, usb_dev->descriptor.idProduct);

  if ((usb_dev->descriptor.idVendor  == PCAN_USB_VENDOR_ID)  && 
      (usb_dev->descriptor.idProduct == PCAN_USB_PRODUCT_ID))
  {
    // take the 1st configuration (it's default)
    if ((err = usb_reset_configuration (usb_dev)) < 0) 
    {
      printk(KERN_ERR "%s: usb_set_configuration() failed!\n", DEVICE_NAME);
      goto reject;
    }
    
    // only 1 interface is supported
    if ((err = usb_set_interface (usb_dev, 0, 0)) < 0) 
    {
      printk(KERN_ERR "%s: usb_set_interface() failed!\n", DEVICE_NAME);
      goto reject;
    }
    
    // allocate memory for my device
    if ((dev = (struct pcandev *)kmalloc(sizeof(struct pcandev), GFP_ATOMIC)) == NULL)
    {
      printk(KERN_ERR "%s: pcan_usb_plugin - memory allocation failed!\n", DEVICE_NAME);
      err = -ENOMEM;
      goto reject;
    }
    memset(dev, 0x00, sizeof(*dev));

    dev->wInitStep = 0;

    u   = &dev->port.usb;

    // init structure elements to defaults
    pcan_soft_init(dev, "usb", HW_USB);

    // preset finish flags
    atomic_set(&u->param_xmit_finished, 0);

    // preset active URB counter
    atomic_set(&u->active_urbs, 0);
  
    // override standard device access functions
    dev->device_open      = pcan_usb_device_open;
    dev->device_release   = pcan_usb_device_release;
    dev->device_write     = pcan_usb_device_write;
    #ifdef NETDEV_SUPPORT
    dev->netdevice_write  = pcan_usb_device_write_frame;
    #endif

    // init process wait queues
    init_waitqueue_head(&dev->read_queue);
    init_waitqueue_head(&dev->write_queue);

    // set this before any instructions, fill struct pcandev, part 1  
    dev->wInitStep   = 0;           
    dev->readreg     = NULL;
    dev->writereg    = NULL;
    dev->cleanup     = pcan_usb_cleanup;
    dev->req_irq     = pcan_usb_req_irq;
    dev->free_irq    = pcan_usb_free_irq;
    dev->open        = pcan_usb_open;
    dev->release     = pcan_usb_release;
    dev->filter      = pcan_create_filter_chain();

    // store pointer to kernel supplied usb_dev
    u->usb_dev          = usb_dev;
    u->ucHardcodedDevNr = (u8)(usb_dev->descriptor.bcdDevice & 0xff);
    u->ucRevision       = (u8)(usb_dev->descriptor.bcdDevice >> 8);
    u->pUSBtime         = NULL;

    printk(KERN_INFO "%s: usb hardware revision = %d\n", DEVICE_NAME, u->ucRevision);

    // get endpoint addresses (numbers) and associated max data length (only from setting 0)
    iface_desc = &interface->altsetting[0];
    for (i = 0; i < iface_desc->desc.bNumEndpoints; i++)
    {
      endpoint = &iface_desc->endpoint[i].desc;
      
      u->Endpoint[i].ucNumber = endpoint->bEndpointAddress;
      u->Endpoint[i].wDataSz  = endpoint->wMaxPacketSize;
    }

    init_waitqueue_head(&u->usb_wait_queue);

    dev->wInitStep = 1;

    // add into list of devices
    list_add_tail(&dev->list, &pcan_drv.devices);  // add this device to the list        
    dev->wInitStep = 2;
    
    // assign the device as plugged in
    dev->ucPhysicallyInstalled = 1;

    pcan_drv.wDeviceCount++;
    usb_devices++; 
    dev->wInitStep = 3;
    
    if ((err = pcan_usb_allocate_resources(dev)))
      goto reject;

    dev->wInitStep = 4;

    usb_set_intfdata(interface, dev);
    if ((err = usb_register_dev(interface, &pcan_class)) < 0)
    {
      usb_set_intfdata(interface, NULL);
      goto reject;
    }
    
    dev->nMinor = interface->minor;
    
    // get serial number early
    pcan_usb_getSerialNumber(dev);
    
    #ifdef NETDEV_SUPPORT
    pcan_netdev_register(dev);
    #endif
    
    printk(KERN_INFO "%s: usb device minor %d found\n", DEVICE_NAME, dev->nMinor);

    return 0;

    reject:
    pcan_usb_cleanup(dev);
    
    printk(KERN_ERR "%s: pcan_usb_plugin() failed! (%d)\n", DEVICE_NAME, err);
  }
  
  return err;
}
#else
#ifdef LINUX_24
static void *pcan_usb_plugin(struct usb_device *usb_dev, unsigned int interface, const struct usb_device_id *id_table)
#else
static void *pcan_usb_plugin(struct usb_device *usb_dev, unsigned int interface)
#endif
{
  struct pcandev *dev = NULL;
  int    err = 0;
  int    i;
  USB_PORT *u;
  struct usb_interface_descriptor *current_interface_setting;

  DPRINTK(KERN_DEBUG "%s: pcan_usb_plugin(0x%04x, 0x%04x, %d)\n", DEVICE_NAME,
          usb_dev->descriptor.idVendor, usb_dev->descriptor.idProduct, interface);

  if ((usb_dev->descriptor.idVendor  == PCAN_USB_VENDOR_ID)  &&
      (usb_dev->descriptor.idProduct == PCAN_USB_PRODUCT_ID))
  {
    // take the 1st configuration (it's default)
    if (usb_set_configuration (usb_dev, usb_dev->config[0].bConfigurationValue) < 0)
    {
      printk(KERN_ERR "%s: usb_set_configuration() failed!\n", DEVICE_NAME);
      goto reject;
    }

    // only 1 interface is supported
    if (usb_set_interface (usb_dev, 0, 0) < 0)
    {
      printk(KERN_ERR "%s: usb_set_interface() failed!\n", DEVICE_NAME);
      goto reject;
    }
    
    // allocate memory for my device
    if ((dev = (struct pcandev *)kmalloc(sizeof(struct pcandev), GFP_ATOMIC)) == NULL)
    {
      printk(KERN_ERR "%s: pcan_usb_plugin - memory allocation failed!\n", DEVICE_NAME);
      goto reject;
    }

    dev->wInitStep = 0;

    u   = &dev->port.usb;

    // init structure elements to defaults
    pcan_soft_init(dev, "usb", HW_USB);

    // preset finish flags
    atomic_set(&u->param_xmit_finished, 0);

    // preset active URB counter
    atomic_set(&u->active_urbs, 0);

    // override standard device access functions
    dev->device_open      = pcan_usb_device_open;
    dev->device_release   = pcan_usb_device_release;
    dev->device_write     = pcan_usb_device_write;
    #ifdef NETDEV_SUPPORT
    dev->netdevice_write  = pcan_usb_device_write_frame;
    #endif

    // init process wait queues
    init_waitqueue_head(&dev->read_queue);
    init_waitqueue_head(&dev->write_queue);

    // set this before any instructions, fill struct pcandev, part 1
    dev->wInitStep   = 0;           
    dev->readreg     = NULL;
    dev->writereg    = NULL;
    dev->cleanup     = pcan_usb_cleanup;
    dev->req_irq     = pcan_usb_req_irq;
    dev->free_irq    = pcan_usb_free_irq;
    dev->open        = pcan_usb_open;
    dev->release     = pcan_usb_release;

    if ((err = assignMinorNumber(dev)))
      goto reject;

    // store pointer to kernel supplied usb_dev
    u->usb_dev          = usb_dev;
    u->ucHardcodedDevNr = (u8)(usb_dev->descriptor.bcdDevice & 0xff);
    u->ucRevision       = (u8)(usb_dev->descriptor.bcdDevice >> 8);
    u->pUSBtime         = NULL;

    printk(KERN_INFO "%s: usb hardware revision = %d\n", DEVICE_NAME, u->ucRevision);

    // get endpoint addresses (numbers) and associated max data length
    current_interface_setting = &usb_dev->actconfig->interface->altsetting[usb_dev->actconfig->interface->act_altsetting];
    for (i = 0; i < 4; i++)
    {
      u->Endpoint[i].ucNumber = current_interface_setting->endpoint[i].bEndpointAddress;
      u->Endpoint[i].wDataSz  = current_interface_setting->endpoint[i].wMaxPacketSize;
    }

    init_waitqueue_head(&u->usb_wait_queue);

    dev->wInitStep = 1;

    // add into list of devices
    list_add_tail(&dev->list, &pcan_drv.devices);  // add this device to the list
    dev->wInitStep = 2;

    // assign the device as plugged in
    dev->ucPhysicallyInstalled = 1;

    pcan_drv.wDeviceCount++;
    usb_devices++; 
    dev->wInitStep = 3;

    if ((err = pcan_usb_allocate_resources(dev)))
      goto reject;

    dev->wInitStep = 4;

    printk(KERN_INFO "%s: usb device minor %d found\n", DEVICE_NAME, dev->nMinor);

    #ifdef NETDEV_SUPPORT
    pcan_netdev_register(dev);
    #endif

    return (void *)dev;

    reject:
    pcan_usb_cleanup(dev);
    
    printk(KERN_ERR "%s: pcan_usb_plugin() failed! (%d)\n", DEVICE_NAME, err);
  }

  return NULL;
}
#endif

#ifdef NETDEV_SUPPORT
static void pcan_usb_plugout_netdev(struct pcandev *dev)
{
  struct net_device *ndev = dev->netdev;

  if (ndev) {
    netif_stop_queue(ndev);
    pcan_netdev_unregister(dev);
  }
}
#endif

//****************************************************************************
// is called at plug out of device
//
#ifdef LINUX_26
static void pcan_usb_plugout(struct usb_interface *interface)
{
  struct pcandev *dev = usb_get_intfdata(interface);

  if (dev)
  {
    DPRINTK(KERN_DEBUG "%s: pcan_usb_plugout(%d)\n", DEVICE_NAME, dev->nMinor);

    #ifdef NETDEV_SUPPORT
    pcan_usb_plugout_netdev(dev);
    #endif

    usb_set_intfdata(interface, NULL);
    
    usb_deregister_dev(interface, &pcan_class);
    
    // mark this device as plugged out
    dev->ucPhysicallyInstalled = 0;

    // do not remove resources if the device is still in use
    if (!dev->nOpenPaths)
      pcan_usb_cleanup(dev);
  }
}
#else
static void pcan_usb_plugout(struct usb_device *usb_dev, void *drv_context)
{
  struct pcandev *dev = (struct pcandev *)drv_context;

  if (dev)
  {
    DPRINTK(KERN_DEBUG "%s: pcan_usb_plugout(%d)\n", DEVICE_NAME, dev->nMinor);

    #ifdef NETDEV_SUPPORT
    pcan_usb_plugout_netdev(dev);
    #endif

    // mark this device as plugged out
    dev->ucPhysicallyInstalled = 0;

    // do not remove resources if the device is still in use
    if (!dev->nOpenPaths)
      pcan_usb_cleanup(dev);
  }
}
#endif

//****************************************************************************
// small interface to rest of driver, only init and deinit
//
static int pcan_usb_init(void)
{
  DPRINTK(KERN_DEBUG "%s: pcan_usb_init()\n", DEVICE_NAME);

  memset (&pcan_drv.usbdrv, 0, sizeof(pcan_drv.usbdrv));
  
  #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,24) && LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
  pcan_drv.usbdrv.owner       = THIS_MODULE;
  #endif
  
  pcan_drv.usbdrv.probe       = pcan_usb_plugin;
  pcan_drv.usbdrv.disconnect  = pcan_usb_plugout;
  pcan_drv.usbdrv.name        = DEVICE_NAME;
  pcan_drv.usbdrv.id_table    = pcan_usb_ids;

  return usb_register(&pcan_drv.usbdrv);
}

void pcan_usb_deinit(void)
{
  DPRINTK(KERN_DEBUG "%s: pcan_usb_deinit()\n", DEVICE_NAME);

  if (pcan_drv.usbdrv.probe == pcan_usb_plugin)
  {
    // then it was registered
    // unregister usb parts, makes a plugout of registered devices
    usb_deregister(&pcan_drv.usbdrv);
  }
}

//----------------------------------------------------------------------------
// init for usb based devices from peak
int pcan_usb_register_devices(void)
{
  int err;

  DPRINTK(KERN_DEBUG "%s: pcan_usb_register_devices()\n", DEVICE_NAME);

  if (!(err = pcan_usb_init()))
  {
    DPRINTK(KERN_DEBUG "%s: pcan_usb_register_devices() is OK\n", DEVICE_NAME);
  }

  return err;
}

 


