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
// Contributions: Oliver Hartkopp (oliver.hartkopp@volkswagen.de)
//****************************************************************************

//****************************************************************************
//
// pcan_netdev.c - CAN network device support functions
//
// $Id: pcan_netdev.c 525 2007-10-17 13:06:25Z ohartkopp $
//
// For CAN netdevice / socketcan specific questions please check the
// Mailing List <socketcan-users@lists.berlios.de>
// Project homepage http://developer.berlios.de/projects/socketcan
//
//****************************************************************************

#include <src/pcan_common.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <src/pcan_main.h>
#include <src/pcan_fops.h>  /* pcan_open_path(), pcan_release_path() */
#include <src/pcan_netdev.h>

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
// Functions for AF_CAN netdevice adaptation
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// AF_CAN netdevice: open device
//----------------------------------------------------------------------------
static int pcan_netdev_open(struct net_device *dev)
{
  struct can_priv *priv = netdev_priv(dev);
  struct pcandev  *pdev = priv->pdev;

  DPRINTK(KERN_DEBUG "%s: %s %s\n", DEVICE_NAME, __FUNCTION__, dev->name);

  if (pcan_open_path(pdev))
    return -ENODEV;

  netif_start_queue(dev);

  return 0;
}


//----------------------------------------------------------------------------
// AF_CAN netdevice: close device
//----------------------------------------------------------------------------
static int pcan_netdev_close(struct net_device *dev)
{
  struct can_priv *priv = netdev_priv(dev);
  struct pcandev  *pdev = priv->pdev;

  DPRINTK(KERN_DEBUG "%s: %s %s\n", DEVICE_NAME, __FUNCTION__, dev->name);

  pcan_release_path(pdev);

  netif_stop_queue(dev);

  return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
//----------------------------------------------------------------------------
// AF_CAN netdevice: get statistics for device
//----------------------------------------------------------------------------
static struct net_device_stats *pcan_netdev_get_stats(struct net_device *dev)
{
  struct can_priv *priv = netdev_priv(dev);

  /* TODO: read statistics from chip */
  return &priv->stats;
}
#endif


//----------------------------------------------------------------------------
// AF_CAN netdevice: timeout handler for device
//----------------------------------------------------------------------------
static void pcan_netdev_tx_timeout(struct net_device *dev)
{
  struct net_device_stats *stats = dev->get_stats(dev);

  stats->tx_errors++;
  netif_wake_queue(dev);
}


//----------------------------------------------------------------------------
// AF_CAN netdevice: transmit handler for device
//----------------------------------------------------------------------------
static int pcan_netdev_tx(struct sk_buff *skb, struct net_device *dev)
{
  struct can_priv *priv = netdev_priv(dev);
  struct pcandev  *pdev = priv->pdev;
  struct net_device_stats *stats = dev->get_stats(dev);
  struct can_frame *cf = (struct can_frame*)skb->data;

  DPRINTK(KERN_DEBUG "%s: %s %s\n", DEVICE_NAME, __FUNCTION__, dev->name);

  netif_stop_queue(dev);
  atomic_set(&pdev->DataSendReady, 0); /* for pcan chardevice */

  pdev->netdevice_write(pdev, cf); /* ignore return value */

  stats->tx_packets++;
  stats->tx_bytes += cf->can_dlc;

  dev->trans_start = jiffies;

  dev_kfree_skb(skb);

  return 0;
}


//----------------------------------------------------------------------------
// AF_CAN netdevice: receive function (put can_frame to netdev queue)
//----------------------------------------------------------------------------
int pcan_netdev_rx(struct pcandev *dev, struct can_frame *cf, struct timeval *tv)
{
  struct net_device *ndev = dev->netdev;
  struct net_device_stats *stats = ndev->get_stats(ndev);
  struct sk_buff *skb;

  DPRINTK(KERN_DEBUG "%s: %s %s\n", DEVICE_NAME, __FUNCTION__, ndev->name);
        
  skb = dev_alloc_skb(sizeof(struct can_frame));

  if (skb == NULL)
    return -ENOMEM;

  skb->dev      = ndev;
  skb->protocol = htons(ETH_P_CAN);
  skb->pkt_type = PACKET_BROADCAST;
  skb->ip_summed = CHECKSUM_UNNECESSARY;

  #if 0
  /*
   * Currently the driver only supports timestamp setting at host arrival time.
   * Therefore the netdev support can used the timestamp provided by netif_rx()
   * which is set when there is no given timestamp (and when network timestamps
   * are not disabled by default in the host). So we just use the mechanics
   * like any other network device does ...
   */
  if (tv)
    #ifdef LINUX_26
    skb_set_timestamp(skb, tv);
    #else
    skb->stamp = *tv;
    #endif
  #endif

  memcpy(skb_put(skb, sizeof(struct can_frame)), cf, sizeof(struct can_frame));
  
  netif_rx(skb);

  ndev->last_rx = jiffies;
  stats->rx_packets++;
  stats->rx_bytes += cf->can_dlc;

  return 0;
}

//----------------------------------------------------------------------------
// AF_CAN netdevice: initialize data structure
//----------------------------------------------------------------------------
void pcan_netdev_init(struct net_device *dev)
{
  /* Fill in the the fields of the device structure with AF_CAN generic values */

  if (!dev)
    return;

  dev->change_mtu           = NULL;
  dev->set_mac_address      = NULL;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
  dev->hard_header          = NULL;
  dev->rebuild_header       = NULL;
  dev->hard_header_cache    = NULL;
  dev->header_cache_update  = NULL;
  dev->hard_header_parse    = NULL;
#else
  dev->header_ops           = NULL;
#endif

  dev->type             = ARPHRD_CAN;
  dev->hard_header_len  = 0;
  dev->mtu              = sizeof(struct can_frame);
  dev->addr_len         = 0;
  dev->tx_queue_len     = 10;

  dev->flags            = IFF_NOARP;

  dev->open             = pcan_netdev_open;
  dev->stop             = pcan_netdev_close;
  dev->hard_start_xmit  = pcan_netdev_tx;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
  dev->get_stats        = pcan_netdev_get_stats;
#endif

  dev->tx_timeout       = pcan_netdev_tx_timeout;
  dev->watchdog_timeo   = TX_TIMEOUT;
}

//----------------------------------------------------------------------------
// AF_CAN netdevice: try to reassign netdev name according to user needs
//----------------------------------------------------------------------------
void pcan_netdev_create_name(char *name, struct pcandev *pdev)
{
  extern char *assign; /* module param: assignment for netdevice names */

  if (!assign) /* auto assignment */
    return;

  if (!strncmp(assign, "peak", 4)) {

    // assign=peak
    snprintf(name, IFNAMSIZ-1, "can%d", pdev->nMinor); /* easy: /dev/pcanXX -> canXX */

  } else {

    // e.g. assign=pcan32:can1,pcan41:can2

    int peaknum, netnum;
    char *ptr = assign;

    while (ptr < (assign + strlen(assign))) {
      ptr = strchr(ptr, 'p'); /* search first 'p' from pcanXX */
      if (!ptr)
        return; /* no match => quit */

      if (sscanf(ptr, "pcan%d:can%d", &peaknum, &netnum) != 2) {
        printk(KERN_INFO "%s: bad parameter format in netdevice assignment.\n", DEVICE_NAME);
        return; /* bad parameter format => quit */
      }

      if (peaknum == pdev->nMinor) {
        snprintf(name, IFNAMSIZ-1, "can%d", netnum);
        break; /* done */
      }
      ptr++; /* search for next 'p' */
    }
  }

  if (name[0]) {
    /* check wanted assigned 'name' against existing device names */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
    if (__dev_get_by_name(name)) {
#else
    if (__dev_get_by_name(&init_net, name)) {
#endif
      printk(KERN_INFO "%s: netdevice name %s to be assigned exists already.\n",
             DEVICE_NAME, name);
      name[0] = 0; /* mark for auto assignment */
    }
  }
}

//----------------------------------------------------------------------------
// AF_CAN netdevice: register network device
//----------------------------------------------------------------------------
int pcan_netdev_register(struct pcandev *pdev)
{
  struct net_device *ndev;
  struct can_priv *priv;
  char name[IFNAMSIZ] = {0};

  pcan_netdev_create_name(name, pdev);

  if (!name[0])
    strncpy(name, CAN_NETDEV_NAME, IFNAMSIZ-1); /* use the default: autoassignment */

#ifdef LINUX_26

  ndev = alloc_netdev(sizeof(struct can_priv), name, pcan_netdev_init);

  if (!ndev) {
    printk(KERN_ERR "%s: out of memory\n", DEVICE_NAME);
    return 1;
  }
  priv = netdev_priv(ndev);

  if (register_netdev(ndev)) {
    printk(KERN_INFO "%s: Failed registering netdevice\n", DEVICE_NAME);
    free_netdev(ndev);
    return 1;
  }

#else

  ndev = (struct net_device*)kmalloc(sizeof(struct net_device), GFP_KERNEL);

  if (!ndev) {
    printk(KERN_ERR "%s: out of memory\n", DEVICE_NAME);
    return 1;
  }

  memset(ndev, 0, sizeof(struct net_device));

  priv = (struct can_priv*)kmalloc(sizeof(struct can_priv), GFP_KERNEL);

  if (!priv) {
    printk(KERN_ERR "%s: out of memory\n", DEVICE_NAME);
    kfree(ndev);
    return 1;
  }

  memset(priv, 0, sizeof(struct can_priv));
  ndev->priv = priv;

  /* fill net_device structure */
  pcan_netdev_init(ndev);

  strncpy(ndev->name, name, IFNAMSIZ-1); /* name the device */
  SET_MODULE_OWNER(ndev);

  if (register_netdev(ndev)) {
    printk(KERN_INFO "%s: Failed registering netdevice\n", DEVICE_NAME);
    kfree(priv);
    kfree(ndev);
    return 1;
  }

#endif

  // Make references between pcan device and netdevice
  priv->pdev   = pdev;
  pdev->netdev = ndev;

  printk(KERN_INFO "%s: registered netdevice %s for pcan %s hw (minor %d)\n",
         DEVICE_NAME, ndev->name, pdev->type, pdev->nMinor);

  return 0;
}

//----------------------------------------------------------------------------
// AF_CAN netdevice: unregister network device
//----------------------------------------------------------------------------
int pcan_netdev_unregister(struct pcandev *pdev)
{
  struct net_device *ndev = pdev->netdev;
  struct can_priv *priv;

  if (!ndev)
    return 1;

  DPRINTK(KERN_DEBUG "%s: %s %s\n", DEVICE_NAME, __FUNCTION__, ndev->name);

  priv = netdev_priv(ndev);

  unregister_netdev(ndev);

#ifndef LINUX_26
  if (priv)
    kfree(priv);
#endif
  
  pdev->netdev = NULL; /* mark as unregistered */

  return 0;
}

//----------------------------------------------------------------------------
// End of functions for AF_CAN netdevice adaptation
//----------------------------------------------------------------------------
 
