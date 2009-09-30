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
// pcan_netdev.h - CAN network device support defines / prototypes
//
// $Id: pcan_netdev.h 524 2007-10-16 13:11:15Z ohartkopp $
//
// For CAN netdevice / socketcan specific questions please check the
// Mailing List <socketcan-users@lists.berlios.de>
// Project homepage http://developer.berlios.de/projects/socketcan
//
//****************************************************************************

#ifndef PCAN_NETDEV_H
#define PCAN_NETDEV_H

#include <linux/netdevice.h>

#define CAN_NETDEV_NAME "can%d"
#define TX_TIMEOUT      (50*HZ/1000) /* 50ms */ 

#ifndef LINUX_26
#define netdev_priv(dev) ((dev)->priv)
#endif

/* private data structure for netdevice */

struct can_priv
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
  struct net_device_stats stats; /* standard netdev statistics */
#endif
  struct pcandev          *pdev; /* back reference to PCAN device */
};

int pcan_netdev_register(struct pcandev *dev);
int pcan_netdev_unregister(struct pcandev *dev);
int pcan_netdev_rx(struct pcandev *dev, struct can_frame *cf, struct timeval *tv);

#endif /* PCAN_NETDEV_H */
