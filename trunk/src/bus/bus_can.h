/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... can.h
 *  Author ............. Brian Zenowich
 *                       Traveler Hauptman
 *                       Sam Clanton
 *                       Christopher Dellin
 *  Creation Date ...... 24 Mar 2003
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2003-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2003 Mar 24 - BZ
 *      File created & documented.
 *    2004 Dec 16 - BZ, SC, TH
 *      Initial port to linux + RTAI
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt
 *                                                                          *
 * ======================================================================== */

/** \file can.h
Handles all communication with the robot over the CAN bus.
    Requires library files "libcan.a" and "libmitop.a".
*/

#define BT_BUS_CAN_MBXID (0)
#define BT_BUS_CAN_BASEID (0)
#define BT_BUS_CAN_ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BT_BUS_CAN_BASEID)

#define BT_BUS_CAN_NODE2ADDR(x) \
   (((BT_BUS_CAN_MBXID + BT_BUS_CAN_BASEID) << 5) \
   | ((x) + BT_BUS_CAN_BASEID))

#define BT_BUS_CAN_GROUPID(n) \
   (((BT_BUS_CAN_MBXID + BT_BUS_CAN_BASEID) << 5) | (0x0400 + (n)))

struct bt_bus_can_device;

/* Public Functions */

struct bt_bus_can_device * bt_bus_can_create(int port);
void bt_bus_can_destroy(struct bt_bus_can_device * dev);
int bt_bus_can_clearmsg(struct bt_bus_can_device * dev);

void bt_bus_can_iterate_start(struct bt_bus_can_device * dev);
int bt_bus_can_iterate_next(struct bt_bus_can_device * dev,
                      int * nextid, int * nextstatus);

/* Do we need this? */
void bt_bus_can_set_max_property(int prop);
int bt_bus_can_get_property(struct bt_bus_can_device * dev, int who, int property, long *reply);
int bt_bus_can_set_property(struct bt_bus_can_device * dev, int who, int property, int verify, long value);

int bt_bus_can_get_packed(struct bt_bus_can_device * dev, int group, int howMany, long *pos, int pos_prop);
int bt_bus_can_set_torques(struct bt_bus_can_device * dev, int group, int *values, int torque_prop);
