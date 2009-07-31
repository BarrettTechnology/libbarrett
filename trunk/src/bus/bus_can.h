/** Definition of bt_bus_can, an abstracted CAN device driver.
 *
 * \file bus_can.h
 * \author Brian Zenowich
 * \author Traveler Hauptman
 * \author Sam Clanton
 * \author Christopher Dellin
 * \date 2003-2009
 */

/* Copyright 2003, 2004, 2005, 2006, 2007, 2008, 2009
 *           Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */

/** \file bus_can.h
 *
 * \section sec_intro Introduction
 *
 * bt_bus_can is an abstracted CAN device driver.  It supports getting and
 * setting properties to CAN nodes, sending and receiving packed data packets
 * to multiple nodes, and iterating over nodes.
 */

#ifndef BT_BUS_CAN_H
#define BT_BUS_CAN_H
#ifdef __cplusplus
extern "C" {
#endif

#define BT_BUS_CAN_MBXID (0)
#define BT_BUS_CAN_BASEID (0)
#define BT_BUS_CAN_ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BT_BUS_CAN_BASEID)

#define BT_BUS_CAN_NODE2ADDR(x) \
   (((BT_BUS_CAN_MBXID + BT_BUS_CAN_BASEID) << 5) \
   | ((x) + BT_BUS_CAN_BASEID))

#define BT_BUS_CAN_GROUPID(n) \
   (((BT_BUS_CAN_MBXID + BT_BUS_CAN_BASEID) << 5) | (0x0400 + (n)))


/** An abstracted CAN device. */
struct bt_bus_can;


/** Create a bt_bus_can object.
 *
 * The bt_bus_can_create() function opens the CAN port given as its argument.
 * The port argument is zero-indexed.
 *
 * \param[out] devptr The bt_bus_can object on success, or 0 on failure
 * \param[in] port The CAN port to open (0, 1, etc.)
 * \retval 0 Success
 */
int bt_bus_can_create(struct bt_bus_can ** devptr, int port);


/** Destroy a bt_bus_can object.
 *
 * \param[in] dev The bt_bus_can device to destroy
 * \retval 0 Success
 */
int bt_bus_can_destroy(struct bt_bus_can * dev);


/** Clear any messages presently on the bus.
 *
 * \param[in] dev The bt_bus_can device to clear
 * \retval 0 Success
 */
int bt_bus_can_clearmsg(struct bt_bus_can * dev);


/** Start iterating over a set of CAN nodes.
 *
 * \param[in] dev The bt_bus_can device over which to iterate
 * \retval 0 Success
 */
int bt_bus_can_iterate_start(struct bt_bus_can * dev);


/** Retreive the next node's ID and status.
 *
 * \param[in] dev The bt_bus_can device over which to iterate
 * \param[out] nextid The location to place the next ID found
 * \param[out] nextstatus The location to place the next status found
 * \retval 1 Another node was found
 * \retval 0 No more nodes are present
 */
int bt_bus_can_iterate_next(struct bt_bus_can * dev, int * nextid,
                            int * nextstatus);


/** Get a property value from a node.
 *
 * This function gets a property value from a given node.
 *
 * \param[in] dev The bt_bus_can device to use
 * \param[in] id The ID of the node
 * \param[in] property The property to get; se bt_bus_properties for a list
 * \param[out] reply The location to save the value
 * \retval 0 Success
 * \retval 1 There was an error reading the packet
 * \retval 2 The returned property wasn't what was asked for
 */
int bt_bus_can_get_property(struct bt_bus_can * dev, int id, int property,
                            long * reply);


/** Set a property value on a node.
 *
 * This function sets a property to a given value on a given node.  If the
 * verify argument is true, this will also perform a get property request to
 * ensure that the value was changed.
 *
 * \param[in] dev The bt_bus_can device to use
 * \param[in] id The ID of the node
 * \param[in] property The property to set; se bt_bus_properties for a list
 * \param[in] verify Whether to verify the set's success
 * \param[in] value The property's value to set
 * \retval 0 Success
 * \retval 2 The verified value didn't match what was set
 */
int bt_bus_can_set_property(struct bt_bus_can * dev, int id, int property,
                            int verify, long value);


/** Get a set of values from a group of nodes.
 *
 * This function sends a request to a group for position (or acceleration)
 * data.
 *
 * \param[in] dev The bt_bus_can device to use
 * \param[in] group The group to ask the positions from
 * \param[in] how_many The number of nodes from which to expect responses
 * \param[out] data The array into which to save the received positions
 * \param[in] The property to request
 * \retval 0 Success
 */
int bt_bus_can_get_packed(struct bt_bus_can * dev, int group, int how_many,
                          long * data, int prop);


/** Send a packed torques message to a group of nodes.
 *
 * This function sends a packed torque request to a group of up to four
 * nodes.
 *
 * \param[in] dev The bt_bus_can device to use
 * \param[in] group The group to ask the positions from
 * \param[in] values The array of four torques to send
 * \param[in] torque_prop The property to send
 */
int bt_bus_can_set_torques(struct bt_bus_can * dev, int group, int * values,
                           int torque_prop);

#ifdef __cplusplus
}
#endif
#endif /* BT_BUS_CAN_H */
