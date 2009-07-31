/** Definition of bt_discover_client, a UDP discovery client module.
 *
 * \file discover.h
 * \author Christopher Dellin
 * \date 2008-2009
 */

/* Copyright 2008, 2009
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

/** \file discover.h
 *
 * \section sec_intro Introduction
 *
 * A bt_discover_client is a UDP discovery client that broadcasts a UDP
 * packet across the network, and listens for UDP responses from bt_discover
 * servers.  Currently, the server is not implemented as part of libbarrett,
 * but this is scheduled to change.
 */

#ifndef BT_DISCOVER_H
#define BT_DISCOVER_H
#ifdef __cplusplus
extern "C" {
#endif


#include <sys/select.h> /* for fd_set */


/** This is a client entry, including the MAC address and the IP address in
 *  string format.
 */
struct bt_discover_client_entry {
   char mac[18];
   char ip[16];
};


/** This is a client object which can retrieve a list of entries from the
 *  network.
 */
struct bt_discover_client
{
   int sock;
   
   struct bt_discover_client_entry * list;
   int num;
};


/** Create a bt_discover_client object.
 *
 * \return The bt_discover_client object on success, or 0 on failure
 */
struct bt_discover_client * bt_discover_client_create(void);


/** Destroy a bt_discover_client object.
 *
 * \param[in] client The bt_discover_client object to destroy
 * \retval 0 Success
 */
int bt_discover_client_destroy(struct bt_discover_client * client);


/** Send a UDP broadcast packet to discover running servers, and wait for
 *  replies.
 *
 * \note For now, this blocks for about 1 second
 *
 * \param[in] client The bt_discover_client to use
 * \retval 0 Success
 * \retval -1 Could not send broadcast packet
 */
int bt_discover_client_discover(struct bt_discover_client * client);


struct bt_discover_server
{
   int sock;
   char data[34];
};

struct bt_discover_server * bt_discover_server_create(int port, char * iface);
int bt_discover_server_destroy(struct bt_discover_server * server);
int bt_discover_server_select_pre(struct bt_discover_server * s, fd_set * read_set);
int bt_discover_server_select_post(struct bt_discover_server * s, fd_set * read_set);


#ifdef __cplusplus
}
#endif
#endif /* BT_DISCOVER_H */
