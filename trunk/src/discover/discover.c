/** Implementation of bt_discover_client, a UDP discovery client module.
 *
 * \file discover.c
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

#include <stdlib.h>
#include <string.h>

#include <sys/types.h> /* For select() */
#include <sys/select.h> /* for fd_set */
#include <unistd.h> /* For close() */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <syslog.h>

#include "discover.h"

/* Note - see SVN/internal/tools/wamdiscover/client for example
 *        to make this Windows-compatible */


struct bt_discover_client * bt_discover_client_create(void)
{
   struct bt_discover_client * c;
   int optval;
   int err;
   
   /* Create */
   c = (struct bt_discover_client *) malloc(sizeof(struct bt_discover_client));
   if (!c)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Initialize */
   c->list = 0;
   c->num = 0;
   c->sock = socket(PF_INET, SOCK_DGRAM, 0);
   if (c->sock == -1)
   {
      syslog(LOG_ERR,"%s: Could not create socket.",__func__);
      free(c);
      return 0;
   }
   
   /* Toggle broadcast flag on socket */
   optval = 1;
   err = setsockopt(c->sock, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval) );
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not toggle broadcast flag.",__func__);
      close(c->sock);
      free(c);
      return 0;
   }
   
   return c;
}


int bt_discover_client_destroy(struct bt_discover_client * c)
{
   if (c->list)
      free(c->list);
   close(c->sock);
   free(c);
   return 0;
}


int bt_discover_client_discover(struct bt_discover_client * c)
{
   struct sockaddr_in their_addr;
   int err;
   
   /* Free the list if we already have one */
   if (c->list)
   {
      free(c->list);
      c->list = 0;
      c->num = 0;
   }
   
   /* Then, send the broadcast UDP packet */
   their_addr.sin_family = AF_INET;
   their_addr.sin_port = htons(1337);
   their_addr.sin_addr.s_addr = INADDR_BROADCAST;
   err = sendto(c->sock, 0, 0, 0, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
   if (err == -1)
   {
      syslog(LOG_ERR,"%s: Could not send broadcast packet.",__func__);
      return -1;
   }
   
   /* Select() until 1 second timeout ... */
   while (1)
   {
      fd_set set;
      struct timeval timeout;
      char data[34];
      int numbytes;
      
      FD_ZERO( &set);
      FD_SET( c->sock, &set );
      timeout.tv_sec = 1;
      timeout.tv_usec = 0;
      
      select(FD_SETSIZE, &set, 0, 0, &timeout);
      
      /* If we didn't read anything, we're done*/
      if ( ! FD_ISSET( c->sock, &set ) )
         break;
      
      /* Read 33 bytes from the socket */
      numbytes = recvfrom(c->sock, data, 33, 0, 0, 0);
      if (numbytes == -1)
         return -1;
      data[numbytes] = '\0';
      
      /* Extend the list array */
      if (!c->list)
         c->list = (struct bt_discover_client_entry *) malloc( sizeof(struct bt_discover_client_entry) );
      else
         c->list = (struct bt_discover_client_entry *) realloc( c->list, (c->num+1)*sizeof(struct bt_discover_client_entry) );
      
      /* Copy in the data */
      strncpy(c->list[c->num].mac,data,17);
      c->list[c->num].mac[17] = '\0';
      strncpy(c->list[c->num].ip,data+18,15);
      c->list[c->num].ip[strlen(data)-18] = '\0';
      
      c->num++;
      
   }
   
   return 0;
   
}
