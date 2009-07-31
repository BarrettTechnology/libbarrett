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

#define _BSD_SOURCE /* For struct ifreq in net/if.h */

#include <stdlib.h>
#include <stdio.h> /* For sprintf() */
#include <string.h>

#include <sys/select.h> /* for fd_set */
#include <unistd.h> /* For close() */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h> /* For struct ifreq */
#include <sys/ioctl.h> /* For ioctl() stuff */


#include <syslog.h>

#include "discover.h"

/* Note - see SVN/internal/tools/wamdiscover/client for example
 *        to make this Windows-compatible */

/* Return Data Format is 17 + 1 + (7 to 15) = (25 to 33) bytes
 * "XX:XX:XX:XX:XX:XX|xxx.xxx.xxx.xxx" 
 */

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


struct bt_discover_server * bt_discover_server_create(int port, char * iface)
{
   int err;
   struct bt_discover_server * s;

   struct sockaddr_in bind_addr;
   struct ifreq ifr_mac;
   struct ifreq ifr_ip;

   s = (struct bt_discover_server *) malloc(sizeof(struct bt_discover_server));
   if (!s)
      return 0;

   s->sock = -1;

   /* Create the UDP socket */
   s->sock = socket(PF_INET, SOCK_DGRAM, 0);
   if (s->sock == -1)
   {
      syslog(LOG_ERR,"%s: Couldn't create UDP socket.",__func__);
      bt_discover_server_destroy(s);
      return 0;
   }

   /* Bind to address */
   bind_addr.sin_family = AF_INET;
   bind_addr.sin_port = htons(port);
   bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
   err = bind(s->sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
   if (err == -1)
   {
      syslog(LOG_ERR,"%s: Couldn't bind UDP to the port.",__func__);
      bt_discover_server_destroy(s);
      return 0;
   }

   /* Get some information about the interface */
   strcpy(ifr_mac.ifr_name,iface);
   strcpy(ifr_ip.ifr_name,iface);
   err = ioctl(s->sock,SIOCGIFHWADDR,&ifr_mac)
      || ioctl(s->sock,SIOCGIFADDR,&ifr_ip);
   if (err)
   {
      syslog(LOG_ERR,"%s: Couldn't get the interface's MAC/IP address.",__func__);
      bt_discover_server_destroy(s);
      return 0;
   }   
   sprintf(s->data,"%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X|%d.%d.%d.%d",
      (unsigned char)ifr_mac.ifr_hwaddr.sa_data[0],
      (unsigned char)ifr_mac.ifr_hwaddr.sa_data[1],
      (unsigned char)ifr_mac.ifr_hwaddr.sa_data[2],
      (unsigned char)ifr_mac.ifr_hwaddr.sa_data[3],
      (unsigned char)ifr_mac.ifr_hwaddr.sa_data[4],
      (unsigned char)ifr_mac.ifr_hwaddr.sa_data[5],
      (unsigned char)ifr_ip.ifr_ifru.ifru_addr.sa_data[2],
      (unsigned char)ifr_ip.ifr_ifru.ifru_addr.sa_data[3],
      (unsigned char)ifr_ip.ifr_ifru.ifru_addr.sa_data[4],
      (unsigned char)ifr_ip.ifr_ifru.ifru_addr.sa_data[5]);
   syslog(LOG_ERR,"MAC|IP identified: |%s|\n",s->data);

   return s;
}

int bt_discover_server_destroy(struct bt_discover_server * s)
{
   if (s->sock != -1)
      close(s->sock);
   free(s);
   return 0;
}

int bt_discover_server_select_pre(struct bt_discover_server * s, fd_set * read_set)
{
   FD_SET(s->sock,read_set);
   return 0;
}

int bt_discover_server_select_post(struct bt_discover_server * s, fd_set * read_set)
{
   struct sockaddr_in their_addr;
   size_t addr_len;
   int numbytes;
   
   if (! FD_ISSET(s->sock,read_set) )
      return 0;

   /* Await an incoming packet */
   addr_len = sizeof(their_addr);
   numbytes = recvfrom(s->sock, 0, 0, 0,
                       (struct sockaddr *)&their_addr, &addr_len);
   if (numbytes == -1)
   {
      syslog(LOG_ERR,"%s: Error receiving packet.",__func__);
      return -1;
   }

   /* Send the data ... */
   numbytes = sendto(s->sock, s->data, strlen(s->data), 0,
                     (struct sockaddr *)&their_addr, sizeof(their_addr));
   if (numbytes == -1)
   {
      syslog(LOG_ERR,"%s: Error sending packet.",__func__);
      return -1;
   }
   
   syslog(LOG_ERR,"Successfully responded to a UDP Broadcast.");
   return 0;
}









