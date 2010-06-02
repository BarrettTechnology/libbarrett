#include <stdlib.h>
#include <string.h>

#ifdef WINDOWS
#  include <winsock2.h>
#else
#  include <unistd.h> /* For close() */
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <stdio.h>
#endif

#include "common.h"


/*
 * List stuff
 */
#define DISPLAY_SEPARATOR " @ "
#define DISPLAY_SEPARATOR_LEN 3
#define BUF_SIZE (34 + DISPLAY_SEPARATOR_LEN)   /* Same as in wamudpd.c, adjusted for display separator length */
struct wam_list_entry
{
   char display_str[BUF_SIZE];
   char * ip_str;
};

struct wam_list_entry * wam_list = 0;
int wam_list_num = 0;

int list_add_entry(char * data) {
   int strLen;
   
   /* Expand the array */
   if (!wam_list_num)
      wam_list = (struct wam_list_entry *) malloc( sizeof(struct wam_list_entry) );
   else
      wam_list = (struct wam_list_entry *) realloc( wam_list, (wam_list_num+1)*sizeof(struct wam_list_entry) );
   
   /* Init the new entry */
   strncpy(wam_list[wam_list_num].display_str, data, BUF_SIZE);
   strcpy(strrchr(wam_list[wam_list_num].display_str, '|'), DISPLAY_SEPARATOR);   /* Find the |, copy the new separator there */
   
   strLen = strlen(wam_list[wam_list_num].display_str);
   wam_list[wam_list_num].ip_str = wam_list[wam_list_num].display_str + strLen;
   strncpy(wam_list[wam_list_num].ip_str, data + strLen - DISPLAY_SEPARATOR_LEN + 1, BUF_SIZE - strLen);
   
   return wam_list_num++;
}

int list_get_size()
{
   return wam_list_num;
}

char * list_get_display_str(int idx) {
   return wam_list[idx].display_str;
}

char * list_get_ipstr(int idx)
{
   return wam_list[idx].ip_str;
}




/*
 * Socket stuff
 */

#ifdef WINDOWS
SOCKET sock;
#else
int sock;
#endif

/* Initialize the socket */
int init_socket()
{
   int optval;
   int err;
   
   /* Initialize Winsock */
#ifdef WINDOWS
   {
      WSADATA wsaData;
      WSAStartup(MAKEWORD(2,2), &wsaData);
   }
#endif
   
   /* Create the socket */
#ifdef WINDOWS
   sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
   if (sock == INVALID_SOCKET)
#else
   sock = socket(PF_INET, SOCK_DGRAM, 0);
   if (sock == -1)
#endif
      return -1;
   
   optval = 1;
#ifdef WINDOWS
   err = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&optval, sizeof(optval) );
#else
   err = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval) );
#endif
   
   if (err)
      return -2;
   
   return 0;
}

#ifdef WINDOWS
SOCKET get_socket()
#else
int get_socket()
#endif
{
   return sock;
}

int close_socket()
{
#ifdef WINDOWS
   closesocket(sock);
   WSACleanup();
#else
   close(sock);
#endif
   return 0;
}


int send_broadcast()
{
   struct sockaddr_in their_addr;
   
   if (wam_list_num)
   {
      free(wam_list);
      wam_list_num = 0;
   }
   
   /* Then, send the broadcast UDP packet */
   their_addr.sin_family = AF_INET;
   their_addr.sin_port = htons(1337);
   their_addr.sin_addr.s_addr = INADDR_BROADCAST;
   return sendto(sock, 0, 0, 0, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
}

/* Returns the index of the newly-inserted entry,
 * or negative on failure */
int receive_packet()
{
   char data[BUF_SIZE];
   int numbytes;
   
   numbytes = recvfrom(sock, data, BUF_SIZE - 1, 0, 0, 0);
   if (numbytes == -1)
      return -1;
   data[numbytes] = '\0';
   
   /* Check this entry for the right format */
   
   /* Put this entry into the WAM list */
   return list_add_entry(data);
}

