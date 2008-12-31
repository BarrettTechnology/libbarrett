/* ======================================================================== *
 *  Module ............. jsonserve
 *  File ............... jsonserve.c
 *  Author ............. Christopher Dellin
 *  Creation Date ...... 10 Dec 2008
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  NOTES:
 *    Here's an example asynchronous server using JSON-RPC.
 *
 *  REVISION HISTORY:
 *    2008 Dec 10 - CD
 *      Created!
 *                                                                          *
 * ======================================================================== */

/** \file jsonserve.c
    Asynch Server Using JSON-RPC!
 */

/* System Libraries */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h> /* For select() */
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h> /* For inet_ntoa() */

/* Package Dependencies */
#include <syslog.h>
#include <libconfig.h>

/* Include the high-level WAM header file */
#include <libbt/wam.h>
#include <libbt/gsl.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/* Woo JSON! */
#include <json/json.h>

#define BUFLEN 1023 /* No requests are allowed to be longer than this */
#define MYPORT 100

/* List of open connections */
struct connection {
   int sock;
   char * buf;
   int buf_already;
};
struct connection * conns;
int conns_num;

/* We have a global flag and signal handler
 * to allow the user to close the program
 * with [Control+C] */
int going;
void sigint_handler()
{
   going = 0;
}

/* Program entry point */
int main(int argc, char ** argv)
{
   int err;
   struct sockaddr_in listener_addr;
   
   /* Lock memory */
   mlockall(MCL_CURRENT | MCL_FUTURE);
   
   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   
   /* Create the listener socket (conns[0]) */
   conns = (struct connection *) malloc( sizeof(struct connection) );
   conns_num = 1;
   
   /* Start listening on a port for incoming connections ... */
   conns[0].sock = socket(PF_INET, SOCK_STREAM, 0);
   printf("we're talking socket %d.\n",conns[0].sock);
   if (conns[0].sock < 0)
   {
      printf("could not create socket.\n");
      closelog();
      return -1;
   }
   
   /* Bind to a port */
   listener_addr.sin_family = AF_INET;
   listener_addr.sin_port = htons(MYPORT);
   listener_addr.sin_addr.s_addr = htonl(INADDR_ANY);
   err = bind(conns[0].sock, (struct sockaddr *)&listener_addr, sizeof(listener_addr));
   if (err)
   {
      printf("could not bind to port %d.\n",MYPORT);
      closelog();
      return -1;
   }

   /* Start listening */
   err = listen( conns[0].sock, 10 );
   if (err)
   {
      printf("could not listen on socket.\n");
      closelog();
      return -1;
   }

   /* Loop until Control+C ... */
   going = 1;
   while (going)
   {
      int s;
      fd_set read_set;
      FD_ZERO(&read_set);
      
      for (s=0; s<conns_num; s++)
         FD_SET(conns[s].sock, &read_set);

      /* Wait for activity ... */
      if (select(FD_SETSIZE, &read_set, NULL, NULL, NULL) < 0)
      {
         printf("select() error!");
         going = 0;
         break;
      }
      
      /* Accept new connections ... */
      if (FD_ISSET(conns[0].sock,&read_set))
      {
         int new;
         struct sockaddr_in new_addr;
         socklen_t new_addr_size;
         
         printf("a\n");
         new_addr_size = sizeof(new_addr);
         printf("we're talking socket %d.\n",conns[0].sock);
         /*new = accept(conns[0].sock, (struct sockaddr *) &new_addr, &new_addr_size);*/
         sleep(3);
         printf("b\n");
         new = accept (conns[0].sock, (struct sockaddr *) (&new_addr), (socklen_t *) &new_addr_size);
         printf("c\n");
         sleep(3);
         printf("d\n");
         if (new < 0)
         {
            printf("accept() error!");
            going = 0;
            break;
         }
         
         
         
         printf("New connection from host %s, port %hd.\n",
                inet_ntoa(new_addr.sin_addr), ntohs(new_addr.sin_port));
         
         conns = (struct connection *) realloc( conns, (conns_num+1) * sizeof(struct connection) );
         conns[conns_num].sock = new;
         conns[conns_num].buf = (char *) malloc( (BUFLEN+1) * sizeof(char) );
         conns[conns_num].buf_already = 0;
         conns_num++;
      }

      /* Read a method request from an open connection */
      for (s=1; s<conns_num; s++)
      {
         int i;
         
         /* Read from the socket ... */
         err = read( conns[s].sock, conns[s].buf + conns[s].buf_already, BUFLEN - conns[s].buf_already );
         if (err < 0)
         {
            printf("read() error!");
            going = 0;
            break;
         }
         if (err == 0)
         {
            printf("closing connection.\n");
            close( conns[s].sock );
            free( conns[s].buf );
            for (i=s; i<conns_num-1; i++)
               conns[i] = conns[i+1];
            conns = (struct connection *) realloc( conns, (conns_num-1) * sizeof(struct connection) );
            conns_num--;
         }
         
         conns[s].buf_already += err;
         if (conns[s].buf_already == BUFLEN)
         {
            printf("buffer full!");
            going = 0;
            break;
         }
         
         /* Read as many messages as we can */
         while (1)
         {
            char * end;
            int msg_len; /* including inserted trailing '\0' */
            
            end = memchr( conns[s].buf, '\n', conns[s].buf_already );
            if (!end)
               break;
            
            (*end) = '\0';
            msg_len = end - conns[s].buf + 1;
            
            printf("Received |%s|!\n",conns[s].buf);
            
            for (i=0; i<conns[s].buf_already-msg_len; i++)
               conns[s].buf[i] = conns[s].buf[msg_len+i];
            
            conns[s].buf_already -= msg_len;
         }
      }

   }
   
   
   /* Close the listener socket */
   close(conns[0].sock);
   
   /* Close syslog */
   closelog();
   
   return 0;
}
