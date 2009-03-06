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

#include <sys/mman.h>

/* Package Dependencies */
#include <syslog.h>

/* Woo JSON! */
#include <json/json.h>

#include <libbt/wam_gw_server.h>

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
   struct bt_wam_gw_server * gw;
   
   /* Lock memory */
   mlockall(MCL_CURRENT | MCL_FUTURE);
   
   /* Initialize syslog */
   openlog("wamgwd", LOG_CONS | LOG_NDELAY, LOG_USER);
   
   gw = bt_wam_gw_server_create();
   if (!gw)
   {
      printf("could not create wam gw server.\n");
      exit(-1);
   }

   /* Loop until Control+C ... */
   going = 1;
   while (going)
   {
      fd_set read_set;
      FD_ZERO(&read_set);
      
      bt_wam_gw_server_fdset(gw, &read_set);

      /* Wait for activity ... */
      if (select(FD_SETSIZE, &read_set, NULL, NULL, NULL) < 0)
      {
         printf("select() error!");
         going = 0;
         break;
      }
      
      bt_wam_gw_server_handle(gw, &read_set);
   }

   bt_wam_gw_server_destroy(gw);
   
   /* Close syslog */
   closelog();
   
   return 0;
}
