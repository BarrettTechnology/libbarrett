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
#include <sys/types.h> /* For select() */
#include <sys/select.h> /* for fd_set */

/* Package Dependencies */
#include <syslog.h>

#include "rpc.h"
#include "rpc_tcpjson.h"
#include "wam_rpc.h"

#include "discover.h"

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
   struct bt_rpc_server * rpc;
   struct bt_discover_server * discover;
   
   /* Lock memory */
   mlockall(MCL_CURRENT | MCL_FUTURE);
   
   /* Initialize syslog */
   openlog("wamgwd", LOG_CONS | LOG_NDELAY, LOG_USER);
   
   rpc = bt_rpc_server_create();
   if (!rpc)
   {
      printf("could not create rpc server.\n");
      exit(-1);
   }
   
   /* Set up the RPC server */
   bt_rpc_server_add_interface(rpc, bt_wam_rpc);
   bt_rpc_server_add_listener(rpc, bt_rpc_tcpjson);

   discover = bt_discover_server_create(1337,"eth0");
   if (!discover)
   {
      printf("could not create discover server.\n");
      bt_rpc_server_destroy(rpc);
      exit(-1);
   }

   /* Loop until Control+C ... */
   going = 1;
   while (going)
   {
      int err;
      fd_set read_set;

      FD_ZERO(&read_set);
      
      bt_rpc_server_select_pre(rpc,&read_set);
      bt_discover_server_select_pre(discover,&read_set);

      /* Select */
      err = select(FD_SETSIZE, &read_set, NULL, NULL, NULL);
      if (err == -1)
      {
         going = 0;
      }

      bt_rpc_server_select_post(rpc,&read_set);
      bt_discover_server_select_post(discover,&read_set);
   }

   bt_rpc_server_destroy(rpc);
   bt_discover_server_destroy(discover);
   
   /* Close syslog */
   closelog();
   
   return 0;
}
