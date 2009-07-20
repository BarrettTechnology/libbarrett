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

#include "rpc.h"
#include "rpc_tcpjson.h"

#include "wam_rpc.h"
#include "file_rpc.h"

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
   bt_rpc_server_add_interface(rpc, bt_file_rpc);
   bt_rpc_server_add_listener(rpc, bt_rpc_tcpjson);

   /* Loop until Control+C ... */
   going = 1;
   while (going)
   {
      bt_rpc_server_select(rpc);
   }

   bt_rpc_server_destroy(rpc);
   
   /* Close syslog */
   closelog();
   
   return 0;
}
