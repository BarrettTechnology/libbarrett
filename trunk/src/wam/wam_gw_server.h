/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wam_gw_server.h
 *  Author ............. Sam Clanton
 *                       Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2004 Q3
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2004-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *    a high-level asynchronous non-realtime interface to the WAM
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt, split from btwam into wam and wambot
 *
 * ======================================================================== */

#include <sys/time.h> /* this is needed for fd_set */
#include <unistd.h>

#include "wam.h"

#include <json/json.h>

/* List of open connections
 * note: this shouldn't be global! */
struct connection
{
   int sock;
   char * buf;
   int buf_already;
};

struct bt_wam_gw_server
{
   int listener;
   
   /*struct printbuf * pb;*/
   char writebuf[101];

   /* List of open connections */
   struct connection * conns;
   int conns_num;

   /* List of open wams */
   struct bt_wam ** wams;
   int wams_num;

};

struct bt_wam_gw_server * bt_wam_gw_server_create();
int bt_wam_gw_server_destroy(struct bt_wam_gw_server * gw);

int bt_wam_gw_server_fdset(struct bt_wam_gw_server * gw, fd_set * setptr);
int bt_wam_gw_server_handle(struct bt_wam_gw_server * gw, fd_set * setptr);
