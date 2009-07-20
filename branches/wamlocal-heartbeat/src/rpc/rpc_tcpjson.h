/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... rpc_tcpjson.h
 *  Author ............. Christopher Dellin
 *  Creation Date ...... 2009 Mar 11
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2009   Barrett Technology <support@barrett.com>
 *
 * ======================================================================== */

#include "rpc.h"

#define PORT 1338
#define BUFLEN 1023 /* No requests are allowed to be longer than this */

const struct bt_rpc_type * bt_rpc_tcpjson;

/* A listener instance */
struct bt_rpc_tcpjson_listener
{
   /* Include the base */
   struct bt_rpc_listener base;
};

struct bt_rpc_tcpjson_callee
{
   struct bt_rpc_callee base;
   char buf[BUFLEN+1];
   int buf_already;
   
   char writebuf[402]; /* For sending things back */
   char * strbuf; /* For library functions that return strings */
   int strbuf_len;
};

struct bt_rpc_tcpjson_caller
{
   struct bt_rpc_caller base;
   int fd;
   char buf[BUFLEN+1];
   int buf_already;
};
