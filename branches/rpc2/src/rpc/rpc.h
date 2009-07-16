/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... rpc.h
 *  Author ............. Christopher Dellin
 *  Creation Date ...... 2009 Mar 11
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2009   Barrett Technology <support@barrett.com>
 *
 * ======================================================================== */

#ifndef BT_RPC_H
#define BT_RPC_H

#include "rpc_int.h"
#include "rpc_prot.h"

/* An rpc object itself */
struct bt_rpc_server
{
   /* We maintain a list of interfaces to use */
   struct bt_rpc_interface ** interfaces;
   int interfaces_num;

   /* We maintain a list of listeners */
   struct bt_rpc_prot_listener ** listeners;
   int listeners_num;
   
   /* We maintain a list of callees */
   struct bt_rpc_prot_callee ** callees;
   int callees_num;
};

/* Functions for caller creation */
struct bt_rpc_caller * bt_rpc_caller_search_create(char * prefixhost, ...);
const struct bt_rpc_type * bt_rpc_type_search(char * prefix);

/* Functions for bt_rcp_server */
struct bt_rpc_server * bt_rpc_server_create();
int bt_rpc_server_destroy(struct bt_rpc_server * server);
int bt_rpc_server_add_interface(struct bt_rpc_server * server, const struct bt_rpc_interface_funcs * interface_funcs);
int bt_rpc_server_add_listener(struct bt_rpc_server * server, const struct bt_rpc_type * type);
int bt_rpc_server_select(struct bt_rpc_server * server);

/* These are used by callees to do things */
struct bt_rpc_interface * bt_rpc_server_interface_lookup(struct bt_rpc_server * server, const char * funcname);
const struct bt_rpc_interface_func * bt_rpc_interface_func_lookup(const struct bt_rpc_interface_funcs * funcs, const char * funcname);
int bt_rpc_interface_object_add(struct bt_rpc_interface * interface, void * vptr);
int bt_rpc_interface_object_check(struct bt_rpc_interface * interface, void * vptr);
int bt_rpc_interface_object_remove(struct bt_rpc_interface * interface, void * vptr);



/* Functions used by interfaces (like bt_rpc_arg_pre() and _post()) */



#endif /* BT_RPC_H */
