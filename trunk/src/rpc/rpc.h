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

/* Bound methods */
#define bt_rpc_listener_create(rt) ((rt)->listener_create())
#define bt_rpc_listener_destroy(l) ((l)->type->listener_destroy(l))
#define bt_rpc_listener_callee_create(l) ((l)->type->listener_callee_create((l)))
#define bt_rpc_callee_destroy(c) ((c)->type->callee_destroy(c))
#define bt_rpc_callee_handle(c,s) ((c)->type->callee_handle(c,s))

/* Forward declaration of instances */
struct bt_rpc_listener;
struct bt_rpc_callee;
struct bt_rpc_caller;
struct bt_rpc_server;

struct bt_rpc_type
{
   /* The name of the rpc protocol, like "tcp+json" */
   char name[20];
   
   /* Listener functions */
   struct bt_rpc_listener * (* listener_create)();
   int (* listener_destroy)(struct bt_rpc_listener *);
   struct bt_rpc_callee * (* listener_callee_create)(struct bt_rpc_listener *);
   
   /* Callee functions */
   int (* callee_destroy)(struct bt_rpc_callee *);
   int (* callee_handle)(struct bt_rpc_callee *, struct bt_rpc_server *);
   
};

/* An rpc_listener instance */
struct bt_rpc_listener
{
   const struct bt_rpc_type * type;
   int fd;
};
struct bt_rpc_callee
{
   const struct bt_rpc_type * type;
   int fd;
};




/* RPC interfaces */
enum bt_rpc_interface_func_type
{
   BT_RPC_FUNC_OBJ_STR_CREATE,
   BT_RPC_FUNC_INT_OBJ_DESTROY,
   BT_RPC_FUNC_INT_OBJ,
   BT_RPC_FUNC_STR_OBJ,
   BT_RPC_FUNC_INT_OBJ_INT,
   BT_RPC_FUNC_INT_OBJ_DOUBLE,
   BT_RPC_FUNC_INT_OBJ_VECTOR
};

struct bt_rpc_interface_func
{
   void (*ptr)();
   char name[50];
   enum bt_rpc_interface_func_type type;
};

struct bt_rpc_interface_funcs
{
   char prefix[30];
   const struct bt_rpc_interface_func * list;
};

struct bt_rpc_object
{
   char name[30];
   void * vptr;
};

struct bt_rpc_interface
{
   const struct bt_rpc_interface_funcs * funcs;
   
   /* We maintain a list of objects created */
   struct bt_rpc_object ** objects;
   int objects_num;
};




/* An rpc object itself */
struct bt_rpc_server
{
   /* We maintain a list of interfaces to use */
   struct bt_rpc_interface ** interfaces;
   int interfaces_num;

   /* We maintain a list of listeners */
   struct bt_rpc_listener ** listeners;
   int listeners_num;
   
   /* We maintain a list of callees */
   struct bt_rpc_callee ** callees;
   int callees_num;
};

struct bt_rpc_server * bt_rpc_server_create();
int bt_rpc_server_destroy(struct bt_rpc_server * server);
int bt_rpc_server_add_interface(struct bt_rpc_server * server, const struct bt_rpc_interface_funcs * interface_funcs);
int bt_rpc_server_add_listener(struct bt_rpc_server * server, const struct bt_rpc_type * type);
int bt_rpc_server_select(struct bt_rpc_server * server);

/* These are used by callees to do things */
struct bt_rpc_interface * bt_rpc_server_interface_lookup(struct bt_rpc_server * server, char * funcname);
const struct bt_rpc_interface_func * bt_rpc_interface_func_lookup(struct bt_rpc_interface * interface, char * funcname);
int bt_rpc_interface_add_object(struct bt_rpc_interface * interface, char * name, void * vptr);
void * bt_rpc_interface_lookup_object(struct bt_rpc_interface * interface, char * name);
int bt_rpc_interface_remove_object(struct bt_rpc_interface * interface, char * name);

#endif /* BT_RPC_H */
