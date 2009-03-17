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
#define bt_rpc_caller_create(rt,h) ((rt)->caller_create(h))
#define bt_rpc_caller_destroy(c) ((c)->type->caller_destroy(c))
#define bt_rpc_caller_handle(c,fs,f,...) ((c)->type->caller_handle(c,fs,f,__VA_ARGS__))

/* Forward declaration of instances */
struct bt_rpc_listener;
struct bt_rpc_callee;
struct bt_rpc_caller;
struct bt_rpc_server;
struct bt_rpc_interface_funcs;

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
   
   /* Caller functions */
   struct bt_rpc_caller * (* caller_create)(char * host);
   int (* caller_destroy)(struct bt_rpc_caller *);
   int (* caller_handle)(struct bt_rpc_caller *, const struct bt_rpc_interface_funcs * funcs, const char * function, ...);
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
struct bt_rpc_caller
{
   const struct bt_rpc_type * type;
};



/* RPC interfaces */
enum bt_rpc_interface_func_type
{
/*   BT_RPC_FUNC_OBJ_STR_CREATE,*/
   BT_RPC_FUNC_OBJ_STR_INT_CREATE,
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

struct bt_rpc_interface
{
   const struct bt_rpc_interface_funcs * funcs;
   
   /* We maintain a list of objects created */
   void ** objects;
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

#endif /* BT_RPC_H */
