/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... rpc_prot.h
 *  Author ............. Christopher Dellin
 *  Creation Date ...... 2009 Mar 11
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2009   Barrett Technology <support@barrett.com>
 *
 * ======================================================================== */


/* Forward declaration of instances */
struct bt_rpc_prot_listener;
struct bt_rpc_prot_callee;
#if 0
struct bt_rpc_prot_caller;
#endif

struct bt_rpc_prot
{
   /* The name of the rpc protocol, like "tcp+json" */
   char name[20];
   
   /* Listener functions */
   struct bt_rpc_listener * (* listener_create)();
   int (* listener_destroy)(struct bt_rpc_listener *);
   struct bt_rpc_callee * (* listener_callee_create)(struct bt_rpc_listener *);
   
   /* Callee functions */
   int (* callee_destroy)(struct bt_rpc_callee *);
   /*int (* callee_handle)(struct bt_rpc_callee *, struct bt_rpc_server *);*/
   int (* callee_call_create)(struct bt_rpc_callee *, char * fname, void ** callhandle);
   int (* callee_call_destroy)(struct bt_rpc_callee *, void * callhandle);
   
   int (* callee_call_get_chars)(void * callhandle, int idx, int num, char * valptr);
   int (* callee_call_get_ints)(void * callhandle, int idx, int num, int * valptr);
   int (* callee_call_get_floats)(void * callhandle, int idx, int num, float * valptr);
   int (* callee_call_get_doubles)(void * callhandle, int idx, int num, double * valptr);

   int (* callee_call_put_chars)(void * callhandle, int idx, int num, char * valptr);
   int (* callee_call_put_ints)(void * callhandle, int idx, int num, int * valptr);
   int (* callee_call_put_floats)(void * callhandle, int idx, int num, float * valptr);
   int (* callee_call_put_doubles)(void * callhandle, int idx, int num, double * valptr);
   
#if 0
   /* Caller functions */
   struct bt_rpc_caller * (* caller_create)(char * host);
   int (* caller_destroy)(struct bt_rpc_caller *);
   int (* caller_handle)(struct bt_rpc_caller *, const struct bt_rpc_interface_funcs * funcs, const char * function, ...);
#endif
};

/* An rpc_listener instance */
struct bt_rpc_prot_listener
{
   const struct bt_rpc_prot * type;
   int fd;
};
struct bt_rpc_prot_callee
{
   const struct bt_rpc_prot * type;
   int fd;
};
#if 0
struct bt_rpc_caller
{
   const struct bt_rpc_type * type;
};
#endif
