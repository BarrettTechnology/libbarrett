/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... rpc.c
 *  Author ............. Christopher Dellin
 *  Creation Date ...... 2009 Mar 11
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2009   Barrett Technology <support@barrett.com>
 *
 * ======================================================================== */

#include <sys/types.h> /* For select() */
#include <sys/select.h> /* for fd_set */

#include <stdlib.h>
#include <syslog.h>
#include <string.h>

#include "rpc.h"
#include "rpc_tcpjson.h"

/* List of available types */
static const struct bt_rpc_type * type_list[] =
{
   &bt_rpc_tcpjson_type,
   0
};

/* Functions for caller creation */
const struct bt_rpc_type * bt_rpc_type_search(char * prefix)
{
   int i;
   for (i=0; type_list[i]; i++)
   if (strcmp(prefix,type_list[i]->name)==0)
      return type_list[i];
   return 0;
}

/* Define what a server is */
struct bt_rpc_server * bt_rpc_server_create()
{
   struct bt_rpc_server * s;
   
   /* Create */
   s = (struct bt_rpc_server *) malloc(sizeof(struct bt_rpc_server));
   if (!s)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Initialize */
   s->interfaces = 0;
   s->interfaces_num = 0;
   s->listeners = 0;
   s->listeners_num = 0;
   s->callees = 0;
   s->callees_num = 0;
   
   return s;
}

int bt_rpc_server_destroy(struct bt_rpc_server * s)
{
   int i;
   
   /* Interfaces are not dynamically allocated */
   if (s->interfaces)
      free(s->interfaces);
   
   /* Callees are, though */
   for (i=0; i<s->callees_num; i++)
      bt_rpc_callee_destroy(s->callees[i]);
   if (s->callees)
      free(s->callees);
   
   /* Listeners are, also */
   for (i=0; i<s->listeners_num; i++)
      bt_rpc_listener_destroy(s->listeners[i]);
   if (s->listeners)
      free(s->listeners);
   
   free(s);
   return 0;
}

int bt_rpc_server_add_interface(struct bt_rpc_server * s, const struct bt_rpc_interface_funcs * interface_funcs)
{
   struct bt_rpc_interface * interface;
   
   /* Attempt to make the new interface */
   interface = (struct bt_rpc_interface *) malloc(sizeof(struct bt_rpc_interface));
   if (!interface)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return -1;
   }
   /* Initialize */
   interface->funcs = interface_funcs;
   interface->objects = 0;
   interface->objects_num = 0;
   /* Extend the storage array */
   if (!s->interfaces_num)
      s->interfaces = (struct bt_rpc_interface **) malloc(sizeof(struct bt_rpc_interface *));
   else
      s->interfaces = (struct bt_rpc_interface **) realloc(s->interfaces,
                                                           (s->interfaces_num+1)*sizeof(struct bt_rpc_interface *));
   if (!s->interfaces)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      free(interface);
      return -1;
   }
   /* Save the interface */
   s->interfaces[s->interfaces_num] = interface;
   s->interfaces_num++;
   return 0;
}

int bt_rpc_server_add_listener(struct bt_rpc_server * s, const struct bt_rpc_type * type)
{
   struct bt_rpc_listener * l;
   
   /* Attempt to make the new listener */
   l = bt_rpc_listener_create(type);
   if (!l)
   {
      syslog(LOG_ERR,"%s: Could not create listener.",__func__);
      return -2;
   }
   /* Extend the storage array */
   if (!s->listeners_num)
      s->listeners = (struct bt_rpc_listener **) malloc(sizeof(struct bt_rpc_listener *));
   else
      s->listeners = (struct bt_rpc_listener **) realloc(s->listeners,
                                                           (s->listeners_num+1)*sizeof(struct bt_rpc_listener *));
   if (!s->listeners)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_rpc_listener_destroy(l);
      return -1;
   }
   /* Save the listener */
   s->listeners[s->listeners_num] = l;
   s->listeners_num++;
   return 0;
}

int bt_rpc_server_select(struct bt_rpc_server * s)
{
   int i;
   fd_set read_set;
   int err;
   
   FD_ZERO(&read_set);
   
   /* Wait for all the listeners */
   for (i=0; i<s->listeners_num; i++)
      FD_SET(s->listeners[i]->fd,&read_set);
   
   /* Wait for all the callees */
   for (i=0; i<s->callees_num; i++)
      FD_SET(s->callees[i]->fd,&read_set);
   
   /* Select */
   err = select(FD_SETSIZE, &read_set, NULL, NULL, NULL);
   if (err == -1)
   {
      syslog(LOG_ERR,"%s: select() error.",__func__);
      return -1;
   }
   
   /* Handle any listener stuff */
   for (i=0; i<s->listeners_num; i++)
   if (FD_ISSET(s->listeners[i]->fd,&read_set))
   {
      struct bt_rpc_callee * callee;
      callee = bt_rpc_listener_callee_create(s->listeners[i]);
      if (!callee)
      {
         syslog(LOG_ERR,"%s: Could not create callee from listener.",__func__);
         continue;
      }
      /* Add the callee to the server */
      if (!s->callees_num)
         s->callees = (struct bt_rpc_callee **) malloc(sizeof(struct bt_rpc_callee *));
      else
         s->callees = (struct bt_rpc_callee **) realloc(s->callees,
                                                        (s->callees_num+1)*sizeof(struct bt_rpc_callee *));
      if (!s->callees)
      {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         bt_rpc_callee_destroy(callee);
         continue;
      }
      /* Save the callee */
      s->callees[s->callees_num] = callee;
      s->callees_num++;
   }
   
   /* Handle any callee stuff */
   for (i=0; i<s->callees_num; i++)
   if (FD_ISSET(s->callees[i]->fd,&read_set))
   {
      err = bt_rpc_callee_handle(s->callees[i],s);
      if (err == 1)
      {
         int j;
         bt_rpc_callee_destroy(s->callees[i]);
         for (j=i; j<s->callees_num-1; j++)
            s->callees[j] = s->callees[j+1];
         if (s->callees_num > 1)
            s->callees = (struct bt_rpc_callee **) realloc(s->callees,
                                                        (s->callees_num-1)*sizeof(struct bt_rpc_callee *));
         else
            free(s->callees);
         /* Handle errors here? */
         s->callees_num--;
      }
      /* Handle errors? */
   }
   
   return 0;
}

struct bt_rpc_interface * bt_rpc_server_interface_lookup(struct bt_rpc_server * s, const char * funcname)
{
   int i;
   for (i=0; i<s->interfaces_num; i++)
   if (strncmp(funcname,s->interfaces[i]->funcs->prefix,strlen(s->interfaces[i]->funcs->prefix))==0)
      return s->interfaces[i];
   return 0;
}

const struct bt_rpc_interface_func * bt_rpc_interface_func_lookup(const struct bt_rpc_interface_funcs * funcs, const char * funcname)
{
   int i;
   for (i=0; funcs->list[i].ptr; i++)
   if (strcmp(funcname,funcs->list[i].name)==0)
      return &(funcs->list[i]);
   return 0;
}

int bt_rpc_interface_object_add(struct bt_rpc_interface * interface, void * vptr)
{
   /* Extend the storage array */
   if (!interface->objects_num)
      interface->objects = (void **) malloc(sizeof(void *));
   else
      interface->objects = (void **) realloc(interface->objects,
                                             (interface->objects_num+1)*sizeof(void *));
   if (!interface->objects)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return -1;
   }
   /* Save the object */
   interface->objects[interface->objects_num] = vptr;
   interface->objects_num++;
   return 0;
}

int bt_rpc_interface_object_check(struct bt_rpc_interface * interface, void * vptr)
{
   int i;
   for (i=0; i<interface->objects_num; i++)
   if (vptr == interface->objects[i])
      return 0;
   return -1; /* Not found */
}

int bt_rpc_interface_object_remove(struct bt_rpc_interface * interface, void * vptr)
{
   int i;
   for (i=0; i<interface->objects_num; i++)
   if (vptr == interface->objects[i])
   {
      int j;
      for (j=i; j<interface->objects_num-1; j++)
         interface->objects[j] = interface->objects[j+1];
      if (interface->objects_num > 1)
         interface->objects = (void **) realloc(interface->objects,
                                                (interface->objects_num-1)*sizeof(void *));
      else
         free(interface->objects);
      /* Ignore error? */
      interface->objects_num--;
      return 0;
   }
   return -1; /* Not found */
}
