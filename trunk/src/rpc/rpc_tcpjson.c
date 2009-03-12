/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... rpc_tcpjson.c
 *  Author ............. Christopher Dellin
 *  Creation Date ...... 2009 Mar 11
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2009   Barrett Technology <support@barrett.com>
 *
 * ======================================================================== */

#include <stdlib.h>     /* For malloc(), etc */
#include <stdio.h>      /* For sprintf() */
#include <string.h>     /* For memchr() */
#include <sys/socket.h> /* For socket() */
#include <unistd.h>     /* For close() */
#include <netinet/in.h> /* For struct sockaddr_in, INADDR_ANY, etc */
#include <arpa/inet.h>  /* For inet_ntoa() */

#include <syslog.h>

#include <json/json.h>

#include "rpc_tcpjson.h"

/* Define the type (see rpc.h for details) */
static struct bt_rpc_listener * listener_create();
static int listener_destroy(struct bt_rpc_listener * base);
static struct bt_rpc_callee * listener_callee_create(struct bt_rpc_listener * base);
static int callee_destroy(struct bt_rpc_callee * base);
static int callee_handle(struct bt_rpc_callee * base, struct bt_rpc_server * s);

static const struct bt_rpc_type bt_rpc_tcpjson_type = {
   "tcp+json",
   &listener_create,
   &listener_destroy,
   &listener_callee_create,
   &callee_destroy,
   &callee_handle
};
const struct bt_rpc_type * bt_rpc_tcpjson = &bt_rpc_tcpjson_type;

/* Private function prototypes */
static int msg_parse(struct bt_rpc_tcpjson_callee * c, struct bt_rpc_server * s, char * msg);

/* Create a new listener */
static struct bt_rpc_listener * listener_create()
{
   int err;
   struct bt_rpc_tcpjson_listener * l;
   struct sockaddr_in listener_addr;
   
   /* Create */
   l = (struct bt_rpc_tcpjson_listener *) malloc(sizeof(struct bt_rpc_tcpjson_listener));
   if (!l)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Initialize */
   l->base.type = bt_rpc_tcpjson;
   l->base.fd = -1;
   
   /* Create the socket */
   l->base.fd = socket(PF_INET, SOCK_STREAM, 0);
   if (l->base.fd == -1)
   {
      syslog(LOG_ERR,"%s: Could not create socket.",__func__);
      listener_destroy(&l->base);
      return 0;
   }
   
   /* Bind to a port */
   listener_addr.sin_family = AF_INET;
   listener_addr.sin_port = htons(PORT);
   listener_addr.sin_addr.s_addr = htonl(INADDR_ANY);
   err = bind(l->base.fd, (struct sockaddr *)&listener_addr, sizeof(listener_addr));
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not bind to port %d.",__func__,PORT);
      listener_destroy(&l->base);
      return 0;
   }

   /* Start listening */
   err = listen(l->base.fd, 10);
   if (err)
   {
      syslog(LOG_ERR,"%s: could not listen on socket.",__func__);
      listener_destroy(&l->base);
      return 0;
   }
   
   /* Success! */
   return &l->base;
}

static int listener_destroy(struct bt_rpc_listener * base)
{
   struct bt_rpc_tcpjson_listener * l;
   l = (struct bt_rpc_tcpjson_listener *)base;
   
   if (l->base.fd != -1)
      close(l->base.fd);
   
   free(l);
   return 0;
}

static struct bt_rpc_callee * listener_callee_create(struct bt_rpc_listener * base)
{
   struct bt_rpc_tcpjson_listener * l;
   int new;
   struct sockaddr_in new_addr;
   socklen_t new_addr_size;
   struct bt_rpc_tcpjson_callee * c;
   
   l = (struct bt_rpc_tcpjson_listener *)base;
   
   /* Accept to create new socket */
   new_addr_size = sizeof(new_addr);
   new = accept(l->base.fd, (struct sockaddr *)&new_addr, &new_addr_size);
   if (new == -1)
   {
      syslog(LOG_ERR,"%s: Could not accept new connection.",__func__);
      return 0;
   }
   
   /* Print to the log */
   syslog(LOG_ERR,"%s: New connection from host %s, port %hd.", __func__,
          inet_ntoa(new_addr.sin_addr), ntohs(new_addr.sin_port));
   
   /* Create a callee for this */
   c = (struct bt_rpc_tcpjson_callee *) malloc(sizeof(struct bt_rpc_tcpjson_callee));
   if (!c)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Initialize */
   c->base.type = base->type;
   c->base.fd = new;
   /* c->buf[] statically allocated */
   c->buf_already = 0;
   
   /* Success! */
   return (struct bt_rpc_callee *)c;
}

static int callee_destroy(struct bt_rpc_callee * base)
{
   struct bt_rpc_tcpjson_callee * c;
   c = (struct bt_rpc_tcpjson_callee *)base;
   free(c);
   return 0;
}

static int callee_handle(struct bt_rpc_callee * base, struct bt_rpc_server * s)
{
   int err;
   struct bt_rpc_tcpjson_callee * c;
   c = (struct bt_rpc_tcpjson_callee *)base;
   
   /* Read into the buffer */
   err = read( c->base.fd, c->buf + c->buf_already, BUFLEN - c->buf_already );
   if (err == -1)
   {
      syslog(LOG_ERR,"%s: Read returned error.",__func__);
      return -1;
   }
   if (err == 0)
   {
      syslog(LOG_ERR,"%s: Closed connection.",__func__);
      /* NOTE: WE MIGHT WANT TO SUICIDE HERE? */
      return 1;
   }
   
   /* Increment the buffer read state by the amount read */
   c->buf_already += err;
   if (c->buf_already == BUFLEN)
   {
      syslog(LOG_ERR,"%s: Buffer full!.",__func__);
      c->buf_already = 0;
      return -2;
   }
   
   /* Read as many messages as we can, newline-terminated. */
   while (1)
   {
      int i;
      char * end;
      int msg_len; /* including inserted trailing '\0' */
      
      /* Note: this will spuriously find newlines in strings! */
      end = memchr( c->buf, '\n', c->buf_already );
      if (!end)
         break; /* No newline yet found */
      
      /* We found a message; cap and measure it */
      (*end) = '\0';
      msg_len = end - c->buf + 1;
      
      /* Parse the message, and perhaps send a response */
      msg_parse(c, s, c->buf);
      /* Do something with this error? */
      
      /* Move any extra characters over */
      for (i=0; i<c->buf_already-msg_len; i++)
         c->buf[i] = c->buf[msg_len+i];
      
      /* Decrement the buffer read state by the amount parsed */
      c->buf_already -= msg_len;
   }
   
   return 0;
}













/* Private functions */
static int msg_parse(struct bt_rpc_tcpjson_callee * c, struct bt_rpc_server * s, char * msg)
{
   struct json_object * req;
   char * method;
   struct json_object * params;
   struct bt_rpc_interface * interface;
   const struct bt_rpc_interface_func * func;
   
   /* Parse the message, make sure it has method and params */
   req = json_tokener_parse(msg);
   if (is_error(req))
   {
      syslog(LOG_ERR,"%s: Error parsing request, pointer is %d.",__func__,(int)req);
      return -1;
   }
   if (!json_object_is_type(req,json_type_object))
   {
      syslog(LOG_ERR,"%s: Request is not a JSON object.",__func__);
      json_object_put(req);
      return -1;
   }
   method = json_object_get_string(json_object_object_get(req,"method"));
   if (!method)
   {
      syslog(LOG_ERR,"%s: Request does not have a 'method' entry.",__func__);
      json_object_put(req);
      return -1;
   }
   if (   !(params = json_object_object_get(req,"params"))
       || !json_object_get_array(params))
   {
      syslog(LOG_ERR,"%s: Request does not have a 'params' array.",__func__);
      json_object_put(req);
      return -1;
   }
   
   /* Look up the method */
   interface = bt_rpc_server_interface_lookup(s, method);
   if (!interface)
   {
      syslog(LOG_ERR,"%s: Interface for \"%s\" not implemented.",__func__,method);
      json_object_put(req);
      return -1;
   }
   func = bt_rpc_interface_func_lookup(interface, method);
   if (!func)
   {
      syslog(LOG_ERR,"%s: Method \"%s\" not implemented.",__func__,method);
      json_object_put(req);
      return -1;
   }
   
   /* Call the method */
   switch (func->type)
   {
      int myint;
      int myint2;
      char * str;
      void * vptr;
      case BT_RPC_FUNC_OBJ_STR_CREATE:
         /* Check for one string parameter */
         if (json_object_array_length(params)!=1)
         {
            syslog(LOG_ERR,"%s: Request does not have one parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         str = json_object_get_string(json_object_array_get_idx(params,0));
         if (!str)
         {
             syslog(LOG_ERR,"%s: Request does not have a string parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the create funcion */
         vptr = (*(void * (*)(char *))(func->ptr))(str);
         if (!vptr)
         {
            int err;
            /* Return failure */
            err = write( c->base.fd, "{\"result\":false}\n",
                         strlen("{\"result\":false}\n") );
            /* What should we do with this error??? */
            json_object_put(req);
            return -1;
         }
         /* Save the object into the interface, as the string */
         bt_rpc_interface_add_object(interface, str, vptr);
         /* Check return value? */
         /* Return success */
         {
            int err;
            /* Return failure */
            err = write( c->base.fd, "{\"result\":true}\n",
                         strlen("{\"result\":true}\n") );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
      case BT_RPC_FUNC_INT_OBJ_DESTROY:
         /* Check for one string parameter */
         if (json_object_array_length(params)!=1)
         {
            syslog(LOG_ERR,"%s: Request does not have one parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         str = json_object_get_string(json_object_array_get_idx(params,0));
         if (!str)
         {
            syslog(LOG_ERR,"%s: Request does not have a string parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Look up the object */
         vptr = bt_rpc_interface_lookup_object(interface, str);
         if (!vptr)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the destroy function */
         myint = (*(int (*)(void *))(func->ptr))(vptr);
         /* Remove the object */
         bt_rpc_interface_remove_object(interface, str);
         /* Return success */
         {
            int err;
            /* Return success */
            err = write( c->base.fd, "{\"result\":true}\n",
                         strlen("{\"result\":true}\n") );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
      case BT_RPC_FUNC_INT_OBJ:
         /* Check for one string parameter */
         if (json_object_array_length(params)!=1)
         {
            syslog(LOG_ERR,"%s: Request does not have one parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         str = json_object_get_string(json_object_array_get_idx(params,0));
         if (!str)
         {
            syslog(LOG_ERR,"%s: Request does not have a string parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Look up the object */
         vptr = bt_rpc_interface_lookup_object(interface, str);
         if (!vptr)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the function */
         myint = (*(int (*)(void *))(func->ptr))(vptr);
         /* Return success */
         {
            int err;
            /* Return success */
            sprintf(c->writebuf,"{\"result\":%d}\n",myint);
            err = write( c->base.fd, c->writebuf,
                         strlen(c->writebuf) );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
      case BT_RPC_FUNC_STR_OBJ:
         /* Check for one string parameter */
         if (json_object_array_length(params)!=1)
         {
            syslog(LOG_ERR,"%s: Request does not have one parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         str = json_object_get_string(json_object_array_get_idx(params,0));
         if (!str)
         {
            syslog(LOG_ERR,"%s: Request does not have a string parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Look up the object */
         vptr = bt_rpc_interface_lookup_object(interface, str);
         if (!vptr)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the function */
         (*(int (*)(void *,char *))(func->ptr))(vptr,c->strbuf);
         /* Return success */
         {
            int err;
            /* Return success */
            sprintf(c->writebuf,"{\"result\":%s}\n",c->strbuf);
            err = write( c->base.fd, c->writebuf,
                         strlen(c->writebuf) );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
      case BT_RPC_FUNC_INT_OBJ_INT:
         /* Check for two parameters */
         if (json_object_array_length(params)!=2)
         {
            syslog(LOG_ERR,"%s: Request does not have two parameters.",__func__);
            json_object_put(req);
            return -1;
         }
         str = json_object_get_string(json_object_array_get_idx(params,0));
         if (!str)
         {
            syslog(LOG_ERR,"%s: Request does not have a string parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         myint2 = json_object_get_int(json_object_array_get_idx(params,1));
         /* Look up the object */
         vptr = bt_rpc_interface_lookup_object(interface, str);
         if (!vptr)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the function */
         myint = (*(int (*)(void *,int))(func->ptr))(vptr,myint2);
         /* Return success */
         {
            int err;
            /* Return success */
            sprintf(c->writebuf,"{\"result\":%d}\n",myint);
            err = write( c->base.fd, c->writebuf,
                         strlen(c->writebuf) );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
      default:
         break;
   }
   
   syslog(LOG_ERR,"%s: Method \"%s\" not implemented.",__func__,method);
   json_object_put(req);
   return -1;
}
