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
#include <stdarg.h>     /* For variadics */
#include <sys/socket.h> /* For socket() */
#include <unistd.h>     /* For close() */
#include <netinet/in.h> /* For struct sockaddr_in, INADDR_ANY, etc */
#include <arpa/inet.h>  /* For inet_ntoa() */
#include <netdb.h>      /* For gethostbyname() */

#include <syslog.h>

#include <json/json.h>

#include "rpc_tcpjson.h"

/* Define the type (see rpc.h for details) */
static struct bt_rpc_listener * listener_create();
static int listener_destroy(struct bt_rpc_listener * base);
static struct bt_rpc_callee * listener_callee_create(struct bt_rpc_listener * base);
static int callee_destroy(struct bt_rpc_callee * base);
static int callee_handle(struct bt_rpc_callee * base, struct bt_rpc_server * s);
static struct bt_rpc_caller * caller_create(char * host);
static int caller_destroy(struct bt_rpc_caller *);
static int caller_handle(struct bt_rpc_caller *, const struct bt_rpc_interface_funcs * funcs, const char * function, ...);

static const struct bt_rpc_type bt_rpc_tcpjson_type = {
   "tcp+json",
   &listener_create,
   &listener_destroy,
   &listener_callee_create,
   &callee_destroy,
   &callee_handle,
   &caller_create,
   &caller_destroy,
   &caller_handle
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
      close(new);
      return 0;
   }
   
   c->strbuf = (char *) malloc(302*sizeof(char));
   if (!c->strbuf)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      close(new);
      free(c);
      return 0;
   }
   c->strbuf_len = 302;

   c->doublebuf = (double *) malloc(20*sizeof(double));
   if (!c->doublebuf)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      free(c->strbuf);
      close(new);
      free(c);
      return 0;
   }
   c->doublebuf_len = 20;
   
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
   close(c->base.fd);
   free(c->strbuf);
   free(c->doublebuf);
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
      /* NOTE: We let the rpc server destroy us (and close the socket) */
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
   int err;
   struct json_object * req;
   const char * method;
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
   func = bt_rpc_interface_func_lookup(interface->funcs, method);
   if (!func)
   {
      syslog(LOG_ERR,"%s: Method \"%s\" not implemented.",__func__,method);
      json_object_put(req);
      return -1;
   }
   
   /* Call the method */
   switch (func->type)
   {
      struct json_object * jsonobj;
      int myint;
      int myint2;
      int myint3;
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
         /* Save the object into the interface, if it was created */
         if (vptr)
            bt_rpc_interface_object_add(interface, vptr);
         /* Return success, with the object pointer */
         {
            int err;
            /* Return success */
            sprintf(c->writebuf,"{\"result\":%d}\n",(int)vptr);
            err = write( c->base.fd, c->writebuf,
                         strlen(c->writebuf) );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
#if 0
      case BT_RPC_FUNC_OBJ_CREATE:
         /* Check for one string, one int parameter */
         if (json_object_array_length(params)!=0)
         {
            syslog(LOG_ERR,"%s: Request does not have zero parameters.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the create funcion */
         vptr = (*(void * (*)())(func->ptr))();
         /* Save the object into the interface, if it was created */
         if (vptr)
            bt_rpc_interface_object_add(interface, vptr);
         /* Return success, with the object pointer */
         {
            int err;
            /* Return success */
            sprintf(c->writebuf,"{\"result\":%d}\n",(int)vptr);
            err = write( c->base.fd, c->writebuf,
                         strlen(c->writebuf) );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
#endif
      case BT_RPC_FUNC_OBJ_STR_INT_CREATE:
         /* Check for one string, one int parameter */
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
         /* Cast and call the create funcion */
         vptr = (*(void * (*)(char *,int))(func->ptr))(str,myint2);
         /* Save the object into the interface, if it was created */
         if (vptr)
            bt_rpc_interface_object_add(interface, vptr);
         /* Return success, with the object pointer */
         {
            int err;
            /* Return success */
            sprintf(c->writebuf,"{\"result\":%d}\n",(int)vptr);
            err = write( c->base.fd, c->writebuf,
                         strlen(c->writebuf) );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
      case BT_RPC_FUNC_INT_OBJ_DESTROY:
         /* Check for one int (vptr) parameter */
         if (json_object_array_length(params)!=1)
         {
            syslog(LOG_ERR,"%s: Request does not have one parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         vptr = (void *) json_object_get_int(json_object_array_get_idx(params,0));
         /* Check object */
         err = bt_rpc_interface_object_check(interface, vptr);
         if (err)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the destroy function */
         myint = (*(int (*)(void *))(func->ptr))(vptr);
         /* Remove the object */
         bt_rpc_interface_object_remove(interface, vptr);
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
      case BT_RPC_FUNC_INT_OBJ:
         /* Check for one int (vptr) parameter */
         if (json_object_array_length(params)!=1)
         {
            syslog(LOG_ERR,"%s: Request does not have one parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         vptr = (void *) json_object_get_int(json_object_array_get_idx(params,0));
         /* Check object */
         err = bt_rpc_interface_object_check(interface, vptr);
         if (err)
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
         /* Check for one int (vptr) parameter */
         if (json_object_array_length(params)!=1)
         {
            syslog(LOG_ERR,"%s: Request does not have one parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         vptr = (void *) json_object_get_int(json_object_array_get_idx(params,0));
         /* Check object */
         err = bt_rpc_interface_object_check(interface, vptr);
         if (err)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the function */
         (*(char * (*)(void *,char *))(func->ptr))(vptr,c->strbuf);
         /* Return success */
         {
            int err;
            /* Return success */
            sprintf(c->writebuf,"{\"result\":\"%s\"}\n",c->strbuf);
            err = write( c->base.fd, c->writebuf,
                         strlen(c->writebuf) );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
      case BT_RPC_FUNC_STR_OBJ_INT:
         /* Check for one int (vptr) parameter, one int parameter */
         if (json_object_array_length(params)!=2)
         {
            syslog(LOG_ERR,"%s: Request does not have one parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         vptr = (void *) json_object_get_int(json_object_array_get_idx(params,0));
         myint2 = json_object_get_int(json_object_array_get_idx(params,1));
         /* Check object */
         err = bt_rpc_interface_object_check(interface, vptr);
         if (err)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the function */
         (*(char * (*)(void *,int,char *))(func->ptr))(vptr,myint2,c->strbuf);
         /* Return success */
         {
            int err;
            /* Return success */
            sprintf(c->writebuf,"{\"result\":\"%s\"}\n",c->strbuf);
            err = write( c->base.fd, c->writebuf,
                         strlen(c->writebuf) );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
      case BT_RPC_FUNC_INT_OBJ_INT:
         /* Check for one int (vptr) parameter, one int parameter */
         if (json_object_array_length(params)!=2)
         {
            syslog(LOG_ERR,"%s: Request does not have one parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         vptr = (void *) json_object_get_int(json_object_array_get_idx(params,0));
         myint2 = json_object_get_int(json_object_array_get_idx(params,1));
         /* Check object */
         err = bt_rpc_interface_object_check(interface, vptr);
         if (err)
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
      case BT_RPC_FUNC_INT_OBJ_STR:
         /* Check for two parameters, an obj and a string */
         if (json_object_array_length(params)!=2)
         {
            syslog(LOG_ERR,"%s: Request does not have two parameters.",__func__);
            json_object_put(req);
            return -1;
         }
         vptr = (void *) json_object_get_int(json_object_array_get_idx(params,0));
         /* Check object */
         err = bt_rpc_interface_object_check(interface, vptr);
         if (err)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Get string */
         str = json_object_get_string(json_object_array_get_idx(params,1));
         if (!str)
         {
            syslog(LOG_ERR,"%s: Request does not have a string parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the funcion */
         myint = (*(int (*)(void *,char *))(func->ptr))(vptr,str);
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
      case BT_RPC_FUNC_INT_OBJ_INT_INT:
         /* Check for one int (vptr) parameter, two int parameters */
         if (json_object_array_length(params)!=3)
         {
            syslog(LOG_ERR,"%s: Request does not have three parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         vptr = (void *) json_object_get_int(json_object_array_get_idx(params,0));
         myint2 = json_object_get_int(json_object_array_get_idx(params,1));
         myint3 = json_object_get_int(json_object_array_get_idx(params,2));
         /* Check object */
         err = bt_rpc_interface_object_check(interface, vptr);
         if (err)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the function */
         myint = (*(int (*)(void *,int,int))(func->ptr))(vptr,myint2,myint3);
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
      case BT_RPC_FUNC_INT_OBJ_GETLINE:
         /* Check for one int (vptr) parameter */
         if (json_object_array_length(params)!=1)
         {
            syslog(LOG_ERR,"%s: Request does not have one parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         vptr = (void *) json_object_get_int(json_object_array_get_idx(params,0));
         /* Check object */
         err = bt_rpc_interface_object_check(interface, vptr);
         if (err)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Cast and call the function */
         myint = (*(int (*)(void *,char **,int *))(func->ptr))(vptr,&c->strbuf,&c->strbuf_len);
         /* Return success */
         {
            int err;
            /* Return success */
            sprintf(c->writebuf,"{\"result\":\"%s\"}\n",c->strbuf);
            err = write( c->base.fd, c->writebuf,
                         strlen(c->writebuf) );
            /* What should we do with this error??? */
         }
         json_object_put(req);
         return 0;
      case BT_RPC_FUNC_INT_OBJ_NDOUBLE:
         /* Check for two parameters, an obj and an array */
         if (json_object_array_length(params)!=2)
         {
            syslog(LOG_ERR,"%s: Request does not have two parameters.",__func__);
            json_object_put(req);
            return -1;
         }
         vptr = (void *) json_object_get_int(json_object_array_get_idx(params,0));
         /* Check object */
         err = bt_rpc_interface_object_check(interface, vptr);
         if (err)
         {
            syslog(LOG_ERR,"%s: Could not find object.",__func__);
            json_object_put(req);
            return -1;
         }
         /* Get array */
         jsonobj = json_object_array_get_idx(params,1);
         if (!json_object_get_array(jsonobj))
         {
            syslog(LOG_ERR,"%s: Request does not have array parameter.",__func__);
            json_object_put(req);
            return -1;
         }
         myint3 = json_object_array_length(jsonobj);
         /* Enlarge the double buffer if necessary */
         if (myint3 > c->doublebuf_len)
         {
            c->doublebuf = (double *) realloc( c->doublebuf, myint3*sizeof(double) );
            c->doublebuf_len = myint3;
         }
         for (myint2=0; myint2<myint3; myint2++)
         {
            c->doublebuf[myint2] = json_object_get_double(
                           json_object_array_get_idx(jsonobj,myint2) );
         }
         /* Cast and call the funcion */
         myint = (*(int (*)(void *,int,double *))(func->ptr))(vptr,myint3,c->doublebuf);
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


static struct bt_rpc_caller * caller_create(char * host)
{
   int err;
   struct bt_rpc_tcpjson_caller * cr;
   struct sockaddr_in callee_addr;
   struct hostent * hostinfo;
   
   /* Create */
   cr = (struct bt_rpc_tcpjson_caller *) malloc(sizeof(struct bt_rpc_tcpjson_caller));
   if (!cr)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Initialize */
   cr->base.type = bt_rpc_tcpjson;
   cr->fd = -1;
   
   /* Create the socket */
   cr->fd = socket(PF_INET, SOCK_STREAM, 0);
   if (cr->fd == -1)
   {
      syslog(LOG_ERR,"%s: Could not create socket.",__func__);
      caller_destroy(&cr->base);
      return 0;
   }
   
   /* Look up host */
   callee_addr.sin_family = AF_INET;
   callee_addr.sin_port = htons(PORT);
   hostinfo = gethostbyname(host);
   if (!hostinfo)
   {
      syslog(LOG_ERR,"%s: Could not find host %s.",__func__,host);
      caller_destroy(&cr->base);
      return 0;
   }
   callee_addr.sin_addr = *(struct in_addr *)(hostinfo->h_addr_list[0]);
   
   /* Attempt to connect */
   err = connect(cr->fd, (struct sockaddr *)&callee_addr, sizeof(callee_addr));
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not connect to host %s.",__func__,host);
      caller_destroy(&cr->base);
      return 0;
   }
   
   /* Success! */
   return (struct bt_rpc_caller *)cr;
}

static int caller_destroy(struct bt_rpc_caller * base)
{
   struct bt_rpc_tcpjson_caller * cr;
   cr = (struct bt_rpc_tcpjson_caller *)base;
   
   if (cr->fd != -1)
      close(cr->fd);
   
   free(cr);
   return 0;
}

static int caller_handle(struct bt_rpc_caller * base, const struct bt_rpc_interface_funcs * funcs, const char * function, ...)
{
   int err;
   va_list ap;
   struct bt_rpc_tcpjson_caller * cr;
   const struct bt_rpc_interface_func * func;
   struct json_object * req;
   
   cr = (struct bt_rpc_tcpjson_caller *)base;
   va_start(ap, function);
   
   /* Look up funcion */
   func = bt_rpc_interface_func_lookup(funcs, function);
   if (!func)
   {
      syslog(LOG_ERR,"%s: Could not find function %s.",__func__,function);
      va_end(ap);
      return -1;
   }
   
   /* Switch on type,
    * put parameters into cr->buf */
   switch (func->type)
   {
      char * mystr;
      void * vptr;
      int myint;
      int myint2;
      double * doubleptr;
      case BT_RPC_FUNC_OBJ_STR_CREATE:
         mystr = va_arg(ap, char *);
         sprintf(cr->buf,"{'method':'%s','params':['%s']}\n", function, mystr);
         break;
#if 0
      case BT_RPC_FUNC_OBJ_CREATE:
         sprintf(cr->buf,"{'method':'%s','params':[]}\n", function);
         break;
#endif
      case BT_RPC_FUNC_OBJ_STR_INT_CREATE:
         mystr = va_arg(ap, char *);
         myint = va_arg(ap, int);
         sprintf(cr->buf,"{'method':'%s','params':['%s',%d]}\n", function, mystr, myint);
         break;
      case BT_RPC_FUNC_INT_OBJ_DESTROY:
      case BT_RPC_FUNC_INT_OBJ:
      case BT_RPC_FUNC_STR_OBJ:
      case BT_RPC_FUNC_INT_OBJ_GETLINE:
         vptr = va_arg(ap, void *);
         sprintf(cr->buf,"{'method':'%s','params':[%d]}\n", function, (int)vptr);
         break;
      case BT_RPC_FUNC_STR_OBJ_INT:
      case BT_RPC_FUNC_INT_OBJ_INT:
         vptr = va_arg(ap, void *);
         myint = va_arg(ap, int);
         sprintf(cr->buf,"{'method':'%s','params':[%d,%d]}\n", function,
                 (int)vptr, myint);
         break;
      case BT_RPC_FUNC_INT_OBJ_INT_INT:
         vptr = va_arg(ap, void *);
         myint = va_arg(ap, int);
         myint2 = va_arg(ap, int);
         sprintf(cr->buf,"{'method':'%s','params':[%d,%d,%d]}\n", function,
                 (int)vptr, myint, myint2);
         break;
      case BT_RPC_FUNC_INT_OBJ_STR:
         vptr = va_arg(ap, void *);
         mystr = va_arg(ap, char *);
         sprintf(cr->buf,"{'method':'%s','params':[%d,'%s']}\n", function, (int)vptr, mystr);
         break;
      case BT_RPC_FUNC_INT_OBJ_NDOUBLE:
         vptr = va_arg(ap, void *);
         myint = va_arg(ap, int);
         doubleptr = va_arg(ap, double *);
         sprintf(cr->buf,"{'method':'%s','params':[%d,[", function, (int)vptr);
         if (myint)
            sprintf(cr->buf + strlen(cr->buf),"%.4f",doubleptr[0]);
         for (myint2=1; myint2<myint; myint2++)
            sprintf(cr->buf + strlen(cr->buf),",%.4f",doubleptr[myint2]);
         sprintf(cr->buf + strlen(cr->buf),"]]}\n");
         break;
      default:
         syslog(LOG_ERR,"%s: Non-supported function type.",__func__);
         va_end(ap);
         return -1;
   }
   
   /* Send the function */
   err = write( cr->fd, cr->buf, strlen(cr->buf) );
   /* Check error? */
   
   /* Await response */
   cr->buf_already = 0;
   while (1)
   {
      char * end;
      
      err = read( cr->fd, cr->buf + cr->buf_already, BUFLEN - cr->buf_already );
      if (err == -1)
      {
         syslog(LOG_ERR,"%s: Read returned error.",__func__);
         va_end(ap);
         return -1;
      }
      if (err == 0)
      {
         syslog(LOG_ERR,"%s: Closed connection.",__func__);
         /* What do we do here? */
         va_end(ap);
         return -1;
      } 
      
      /* Increment the buffer read state by the amount read */
      cr->buf_already += err;
      if (cr->buf_already == BUFLEN)
      {
         syslog(LOG_ERR,"%s: Buffer full!.",__func__);
         cr->buf_already = 0;
         return -2;
      }
      
      /* Put a '\0' at the '\n' ... */
      end = memchr( cr->buf, '\n', cr->buf_already );
      if (end)
      {
         (*end) = '\0';
         break; 
      }
      
      /* No newline yet found, keep reading */
   }
   
   /* Parse the message, make sure it's an object */
   req = json_tokener_parse(cr->buf);
   if (is_error(req))
   {
      syslog(LOG_ERR,"%s: Error parsing request, pointer is %d.",__func__,(int)req);
      syslog(LOG_ERR,"%s: Request: |%s|",__func__,cr->buf);
      va_end(ap);
      return -1;
   }
   if (!json_object_is_type(req,json_type_object))
   {
      syslog(LOG_ERR,"%s: Request is not a JSON object.",__func__);
      json_object_put(req);
      va_end(ap);
      return -1;
   }
   
   /* Check for error!!!! */
   
   /* Switch on type,
    * put result into (*result), the last vararg */
   switch (func->type)
   {
      void * vptr;
      int myint;
      char * mystr;
      void * result;
      char ** strptr;
      int * intptr;

      case BT_RPC_FUNC_OBJ_STR_CREATE:
#if 0
      case BT_RPC_FUNC_OBJ_CREATE:
#endif
      case BT_RPC_FUNC_OBJ_STR_INT_CREATE:
         vptr = (void *) json_object_get_int(json_object_object_get(req,"result"));
         result = va_arg(ap, void *);
         *((void **)result) = vptr;
         break;
      case BT_RPC_FUNC_INT_OBJ_DESTROY:
      case BT_RPC_FUNC_INT_OBJ:
      case BT_RPC_FUNC_INT_OBJ_INT:
      case BT_RPC_FUNC_INT_OBJ_STR:
      case BT_RPC_FUNC_INT_OBJ_INT_INT:
      case BT_RPC_FUNC_INT_OBJ_NDOUBLE:
         myint = json_object_get_int(json_object_object_get(req,"result"));
         result = va_arg(ap, void *);
         *((int *)result) = myint;
         break;
      case BT_RPC_FUNC_STR_OBJ:
      case BT_RPC_FUNC_STR_OBJ_INT:
         mystr = json_object_get_string(json_object_object_get(req,"result"));
         /* TODO check length! */
         result = va_arg(ap, void *);
         strcpy((char *)result,mystr);
         break;
      case BT_RPC_FUNC_INT_OBJ_GETLINE:
         mystr = json_object_get_string(json_object_object_get(req,"result"));
         strptr = va_arg(ap, char **);
         intptr = va_arg(ap, int *);
         result = va_arg(ap, void *);
         myint = strlen(mystr);
         /* Allocate (*strptr) if passed as 0 */
         if (!(*strptr))
         {
            (*strptr) = (char *) malloc( (myint+1) * sizeof(char) );
            if (!(*strptr))
            {
               syslog(LOG_ERR,"%s: Out of memory.",__func__);
               *((int *)result) = -1;
               break;
            }
            (*intptr) = myint + 1;
         }
         /* Enlarge (*strptr) if necessary */
         if ((*intptr) < myint + 1)
         {
            (*strptr) = (char *) realloc( (*strptr), (myint+1) * sizeof(char) );
            if (!(*strptr))
            {
               syslog(LOG_ERR,"%s: Out of memory.",__func__);
               *((int *)result) = -1;
               break;
            }
            (*intptr) = myint + 1;
         }
         /* Copy in the string */
         strcpy((*strptr),mystr);
         *((int *)result) = -1; /* TODO: actually get this over the wire */
         break;
      default:
         syslog(LOG_ERR,"%s: Non-supported function type.",__func__);
         json_object_put(req);
         va_end(ap);
         return -1;
   }
   
   json_object_put(req);
   va_end(ap);
   return 0;
}
