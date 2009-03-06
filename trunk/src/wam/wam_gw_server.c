/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wam_gw_server.c
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

/* System Libraries */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h> /* For select() */
/*#include <sys/mman.h>*/
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h> /* For inet_ntoa() */

#include <syslog.h>
#include <json/json.h>

#include "wam_gw_server.h"

#include "wam_internal.h"

#define PORT 1338
#define BUFLEN 1023 /* No requests are allowed to be longer than this */


/* Here comes some magic! */
enum func_type
{
   FUNC_TYPE_INT_FROM_WAM,
   FUNC_TYPE_STR_FROM_WAM,
   FUNC_TYPE_INT_FROM_WAM_INT,
   FUNC_TYPE_INT_FROM_WAM_DOUBLE,
   FUNC_TYPE_INT_FROM_WAM_VECTOR
};

struct func_definition
{
   void (*func)();
   char name[30];
   enum func_type type;
};

/* Note: bt_wam_create() is dealt with separately */
static struct func_definition funcs[] = 
{
   {(void (*)())&bt_wam_isgcomp,     "bt_wam_isgcomp",     FUNC_TYPE_INT_FROM_WAM},
   {(void (*)())&bt_wam_setgcomp,    "bt_wam_setgcomp",    FUNC_TYPE_INT_FROM_WAM_INT},
   {(void (*)())&bt_wam_teach_start, "bt_wam_teach_start", FUNC_TYPE_INT_FROM_WAM},
   {(void (*)())&bt_wam_teach_end,   "bt_wam_teach_end",   FUNC_TYPE_INT_FROM_WAM},
   {(void (*)())&bt_wam_playback,    "bt_wam_playback",    FUNC_TYPE_INT_FROM_WAM},
   {0,{0},0}
};


/* Private function prototypes */
static int msg_parse(struct bt_wam_gw_server * gw, int conn, char * msg);


/* Public functions */
struct bt_wam_gw_server * bt_wam_gw_server_create()
{
   int err;
   struct bt_wam_gw_server * gw;
   struct sockaddr_in listener_addr;
   
   /* Create the gw_server structure itself,
    * with 0 active connections */
   gw = (struct bt_wam_gw_server *) malloc( sizeof(struct bt_wam_gw_server) );
   if (!gw) return 0;
   gw->conns = 0;
   gw->conns_num = 0;
   gw->wams = 0;
   gw->wams_num = 0;
#if 0
   /* Make a new printbuf object for JSON parsing */
   gw->pb = printbuf_new();
   if (!gw->pb)
   {
      syslog(LOG_ERR,"could not create JSON print buffer.");
      free(gw);
      return 0;
   }
#endif
   /* Start listening on a port for incoming connections ... */
   gw->listener = socket(PF_INET, SOCK_STREAM, 0);
   if (gw->listener < 0)
   {
      syslog(LOG_ERR,"could not create socket.");
      free(gw);
      return 0;
   }
   
   /* Bind to a port */
   listener_addr.sin_family = AF_INET;
   listener_addr.sin_port = htons(PORT);
   listener_addr.sin_addr.s_addr = htonl(INADDR_ANY);
   err = bind(gw->listener, (struct sockaddr *)&listener_addr, sizeof(listener_addr));
   if (err)
   {
      syslog(LOG_ERR,"could not bind to port %d.",PORT);
      free(gw);
      return 0;
   }

   /* Start listening */
   err = listen( gw->listener, 10 );
   if (err)
   {
      syslog(LOG_ERR,"could not listen on socket.");
      free(gw);
      return 0;
   }
   
   return gw;
}

int bt_wam_gw_server_fdset(struct bt_wam_gw_server * gw, fd_set * setptr)
{
   int i;
   FD_SET(gw->listener,setptr);
   for (i=0; i<gw->conns_num; i++)
      FD_SET(gw->conns[i].sock,setptr);
   return 0;
}

/* This function handles buffers and the like, and calls
 * the msg_parse() function below. */
int bt_wam_gw_server_handle(struct bt_wam_gw_server * gw, fd_set * setptr)
{
   int i;
   int err;
   
   /* Accept new connections */
   if (FD_ISSET(gw->listener,setptr))
   {
      int new;
      struct sockaddr_in new_addr;
      socklen_t new_addr_size;
      
      new_addr_size = sizeof(new_addr);
      new = accept(gw->listener, (struct sockaddr *)&new_addr, &new_addr_size);
      if (new < 0)
      {
         syslog(LOG_ERR,"could not accept new connection");
      }
      else
      {
         syslog(LOG_ERR,"New connection from host %s, port %hd.",
                inet_ntoa(new_addr.sin_addr), ntohs(new_addr.sin_port));
         
         if (!gw->conns_num)
            gw->conns = (struct connection *) malloc( sizeof(struct connection) );
         else
            gw->conns = (struct connection *) realloc( gw->conns, (gw->conns_num+1) * sizeof(struct connection) );
         gw->conns[gw->conns_num].sock = new;
         gw->conns[gw->conns_num].buf = (char *) malloc( (BUFLEN+1) * sizeof(char) );
         gw->conns[gw->conns_num].buf_already = 0;
         gw->conns_num++;
      }
   }
   
   /* Handle incoming messages from existing connections */
   for (i=0; i<gw->conns_num; i++)
   if (FD_ISSET(gw->conns[i].sock,setptr))
   {
      /* Read into the buffer */
      err = read( gw->conns[i].sock, gw->conns[i].buf + gw->conns[i].buf_already,
                  BUFLEN - gw->conns[i].buf_already );
      if (err < 0)
      {
         syslog(LOG_ERR,"read() error!");
         break;
      }
      if (err == 0)
      {
         int j;
         syslog(LOG_ERR,"closing connection.");
         close( gw->conns[i].sock );
         free( gw->conns[i].buf );
         for (j=i; j<gw->conns_num-1; i++)
            gw->conns[j] = gw->conns[j+1];
         if (gw->conns_num == 1)
            free(gw->conns);
         else
            gw->conns = (struct connection *) realloc( gw->conns, (gw->conns_num-1) * sizeof(struct connection) );
         gw->conns_num--;
      }
      
      gw->conns[i].buf_already += err;
      if (gw->conns[i].buf_already == BUFLEN)
      {
         syslog(LOG_ERR,"buffer full!");
         /* For now, just give up. */
         gw->conns[i].buf_already = 0;
         break;
      }
      
      /* Read as many messages as we can */
      while (1)
      {
         int j;
         char * end;
         int msg_len; /* including inserted trailing '\0' */
         
         /* Note: this will spuriously find newlines in strings! */
         end = memchr( gw->conns[i].buf, '\n', gw->conns[i].buf_already );
         if (!end)
            break; /* No newline yet found */
         
         (*end) = '\0';
         msg_len = end - gw->conns[i].buf + 1;
         
         /* Parse the message, and perhaps send a response */
         msg_parse(gw, i, gw->conns[i].buf);
         
         for (j=0; j<gw->conns[i].buf_already-msg_len; j++)
            gw->conns[i].buf[j] = gw->conns[i].buf[msg_len+j];
         
         gw->conns[i].buf_already -= msg_len;
      }
   }
   
   return 0;
}

int bt_wam_gw_server_destroy(struct bt_wam_gw_server * gw)
{
   int i;
   for (i=0; i<gw->conns_num; i++)
   {
      close( gw->conns[i].sock );
      free( gw->conns[i].buf );
   }
   close(gw->listener);
   free(gw->conns);
   free(gw);
   return 0;
}


/* Private functions */
static int msg_parse(struct bt_wam_gw_server * gw, int conn, char * msg)
{
   struct json_object * req;
   char * method;
   struct json_object * params;
   char * wamname;
   struct bt_wam * wam;
   int i;
   
   req = json_tokener_parse(msg);
   if (is_error(req))
   {
      printf("Error parsing request, pointer is %d.\n",(int)req);
      return -1;
   }
   
   /* Make sure it's an object */
   if (!json_object_is_type(req,json_type_object))
   {
      printf("Request is not a JSON object.\n");
      json_object_put(req);
      return -1;
   }
   
   /* Get the string associated with 'method' */
   method = json_object_get_string(json_object_object_get(req,"method"));
   if (!method)
   {
      printf("Request does not have a 'method' entry.\n");
      json_object_put(req);
      return -1;
   }
   
   /* Check for params */
   if (   !(params = json_object_object_get(req,"params"))
       || !json_object_get_array(params))
   {
      printf("Request does not have a 'params' array.\n");
      json_object_put(req);
      return -1;
   }
   
   /* bt_wam_create, special case ... */
   if (strcmp(method,"bt_wam_create")==0)
   {      
      /* Check for one parameter */
      if (json_object_array_length(params)!=1)
      {
         printf("Request does not have one parameter.\n");
         json_object_put(req);
         return -1;
      }
      
      /* Get the parameter, a string */
      wamname = json_object_get_string(json_object_array_get_idx(params,0));
      if (!wamname)
      {
         printf("Request does not have a string parameter.\n");
         json_object_put(req);
         return -1;
      }
      
      /* Attempt to open this wam name! */
      wam = bt_wam_create(wamname);
      if (!wam)
      {
         int err;
         /* Return failure */
         err = write( gw->conns[conn].sock, "{\"result\":false}\n",
                                  strlen("{\"result\":false}\n") );
         /* What should we do with this error??? */
         json_object_put(req);
         return -1;
      }
      
      /* Save the WAM in our local list */
      if (!gw->wams_num)
         gw->wams = (struct bt_wam **) malloc(sizeof(struct bt_wam *));
      else
         gw->wams = (struct bt_wam **) realloc(gw->wams,(gw->wams_num+1)*sizeof(struct bt_wam *));
      if (!gw->wams)
      {
         printf("Memory allocation error.\n");
         bt_wam_destroy(wam);
         json_object_put(req);
         return -1;
      }
      gw->wams[gw->wams_num] = wam;
      gw->wams_num++;
      
      /* Return success */
      {
         int err;
         /* Return failure */
         err = write( gw->conns[conn].sock, "{\"result\":true}\n",
                                  strlen("{\"result\":true}\n") );
         /* What should we do with this error??? */
      }
      
      json_object_put(req);
      return 0;
   }
   
   /* Check that the wam name is the first arg */
   if (json_object_array_length(params) < 1)
   {
      printf("Request does not have at least one parameter.\n");
      json_object_put(req);
      return -1;
   }
   /* Get the parameter, a string */
   wamname = json_object_get_string(json_object_array_get_idx(params,0));
   if (!wamname)
   {
      printf("Request does not have a wam name string parameter.\n");
      json_object_put(req);
      return -1;
   }
   /* Perform lookup */
   wam = 0;
   for (i=0; i<gw->wams_num; i++)
   if (strcmp(gw->wams[i]->name,wamname)==0)
   {
      wam = gw->wams[i];
      break;
   }
   if (!wam)
   {
      printf("That wam is not currently open.\n");
      json_object_put(req);
      return -1;
   }
   
   /* Do the destroy specially too */
   if (strcmp(method,"bt_wam_destroy")==0)
   {
      bt_wam_destroy(wam);
      /* Remove from the list */
      for (i=0; i<gw->wams_num; i++)
      if (gw->wams[i] == wam)
      {
         int j;
         for (j=i; j<gw->wams_num-1; j++)
            gw->wams[j] = gw->wams[j+1];
         break;
      }
      gw->wams_num--;
      if (!gw->wams_num)
         free(gw->wams);
      else
         gw->wams = (struct bt_wam **) realloc( gw->wams, gw->wams_num * sizeof(struct bt_wam *) );
      /* Check for errors here? */
      
      /* Return success */
      {
         int err;
         /* Return failure */
         err = write( gw->conns[conn].sock, "{\"result\":true}\n",
                                  strlen("{\"result\":true}\n") );
         /* What should we do with this error??? */
      }
      
      json_object_put(req);
      return 0;
   }
   
   /* Check all other functions, which have the wam as the first arg */
   for (i=0; funcs[i].func; i++)
   if (strcmp(method,funcs[i].name)==0)
   {
      int an_int;
      int ret;
      switch (funcs[i].type)
      {
         case FUNC_TYPE_INT_FROM_WAM:
            /* Cast and call! */
            ret = (*(int (*)(struct bt_wam *))(funcs[i].func))(wam);
            /* Return the result */
            sprintf(gw->writebuf,"{\"result\":%d}\n",ret);
            write( gw->conns[conn].sock, gw->writebuf, strlen(gw->writebuf) );
            /* What should we do with this error??? */
            json_object_put(req);
            return 0;
         case FUNC_TYPE_INT_FROM_WAM_INT:
            /* Get the int */
            if (json_object_array_length(params) != 2)
            {
               printf("Method \"%s\" not enough parameters.\n",method);
               json_object_put(req);
               return -1;
            }
            an_int = json_object_get_int(json_object_array_get_idx(params,1));
            /* Cast and call! */
            ret = (*(int (*)(struct bt_wam *, int))(funcs[i].func))(wam, an_int);
            /* Return the result */
            sprintf(gw->writebuf,"{\"result\":%d}\n",ret);
            write( gw->conns[conn].sock, gw->writebuf, strlen(gw->writebuf) );
            /* What should we do with this error??? */
            json_object_put(req);
            return 0;
         default:
            printf("Method \"%s\" not implemented.\n",method);
            json_object_put(req);
            return -1;
      }
      /* Remove the wam pointer if this was bt_wam_destroy() */
      break;
   }
   
   printf("Method \"%s\" not found.\n",method);
   json_object_put(req);
   return 0;
}
