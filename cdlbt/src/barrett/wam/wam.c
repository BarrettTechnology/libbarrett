
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <fcntl.h>    /* For open() */
#include <sys/stat.h> /* For file permissions */
#include <unistd.h>    /* For close(), getpid() */
#include <sys/types.h> /* For getpid() */

#include "wam.h"
#include "wam_rpc.h"
#include "../rpc/rpc_tcpjson.h"


#ifdef HAS_WAM_LOCAL
# include "wam_local.h"
# include "wam_list.h"
# include <libconfig.h> /* We should push this into wam_local.c ... */
#endif

#if 0
/* Eventaully, this will look like this ... */

#include "rpc.h"

#define RPC_FUNCS \
   CFUNC( bt_wam_create,   bt_wam_local_create,   struct bt_wam, P2(T(CHAR),T(INT)) ) \
    FUNC( bt_wam_destroy,  bt_wam_local_destroy,  T(INT),    P1(H(struct bt_wam))         ) \
    FUNC( bt_wam_do_thing, bt_wam_local_do_thing, T(INT),    P2(H(struct bt_wam), T(INT))  ) \
    FUNC( bt_wam_use,      bt_wam_local_use,      T(INT),    P2(H(struct bt_wam), H(struct bt_refgen)) ) \
    FUNC( bt_wam_goto,     bt_wam_local_goto,     T(INT),    P2(H(struct bt_wam), R(DOUBLE,CONST,IN,2,0)) ) \
    FUNC( bt_wam_getline,  bt_wam_local_getline,  T(INT),    P3(H(struct bt_wam), G(CHAR,OUT,100,2), R(INT,CONST,OUT,1,0)) ) \
    FUNC( bt_wam_getstr,   bt_wam_local_getstr,   R(DOUBLE,CONST,OUT,2,0), P1(H(struct bt_wam)) )

#include "rpc_generate_c.h"

#endif

/* This could conceivably go in another header file ... */

static char proxy_err[] = "(proxy-err)";

struct bt_wam
{
   struct bt_rpc_caller * caller;
   void * obj;
};


/* Just a shortcut */
int bt_wam_create(struct bt_wam ** wamptr, char * wamname)
{
   return bt_wam_create_opt(wamptr,wamname,0);
}

int bt_wam_create_opt(struct bt_wam ** wamptr, char * wamname, enum bt_wam_opt opts)
{
   /* Does it have a '/' character in it? */
   char prefixhost[100]; /* TODO size */
   char * sep;
   char * host;
   char * rname;

   (*wamptr) = 0;
   
   strcpy(prefixhost,wamname);
   host = 0;
   rname = 0;
   
   sep = strchr(prefixhost,':');
   if (sep && sep[1] == '/' && sep[2] == '/')
   {
      host = sep + 3;
      
      sep = strchr(host,'/');
      if (sep)
      {
         sep[0] = '\0';
         rname = sep + 1;
      }
   }
   
   /* If it's a network name, open over network */
   if (rname)
   {
      int err;
      struct bt_wam * wam;
      struct bt_rpc_caller * rpc_caller;
      void * obj;
      int myint;
      
      syslog(LOG_ERR,"%s: Opening remote wam, rcp prefixhost %s, rname %s.",
             __func__,prefixhost,rname);
      
      /* Create the caller */
      bt_rpc_caller_search_create(&rpc_caller, prefixhost,
                                  bt_rpc_tcpjson, 0);
      if (!rpc_caller)
      {
         syslog(LOG_ERR,"%s: Could not create caller.",__func__);
         return -1;
      }
      
      /* Create wam_proxy object, save caller, obj */
      wam = (struct bt_wam *) malloc(sizeof(struct bt_wam));
      if (!wam)
      {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         bt_rpc_caller_destroy(rpc_caller);
         return -1;
      }
      
      /* Attempt to open the remote wam */
      err = bt_rpc_caller_handle(rpc_caller,bt_wam_rpc,"bt_wam_create_opt",rname,opts,&obj,&myint);
      if (err || !obj)
      {
         syslog(LOG_ERR,"%s: Could not open remote WAM.",__func__);
         free(wam);
         bt_rpc_caller_destroy(rpc_caller);
         return -1;
      }
      
      /* Initialize */
      wam->caller = rpc_caller;
      wam->obj = obj;
      (*wamptr) = wam;
      return myint;
   }
   
#ifdef HAS_WAM_LOCAL
   /* OK, it's local. If we're not ASYNC_ONLY, open locally ... */
   {
      /* First, see if we have a local config file at {WAMCONFIGDIR}{NAME}.config */
      int err;
      struct bt_wam * wam;
      struct bt_wam_local * wam_local;
      
      syslog(LOG_ERR,"%s: Opening local wam %s.",__func__,wamname);
      
      /* Make the wam */
      wam = (struct bt_wam *) malloc(sizeof(struct bt_wam));
      if (!wam)
      {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         return -1;
      }

      err = bt_wam_local_create(&wam_local, wamname, opts);
      if (!wam_local)
      {
         free(wam);
         return -1;
      }
      
      wam->obj = (void *)wam_local;
      wam->caller = 0;
      (*wamptr) = wam;
      return err;
   }
#endif
   
   /* No wam found.*/
   return -1;
}

int bt_wam_destroy(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
   {
      bt_wam_local_destroy(wam->obj);
      free(wam);
      return 0;
   }
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      bt_rpc_caller_destroy(wam->caller);
      free(wam);
      return myint;
   }
}

struct bt_wam_local * bt_wam_get_local(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return wam->obj;
   else
#endif
   {
      return 0;
   }
}

int bt_wam_loop_start(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_loop_start(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_loop_stop(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_loop_stop(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_dof(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_dof(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

char * bt_wam_str_jposition(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_str_jposition(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

char * bt_wam_str_jvelocity(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_str_jvelocity(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

char * bt_wam_str_jtorque(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_str_jtorque(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

char * bt_wam_str_cposition(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_str_cposition(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

char * bt_wam_str_crotation_r1(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_str_crotation_r1(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

char * bt_wam_str_crotation_r2(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_str_crotation_r2(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

char * bt_wam_str_crotation_r3(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_str_crotation_r3(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}



int bt_wam_isgcomp(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_isgcomp(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_setgcomp(struct bt_wam * wam, int onoff)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_setgcomp(wam->obj,onoff);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, onoff, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}




int bt_wam_idle(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_idle(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_hold(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_hold(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_is_holding(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_is_holding(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}



char * bt_wam_str_con_position(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_str_con_position(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

char * bt_wam_get_current_controller_name(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_get_current_controller_name(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

char * bt_wam_get_current_controller_space(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_get_current_controller_space(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}


int bt_wam_controller_toggle(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_controller_toggle(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_control_use_name(struct bt_wam * wam, char * name)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_control_use_name(wam->obj,name);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, name, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_control_use_space(struct bt_wam * wam, char * space)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_control_use_space(wam->obj,space);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, space, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}






int bt_wam_refgen_clear(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_refgen_clear(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}


char * bt_wam_refgen_active_name(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_refgen_active_name(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

char * bt_wam_refgen_loaded_name(struct bt_wam * wam, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_refgen_loaded_name(wam->obj,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

int bt_wam_refgen_save(struct bt_wam * wam, char * filename)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_refgen_save(wam->obj,filename);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, filename, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_refgen_load(struct bt_wam * wam, char * filename)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_refgen_load(wam->obj,filename);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, filename, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}



int bt_wam_set_velocity(struct bt_wam * wam, double vel)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_set_velocity(wam->obj,vel);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, vel, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_set_acceleration(struct bt_wam * wam, double vel)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_set_velocity(wam->obj,vel);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, vel, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_moveto(struct bt_wam * wam, int n, double * dest)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_moveto(wam->obj,n,dest);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, n, dest, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_movehome(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_movehome(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_moveisdone(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_moveisdone(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}



int bt_wam_is_teaching(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_is_teaching(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_teach_start(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_teach_start(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_teach_end(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_teach_end(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}


int bt_wam_run(struct bt_wam * wam)
{
#ifdef HAS_WAM_LOCAL
   if (!wam->caller)
      return bt_wam_local_run(wam->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(wam->caller, bt_wam_rpc, __func__, wam->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}









struct bt_wam_list
{
   struct bt_rpc_caller * caller;
   void * obj;
};

int bt_wam_list_create(struct bt_wam_list ** listptr, char * wamloc)
{
   int err;
   struct bt_wam_list * list;
   struct bt_rpc_caller * rpc_caller;
   void * obj;
   int myint;
   
   /* Create */
   (*listptr) = 0;
   list = (struct bt_wam_list *) malloc(sizeof(struct bt_wam_list));
   if (!list)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return -1;
   }

   /* Handle the local case */
   if (!wamloc || strlen(wamloc) == 0)
   {
#ifdef HAS_WAM_LOCAL
      struct bt_wam_list_local * wam_list_local;
      err = bt_wam_list_local_create(&wam_list_local);
      if (!wam_list_local)
      {
         syslog(LOG_ERR,"%s: Could not create local wam list.",__func__);
         free(list);
         return -1;
      }
      list->obj = (void *)wam_list_local;
      list->caller = 0;
      (*listptr) = list;
      return err;
#endif
      free(list);
      return -1;
   }

   syslog(LOG_ERR,"%s: Opening remote wam list, rcp prefixhost %s.",__func__,wamloc);
   
   /* Create the caller */
   bt_rpc_caller_search_create(&rpc_caller, wamloc,
                               bt_rpc_tcpjson, 0);
   if (!rpc_caller)
   {
      syslog(LOG_ERR,"%s: Could not create caller.",__func__);
      free(list);
      return -1;
   }
   
   /* Attempt to open the remote wam list */
   err = bt_rpc_caller_handle(rpc_caller,bt_wam_rpc,"bt_wam_list_create","",&obj,&myint);
   if (err || !obj)
   {
      syslog(LOG_ERR,"%s: Could not open remote WAM list.",__func__);
      bt_rpc_caller_destroy(rpc_caller);
      free(list);
      return -1;
   }
   
   /* Initialize */
   list->caller = rpc_caller;
   list->obj = obj;
   (*listptr) = list;
   return myint;
}

int bt_wam_list_destroy(struct bt_wam_list * list)
{
#ifdef HAS_WAM_LOCAL
   if (!list->caller)
   {
      bt_wam_list_local_destroy(list->obj);
      free(list);
      return 0;
   }
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(list->caller, bt_wam_rpc, __func__, list->obj, &myint))
         return -1; /* Could not forward over RPC */
      bt_rpc_caller_destroy(list->caller);
      free(list);
      return myint;
   }
}

int bt_wam_list_get_num(struct bt_wam_list * list)
{
#ifdef HAS_WAM_LOCAL
   if (!list->caller)
      return bt_wam_list_local_get_num(list->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(list->caller, bt_wam_rpc, __func__, list->obj, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

char * bt_wam_list_get_name(struct bt_wam_list * list, int i, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!list->caller)
      return bt_wam_list_local_get_name(list->obj,i,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(list->caller, bt_wam_rpc, __func__, list->obj, i, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}

enum bt_wam_list_entry_status bt_wam_list_get_status(struct bt_wam_list * list, int i)
{
#ifdef HAS_WAM_LOCAL
   if (!list->caller)
      return bt_wam_list_local_get_status(list->obj,i);
   else
#endif
   {
      enum bt_wam_list_entry_status myint;
      if (bt_rpc_caller_handle(list->caller, bt_wam_rpc, __func__, list->obj, i, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_wam_list_get_pid(struct bt_wam_list * list, int i)
{
#ifdef HAS_WAM_LOCAL
   if (!list->caller)
      return bt_wam_list_local_get_pid(list->obj,i);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(list->caller, bt_wam_rpc, __func__, list->obj, i, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

char * bt_wam_list_get_programname(struct bt_wam_list * list, int i, char * buf)
{
#ifdef HAS_WAM_LOCAL
   if (!list->caller)
      return bt_wam_list_local_get_programname(list->obj,i,buf);
   else
#endif
   {
      if (bt_rpc_caller_handle(list->caller, bt_wam_rpc, __func__, list->obj, i, buf))
         return proxy_err; /* Could not forward over RPC */
      return buf;
   }
}


