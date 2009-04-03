/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... file.c
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
 *
 *  REVISION HISTORY:
 *
 * ======================================================================== */

#define _GNU_SOURCE /* For getline() */

#include <stdlib.h>

#include <stdio.h>
#include <string.h>

#include <syslog.h>

#include "file_rpc.h"
#include "file.h"

#include "rpc.h"
#include "rpc_tcpjson.h"


struct bt_file
{
   struct bt_rpc_caller * caller;
   void * obj;
};

struct bt_file * bt_file_create(char * fileloc)
{
   /* Does it have a '/' character in it? */
   char prefixhost[100]; /* TODO size */
   char * sep;
   char * host;
   char * rname;
   
   strcpy(prefixhost,fileloc);
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
      struct bt_file * file;
      struct bt_rpc_caller * rpc_caller;
      void * obj;
      
      syslog(LOG_ERR,"%s: Opening remote file, rcp prefixhost %s, rname %s.",
             __func__,prefixhost,rname);
      
      /* Create the caller */
      rpc_caller = bt_rpc_caller_search_create(prefixhost,
                                               bt_rpc_tcpjson,
                                               0);
      if (!rpc_caller)
      {
         syslog(LOG_ERR,"%s: Could not create caller.",__func__);
         return 0;
      }
      
      /* Create file object */
      file = (struct bt_file *) malloc(sizeof(struct bt_file));
      if (!file)
      {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         bt_rpc_caller_destroy(rpc_caller);
         return 0;
      }
      
      /* Attempt to open the remote file */
      err = bt_rpc_caller_handle(rpc_caller,bt_file_rpc,"bt_file_create",rname,&obj);
      if (err || !obj)
      {
         syslog(LOG_ERR,"%s: Could not open remote file.",__func__);
         free(file);
         bt_rpc_caller_destroy(rpc_caller);
         return 0;
      }
      
      /* Initialize as remote */
      file->caller = rpc_caller;
      file->obj = obj;
      return file;
   }
   
#ifndef ASYNC_ONLY
   /* OK, it's local. If we're not ASYNC_ONLY, open locally ... */
   {
      struct bt_file * file;
      char fullpath[200]; /* TODO: Do this right! */
      
      /* Create file object */
      file = (struct bt_file *) malloc(sizeof(struct bt_file));
      if (!file)
      {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         return 0;
      }
      
      /* Open the file */
      /* TODO: Note how dangerous this is! 
       * What if fileloc were "../etc/passwd"! */
      sprintf(fullpath,"/root/%s",fileloc);
      file->obj = fopen(fullpath,"r+");
      if (!file->obj)
      {
         syslog(LOG_ERR,"%s: Could not open local file %s.",__func__,fullpath);
         free(file);
         return 0;
      }
      
      /* Initialize as local*/
      file->caller = 0;
      return file;
   }
#endif
   
   /* No file found.*/
   return 0;
}

int bt_file_destroy(struct bt_file * file)
{
#ifndef ASYNC_ONLY
   if (!file->caller)
   {
      fclose(file->obj);
      free(file);
      return 0;
   }
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(file->caller, bt_file_rpc, __func__, file->obj, &myint))
         return -1; /* Could not forward over RPC */
      bt_rpc_caller_destroy(file->caller);
      free(file);
      return myint;
   }
}

int bt_file_fputs(struct bt_file * file, char * line)
{
#ifndef ASYNC_ONLY
   if (!file->caller)
   {
      int err;
      err = fputs(line,file->obj);
      if (err == EOF) return -1;
      return 0;
   }
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(file->caller, bt_file_rpc, __func__, file->obj, line, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_file_fseek(struct bt_file * file, int offset, enum bt_file_seek seek)
{
#ifndef ASYNC_ONLY
   if (!file->caller)
   {
      int err;
      int whence;
      whence = (seek == BT_FILE_SEEK_SET) ? SEEK_SET :
                  ((seek == BT_FILE_SEEK_CUR) ? SEEK_CUR : SEEK_END);
      err = fseek(file->obj,offset,whence);
      if (err) return -1;
      return 0;
   }
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(file->caller, bt_file_rpc, __func__, file->obj, offset, seek, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}

int bt_file_getline(struct bt_file * file, char ** lineptr, int * linelen)
{
#ifndef ASYNC_ONLY
   if (!file->caller)
      return getline(lineptr,(size_t *)linelen,file->obj);
   else
#endif
   {
      int myint;
      if (bt_rpc_caller_handle(file->caller, bt_file_rpc, __func__, file->obj, lineptr, linelen, &myint))
         return -1; /* Could not forward over RPC */
      return myint;
   }
}




