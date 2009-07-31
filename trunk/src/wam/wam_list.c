
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <syslog.h>
#include <dirent.h>   /* For directories */

#include "wam_list.h"

int bt_wam_list_local_create(struct bt_wam_list_local ** listptr)
{
   struct bt_wam_list_local * list;
   DIR * etcwam;
   struct dirent * file;
   
   /* Create */
   (*listptr) = 0;
   list = (struct bt_wam_list_local *) malloc(sizeof(struct bt_wam_list_local));
   if (!list)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return -1;
   }
   
   /* Initialize */
   list->entries = 0;
   list->num = 0;
   
   /* Open the /etc/wam directory */
   etcwam = opendir(WAMCONFIGDIR);
   if (!etcwam)
   {
      syslog(LOG_ERR,"%s: Directory %s not found.",__func__,WAMCONFIGDIR);
      (*listptr) = list;
      return 0;
   }
   
   /* Note: this is not thread-safe. */
   while ((file = readdir(etcwam)))
   {
      int n;
      /* Check if the name is ***.config */
      n = strlen(file->d_name);
      if (n < 8) continue;
      if (strcmp(".config",file->d_name + n - 7)==0)
      {
         struct bt_wam_list_entry * entry;
         char lockfilename[WAMLOCKDIRLEN+WAMNAMELEN+1];
         FILE * lockfile;
         char pscommand[100];
         FILE * pspipe;
         int err;
         
         entry = (struct bt_wam_list_entry *) malloc(sizeof(struct bt_wam_list_entry));
         if (!list->num)
            list->entries = (struct bt_wam_list_entry **)
                            malloc(sizeof(struct bt_wam_list_entry *));
         else
            list->entries = (struct bt_wam_list_entry **)
                            realloc(list->entries,(list->num+1)*sizeof(struct bt_wam_list_entry *));
         list->entries[list->num] = entry;
         list->num++;
         
         strncpy(entry->name,file->d_name,n-7);
         entry->name[n-7] = '\0';
         
         /* Attempt to read the lock file */
         strcpy(lockfilename,WAMLOCKDIR);
         strcat(lockfilename,entry->name);
         lockfile = fopen(lockfilename,"r");
         if (!lockfile)
         {
            entry->status = BT_WAM_LIST_ENTRY_STATUS_FREE;
            entry->programpid = 0;
            strcpy(entry->programname,"(none)");
            continue;
         }
         
         /* Copy in the pid */
         err = fscanf(lockfile,"%10d\n",&(entry->programpid));
         if (err != 1)
         {
            entry->status = BT_WAM_LIST_ENTRY_STATUS_DEFUNCT;
            entry->programpid = 0;
            strcpy(entry->programname,"(none)");
            fclose(lockfile);
            continue;
         }
         
         /* Attempt to grab the program name from ps */
         sprintf(pscommand,"ps -p %d -o comm=",entry->programpid);
         pspipe = popen(pscommand,"r");
         if (!pspipe)
         {
            entry->status = BT_WAM_LIST_ENTRY_STATUS_INUSE;
            strcpy(entry->programname,"(unknown)");
            fclose(lockfile);
            continue;
         }
         
         err = fscanf(pspipe,"%s\n",entry->programname);
         if (err != 1)
         {
            entry->status = BT_WAM_LIST_ENTRY_STATUS_DEFUNCT;
            strcpy(entry->programname,"(unknown)");
            pclose(pspipe);
            fclose(lockfile);
            continue;
         }
         
         entry->status = BT_WAM_LIST_ENTRY_STATUS_INUSE;
         
         pclose(pspipe);
         fclose(lockfile);
      }
   }
   
   closedir(etcwam);
   (*listptr) = list;
   return 0;
}

int bt_wam_list_local_destroy(struct bt_wam_list_local * list)
{
   int i;
   
   for (i=0; i<list->num; i++)
      free(list->entries[i]);
   if (list->num)
      free(list->entries);
   free(list);
   return 0;
}

int bt_wam_list_local_get_num(struct bt_wam_list_local * list)
{
   return list->num;
}

char * bt_wam_list_local_get_name(struct bt_wam_list_local * list, int i, char * buf)
{
   if (i >= list->num)
      return 0;
   strcpy(buf,list->entries[i]->name);
   return buf;
}

enum bt_wam_list_entry_status bt_wam_list_local_get_status(struct bt_wam_list_local * list, int i)
{
   if (i >= list->num)
      return -1;
   return list->entries[i]->status;
}

int bt_wam_list_local_get_pid(struct bt_wam_list_local * list, int i)
{
   if (i >= list->num)
      return -1;
   return list->entries[i]->programpid;
}

char * bt_wam_list_local_get_programname(struct bt_wam_list_local * list, int i, char * buf)
{
   if (i >= list->num)
      return 0;
   strcpy(buf,list->entries[i]->programname);
   return buf;
}


