/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wam.c
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

#include <stdlib.h> /* For popen(), pclose() */
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <math.h>     /* For sqrt() */

#include <fcntl.h>    /* For open() */
#include <sys/stat.h> /* For file permissions */
#include <unistd.h>    /* For close(), getpid() */
#include <sys/types.h> /* For getpid() */

#include <libconfig.h>
#include <syslog.h>

/*#include "wam.h"*/
#include "wam_local.h"

#include "gsl.h"



/* Local function prototype */
struct bt_wam_local * bt_wam_local_create_cfg(char * wamname, config_setting_t * wamconfig, enum bt_wam_opt opts);

/* Here we take care of the configuration file and lock files */
struct bt_wam_local * bt_wam_local_create(char * wamname, enum bt_wam_opt opts)
{
   /* First, see if we have a local config file at {WAMCONFIGDIR}{NAME}.config */
   char filename[WAMCONFIGDIRLEN+WAMNAMELEN+1];
   char lockfilename[WAMLOCKDIRLEN+WAMNAMELEN+1];
   char lockfiletxt[12];
   int err;
   struct config_t cfg;
   struct bt_wam_local * wam;
   
   syslog(LOG_ERR,"%s: Opening local wam %s.",__func__,wamname);
   
   strcpy(filename,WAMCONFIGDIR);
   strcat(filename,wamname);
   strcat(filename,".config");
   
   config_init(&cfg);
   syslog(LOG_ERR,"Attempting to open '%s' ...",filename);
   err = config_read_file(&cfg,filename);
   if (err != CONFIG_TRUE)
   {
      syslog(LOG_ERR,"libconfig error: %s, line %d\n",
          config_error_text(&cfg), config_error_line(&cfg));
      config_destroy(&cfg);
      /* Failure */
      return 0;
   }
   
   /* Check lock file */
   strcpy(lockfilename,WAMLOCKDIR);
   strcat(lockfilename,wamname);
   err = open(lockfilename,
              O_WRONLY | O_CREAT | O_EXCL,
              S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
   if (err == -1)
   {
      syslog(LOG_ERR,"%s: Lock file could not be created. Already in use?",__func__);
      config_destroy(&cfg);
      return 0;
   }
   
   /* Write the PID to the file, close the file */
   sprintf(lockfiletxt,"%10d\n",getpid());
   write(err,lockfiletxt,11);
   close(err);

   wam = bt_wam_local_create_cfg(wamname, config_lookup(&cfg,"wam"), opts);
   if (!wam)
   {
      /* Check if failed, unlink lock file */
      config_destroy(&cfg);
      unlink(lockfilename);
      return 0;
   }
   
   config_destroy(&cfg);
   return wam;
}

/* Here are the non-realtime create/destroy functions */
struct bt_wam_local * bt_wam_local_create_cfg(char * wamname, config_setting_t * wamconfig, enum bt_wam_opt opts)
{
   struct bt_wam_local * wam;
   struct bt_wam_thread_helper * helper;
   
   /* Set up realtime stuff */
   /* Allow hard real time process scheduling for non-root users */
   bt_os_rt_allow_nonroot();
   
   /* Make a new wam structure */
   wam = (struct bt_wam_local *) malloc( sizeof(struct bt_wam_local) );
   if (!wam)
   {
      syslog(LOG_ERR,"%s: No memory for a new WAM.",__func__);
      return 0;
   }
   
   /* Initialize the wam */
   strncpy(wam->name, wamname, WAMNAMELEN);
   wam->name[WAMNAMELEN] = '\0'; /* Not sure if we need this ... */
   wam->loop_go = 1;
   wam->gcomp = 0;
   wam->callback = 0;
   wam->vel = 0.5;
   wam->acc = 0.5;
   wam->refgen_tempmove = 0;
   wam->refgen_loaded = 0;
   wam->refgen_active = 0;
   wam->refgen_loaded_idestroy = 0;
   wam->refgen_types = 0;
   wam->refgen_types_num = 0;
   wam->teaching = 0;
   wam->count = 0;
   wam->ts = 0;
   wam->rt_thread = 0;
   wam->nonrt_thread = 0;
   
   /* Set anything in options */
   if (opts & BT_WAM_OPT_NO_LOOP_START)
      wam->loop_go = 0;
   
   /* Initialize the realtime stuff */
   wam->wambot = 0;
   wam->kin = 0;
   wam->dyn = 0;
   wam->grav = 0;
   wam->log = 0;
   wam->ts_log = 0;
   wam->con_joint = 0;
   wam->con_joint_legacy = 0;
   wam->con_cartesian_xyz = 0;
   wam->con_list = 0;

   /* Spin off the non-realtime log-saving thread */
   wam->nonrt_thread = bt_os_thread_create(BT_OS_NONRT, "NONRTT", 10, bt_wam_thread_nonrt, (void *)wam);
   if (!wam->nonrt_thread)
   {
      syslog(LOG_ERR,"%s: Could not create non-realtime thread.",__func__);
      bt_wam_destroy((struct bt_wam *)wam);
      return 0;
   }
   
   /* Spin off the realtime thread to set everything up */
   helper = bt_wam_thread_helper_create(wam,wamconfig);
   if (!helper)
   {
      syslog(LOG_ERR,"%s: Could not create setup helper.",__func__);
      bt_wam_destroy((struct bt_wam *)wam);
      return 0;
   }
   /* TODO: FIX THIS! */
   /*wam->rt_thread = bt_os_thread_create(BT_OS_RT, "CONTRL", 90, rt_wam, (void *)helper);*/
   wam->rt_thread = bt_os_thread_create(BT_OS_RT, wam->name, 90, bt_wam_thread, (void *)helper);
   if (!wam->rt_thread)
   {
      syslog(LOG_ERR,"%s: Could not create realtime thread.",__func__);
      bt_wam_thread_helper_destroy(helper);
      bt_wam_destroy((struct bt_wam *)wam);
      return 0;
   }
   
   /* Wait until the thread is done starting */
   while (!helper->is_setup)
      bt_os_usleep(10000);
   
   /* Check for setup failure */
   if (helper->setup_failed)
   {
      syslog(LOG_ERR,"%s: WAM realtime setup failed.",__func__);
      bt_wam_thread_helper_destroy(helper);
      bt_wam_destroy((struct bt_wam *)wam);
      return 0;
   }
   
   /* Success! */
   bt_wam_thread_helper_destroy(helper);
   return wam;
}

int bt_wam_local_destroy(struct bt_wam_local * wam)
{  
   char lockfilename[WAMLOCKDIRLEN+WAMNAMELEN+1];
   
   /* Tell the non-realtime thread to exit */
   if (wam->nonrt_thread)
   {
      bt_os_thread_stop(wam->nonrt_thread);
      bt_os_thread_destroy(wam->nonrt_thread);
   }
   
   if (wam->rt_thread)
   {
      bt_os_thread_stop(wam->rt_thread);
      bt_os_thread_destroy(wam->rt_thread);
   }
   
#if 0
   /* ATTEMPT TO Decode the binary log file */
   bt_log_decode_file("datafile.dat", "dat.oct", 1, 1); /* Woo octave! */
   bt_log_decode_file("ts_log.dat", "ts_log.csv", 1, 0); /* Header, no octave */
#endif

   /* Delete the lock file */
   strcpy(lockfilename,WAMLOCKDIR);
   strcat(lockfilename,wam->name);
   unlink(lockfilename);
   
   free(wam);
   
   return 0;
}

int bt_wam_local_loop_start(struct bt_wam_local * wam)
{
   wam->loop_go = 1;
   return 0;
}

int bt_wam_local_loop_stop(struct bt_wam_local * wam)
{
   wam->loop_go = 0;
   return 0;
}


/* String formatting functions */
char * bt_wam_local_str_jposition(struct bt_wam_local * wam, char * buf)
{
   return bt_gsl_vector_sprintf(buf,wam->jposition);
}

char * bt_wam_local_str_jvelocity(struct bt_wam_local * wam, char * buf)
{
   return bt_gsl_vector_sprintf(buf,wam->jvelocity);
}

char * bt_wam_local_str_jtorque(struct bt_wam_local * wam, char * buf)
{
   return bt_gsl_vector_sprintf(buf,wam->jtorque);
}

char * bt_wam_local_str_cposition(struct bt_wam_local * wam, char * buf)
{
   return bt_gsl_vector_sprintf(buf,wam->cposition);
}

char * bt_wam_local_str_crotation_r1(struct bt_wam_local * wam, char * buf)
{
   gsl_vector_view view;
   view = gsl_matrix_row(wam->crotation,0);
   return bt_gsl_vector_sprintf(buf,&view.vector);
}

char * bt_wam_local_str_crotation_r2(struct bt_wam_local * wam, char * buf)
{
   gsl_vector_view view;
   view = gsl_matrix_row(wam->crotation,1);
   return bt_gsl_vector_sprintf(buf,&view.vector);
}

char * bt_wam_local_str_crotation_r3(struct bt_wam_local * wam, char * buf)
{
   gsl_vector_view view;
   view = gsl_matrix_row(wam->crotation,2);
   return bt_gsl_vector_sprintf(buf,&view.vector);
}



/* Here are the asynchronous WAM functions */
int bt_wam_local_isgcomp(struct bt_wam_local * wam)
{
   return wam->gcomp;
}

int bt_wam_local_setgcomp(struct bt_wam_local * wam, int onoff)
{
   wam->gcomp = onoff ? 1 : 0;
   return 0;
}



/* Wrappers around the active controller */
int bt_wam_local_idle(struct bt_wam_local * wam)
{
   /* Make sure we're not doing any trajectories */
   wam->refgen_active = 0;
   return bt_control_idle(wam->con_active);
}

int bt_wam_local_hold(struct bt_wam_local * wam)
{
   /* Make sure we're not doing any trajectories */
   wam->refgen_active = 0;
   return bt_control_hold(wam->con_active);
}

int bt_wam_local_is_holding(struct bt_wam_local * wam)
{
   return bt_control_is_holding(wam->con_active);
}



char * bt_wam_local_str_con_position(struct bt_wam_local * wam, char * buf)
{
   if (!wam->con_active)
   {
      strcpy(buf,"none active");
      return buf;
   }
   return bt_gsl_vector_sprintf(buf,wam->con_active->position);
}

char * bt_wam_local_get_current_controller_name(struct bt_wam_local * wam, char * buf)
{
   if (!wam->con_active)
      strcpy(buf,"(none)");
   else
      strcpy(buf,wam->con_active->type->name);
   return buf;
}

char * bt_wam_local_get_current_controller_space(struct bt_wam_local * wam, char * buf)
{
   if (!wam->con_active)
      strcpy(buf,"(none)");
   else
      strcpy(buf,wam->con_active->type->space);
   return buf;
}


int bt_wam_local_controller_toggle(struct bt_wam_local * wam)
{
   int i, j;
   struct bt_control * con_new;

   con_new = 0;

   /* If we're running a refgen, nope! */
   if (wam->refgen_active)
      return -1;

   /* Get i to be the next controller in the list,
    * or 0 if con_active is a custom controller */
   for (i=0; i<wam->con_num; i++)
      if (wam->con_list[i] == wam->con_active)
      {
         i++;
         break;
      }
   if (i >= wam->con_num)
      i = 0;

   /* If there's no loaded refgen, just use the next available controller */
   if (!wam->refgen_loaded)
   {
      con_new = wam->con_list[i];
   }
   else
      /* If there is a loaded refgen, we need to find one with the same space */
      for (j=0; j<wam->con_num; j++)
      {
         if (strcmp(wam->con_active->type->space,wam->con_list[(i+j)%(wam->con_num)]->type->space)==0)
         {
            con_new = wam->con_list[(i+j)%(wam->con_num)];
            break;
         }
      }

   if (!con_new)
      return -1;

   /* Set new controller to same state as the old one */
   bt_control_idle(con_new);
   if (bt_control_is_holding(wam->con_active))
      bt_control_hold(con_new);

   wam->con_active = con_new;

   return 0;
}


int bt_wam_local_control_use(struct bt_wam_local * wam, struct bt_control * control)
{
   /* If we're running a refgen, nope! */
   if (wam->refgen_active)
      return -1;

   /* If there's a refgen loaded, make sure the space is the same */
   if (strcmp(control->type->space,wam->con_active->type->space)!=0)
      return -1;

   /* Set new controller to same state as the old one */
   bt_control_idle(control);
   if (bt_control_is_holding(wam->con_active))
      bt_control_hold(control);
   
   /* OK fine, use the external controller */
   wam->con_active = control;
   return 0;
}




/* Clear any loaded refgen */
int bt_wam_local_refgen_clear(struct bt_wam_local * wam)
{
   /* If we're currently running a refgen, stop it */
   if (wam->refgen_active)
      wam->refgen_active = 0;

   /* Remove the tempmove refgen, if it exists */
   if (wam->refgen_tempmove)
   {
      bt_refgen_destroy(wam->refgen_tempmove);
      wam->refgen_tempmove = 0;
   }

   /* Remove the loaded refgen, if it exists */
   if (wam->refgen_loaded)
   {
      if (wam->refgen_loaded_idestroy)
         bt_refgen_destroy(wam->refgen_loaded);
      wam->refgen_loaded = 0;
   }
   
   return 0;
}


char * bt_wam_local_refgen_active_name(struct bt_wam_local * wam, char * buf)
{
   if (!wam->refgen_active)
      strcpy(buf,"(none)");
   else
      strcpy(buf,wam->refgen_active->type->name);
   return buf;
}

char * bt_wam_local_refgen_loaded_name(struct bt_wam_local * wam, char * buf)
{
   if (!wam->refgen_loaded)
      strcpy(buf,"(none)");
   else
      strcpy(buf,wam->refgen_loaded->type->name);
   return buf;
}



int bt_wam_local_refgen_save(struct bt_wam_local * wam, char * filename)
{
   /* If we're currently using the refgen, don't try to save it */
   if (wam->refgen_active)
      return -1;

   /* If there isn't a loaded refgen, ignore */
   if (!wam->refgen_loaded)
      return -1;

   /* If the loaded refgen doesn't support load/save, ignore */
   if (!bt_refgen_has_loadsave(wam->refgen_loaded))
      return -1;

   /* Attempt to save */
   {
      config_t cfg;
      config_setting_t * root;
      config_setting_t * refgen;
      config_setting_t * refgen_data;
      config_setting_t * setting;
      
      config_init(&cfg);
      root = config_root_setting(&cfg);
      refgen = config_setting_add(root,"refgen",CONFIG_TYPE_GROUP);

      setting = config_setting_add(refgen,"type",CONFIG_TYPE_STRING);
      config_setting_set_string(setting,wam->refgen_loaded->type->name);

      setting = config_setting_add(refgen,"control",CONFIG_TYPE_STRING);
      config_setting_set_string(setting,wam->con_active->type->name);

      setting = config_setting_add(refgen,"control-space",CONFIG_TYPE_STRING);
      config_setting_set_string(setting,wam->con_active->type->space);
      
      refgen_data = config_setting_add(root,"refgen-data",CONFIG_TYPE_GROUP);
      bt_refgen_save(wam->refgen_loaded,refgen_data);

      config_write_file(&cfg,filename);

      config_destroy(&cfg);
   }

   return 0;
}

int bt_wam_local_refgen_load(struct bt_wam_local * wam, char * filename)
{
   int i;
   int err;
   config_t cfg;
   config_setting_t * root;
   config_setting_t * refgen;
   config_setting_t * refgen_data;
   const char * strtype;
   const char * strcontrol;
   const char * strspace;
   struct bt_control * con_match;
   const struct bt_refgen_type * refgen_type_match;
   struct bt_refgen * new_refgen;

   if (wam->refgen_active)
      return -1;

   /* First, try to load; if it works, overwrite any currently-loaded refgen */
   
   config_init(&cfg);
   err = config_read_file(&cfg,filename);
   if (err != CONFIG_TRUE)
   {
      config_destroy(&cfg);
      return -1;
   }

   root = config_root_setting(&cfg);

   refgen = config_setting_get_member(root,"refgen");
   refgen_data = config_setting_get_member(root,"refgen-data");

   config_setting_lookup_string(refgen,"type",&strtype);
   config_setting_lookup_string(refgen,"control",&strcontrol);
   config_setting_lookup_string(refgen,"control-space",&strspace);

   /* Find a matching controller and refgen type */
   con_match = 0;
   refgen_type_match = 0;

   /* Try to match based on controller name */
   if (strcmp(strcontrol,wam->con_active->type->name)==0)
      con_match = wam->con_active;
   else
      for (i=0; i<wam->con_num; i++)
         if (strcmp(strcontrol,wam->con_list[i]->type->name)==0)
         {
            con_match = wam->con_list[i];
            break;
         }

   /* If no match yet, try to match based on controller space */
   if (!con_match)
   {
      if (strcmp(strspace,wam->con_active->type->space)==0)
         con_match = wam->con_active;
      else
         for (i=0; i<wam->con_num; i++)
            if (strcmp(strspace,wam->con_list[i]->type->space)==0)
            {
               con_match = wam->con_list[i];
               break;
            }
   }

   /* Try to match refgen type */
   for (i=0; i<wam->refgen_types_num; i++)
      if (strcmp(strtype,wam->refgen_types[i]->name)==0)
      {
         refgen_type_match = wam->refgen_types[i];
      }

   /* If we didn't find matches, give up
    * It also must support creating blank refgens */
   if (!con_match || !refgen_type_match || !bt_refgen_has_create(refgen_type_match))
   {
      config_destroy(&cfg);
      return -1;
   }

   /* Create the new refgen */
   new_refgen = bt_refgen_create(refgen_type_match,con_match->n);
   if (!new_refgen)
   {
      config_destroy(&cfg);
      return -1;
   }

   /* Load the refgen with the refgen-data */
   if (!bt_refgen_has_loadsave(new_refgen))
   {
      config_destroy(&cfg);
      bt_refgen_destroy(new_refgen);
      return -1;
   }
   
   err = bt_refgen_load(new_refgen,refgen_data);
   if (err)
   {
      config_destroy(&cfg);
      bt_refgen_destroy(new_refgen);
      return -1;
   }

   /* OK, we have a loaded refgen; replace any existing ones in the wam,
    * and replace the controller as well! */
   bt_wam_local_refgen_clear(wam);
   
   if (bt_control_is_holding(wam->con_active))
      bt_control_hold(con_match);
   else
      bt_control_idle(con_match);
   wam->con_active = con_match;

   wam->refgen_loaded = new_refgen;
   wam->refgen_loaded_idestroy = 1;

   return 0;
}



int bt_wam_local_refgen_addtype(struct bt_wam_local * wam, const struct bt_refgen_type * type)
{
   wam->refgen_types = (const struct bt_refgen_type **) realloc(wam->refgen_types,
                            (wam->refgen_types_num + 1)*sizeof(struct bt_refgen_type *));
   wam->refgen_types[wam->refgen_types_num] = type;
   wam->refgen_types_num++;
   return 0;
}

/* Use an already-created refgen as the loaded refgen,
 * removing any currently-loaded refgen;
 * set idestroy = 0 */
int bt_wam_local_refgen_use(struct bt_wam_local * wam, struct bt_refgen * refgen)
{
   bt_wam_local_refgen_clear(wam);

   /* Set up the loaded refgen */
   wam->refgen_loaded = refgen;
   wam->refgen_loaded_idestroy = 0;
   
   return 0;   
}




int bt_wam_local_set_velocity(struct bt_wam_local * wam, double vel)
{
   wam->vel = vel;
   return 0;
}

int bt_wam_local_set_acceleration(struct bt_wam_local * wam, double acc)
{
   wam->acc = acc;
   return 0;
}

int bt_wam_local_moveto(struct bt_wam_local * wam, int n, double * dest)
{
   gsl_vector vecval;

   if (n != wam->con_active->n)
      return -1;

   vecval.size = n;
   vecval.stride = 1;
   vecval.data = dest;
   vecval.block = 0;
   vecval.owner = 0;

   return bt_wam_local_moveto_vec(wam,&vecval);
}


int bt_wam_local_movehome(struct bt_wam_local * wam)
{
   return bt_wam_local_moveto_vec(wam,wam->wambot->home);
}

int bt_wam_local_moveisdone(struct bt_wam_local * wam)
{
   return (wam->refgen_active) ? 0 : 1;
}

/* Move to a location, removing any currently-loaded refgen */
int bt_wam_local_moveto_vec(struct bt_wam_local * wam, gsl_vector * dest)
{
   bt_wam_local_refgen_clear(wam);

   /* This will make wam->con_active->reference the current position */
   bt_control_hold(wam->con_active); 
   
   /* Make the refgen itself */
#if 0
   wam->refgen_list->refgen = (struct bt_refgen *)
      bt_refgen_move_create(&(wam->elapsed_time), wam->jposition, wam->jvelocity, dest, wam->vel, wam->acc);
#endif
   wam->refgen_loaded = bt_refgen_move_create(
                                             wam->con_active->reference,
                                             0, dest, wam->vel, wam->acc);
   if (!wam->refgen_loaded) { return -1;}

   /* Save this refgen as the current one, and start it! */
   wam->start_time = 1e-9 * bt_os_rt_get_time();
   bt_refgen_start(wam->refgen_loaded);
   wam->refgen_active = wam->refgen_loaded;
   
   return 0;
}




int bt_wam_local_is_teaching(struct bt_wam_local * wam)
{
   return (wam->teaching) ? 1 : 0; 
}

int bt_wam_local_teach_start(struct bt_wam_local * wam)
{
   /* Make sure we're in the right mode */
   if (wam->refgen_active) return -1;
   if (wam->teaching) return -1;
   if (bt_control_is_holding(wam->con_active)) return -1;
   if (!wam->con_active) return -1;

   /* Delete any tempmove refgens */
   if (wam->refgen_tempmove)
   {
      bt_refgen_destroy(wam->refgen_tempmove);
      wam->refgen_tempmove = 0;
   }

   /* If there's already a loaded refgen,
    * we'll teach with it.  Otherwise, make a new
    * bt_refgen_teachplay */
   if (!wam->refgen_loaded)
   {
      wam->refgen_loaded = bt_refgen_create(bt_refgen_teachplay,wam->con_active->n);
      wam->refgen_loaded_idestroy = 1;
      if (!wam->refgen_loaded)
         return -1;
   }

   /* Initialize teaching */
   bt_refgen_teach_init(wam->refgen_loaded);

   /* Start teaching */
   bt_refgen_teach_start(wam->refgen_loaded);
   wam->start_time = 1e-9 * bt_os_rt_get_time();
   wam->teaching = 1;
   wam->refgen_active = wam->refgen_loaded;

   return 0;
}




int bt_wam_local_teach_end(struct bt_wam_local * wam)
{
   if (!wam->teaching) return -1;
   wam->teaching = 0;

   /* We should wait for it to finish teaching!!!! */

   wam->refgen_active = 0;

   /* End teaching (converting current refgen to playable refgen) */
   bt_refgen_teach_end(wam->refgen_loaded);
   
   return 0;
   
}


/* Run the loaded refgen */
int bt_wam_local_run(struct bt_wam_local * wam)
{
   gsl_vector * refgen_start;
   
   /* Make sure we're not currently teaching */
   if (wam->teaching) return 1;
   
   /* Make sure there's a loaded refgen */
   if ( ! wam->refgen_loaded )
      return -1;

   /* Delete any existing tempmove refgen */
   if (wam->refgen_tempmove)
   {
      bt_refgen_destroy(wam->refgen_tempmove);
      wam->refgen_tempmove = 0;
   }
   
   /* Get the starting position of the loaded refgen */
   bt_refgen_get_start(wam->refgen_loaded,&refgen_start);

   bt_control_hold(wam->con_active);
   
   /* Create a tempmove refgen to get us there */
#if 0
   wam->refgen_list->refgen = (struct bt_refgen *)
      bt_refgen_move_create(&(wam->elapsed_time), wam->jposition, wam->jvelocity,
                                   teachplay_start, wam->vel, wam->acc);
#endif
   wam->refgen_tempmove = (struct bt_refgen *)
      bt_refgen_move_create(wam->con_active->reference, 0,
                            refgen_start, wam->vel, wam->acc);
   
   if (!wam->refgen_tempmove) { return -1;}
   /* Note, we should free more stuff here! */
   
   /* Save this refgen as the current one, and start it! */
   wam->start_time = 1e-9 * bt_os_rt_get_time();
   bt_refgen_start( wam->refgen_tempmove );
   wam->refgen_active = wam->refgen_tempmove;
   
   return 0;
}

int bt_wam_local_set_callback(struct bt_wam_local * wam,
                              int (*callback)(struct bt_wam_local * wam))
{
   wam->callback = callback;
   return 0;
}

