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

#include "wambot_phys.h"



/* Some global things for timing the threads */
#define TS \
   T(UPDATE) \
   T(KINEMATICS) \
   T(REFGEN) \
   T(CONTROL) \
   T(TEACH) \
   T(GCOMP) \
   T(SETJTOR) \
   T(LOG)
#define T(x) TS_##x,
   enum ts_enum { TS TSNUM };
#undef T
#define T(x) #x,
   char * ts_name[] = { TS 0 };
#undef T



/* Here's the WAM realtime thread; it is only one thread, rt_wam(), which
 * calls rt_wam_create() and rm_wam_destroy() appropriately. */
static void rt_wam(struct bt_os_thread * thread);
static int rt_wam_create(struct bt_wam_local * wam, config_setting_t * wamconfig);
static void rt_wam_destroy(struct bt_wam_local * wam);

/* Here's the WAM non-realtime thread, which takes care of logging,
 * and perhaps some other things. */
static void nonrt_thread_function(struct bt_os_thread * thread);

/* The setup helper is for communication between the non-realtime create()
 * function and the realtime rt_wam() setup function. */
struct setup_helper
{
   struct bt_wam_local * wam;
   config_setting_t * config;
   int is_setup;
   int setup_failed;
};
static struct setup_helper * helper_create(struct bt_wam_local * wam, config_setting_t * config)
{
   struct setup_helper * helper;
   helper = (struct setup_helper *) malloc( sizeof(struct setup_helper) );
   if (!helper) return 0;
   helper->wam = wam;
   helper->config = config;
   helper->is_setup = 0;
   helper->setup_failed = 0;
   return helper;
}
static void helper_destroy(struct setup_helper * helper)
{
   free(helper);
   return;
}

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
   struct setup_helper * helper;
   
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
   wam->nonrt_thread = bt_os_thread_create(BT_OS_NONRT, "NONRTT", 10, nonrt_thread_function, (void *)wam);
   if (!wam->nonrt_thread)
   {
      syslog(LOG_ERR,"%s: Could not create non-realtime thread.",__func__);
      bt_wam_destroy((struct bt_wam *)wam);
      return 0;
   }

   /* Set up the timer */
   wam->ts = bt_os_timestat_create(TSNUM);
   if (!wam->ts)
   {
      syslog(LOG_ERR,"%s: Could not create timestat.",__func__);
      bt_wam_destroy((struct bt_wam *)wam);
      return 0;
   }
   
   /* Spin off the realtime thread to set everything up */
   helper = helper_create(wam,wamconfig);
   if (!helper)
   {
      syslog(LOG_ERR,"%s: Could not create setup helper.",__func__);
      bt_wam_destroy((struct bt_wam *)wam);
      return 0;
   }
   /* TODO: FIX THIS! */
   /*wam->rt_thread = bt_os_thread_create(BT_OS_RT, "CONTRL", 90, rt_wam, (void *)helper);*/
   wam->rt_thread = bt_os_thread_create(BT_OS_RT, wam->name, 90, rt_wam, (void *)helper);
   if (!wam->rt_thread)
   {
      syslog(LOG_ERR,"%s: Could not create realtime thread.",__func__);
      helper_destroy(helper);
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
      helper_destroy(helper);
      bt_wam_destroy((struct bt_wam *)wam);
      return 0;
   }
   
   /* Success! */
   helper_destroy(helper);
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

   /* Print timer means and variances */
   if (wam->ts)
   {
      int i;
      bt_os_rtime means[TSNUM];
      bt_os_rtime vars[TSNUM];
      bt_os_rtime mins[TSNUM];
      bt_os_rtime maxs[TSNUM];
      bt_os_timestat_get(wam->ts,means,vars,mins,maxs);
      syslog(LOG_ERR,"%15s:   us  ns      us  ns      min       max  \n","Timing Stats");
      syslog(LOG_ERR,"%15s--------------------------------------------\n","-------------");
      for (i=0; i<TSNUM; i++)
         syslog(LOG_ERR,"%15s: %4d %03d  +/-%3d %03d   %4d %03d  %4d %03d\n",
                ts_name[i],
                ((int)(means[i]))/1000,      ((int)(means[i]))%1000,
                ((int)(sqrt(vars[i])))/1000, ((int)(sqrt(vars[i])))%1000,
                ((int)(mins[i]))/1000,       ((int)(mins[i]))%1000,
                ((int)(maxs[i]))/1000,       ((int)(maxs[i]))%1000 );
      
      /* Destroy the timestat */
      bt_os_timestat_destroy(wam->ts);
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

char * bt_wam_local_str_con_position(struct bt_wam_local * wam, char * buf)
{
   if (!wam->con_active)
   {
      strcpy(buf,"none active");
      return buf;
   }
   return bt_gsl_vector_sprintf(buf,wam->con_active->position);
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

int bt_wam_local_controller_toggle(struct bt_wam_local * wam)
{
   int i, j;

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
      wam->con_active = wam->con_list[i];
      return 0;
   }

   /* If there is a loaded refgen, we need to find one with the same space */
   for (j=0; j<wam->con_num; j++)
   {
      if (strcmp(wam->con_active->type->space,wam->con_list[(i+j)%(wam->con_num)]->type->space)==0)
      {
         wam->con_active = wam->con_list[(i+j)%(wam->con_num)];
         return 0;
      }
   }

   return -1;
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

int bt_wam_local_control_use(struct bt_wam_local * wam, struct bt_control * control)
{
   /* If we're running a refgen, nope! */
   if (wam->refgen_active)
      return -1;

   /* If there's a refgen loaded, make sure the space is the same */
   if (strcmp(control->type->space,wam->con_active->type->space)!=0)
      return -1;

   /* OK fine, use the external controller */
   wam->con_active = control;
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



int bt_wam_local_refgen_addtype(struct bt_wam_local * wam, const struct bt_refgen_type * type)
{
   wam->refgen_types = (const struct bt_refgen_type **) realloc(wam->refgen_types,
                            (wam->refgen_types_num + 1)*sizeof(struct bt_refgen_type *));
   wam->refgen_types[wam->refgen_types_num] = type;
   wam->refgen_types_num++;
   return 0;
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


/* Move to a location, removing any currently-loaded refgen */
int bt_wam_local_moveto(struct bt_wam_local * wam, gsl_vector * dest)
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

int bt_wam_local_movehome(struct bt_wam_local * wam)
{
   return bt_wam_local_moveto(wam,wam->wambot->home);
}

int bt_wam_local_moveisdone(struct bt_wam_local * wam)
{
   return (wam->refgen_active) ? 0 : 1;
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


/* ===========================================================================
 * Below are separate thread stuffs */

static void rt_wam(struct bt_os_thread * thread)
{
   int err;
   struct setup_helper * helper;
   struct bt_wam_local * wam;
   
   helper = (struct setup_helper *) thread->data;
   wam = helper->wam;
   
   /* Initialize the WAM data structure (wambot, kinematics, gravity, etc) */
   err = rt_wam_create(helper->wam, helper->config);
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not create realtime WAM stuff.",__func__);
      helper->setup_failed = 1;
      helper->is_setup = 1;
      bt_os_thread_exit( thread );
      return;
   }
   
   /* Set the active controller to joint-space */
   wam->con_active = (struct bt_control *) wam->con_joint_legacy;
   
   /* Set velocity safety limit to 2.0 m/s */
   bt_bus_set_property(((struct bt_wambot_phys *)(wam->wambot))->bus, BT_BUS_PUCK_ID_WAMSAFETY,
                       ((struct bt_wambot_phys *)(wam->wambot))->bus->p->VL2, 1, 2.0 * 0x1000);
   /* CHECK RETURN VALUE */
   
   /* Set up the easy-access wam vectors */
   wam->jposition = wam->wambot->jposition;
   wam->jvelocity = wam->wambot->jvelocity;
   wam->jtorque = wam->wambot->jtorque;
   wam->cposition = wam->kin->tool->origin_pos;
   wam->cvelocity = wam->kin->tool_velocity;
   wam->crotation = wam->kin->tool->rot_to_world;
   
   /* Setup is complete! */
   helper->is_setup = 1;
   
   /* Note - the helper will now be destroyed for us,
    * and the create() function will return. */
   
   /* OK, start the control loop ... */
   bt_os_rt_make_periodic(0.002,"CONTRL"); /* Note - only call this once */
   /* CHECK RETURN VALUE */
   
   /* Loop until we're told by destroy() to exit */
   while (!bt_os_thread_isdone(thread))
   {
      double time;
      
      /* Wait for the next control period ... */
      bt_os_rt_set_mode_hard();
      bt_os_rt_task_wait_period();
      
      /* Skip the loop if we're not told to go */
      if (!wam->loop_go) continue;
      
      /* Start timing ... */
      bt_os_timestat_start(wam->ts);
      
      /* Get start-of-task time, increment counter */
      time = 1e-9 * bt_os_rt_get_time();
      if (!wam->count) wam->start_time = time;
      wam->count++;
      
      /* Grab the current joint positions */
      bt_wambot_update( wam->wambot );
      bt_os_timestat_trigger(wam->ts,TS_UPDATE);

      /* Evaluate kinematics
       * NOTE: Should this be common for all refgens/controllers?
       *       It's definitely needed for Barrett Dynamics,
       *       but this is encapsulated in Controllers right now ... */
      bt_kinematics_eval( wam->kin, wam->wambot->jposition, wam->wambot->jvelocity );
      bt_os_timestat_trigger(wam->ts,TS_KINEMATICS);
      
      /* Get the position from the current controller */
      bt_control_get_position(wam->con_active);

      /* Zero the torque 
       * (this is here in case a refgen wants to tweak it,
       * which really isn't what we should be doing ... */
      gsl_vector_set_zero( wam->wambot->jtorque );
      
      /* If there's an active trajectory, grab the reference into the joint controller
       * Note: this is a while loop for the case where the refgen is done,
       *       and we move on to the next refgen. */
      while (wam->refgen_active && !wam->teaching)
      {
         err = bt_refgen_eval( wam->refgen_active,
                               time - wam->start_time,
                               wam->con_active->reference );

         if (!err) break;
         
         if (err == 1) /* finished */
         {
            if ( (wam->refgen_active == wam->refgen_tempmove)
                && (wam->refgen_loaded) )
            {
               wam->start_time = time;
               bt_refgen_start(wam->refgen_loaded);
               wam->refgen_active = wam->refgen_loaded;
            }
            else
            {
               wam->refgen_active = 0;
            }
         }
      }
      bt_os_timestat_trigger(wam->ts,TS_REFGEN);

      /* Do the active controller */
      bt_control_eval( wam->con_active, wam->wambot->jtorque, time );
      bt_os_timestat_trigger(wam->ts,TS_CONTROL);

      /* If we're teaching, trigger the teach trajectory
       * (eventually we want to adjust the trigger rate?) */
      if (wam->teaching && wam->refgen_active && (((wam->count) & 0x4F) == 0) )
         bt_refgen_teach_trigger(wam->refgen_active,
                                 time - wam->start_time,
                                 wam->con_active->position);
      bt_os_timestat_trigger(wam->ts,TS_TEACH);
 
      /* Do gravity compensation (if flagged) */
      if (wam->gcomp) bt_calgrav_eval( wam->grav, wam->wambot->jtorque );
      bt_os_timestat_trigger(wam->ts,TS_GCOMP);

      /* Apply the current joint torques */
      bt_wambot_setjtor( wam->wambot );
      bt_os_timestat_trigger(wam->ts,TS_SETJTOR);
      
#if 0
      /* TEMP - ask puck 1 for its ID */
      {
         long val;
         bt_bus_get_property( ((struct bt_wambot_phys *)(wam->wambot))->bus, 1,
                              ((struct bt_wambot_phys *)(wam->wambot))->bus->p->ID,
                              &val );
      }
      bt_os_timestat_trigger(wam->ts,TS_GETP1);
#endif
            
      /* Log data (including timing statistics) */
      if (wam->log)
         bt_log_trigger( wam->log );
      if (wam->ts_log)
         bt_log_trigger( wam->ts_log );
      bt_os_timestat_trigger(wam->ts,TS_LOG);
      
      /* Calculate timing statistics */
      bt_os_timestat_end(wam->ts);
   }
   
   rt_wam_destroy(wam);
   
   /* Remove this thread from the realtime scheduler */
   bt_os_thread_exit( thread );
   
   return;
}

/* realtime WAM initialization stuff */
static int rt_wam_create(struct bt_wam_local * wam, config_setting_t * wamconfig)
{
#if 0
   int err;
   int i;
#endif
 
   /* Create a wambot object (which sets the dof)
    * NOTE - this should be configurable! */
   wam->wambot = (struct bt_wambot *) bt_wambot_phys_create( config_setting_get_member(wamconfig,"wambot") );
   if (!wam->wambot)
   {
      syslog(LOG_ERR,"%s: Could not create wambot.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
   
   /* Create a kinematics object */
   wam->kin = bt_kinematics_create( config_setting_get_member(wamconfig,"kinematics"), wam->wambot->dof );
   if (!wam->kin)
   {
      syslog(LOG_ERR,"%s: Could not create kinematics.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }

   /* Create a dynamics object */
   wam->dyn = bt_dynamics_create( config_setting_get_member(wamconfig,"dynamics"), wam->wambot->dof, wam->kin );
   if (!wam->dyn)
   {
      syslog(LOG_ERR,"%s: Could not create dynamics.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
   
   /* Create a gravity object */
   wam->grav = bt_calgrav_create( config_setting_get_member(wamconfig,"calgrav"), wam->kin );
   if (!wam->grav)
   {
      syslog(LOG_ERR,"%s: Could not create gravity compensation.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }

#if 0
   /* Create a datalogger
    * For now, we're just logging pos and accelerations */
   wam->log = bt_log_create( 3 );
   if (!wam->log)
   {
      syslog(LOG_ERR,"%s: Could not create logger.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
   
   /* Initialize logging fields */
   bt_log_addfield( wam->log, wam->wambot->jposition->data, wam->wambot->dof, BT_LOG_DOUBLE, "jpos");
   bt_log_addfield( wam->log, wam->wambot->jtorque->data, wam->wambot->dof, BT_LOG_DOUBLE, "jtorq");
   bt_log_addfield( wam->log, wam->wambot->jacceleration->data, wam->wambot->dof, BT_LOG_DOUBLE, "jacc");
   err = bt_log_init( wam->log, 1000, "datafile.dat");
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not initialize logging.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
   
   /* Create the ts datalogger */
   wam->ts_log = bt_log_create( TSNUM+1 );
   if (!wam->ts_log)
   {
      syslog(LOG_ERR,"%s: Could not create ts logger.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
   bt_log_addfield( wam->ts_log, wam->ts->start, 1, BT_LOG_ULONGLONG, "TS_START" );
   for (i=0; i<TSNUM; i++)
      bt_log_addfield( wam->ts_log, wam->ts->times + i, 1, BT_LOG_ULONGLONG, ts_name[i] );
   err = bt_log_init( wam->ts_log, 1000, "ts_log.dat");
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not initialize ts logging.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
#endif

   /* Create a joint-space controller */
   wam->con_joint = bt_control_joint_create(config_setting_get_member(wamconfig,"control_joint"),
                                            wam->dyn,
                                            wam->wambot->jposition, wam->wambot->jvelocity);
   if (!wam->con_joint)
   {
      syslog(LOG_ERR,"%s: Could not create joint-space controller.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }

   /* Create a legacy joint-space controller */
   wam->con_joint_legacy = bt_control_joint_legacy_create(config_setting_get_member(wamconfig,"control_joint_legacy"),
                                            wam->wambot->jposition, wam->wambot->jvelocity);
   if (!wam->con_joint_legacy)
   {
      syslog(LOG_ERR,"%s: Could not create legacy joint-space controller.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
   
   /* Create a Cartesian-xyz-space controller */
   wam->con_cartesian_xyz = bt_control_cartesian_xyz_create(config_setting_get_member(wamconfig,"control_cartesian_xyz"),
                                            wam->kin, wam->dyn );
   if (!wam->con_cartesian_xyz)
   {
      syslog(LOG_ERR,"%s: Could not create Cartesian-xyz-space controller.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
   
   /* Create a Cartesian-xyz-q-space controller */
   wam->con_cartesian_xyz_q = bt_control_cartesian_xyz_q_create(config_setting_get_member(wamconfig,"control_cartesian_xyz_q"),
                                            wam->kin, wam->dyn );
   if (!wam->con_cartesian_xyz_q)
   {
      syslog(LOG_ERR,"%s: Could not create Cartesian-xyz-q-space controller.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
   
   wam->con_list = (struct bt_control **) malloc((4)*sizeof(struct bt_control *));
   wam->con_list[0] = (struct bt_control *) wam->con_joint;
   wam->con_list[1] = (struct bt_control *) wam->con_joint_legacy;
   wam->con_list[2] = (struct bt_control *) wam->con_cartesian_xyz;
   wam->con_list[3] = (struct bt_control *) wam->con_cartesian_xyz_q;
   wam->con_num = 4;

   wam->refgen_types = (const struct bt_refgen_type **) malloc((2)*sizeof(struct bt_refgen_type *));
   wam->refgen_types[0] = bt_refgen_move;
   wam->refgen_types[1] = bt_refgen_teachplay;
   wam->refgen_types_num = 2;

   return 0;
}

static void rt_wam_destroy(struct bt_wam_local * wam)
{
   if (wam->con_list)
      free(wam->con_list);
   if (wam->refgen_types)
      free(wam->refgen_types);
   if (wam->con_joint)
      bt_control_joint_destroy(wam->con_joint);
   if (wam->con_joint_legacy)
      bt_control_joint_legacy_destroy(wam->con_joint_legacy);
   if (wam->con_cartesian_xyz)
      bt_control_cartesian_xyz_destroy(wam->con_cartesian_xyz);
   if (wam->con_cartesian_xyz_q)
      bt_control_cartesian_xyz_q_destroy(wam->con_cartesian_xyz_q);
   if (wam->ts_log)
      bt_log_destroy(wam->ts_log);
   if (wam->log)
      bt_log_destroy(wam->log);
   if (wam->grav)
      bt_calgrav_destroy(wam->grav);
   if (wam->dyn)
      bt_dynamics_destroy(wam->dyn);
   if (wam->kin)
      bt_kinematics_destroy(wam->kin);
   if (wam->wambot)
      bt_wambot_phys_destroy((struct bt_wambot_phys *)wam->wambot);
}

static void nonrt_thread_function(struct bt_os_thread * thread)
{
   struct bt_wam_local * wam = (struct bt_wam_local *) thread->data;
   
   while (!bt_os_thread_isdone(thread))
   {
      if (wam->log)
      {
         /*syslog(LOG_ERR,"Flushing Log Files ...");*/
         bt_log_flush( wam->log );
      }
      
      if (wam->refgen_active && wam->teaching)
      {
         bt_refgen_teach_flush(wam->refgen_active);
      }
      
      bt_os_usleep(1000000);
   }
   
   bt_os_thread_exit( thread );
}




/* WAM List stuff ---------------------------------------- */



