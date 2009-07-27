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

#include <dirent.h>   /* For directories */
#include <fcntl.h>    /* For open() */
#include <sys/stat.h> /* For file permissions */
#include <unistd.h>    /* For close(), getpid() */
#include <sys/types.h> /* For getpid() */

#include <libconfig.h>
#include <syslog.h>

/*#include "wam.h"*/
#include "wam_local.h"
#include "wam_custom.h"

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
   wam->refgen_list = 0;
   wam->refgen_current = 0;
   wam->teach = 0;
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
   bt_log_decode("datafile.dat", "dat.oct", 1, 1); /* Woo octave! */
   bt_log_decode("ts_log.dat", "ts_log.csv", 1, 0); /* Header, no octave */
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
   int i;
   
   for (i=0; i<wam->con_num; i++)
   if (wam->con_list[i] == wam->con_active)
   {
      if (i+1 < wam->con_num)
         wam->con_active = wam->con_list[i+1];
      else
         wam->con_active = wam->con_list[0];
      break;
   }
   return 0;
}

/* Wrappers around the active controller */
int bt_wam_local_idle(struct bt_wam_local * wam)
{
   /* Make sure we're not doing any trajectories */
   wam->refgen_current = 0;
   return bt_control_idle(wam->con_active);
}

int bt_wam_local_hold(struct bt_wam_local * wam)
{
   /* Make sure we're not doing any trajectories */
   wam->refgen_current = 0;
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

char * bt_wam_local_get_current_refgen_name(struct bt_wam_local * wam, char * buf)
{
   if (!wam->refgen_current)
      strcpy(buf,"(none)");
   else
      strcpy(buf,wam->refgen_current->refgen->type->name);
   return buf;
}

int bt_wam_local_refgen_use(struct bt_wam_local * wam, struct bt_refgen * refgen)
{
   struct bt_wam_refgen_list * refgen_list_element;
   gsl_vector * refgen_start;
   
   /* Make sure we're in the right mode */
   if (wam->refgen_current) return -1;
   if (wam->teach) return -1;
   if (!wam->con_active) return -1;
   
   /* Remove any refgens in the list */
   {
      struct bt_wam_refgen_list * refgen_list;
      struct bt_wam_refgen_list * refgen_list_next;
      
      refgen_list = wam->refgen_list;
      while (refgen_list)
      {
         refgen_list_next = refgen_list->next;
         /* respect iown, idelete? */
         if (refgen_list->iown)
            bt_refgen_destroy(refgen_list->refgen);
         free(refgen_list);
         refgen_list = refgen_list_next;
      }
      wam->refgen_list = 0;
   }
   
   /* Make a new refgen list element for the refgen */
   wam->refgen_list = (struct bt_wam_refgen_list *) malloc( sizeof(struct bt_wam_refgen_list) );
   if (!wam->refgen_list) return -1;
   wam->refgen_list->next = 0;
   wam->refgen_list->iown = 0;
   wam->refgen_list->refgen = refgen;
   
   /* Insert the move list element before the refgen */
   refgen_list_element = wam->refgen_list;
   wam->refgen_list = (struct bt_wam_refgen_list *) malloc( sizeof(struct bt_wam_refgen_list) );
   if (!wam->refgen_list) return -1;
   wam->refgen_list->next = refgen_list_element;
   wam->refgen_list->iown = 1;
   
   /* Get the refgen starting point */
   syslog(LOG_ERR,"inside Type: %d",(int)(refgen->type));
   bt_os_usleep(100000);
   bt_refgen_get_start(refgen,&refgen_start);
   
   /* Create the move refgen itself */
   wam->refgen_list->refgen = (struct bt_refgen *)
      bt_refgen_move_create(&(wam->elapsed_time),
                            wam->con_active->position, 0,
                            refgen_start, wam->vel, wam->acc);
   
   if (!wam->refgen_list->refgen) {free(wam->refgen_list); wam->refgen_list=0; return -1;}
   /* Note, we should free more stuff here! */
   
   /* Save this refgen as the current one, and start it! */
   bt_control_hold(wam->con_active);
   wam->start_time = 1e-9 * bt_os_rt_get_time();
   bt_refgen_start( wam->refgen_list->refgen );
   wam->refgen_current = wam->refgen_list;
   
   return 0;   
}

int bt_wam_local_control_use(struct bt_wam_local * wam, struct bt_control * control)
{
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

int bt_wam_local_moveto(struct bt_wam_local * wam, gsl_vector * dest)
{
   /* Remove any refgens in the list */
   {
      struct bt_wam_refgen_list * refgen_list;
      struct bt_wam_refgen_list * refgen_list_next;
      
      refgen_list = wam->refgen_list;
      while (refgen_list)
      {
         refgen_list_next = refgen_list->next;
         /* respect iown, idelete? */
         if (refgen_list->iown)
            bt_refgen_destroy(refgen_list->refgen);
         free(refgen_list);
         refgen_list = refgen_list_next;
      }
      wam->refgen_list = 0;
   }
   
   /* Make a new refgen list element */
   wam->refgen_list = (struct bt_wam_refgen_list *) malloc( sizeof(struct bt_wam_refgen_list) );
   if (!wam->refgen_list) return -1;
   wam->refgen_list->next = 0;
   wam->refgen_list->iown = 1;
   
   /* Make the refgen itself */
#if 0
   wam->refgen_list->refgen = (struct bt_refgen *)
      bt_refgen_move_create(&(wam->elapsed_time), wam->jposition, wam->jvelocity, dest, wam->vel, wam->acc);
#endif
   wam->refgen_list->refgen = (struct bt_refgen *)
      bt_refgen_move_create(&(wam->elapsed_time), wam->con_active->reference, 0, dest, wam->vel, wam->acc);
   if (!wam->refgen_list->refgen) {free(wam->refgen_list); wam->refgen_list=0; return -1;}
   
   /* Save this refgen as the current one, and start it! */
   bt_control_hold(wam->con_active);
   wam->start_time = 1e-9 * bt_os_rt_get_time();
   bt_refgen_start( wam->refgen_list->refgen );
   wam->refgen_current = wam->refgen_list;
   
   return 0;
}

int bt_wam_local_movehome(struct bt_wam_local * wam)
{
   return bt_wam_local_moveto(wam,wam->wambot->home);
}

int bt_wam_local_moveisdone(struct bt_wam_local * wam)
{
   return (wam->refgen_current) ? 0 : 1;
}


int bt_wam_local_is_teaching(struct bt_wam_local * wam)
{
   return (wam->teach) ? 1 : 0; 
}

int bt_wam_local_teach_start(struct bt_wam_local * wam)
{
   /* Make sure we're in the right mode */
   if (wam->refgen_current) return -1;
   if (wam->teach) return -1;
   if (bt_control_is_holding(wam->con_active)) return -1;
   if (!wam->con_active) return -1;
   
   /* Remove any refgens in the list */
   {
      struct bt_wam_refgen_list * refgen_list;
      struct bt_wam_refgen_list * refgen_list_next;
      
      refgen_list = wam->refgen_list;
      while (refgen_list)
      {
         refgen_list_next = refgen_list->next;
         /* respect iown, idelete? */
         if (refgen_list->iown)
            bt_refgen_destroy(refgen_list->refgen);
         free(refgen_list);
         refgen_list = refgen_list_next;
      }
      wam->refgen_list = 0;
   }
   
   /* Make a new refgen list element */
   wam->refgen_list = (struct bt_wam_refgen_list *) malloc( sizeof(struct bt_wam_refgen_list) );
   if (!wam->refgen_list) return -1;
   wam->refgen_list->next = 0;
   wam->refgen_list->iown = 1;
   
   /* Make the refgen itself */
   wam->refgen_list->refgen = (struct bt_refgen *)
      bt_refgen_teachplay_create(&(wam->elapsed_time), wam->con_active->position,"teach");
   if (!wam->refgen_list->refgen) {free(wam->refgen_list); wam->refgen_list=0; return -1;}
   
   /* TODO:
    * For now, we check if the log exists; if it does, we're teaching */
   if (  ((struct bt_refgen_teachplay *)(wam->refgen_list->refgen))->log )
   {
      /* Set the sync side start_time */
      wam->start_time = 1e-9 * bt_os_rt_get_time();
      wam->teach = 1;
   }
   
   return 0;
}

int bt_wam_local_teach_end(struct bt_wam_local * wam)
{
   if (!wam->teach) return -1;
   wam->teach = 0;
   
   /* We should check that it's a teachplay first! */
   bt_refgen_teachplay_save( (struct bt_refgen_teachplay *) (wam->refgen_list->refgen) );
   
   return 0;
   
}

int bt_wam_local_teach_start_custom(struct bt_wam_local * wam, struct bt_refgen * refgen)
{
   /* Make sure we're in the right mode */
   if (wam->refgen_current) return -1;
   if (wam->teach) return -1;
   if (bt_control_is_holding(wam->con_active)) return -1;
   if (!wam->con_active) return -1;
   
   /* Remove any refgens in the list */
   {
      struct bt_wam_refgen_list * refgen_list;
      struct bt_wam_refgen_list * refgen_list_next;
      
      refgen_list = wam->refgen_list;
      while (refgen_list)
      {
         refgen_list_next = refgen_list->next;
         /* respect iown, idelete? */
         if (refgen_list->iown)
            bt_refgen_destroy(refgen_list->refgen);
         free(refgen_list);
         refgen_list = refgen_list_next;
      }
      wam->refgen_list = 0;
   }
   
   /* Make a new refgen list element */
   wam->refgen_list = (struct bt_wam_refgen_list *) malloc( sizeof(struct bt_wam_refgen_list) );
   if (!wam->refgen_list) return -1;
   wam->refgen_list->next = 0;
   wam->refgen_list->iown = 0;
   
   /* Insert the already-made itself */
   wam->refgen_list->refgen = refgen;
   
   /* Set the sync side start_time */
   wam->start_time = 1e-9 * bt_os_rt_get_time();
   wam->teach = 1;
   
   return 0;
}

int bt_wam_local_teach_end_custom(struct bt_wam_local * wam)
{
   if (!wam->teach) return -1;
   wam->teach = 0;
   
   return 0;
}

int bt_wam_local_playback(struct bt_wam_local * wam)
{
   gsl_vector * teachplay_start;
   struct bt_wam_refgen_list * teachplay;
   
   /* Make sure we're not currently teaching */
   if (wam->teach) return 1;
   
   /* Make sure there's a loaded refgen_teachplay */
   if ( wam->refgen_list->refgen->type != bt_refgen_teachplay)
      return -1;
   
   /* Make a new move, from the current location
    * to the start of the refgen */
   teachplay = wam->refgen_list;
   
   /* Insert a new refgen list element */
   wam->refgen_list = (struct bt_wam_refgen_list *) malloc( sizeof(struct bt_wam_refgen_list) );
   if (!wam->refgen_list) return -1;
   wam->refgen_list->next = teachplay;
   wam->refgen_list->iown = 1;
   
   /* Make the move refgen itself */
   bt_refgen_get_start(teachplay->refgen,&teachplay_start);
#if 0
   wam->refgen_list->refgen = (struct bt_refgen *)
      bt_refgen_move_create(&(wam->elapsed_time), wam->jposition, wam->jvelocity,
                                   teachplay_start, wam->vel, wam->acc);
#endif
   wam->refgen_list->refgen = (struct bt_refgen *)
      bt_refgen_move_create(&(wam->elapsed_time),
                            wam->con_active->position, 0,
                            teachplay_start, wam->vel, wam->acc);
   
   if (!wam->refgen_list->refgen) {free(wam->refgen_list); wam->refgen_list=0; return -1;}
   /* Note, we should free more stuff here! */
   
   /* Save this refgen as the current one, and start it! */
   bt_control_hold(wam->con_active);
   wam->start_time = 1e-9 * bt_os_rt_get_time();
   bt_refgen_start( wam->refgen_list->refgen );
   wam->refgen_current = wam->refgen_list;
   
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
      wam->elapsed_time = time - wam->start_time;
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
      while (wam->refgen_current)
      {
         err = bt_refgen_eval( wam->refgen_current->refgen,
                               wam->con_active->reference );
         if (!err) break;
         
         if (err == 1) /* finished */
         {
            /* destroy the current refgen? */
            wam->refgen_current = wam->refgen_current->next;
            if (!wam->refgen_current) break;
            wam->start_time = time;
            wam->elapsed_time = 0;
            bt_refgen_start( wam->refgen_current->refgen );
            /* continue ... */
         }
      }
      bt_os_timestat_trigger(wam->ts,TS_REFGEN);

      /* Do the active controller */
      bt_control_eval( wam->con_active, wam->wambot->jtorque, time );
      bt_os_timestat_trigger(wam->ts,TS_CONTROL);

      /* If we're teaching, trigger the teach trajectory
       * (eventually we want to adjust the trigger rate?) */
      if (wam->teach && (((wam->count) & 0x4F) == 0) )
         bt_refgen_trigger( wam->refgen_list->refgen );
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

   return 0;
}

static void rt_wam_destroy(struct bt_wam_local * wam)
{
   if (wam->con_list)
      free(wam->con_list);
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
      
      if (wam->teach)
      {
         bt_refgen_teachplay_flush(
            (struct bt_refgen_teachplay *) (wam->refgen_list->refgen) );
      }
      
      bt_os_usleep(1000000);
   }
   
   bt_os_thread_exit( thread );
}




/* WAM List stuff ---------------------------------------- */





struct bt_wam_list_local * bt_wam_list_local_create()
{
   struct bt_wam_list_local * list;
   DIR * etcwam;
   struct dirent * file;
   
   /* Create */
   list = (struct bt_wam_list_local *) malloc(sizeof(struct bt_wam_list_local));
   if (!list)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Initialize */
   list->entries = 0;
   list->num = 0;
   
   /* Open the /etc/wam directory */
   etcwam = opendir(WAMCONFIGDIR);
   if (!etcwam)
   {
      syslog(LOG_ERR,"%s: Directory %s not found.",__func__,WAMCONFIGDIR);
      return list;
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
   return list;
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
