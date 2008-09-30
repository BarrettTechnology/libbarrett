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

#include "wam.h"

#include <string.h>
#include <syslog.h>
#include <math.h>     /* For sqrt() */

#include <libconfig.h>

/* Some global things for timing the threads */
#define TS \
   T(UPDATE) T(KINEMATICS) T(LOG) \
   T(GRAV_ZERO) T(GCOMP) T(SPLINE) T(SETJTOR) \
   T(MODE_HARD) T(WAIT_PERIOD)
#define T(x) TS_##x,
   enum ts_enum { TS TSNUM };
#undef T
#define T(x) #x,
   char * ts_name[] = { TS 0 };
#undef T

void rt_wam(bt_os_thread * thread);
int rt_wam_create(struct bt_wam * wam, config_setting_t * wamconfig);
void rt_wam_destroy(struct bt_wam * wam);

void nonrt_thread_function(bt_os_thread * thread);

struct setup_helper
{
   struct bt_wam * wam;
   config_setting_t * config;
   int is_setup;
   int setup_failed;
};

struct setup_helper * helper_create(struct bt_wam * wam, config_setting_t * config)
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

void helper_destroy(struct setup_helper * helper)
{
   free(helper);
   return;
}

struct bt_wam * bt_wam_create(config_setting_t * wamconfig)
{
   struct bt_wam * wam;
   struct setup_helper * helper;
   
   /* Check command-line arguments */
   if (!wamconfig)
   {
      syslog(LOG_ERR,"%s: No WAM configuration given.",__func__);
      return 0;
   }
   
   /* Set up realtime stuff */
   /* Allow hard real time process scheduling for non-root users */
   bt_os_rt_allow_nonroot();
   
   /* Make a new wam structure */
   wam = (struct bt_wam *) malloc( sizeof(struct bt_wam) );
   if (!wam)
   {
      syslog(LOG_ERR,"%s: No memory for a new WAM.",__func__);
      return 0;
   }
   
   /* Spin off the non-realtime log-saving thread */
   wam->nonrt_thread = bt_os_thread_create(BT_OS_NONRT, "LOG", 10, nonrt_thread_function, (void *)wam);
   if (!wam->nonrt_thread)
   {
      syslog(LOG_ERR,"%s: Could not create non-realtime thread.",__func__);
      bt_wam_destroy(wam);
      return 0;
   }
   
   /* Set up the timer */
   wam->ts = bt_os_timestat_create(TSNUM);
   if (!wam->ts)
   {
      syslog(LOG_ERR,"%s: Could not create timestat.",__func__);
      bt_wam_destroy(wam);
      return 0;
   }
   
   /* Set up defaults */
   wam->gcomp = 0;
   
   /* Spin off the realtime thread to set everything up */
   helper = helper_create(wam,wamconfig);
   if (!helper)
   {
      syslog(LOG_ERR,"%s: Could not create setup helper.",__func__);
      bt_wam_destroy(wam);
      return 0;
   }
   wam->rt_thread = bt_os_thread_create(BT_OS_RT, "WAM", 90, rt_wam, (void *)helper);
   if (!wam->rt_thread)
   {
      syslog(LOG_ERR,"%s: Could not create realtime thread.",__func__);
      helper_destroy(helper);
      bt_wam_destroy(wam);
      return 0;
   }
   
   /* Wait until the thread is done starting */
   while (!helper->is_setup)
      bt_os_usleep(10000);
   
   /* Check for setup failure */
   if (helper->setup_failed)
   {
      syslog(LOG_ERR,"%s: WAM Setup failed.",__func__);
      helper_destroy(helper);
      bt_wam_destroy(wam);
      return 0;
   }
   
   /* Success! */
   helper_destroy(helper);
   return wam;
}

void bt_wam_destroy(struct bt_wam * wam)
{
   /* Tell the non-realtime thread to exit */
   if (wam->nonrt_thread)
   {
      wam->nonrt_thread->done = 1;
      bt_os_usleep(30000); /* We can actually check for this! */
   }
   
   /* Tell the realtime thread to exit */
   if (wam->rt_thread)
   {
      wam->rt_thread->done = 1;
      bt_os_usleep(30000); /* We can actually check for this! */
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
      printf("%15s:   us  ns      us  ns      min       max  \n","Timing Stats");
      printf("%15s--------------------------------------------\n","-------------");
      for (i=0; i<TSNUM; i++)
         printf("%15s: %4d %03d  +/-%3d %03d   %4d %03d  %4d %03d\n",
                ts_name[i],
                ((int)(means[i]))/1000,      ((int)(means[i]))%1000,
                ((int)(sqrt(vars[i])))/1000, ((int)(sqrt(vars[i])))%1000,
                ((int)(mins[i]))/1000,       ((int)(mins[i]))%1000,
                ((int)(maxs[i]))/1000,       ((int)(maxs[i]))%1000 );
      
      /* Destroy the timestat (eventually) */
   }
   
   /* ATTEMPT TO Decode the binary log file */
   bt_log_decode("datafile.dat", "dat.oct", 1, 1); /* Woo octave! */
   
   free(wam);
   return;
}

int bt_wam_isgcomp(struct bt_wam * wam)
{
   return wam->gcomp;
}

int bt_wam_setgcomp(struct bt_wam * wam, int onoff)
{
   wam->gcomp = onoff ? 1 : 0;
   return 0;
}


/* Below are separate thread stuffs */

void rt_wam(bt_os_thread * thread)
{
   int err;
   struct setup_helper * helper = (struct setup_helper *) thread->data;
   struct bt_wam * wam = helper->wam;
   
   /* Initialize the WAM data structure (wambot, kinematics, gravity, etc) */
   err = rt_wam_create(helper->wam, helper->config);
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not create realtime WAM stuff.",__func__);
      helper->setup_failed = 1;
      bt_os_thread_exit( thread );
      return;
   }
   
   /* Set the active controller to joint-space */
   wam->con_active = (struct bt_control *) wam->con_joint;
   
   /* Set velocity safety limit tp 2.0 m/s */
   bt_bus_set_property(wam->wambot->bus, SAFETY_PUCK_ID, wam->wambot->bus->p->VL2, 1, 2.0 * 0x1000);
   
   /* Set up the easy-access wam vectors */
   wam->jposition = wam->wambot->jposition;
   wam->jtorque = wam->wambot->jtorque;
   wam->cposition = wam->kin->tool->origin_pos;
   wam->crotation = wam->kin->tool->rot_to_inertial;
   
   /* Setup is complete! */
   helper->is_setup = 1;
   
   /* Note - the helper will now be destroyed for us,
    * and the create() function will return. */
   
   /* OK, start the control loop ... */
   bt_os_make_periodic(0.002,"CONTRL"); /* Note - only call this once */
   bt_os_rt_set_mode_hard();
   
   /* Loop until we're told by destroy() to exit */
   while (!bt_os_thread_done(thread))
   {
      /* Make sure task is still in primary mode,
       * and wait until the next control period */
      bt_os_timestat_start(wam->ts);
      
      /* Grab the current joint positions */
      bt_wambot_update( wam->wambot );
      bt_os_timestat_trigger(wam->ts,TS_UPDATE);
      
      /* Forward kinematics */
      bt_kinematics_eval_forward( wam->kin, wam->wambot->jposition );
      bt_os_timestat_trigger(wam->ts,TS_KINEMATICS);
      
      /* Log data */
      bt_log_trigger( wam->log );
      bt_os_timestat_trigger(wam->ts,TS_LOG);
      
      /* Set the torques to be zero */
      gsl_vector_set_zero( wam->wambot->jtorque );
      bt_os_timestat_trigger(wam->ts,TS_GRAV_ZERO);
     
      /* Do gravity compensation */
      if (wam->gcomp) bt_gravity_eval( wam->grav, wam->wambot->jtorque );
      bt_os_timestat_trigger(wam->ts,TS_GCOMP);
      
      /* Do the active controller  */
      wam->con_active->eval( wam->con_active, wam->wambot->jtorque, 1e-9 * bt_os_rt_get_time() );
      bt_os_timestat_trigger(wam->ts,TS_SPLINE);
      
      /* Apply the current joint torques */
      bt_wambot_setjtor( wam->wambot );
      bt_os_timestat_trigger(wam->ts,TS_SETJTOR);
      
      /* Wait for the next time ... */
      bt_os_rt_set_mode_hard();
      bt_os_timestat_trigger(wam->ts,TS_MODE_HARD);
      bt_os_rt_task_wait_period();
      bt_os_timestat_trigger(wam->ts,TS_WAIT_PERIOD);
      
      /* Calculate timing statistics */
      bt_os_timestat_end(wam->ts);
   }
   
   rt_wam_destroy(wam);
   
   /* Remove this thread from the realtime scheduler */
   bt_os_thread_exit( thread );
   
   return;
}

/* realtime WAM initialization stuff */
int rt_wam_create(struct bt_wam * wam, config_setting_t * wamconfig)
{
   int err;
   
   wam->wambot = 0;
   wam->kin = 0;
   wam->grav = 0;
   wam->log = 0;
   wam->con_joint= 0;
   
   /* Create a wambot object (which sets the dof) */
   wam->wambot = bt_wambot_create( config_setting_get_member(wamconfig,"wambot") );
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
   
   /* Create a gravity object */
   wam->grav = bt_gravity_create( config_setting_get_member(wamconfig,"gravity"), wam->kin );
   if (!wam->grav)
   {
      syslog(LOG_ERR,"%s: Could not create gravity compensation.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
   
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
   
   /* Create a joint-space controller */
   wam->con_joint = bt_control_joint_create(config_setting_get_member(wamconfig,"joint-controller"),
                                            wam->wambot->jposition, wam->wambot->jvelocity);
   if (!wam->con_joint)
   {
      syslog(LOG_ERR,"%s: Could not create joint-space controller.",__func__);
      rt_wam_destroy(wam);
      return 1;
   }
   
   return 0;
}

void rt_wam_destroy(struct bt_wam * wam)
{
   if (wam->con_joint)
      bt_control_joint_destroy(wam->con_joint);
   if (wam->log)
   {
      bt_log_off(wam->log); /* Just to make sure! */
      bt_log_destroy(wam->log);
   }
   if (wam->grav)
      bt_gravity_destroy(wam->grav);
   if (wam->kin)
      bt_kinematics_destroy(wam->kin);
   if (wam->wambot)
      bt_wambot_destroy(wam->wambot);
}


void nonrt_thread_function(bt_os_thread * thread)
{
   struct bt_wam * wam = (struct bt_wam *) thread->data;
   
   while (!bt_os_thread_done(thread))
   {
      if (wam->log)
      {
         syslog(LOG_ERR,"Flushing Log Files ...");
         bt_log_flush( wam->log );
         bt_os_usleep(1000000);
      }
   }
   
   bt_os_thread_exit( thread );
}



