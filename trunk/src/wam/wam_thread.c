
#include <math.h>

#include <syslog.h>

#include "wam_local.h"

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


static int rt_wam_create(struct bt_wam_local * wam, config_setting_t * wamconfig);
static void rt_wam_destroy(struct bt_wam_local * wam);




struct bt_wam_thread_helper * bt_wam_thread_helper_create(struct bt_wam_local * wam, config_setting_t * config)
{
   struct bt_wam_thread_helper * helper;
   helper = (struct bt_wam_thread_helper *) malloc(sizeof(struct bt_wam_thread_helper));
   if (!helper) return 0;
   helper->wam = wam;
   helper->config = config;
   helper->is_setup = 0;
   helper->setup_failed = 0;
   return helper;
}
void bt_wam_thread_helper_destroy(struct bt_wam_thread_helper * helper)
{
   free(helper);
   return;
}



/* ===========================================================================
 * Below are separate thread stuffs */

void bt_wam_thread(struct bt_os_thread * thread)
{
   int err;
   struct bt_wam_thread_helper * helper;
   struct bt_wam_local * wam;
   
   helper = (struct bt_wam_thread_helper *) thread->data;
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
      if (!wam->count) wam->wam_start_time = time;
      wam->wam_time = time - wam->wam_start_time;
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
                               time - wam->refgen_start_time,
                               wam->con_active->reference );

         if (!err) break;
         
         if (err == 1) /* finished */
         {
            if ( (wam->refgen_active == wam->refgen_tempmove)
                && (wam->refgen_loaded) )
            {
               wam->refgen_start_time = time;
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
                                 time - wam->refgen_start_time,
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
      if (wam->user_log)
         bt_log_trigger(wam->user_log);
      if (wam->ts_log)
         bt_log_trigger(wam->ts_log);
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

   /* Set up the timer */
   wam->ts = bt_os_timestat_create(TSNUM);
   if (!wam->ts)
   {
      syslog(LOG_ERR,"%s: Could not create timestat.",__func__);
      rt_wam_destroy(wam);
      return 0;
   }

#if 0

   
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

   return 0;
}

static void rt_wam_destroy(struct bt_wam_local * wam)
{
   if (wam->ts_log)
      bt_log_destroy(wam->ts_log);
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
   if (wam->grav)
      bt_calgrav_destroy(wam->grav);
   if (wam->dyn)
      bt_dynamics_destroy(wam->dyn);
   if (wam->kin)
      bt_kinematics_destroy(wam->kin);
   if (wam->wambot)
      bt_wambot_phys_destroy((struct bt_wambot_phys *)wam->wambot);
}

void bt_wam_thread_nonrt(struct bt_os_thread * thread)
{
   struct bt_wam_local * wam = (struct bt_wam_local *) thread->data;
   
   while (!bt_os_thread_isdone(thread))
   {
      if (wam->user_log)
      {
         /*syslog(LOG_ERR,"Flushing Log Files ...");*/
         bt_log_flush(wam->user_log);
      }

      if (wam->ts_log)
      {
         /*syslog(LOG_ERR,"Flushing Log Files ...");*/
         bt_log_flush(wam->ts_log);
      }
      
      if (wam->refgen_active && wam->teaching)
      {
         bt_refgen_teach_flush(wam->refgen_active);
      }
      
      bt_os_usleep(1000000);
   }
   
   bt_os_thread_exit( thread );
}
