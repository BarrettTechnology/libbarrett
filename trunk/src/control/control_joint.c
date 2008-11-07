/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_joint.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Sam Clanton
 *                       Christopher Dellin
 *  Creation Date ...... Nov 24, 2002
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2002 Nov 24 - TH
 *      File created.
 *    2004 Dec 16 - BZ, SC, TH
 *      Initial port to linux + RTAI
 *    2008 Sept 16 - CD
 *      Ported from btsystem to libbt; btstatecontrol and btcontrol merged
 *
 * ======================================================================== */

#include "control.h"
#include "control_joint.h"

#include "gsl.h"

#include <libconfig.h>
#include <syslog.h>
#include <gsl/gsl_blas.h>

/* Base function pointers */
static const char name[] = "joint-space";
static int idle(struct bt_control * base);
static int hold(struct bt_control * base);
static int is_holding(struct bt_control * base);
static int get_position(struct bt_control * base, gsl_vector * position);
static int set_reference(struct bt_control * base, gsl_vector * reference);
static int eval(struct bt_control * base, gsl_vector * jtorque, double time);

/* Controller-specific functions */
struct bt_control_joint * bt_control_joint_create(config_setting_t * config, gsl_vector * jposition, gsl_vector * jvelocity)
{
   int n;
   struct bt_control_joint * c;
   c = (struct bt_control_joint *) malloc( sizeof(struct bt_control_joint) );
   n = jposition->size;
   
   /* Set base function pointers */
   c->base.name = name;
   c->base.n = n;
   c->base.idle = &idle;
   c->base.hold = &hold;
   c->base.is_holding = &is_holding;
   c->base.get_position = &get_position;
   c->base.set_reference = &set_reference;
   c->base.eval = &eval;
   
   /* Start uninitialized */
   c->is_holding = 0;
   
   /* Save pointers to external input vectors */
   c->jposition = jposition;
   c->jvelocity = jvelocity;
   
   /* Create owned-by-me vectors */
   c->reference = gsl_vector_calloc(n);
   c->Kp = gsl_vector_calloc(n);
   c->Ki = gsl_vector_calloc(n);
   c->Kd = gsl_vector_calloc(n);
   c->integrator = gsl_vector_calloc(n);
   c->temp1 = gsl_vector_calloc(n);
   c->temp2 = gsl_vector_calloc(n);
   
   /* Read in the PID values: */
   {
      int j;
      config_setting_t * pids;
      /* Make sure the configuration looks good */
      if ( !(pids = config_setting_get_member( config, "pids" ))
                || (config_setting_type(pids)   != CONFIG_TYPE_LIST) 
                || (config_setting_length(pids) != n) )
      {
         syslog(LOG_ERR,"%s: The 'pids' configuration is not a %d-element list.",__func__,n);
         bt_control_joint_destroy(c);
         return 0;
      }
      /* Read in the PID values */
      for (j=0; j<n; j++)
      {
         double p,i,d;
         config_setting_t * pid_grp;
         
         pid_grp = config_setting_get_elem( pids, j );
         if (   bt_gsl_config_get_double(config_setting_get_member( pid_grp, "p" ), &p)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "i" ), &i)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "d" ), &d))
         {
            syslog(LOG_ERR,"%s: No p, i, and/or d value",__func__);
            bt_control_joint_destroy(c);
            return 0;
         }
         
         gsl_vector_set(c->Kp,j,p);
         gsl_vector_set(c->Ki,j,i);
         gsl_vector_set(c->Kd,j,d);
      }
   }
   
   return c;
}

void bt_control_joint_destroy(struct bt_control_joint * c)
{
   if (c->reference) gsl_vector_free(c->reference);
   if (c->Kp) gsl_vector_free(c->Kp);
   if (c->Ki) gsl_vector_free(c->Ki);
   if (c->Kd) gsl_vector_free(c->Kd);
   if (c->integrator) gsl_vector_free(c->integrator);
   if (c->temp1) gsl_vector_free(c->temp1);
   if (c->temp2) gsl_vector_free(c->temp2);
   free(c);
   return;
}

static int idle(struct bt_control * base)
{
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   /* Do we need to stop doing anything? */
   c->is_holding = 0;
   return 0;
}

static int hold(struct bt_control * base)
{
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   gsl_vector_memcpy(c->reference,c->jposition);
   gsl_vector_set_zero(c->integrator);
   c->last_time_saved = 0;
   c->is_holding = 1;
   return 0;
}

int is_holding(struct bt_control * base)
{
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   return c->is_holding ? 1 : 0;
}

static int get_position(struct bt_control * base, gsl_vector * position)
{
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   gsl_vector_memcpy( position, c->jposition );
   return 0;
}

static int set_reference(struct bt_control * base, gsl_vector * reference)
{
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   gsl_vector_memcpy( c->reference, reference );
   return 0;
}

#if 0
/* Teaches - into traj_spline, traj_profile */
static int teach_discrete(struct bt_control * base)
{
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   
   /* Make sure we're in idle mode */
   if (c->mode != BT_CONTROL_IDLE)
      return 1;
   
   /* Remove any current spline / profile */
   if (c->traj_spline) bt_trajectory_spline_destroy(c->traj_spline);
   if (c->traj_profile) bt_trajectory_profile_destroy(c->traj_profile);
   
   /* Make a new spline, using the current jposition as the start location */
   c->traj_spline = bt_trajectory_spline_create( c->jposition );
   
   c->mode = BT_CONTROL_IDLE_TEACH_DISCRETE;
   return 0;
}

static int teach_add(struct bt_control * base)
{
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   
   /* Make sure we're currently doing a discrete teach */
   if (c->mode != BT_CONTROL_IDLE_TEACH_DISCRETE)
      return 1;
   
   /* Add the current point to the spline */
   bt_trajectory_spline_add( c->traj_spline, c->jposition );
   
   return 0;
}

static int teach_done(struct bt_control * base)
{
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   
   /* Make sure we're currently doing a discrete teach */
   /* NOTE - EVENTUALLY, THIS IS HOW WE END CONTINUOUS TEACHES TOO! */
   if (c->mode != BT_CONTROL_IDLE_TEACH_DISCRETE)
      return 1;
   
   /* Initialize the spline, starting from rest */
   bt_trajectory_spline_init( c->traj_spline, 0, 0 );
   
   /* Create a new profile, starting from rest */
   c->traj_profile = bt_trajectory_profile_create( 1.0, 1.0, 0.0, c->traj_spline->length);
   
   c->mode = BT_CONTROL_IDLE;
   return 0;
}

/* Playbacks */
static int traj_start(struct bt_control * base, int skip_ready)
{
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   
   /* Make sure we can start playback */
   switch (c->mode)
   {
      case BT_CONTROL_IDLE:
      case BT_CONTROL_HOLD:
      case BT_CONTROL_TRAJ_READY:
         break;
      default:
         return 1;
   }
   
   /* If we're already ready to go, then go! */
   if (c->mode == BT_CONTROL_TRAJ_READY)
   {
      c->start_time_saved = 0;
      c->mode = BT_CONTROL_TRAJ_MOVING;
      return 0;
   }
   
   /* Make a move from here to the starting location */
   if (c->move_spline) bt_trajectory_spline_destroy(c->move_spline);
   if (c->move_profile) bt_trajectory_profile_destroy(c->move_profile);
   
   /* Start at current position */
   c->move_spline = bt_trajectory_spline_create( c->jposition );
   
   /* End at the beginning of the trajectory spline */
   bt_trajectory_spline_get( c->traj_spline, c->temp1, 0.0 );
   bt_trajectory_spline_add( c->move_spline, c->temp1 );
   
   /* Set the direction */
   bt_trajectory_spline_init( c->move_spline, 0, c->jvelocity );
   
   /* Create the profile */
   c->move_profile = bt_trajectory_profile_create( 1.0, 1.0, gsl_blas_dnrm2(c->jvelocity), c->move_spline->length);
   
   /* Start the move */
   c->skip_ready = skip_ready;
   c->start_time_saved = 0;
   c->mode = BT_CONTROL_TRAJ_PREP;
   
   return 0;
}

/* RT - Evaluate */
static int eval(struct bt_control * base, gsl_vector * jtorque, double time)
{
   int err;
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   
   /* Are we currently following a trajectory?
    * Grab into the reference position from the spline. */
   switch (c->mode)
   {
      double s;
      case BT_CONTROL_TRAJ_PREP:
         /* Is this the start? */
         if (!c->start_time_saved)
         {
            c->start_time = time;
            c->start_time_saved = 1;
         }
         /* Grab the current point along the trajectory into the reference position */
         err = bt_trajectory_profile_get( c->move_profile, &s, time - c->start_time );
         /* Is this the end? */
         if (err == 2)
         {
            if (c->skip_ready)
            {
               c->start_time = time;
               c->last_time = time;
               c->mode = BT_CONTROL_TRAJ_MOVING;
               return eval(base,jtorque,time);
            }
            else
            {
               c->mode = BT_CONTROL_TRAJ_READY;
            }
         }
         bt_trajectory_spline_get( c->move_spline, c->reference, s );
         break;
      case BT_CONTROL_TRAJ_MOVING:
         /* Is this the start? */
         if (!c->start_time_saved)
         {
            c->start_time = time;
            c->last_time = time;
            c->start_time_saved = 1;
         }
         /* Grab the current point along the trajectory into the reference position */
         err = bt_trajectory_profile_get( c->traj_profile, &s, time - c->start_time );
         /* Is this the end? */
         if (err == 2)
         {
            c->mode = BT_CONTROL_HOLD;
         }
         bt_trajectory_spline_get( c->move_spline, c->reference, s );
         break;
      default:
         break;
   }
#endif

/* RT - Evaluate */
static int eval(struct bt_control * base, gsl_vector * jtorque, double time)
{
   struct bt_control_joint * c = (struct bt_control_joint *) base;
   
   /* Do PID position control with the current reference */
   if (c->is_holding)
   {
      if (!c->last_time_saved)
      {
         c->last_time = time;
         c->last_time_saved = 1;
      }
      /* Compute the error */
      gsl_vector_memcpy( c->temp1, c->jposition );
      gsl_vector_sub( c->temp1, c->reference );
      /* Increment integrator */
      gsl_vector_memcpy( c->temp2, c->temp1 );
      gsl_vector_scale( c->temp2, time - c->last_time );
      c->last_time = time;
      gsl_vector_add( c->integrator, c->temp2 );
      /* Copy in P term */
      gsl_vector_memcpy( c->temp2, c->temp1 );
      gsl_vector_mul( c->temp2, c->Kp );
      gsl_vector_sub( jtorque, c->temp2 );
      /* Copy in I term */
      gsl_vector_memcpy( c->temp2, c->integrator);
      gsl_vector_mul( c->temp2, c->Ki );
      gsl_vector_sub( jtorque, c->temp2 );
      /* Copy in D term */
      gsl_vector_memcpy( c->temp2, c->jvelocity );
      gsl_vector_mul( c->temp2, c->Kd );
      gsl_vector_sub( jtorque, c->temp2 );
   }
   
   return 0;
}



