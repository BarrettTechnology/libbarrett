/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_joint_legacy.c
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
#include "control_joint_legacy.h"

#include "gsl.h"

#include <libconfig.h>
#include <syslog.h>
#include <gsl/gsl_blas.h>

/* Define the type */
static int idle(struct bt_control * base);
static int hold(struct bt_control * base);
static int is_holding(struct bt_control * base);
static int get_position(struct bt_control * base);
static int eval(struct bt_control * base, gsl_vector * jtorque, double time);
static const struct bt_control_type bt_control_joint_legacy_type = {
   "joint-space-legacy",
   &idle,
   &hold,
   &is_holding,
   &get_position,
   &eval
};
const struct bt_control_type * bt_control_joint_legacy = &bt_control_joint_legacy_type;

/* Controller-specific functions */
struct bt_control_joint_legacy * bt_control_joint_legacy_create(config_setting_t * config, gsl_vector * jposition, gsl_vector * jvelocity)
{
   int n;
   struct bt_control_joint_legacy * c;
   c = (struct bt_control_joint_legacy *) malloc( sizeof(struct bt_control_joint_legacy) );
   n = jposition->size;
   
   /* Set the type, and other generic stuff */
   c->base.type = bt_control_joint_legacy;
   c->base.n = n;
   c->base.position = jposition; /* this points directly in this case */
   c->base.reference = gsl_vector_calloc(n);
   
   /* Start uninitialized */
   c->is_holding = 0;
   
   /* Save pointers to external input vectors */
   c->jposition = jposition;
   c->jvelocity = jvelocity;
   
   /* Create owned-by-me vectors */
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
         bt_control_joint_legacy_destroy(c);
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
            bt_control_joint_legacy_destroy(c);
            return 0;
         }
         
         gsl_vector_set(c->Kp,j,p);
         gsl_vector_set(c->Ki,j,i);
         gsl_vector_set(c->Kd,j,d);
      }
   }
   
   return c;
}

void bt_control_joint_legacy_destroy(struct bt_control_joint_legacy * c)
{
   if (c->base.reference) gsl_vector_free(c->base.reference);
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
   struct bt_control_joint_legacy * c = (struct bt_control_joint_legacy *) base;
   /* Do we need to stop doing anything? */
   c->is_holding = 0;
   return 0;
}

static int hold(struct bt_control * base)
{
   struct bt_control_joint_legacy * c = (struct bt_control_joint_legacy *) base;
   gsl_vector_memcpy(base->reference,base->position);
   gsl_vector_set_zero(c->integrator);
   c->last_time_saved = 0;
   c->is_holding = 1;
   return 0;
}

static int is_holding(struct bt_control * base)
{
   struct bt_control_joint_legacy * c = (struct bt_control_joint_legacy *) base;
   return c->is_holding ? 1 : 0;
}

/* This already points directly! */
static int get_position(struct bt_control * base)
{
   return 0;
}

/* RT - Evaluate */
static int eval(struct bt_control * base, gsl_vector * jtorque, double time)
{
   struct bt_control_joint_legacy * c = (struct bt_control_joint_legacy *) base;
   
   /* Do PID position control with the current reference */
   if (c->is_holding)
   {
      if (!c->last_time_saved)
      {
         c->last_time = time;
         c->last_time_saved = 1;
      }
      /* Compute the error */
      gsl_vector_memcpy( c->temp1, c->base.position );
      gsl_vector_sub( c->temp1, c->base.reference );
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



