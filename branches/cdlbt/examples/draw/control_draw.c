/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_draw.c
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

#include <libbarrett/control.h>
#include "control_draw.h"

#include <libbarrett/dynamics.h>
#include <libbarrett/gsl.h>

#include <libconfig.h>
#include <syslog.h>
#include <gsl/gsl_blas.h>

/* This is just so we know what to look for in the config file */
static char *str_dimension[] = {"x","y","z"};

/* Define the type */
static int idle(struct bt_control * base);
static int hold(struct bt_control * base);
static int is_holding(struct bt_control * base);
static int get_position(struct bt_control * base);
static int eval(struct bt_control * base, gsl_vector * jtorque, double time);
static const struct bt_control_type control_draw_type = {
   "draw-space",
   &idle,
   &hold,
   &is_holding,
   &get_position,
   &eval
};
const struct bt_control_type * control_draw = &control_draw_type;

/* Controller-specific functions */
struct control_draw * control_draw_create(config_setting_t * config,
   struct bt_kinematics * kin, struct bt_dynamics * dyn)
{
   struct control_draw * c;
   c = (struct control_draw *) malloc( sizeof(struct control_draw) );
   
   /* Set the type, and other generic stuff */
   c->base.type = control_draw;
   c->base.n = 3;
   c->base.position = kin->tool->origin_pos;
   c->base.reference = gsl_vector_calloc(3);
   
   /* Start uninitialized */
   c->is_holding = 0;
   
   c->kin = kin;
   c->dyn = dyn;
   
   /* Save pointers to external input vectors */
   c->cvelocity = kin->tool_velocity;
   /*c->jposition = jposition;
   c->jvelocity = jvelocity;*/
   
   c->pressure = gsl_vector_calloc(3);
   c->pressure_onoff = 0;
   
   c->force = gsl_vector_calloc(3);

#if 0
   {
      gsl_matrix_view view;
      
      c->tool_jacobian_linear = (gsl_matrix *) malloc(sizeof(gsl_matrix));
      view = gsl_matrix_submatrix( kin->tool_jacobian, 0,0, 3,dyn->dof );
      *(c->tool_jacobian_linear) = view.matrix;
   }
#endif
   
   /* Create owned-by-me vectors */
   c->Kp = gsl_vector_calloc(3);
   c->Ki = gsl_vector_calloc(3);
   c->Kd = gsl_vector_calloc(3);
   c->integrator = gsl_vector_calloc(3);
   c->temp1 = gsl_vector_calloc(3);
   c->temp2 = gsl_vector_calloc(3);
   
   /* Read in the PID values: */
   {
      int j;
      config_setting_t * pids;
      /* Make sure the configuration looks good */
      if ( !(pids = config_setting_get_member( config, "pids" ))
                || (config_setting_type(pids)   != CONFIG_TYPE_GROUP) 
                || (config_setting_length(pids) != 3) )
      {
         syslog(LOG_ERR,"%s: The 'pids' configuration is not a 3-element group.",__func__);
         control_draw_destroy(c);
         return 0;
      }
      /* Read in the PID values */
      for (j=0; j<3; j++)
      {
         double p,i,d;
         config_setting_t * pid_grp;
         
         pid_grp = config_setting_get_member( pids, str_dimension[j] );
         if (   bt_gsl_config_get_double(config_setting_get_member( pid_grp, "p" ), &p)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "i" ), &i)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "d" ), &d))
         {
            syslog(LOG_ERR,"%s: No p, i, and/or d value",__func__);
            control_draw_destroy(c);
            return 0;
         }
         
         gsl_vector_set(c->Kp,j,p);
         gsl_vector_set(c->Ki,j,i);
         gsl_vector_set(c->Kd,j,d);
      }
   }
   
   return c;
}

void control_draw_destroy(struct control_draw * c)
{
   /*if (c->tool_jacobian_linear) free(c->tool_jacobian_linear);*/
   if (c->pressure) gsl_vector_free(c->pressure);
   if (c->force) gsl_vector_free(c->force);
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

int control_draw_set_pressure_vec(struct control_draw * c, gsl_vector * vec)
{
   gsl_vector_memcpy(c->pressure,vec);
   c->pressure_onoff = 1;
   return 0;
}






static int idle(struct bt_control * base)
{
   struct control_draw * c = (struct control_draw *) base;
   /* Do we need to stop doing anything? */
   c->is_holding = 0;
   return 0;
}

static int hold(struct bt_control * base)
{
   struct control_draw * c = (struct control_draw *) base;
   gsl_vector_memcpy(c->base.reference,c->base.position);
   gsl_vector_set_zero(c->integrator);
   c->last_time_saved = 0;
   c->is_holding = 1;
   return 0;
}

static int is_holding(struct bt_control * base)
{
   struct control_draw * c = (struct control_draw *) base;
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
   struct control_draw * c = (struct control_draw *) base;
   double fp;
   double pp;
   
   /* Do PID position control with the current reference */
   if (c->is_holding)
   {
      if (!c->last_time_saved)
      {
         c->last_time = time;
         c->last_time_saved = 1;
      }
      
      gsl_vector_set_zero(c->force);

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
      gsl_vector_sub( c->force, c->temp2 );
      /* Copy in I term */
      gsl_vector_memcpy( c->temp2, c->integrator);
      gsl_vector_mul( c->temp2, c->Ki );
      gsl_vector_sub( c->force, c->temp2 );
      /* Copy in D term */
      gsl_vector_memcpy( c->temp2, c->cvelocity );
      gsl_vector_mul( c->temp2, c->Kd );
      gsl_vector_sub( c->force, c->temp2 );
      
      if (c->pressure_onoff)
      {
         /* Subtract the pressure-hat component from the force vector,
            and add the pressure vector itself */
         gsl_blas_ddot( c->force, c->pressure, &fp );
         gsl_blas_ddot( c->pressure, c->pressure, &pp );
         gsl_blas_daxpy( 1.0 - fp / pp, c->pressure, c->force );
      }

      /* Multiply by the Jacobian-transpose at the tool */
      gsl_blas_dgemv( CblasTrans, 1.0, c->kin->tool_jacobian_linear,
                      c->force,
                      1.0, jtorque );

#if 0
      /* Evaluate inverse dynamics to produce joint torques */
      bt_dynamics_eval_inverse(c->dyn,
         c->jvelocity, c->jacceleration,
         jtorque );
#endif
   }

   return 0;
}