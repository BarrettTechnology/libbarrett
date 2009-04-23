/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_cartesian_xyz_q.c
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

#include <math.h> /* For sqrt() */
#include <gsl/gsl_math.h> /* For M_PI */

#include "control.h"
#include "control_cartesian_xyz_q.h"

#include "dynamics.h"
#include "gsl.h"

#include <libconfig.h>
#include <syslog.h>
#include <gsl/gsl_blas.h>

/* Note: quats are (w,(i,j,k))*/

/* rotation matrix to quaternion
 * adapted from:
 *
 * "From Quaternion to Matrix and Back"
 * February 27th 2005
 * J.M.P. van Waveren
 * © 2005, Id Software, Inc.
 *
 * www.intel.com/cd/ids/developer/asmo-na/eng/293748.htm
*/
int rot_to_q( gsl_matrix * rot, gsl_vector * quat )
{
   if ( gsl_matrix_get(rot,0,0) + gsl_matrix_get(rot,1,1) + gsl_matrix_get(rot,2,2) > 0.0 )
   {
      double t, s;
      t = gsl_matrix_get(rot,0,0) + gsl_matrix_get(rot,1,1) + gsl_matrix_get(rot,2,2) + 1.0;
      s = 0.5 / sqrt(t);
      gsl_vector_set(quat,0, s * t);
      gsl_vector_set(quat,3, s * ( gsl_matrix_get(rot,0,1) - gsl_matrix_get(rot,1,0) ) );
      gsl_vector_set(quat,2, s * ( gsl_matrix_get(rot,2,0) - gsl_matrix_get(rot,0,2) ) );
      gsl_vector_set(quat,1, s * ( gsl_matrix_get(rot,1,2) - gsl_matrix_get(rot,2,1) ) );
   }
   else if ( gsl_matrix_get(rot,0,0) > gsl_matrix_get(rot,1,1) && gsl_matrix_get(rot,0,0) > gsl_matrix_get(rot,2,2) )
   {
      double t, s;
      t = gsl_matrix_get(rot,0,0) - gsl_matrix_get(rot,1,1) - gsl_matrix_get(rot,2,2) + 1.0;
      s = 0.5 / sqrt(t);
      gsl_vector_set(quat,1, s * t );
      gsl_vector_set(quat,2, s * ( gsl_matrix_get(rot,0,1) + gsl_matrix_get(rot,1,0) ) );
      gsl_vector_set(quat,3, s * ( gsl_matrix_get(rot,2,0) + gsl_matrix_get(rot,0,2) ) );
      gsl_vector_set(quat,0, s * ( gsl_matrix_get(rot,1,2) - gsl_matrix_get(rot,2,1) ) );
   }
   else if ( gsl_matrix_get(rot,1,1) > gsl_matrix_get(rot,2,2) )
   {
      double t, s;
      t = - gsl_matrix_get(rot,0,0) + gsl_matrix_get(rot,1,1) - gsl_matrix_get(rot,2,2) + 1.0;
      s = 0.5 / sqrt(t);
      gsl_vector_set(quat,2, s * t );
      gsl_vector_set(quat,1, s * ( gsl_matrix_get(rot,0,1) + gsl_matrix_get(rot,1,0) ) );
      gsl_vector_set(quat,0, s * ( gsl_matrix_get(rot,2,0) - gsl_matrix_get(rot,0,2) ) );
      gsl_vector_set(quat,3, s * ( gsl_matrix_get(rot,1,2) + gsl_matrix_get(rot,2,1) ) );
   }
   else
   {
      double t, s;
      t = - gsl_matrix_get(rot,0,0) - gsl_matrix_get(rot,1,1) + gsl_matrix_get(rot,2,2) + 1.0;
      s = 0.5 / sqrt(t);
      gsl_vector_set(quat,3, s * t );
      gsl_vector_set(quat,0, s * ( gsl_matrix_get(rot,0,1) - gsl_matrix_get(rot,1,0) ) );
      gsl_vector_set(quat,1, s * ( gsl_matrix_get(rot,2,0) + gsl_matrix_get(rot,0,2) ) );
      gsl_vector_set(quat,2, s * ( gsl_matrix_get(rot,1,2) + gsl_matrix_get(rot,2,1) ) );
   }

   return 0;
}

/* Here, we calculate q_a = q_b * q_c' */
int q_mult_conj( gsl_vector * qa,  gsl_vector * qb,  gsl_vector * qc )
{
   gsl_vector_set(qa,0, + gsl_vector_get(qb,0) * gsl_vector_get(qc,0)
                        + gsl_vector_get(qb,1) * gsl_vector_get(qc,1)
                        + gsl_vector_get(qb,2) * gsl_vector_get(qc,2)
                        + gsl_vector_get(qb,3) * gsl_vector_get(qc,3));
   
   gsl_vector_set(qa,1, - gsl_vector_get(qb,0) * gsl_vector_get(qc,1)
                        + gsl_vector_get(qb,1) * gsl_vector_get(qc,0)
                        - gsl_vector_get(qb,2) * gsl_vector_get(qc,3)
                        + gsl_vector_get(qb,3) * gsl_vector_get(qc,2));
   
   gsl_vector_set(qa,2, - gsl_vector_get(qb,0) * gsl_vector_get(qc,2)
                        + gsl_vector_get(qb,1) * gsl_vector_get(qc,3)
                        + gsl_vector_get(qb,2) * gsl_vector_get(qc,0)
                        - gsl_vector_get(qb,3) * gsl_vector_get(qc,1));
   
   gsl_vector_set(qa,3, - gsl_vector_get(qb,0) * gsl_vector_get(qc,3)
                        - gsl_vector_get(qb,1) * gsl_vector_get(qc,2)
                        + gsl_vector_get(qb,2) * gsl_vector_get(qc,1)
                        + gsl_vector_get(qb,3) * gsl_vector_get(qc,0));
   
   return 0;
}

int q_to_angle_axis( gsl_vector * q, gsl_vector * angleaxis )
{
   double sin_a;
   double angle;
   
   sin_a = sqrt( 1.0 - gsl_vector_get(q,0)*gsl_vector_get(q,0) );
   
   angle = 2 * acos( gsl_vector_get(q,0) ); /* 0 - 2pi */
   if (angle > M_PI ) angle -= 2*M_PI;
   
   if ( fabs(gsl_vector_get(q,0)) < 0.0005 )
   {
      gsl_vector_set_zero(angleaxis);
      return 0;
   }
   
   gsl_vector_set(angleaxis,0, gsl_vector_get(q,1) / sin_a * angle );
   gsl_vector_set(angleaxis,1, gsl_vector_get(q,2) / sin_a * angle );
   gsl_vector_set(angleaxis,2, gsl_vector_get(q,3) / sin_a * angle );
   
   return 0;
}

/* This is just so we know what to look for in the config file */
static char *str_dimension[] = {"x","y","z"};

/* Define the type */
static int idle(struct bt_control * base);
static int hold(struct bt_control * base);
static int is_holding(struct bt_control * base);
static int get_position(struct bt_control * base);
static int eval(struct bt_control * base, gsl_vector * jtorque, double time);
static const struct bt_control_type bt_control_cartesian_xyz_q_type = {
   "Cartesian-xyz-q-space",
   &idle,
   &hold,
   &is_holding,
   &get_position,
   &eval
};
const struct bt_control_type * bt_control_cartesian_xyz_q = &bt_control_cartesian_xyz_q_type;

/* Controller-specific functions */
struct bt_control_cartesian_xyz_q * bt_control_cartesian_xyz_q_create(config_setting_t * config,
   struct bt_kinematics * kin, struct bt_dynamics * dyn)
{
   struct bt_control_cartesian_xyz_q * c;
   
   /* Create */
   c = (struct bt_control_cartesian_xyz_q *) malloc( sizeof(struct bt_control_cartesian_xyz_q) );
   if (!c)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Initialize */
   c->base.position = 0;
   c->base.reference = 0;
   c->pos_xyz = 0;
   c->pos_quat = 0;
   c->ref_xyz = 0;
   c->ref_quat = 0;
   
   /* Set the type, and other generic stuff */
   c->base.type = bt_control_cartesian_xyz_q;
   c->base.n = 7; /* Number of dimensions */
   c->base.position = gsl_vector_calloc(7); /* A place to hold the current position */
   c->base.reference = gsl_vector_calloc(7); /* A place to hold the reference position */
   
   /* Get vector views */
   {
      gsl_vector_view view;
      
      /* Allocate */
      c->pos_xyz  = (gsl_vector *) malloc(sizeof(gsl_vector));
      c->pos_quat = (gsl_vector *) malloc(sizeof(gsl_vector));
      c->ref_xyz  = (gsl_vector *) malloc(sizeof(gsl_vector));
      c->ref_quat = (gsl_vector *) malloc(sizeof(gsl_vector));
      if (   !c->pos_xyz || !c->pos_xyz
          || !c->ref_xyz || !c->ref_quat )
      {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         bt_control_cartesian_xyz_q_destroy(c);
         return 0;
      }
      
      view = gsl_vector_subvector( c->base.position, 0, 3);
      *(c->pos_xyz) = view.vector;
      view = gsl_vector_subvector( c->base.position, 3, 4);
      *(c->pos_quat) = view.vector;
      
      view = gsl_vector_subvector( c->base.reference, 0, 3);
      *(c->ref_xyz) = view.vector;
      view = gsl_vector_subvector( c->base.reference, 3, 4);
      *(c->ref_quat) = view.vector;
   }
   
   /* Start uninitialized */
   c->is_holding = 0;
   
   /* Save some stuff */
   c->kin = kin;
   c->dyn = dyn;
   
   /* Save pointers to external input vectors */
   /*c->cvelocity = kin->tool_velocity;*/
   /*c->jposition = jposition;
   c->jvelocity = jvelocity;*/
   
   c->force = gsl_vector_calloc(3);
   c->torque = gsl_vector_calloc(3);
   
   /* Create owned-by-me vectors */
   c->Kp = gsl_vector_calloc(3);
   c->Ki = gsl_vector_calloc(3);
   c->Kd = gsl_vector_calloc(3);
   c->integrator = gsl_vector_calloc(3);
   c->temp1 = gsl_vector_calloc(3);
   c->temp2 = gsl_vector_calloc(3);
   
   c->temp4vec = gsl_vector_calloc(4);
   
   /* Read in the xyz PID values: */
   {
      int j;
      config_setting_t * pids;
      /* Make sure the configuration looks good */
      if ( !(pids = config_setting_get_member( config, "pids" ))
                || (config_setting_type(pids)   != CONFIG_TYPE_GROUP) 
                || (config_setting_length(pids) != 4) )
      {
         syslog(LOG_ERR,"%s: The 'pids' configuration is not a 4-element group.",__func__);
         bt_control_cartesian_xyz_q_destroy(c);
         return 0;
      }
      /* Read in the XYZ PID values */
      for (j=0; j<3; j++)
      {
         double p,i,d;
         config_setting_t * pid_grp;
         
         pid_grp = config_setting_get_member( pids, str_dimension[j] );
         if ( !pid_grp
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "p" ), &p)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "i" ), &i)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "d" ), &d))
         {
            syslog(LOG_ERR,"%s: No p, i, and/or d value in %s.",__func__,str_dimension[j]);
            bt_control_cartesian_xyz_q_destroy(c);
            return 0;
         }
         
         gsl_vector_set(c->Kp,j,p);
         gsl_vector_set(c->Ki,j,i);
         gsl_vector_set(c->Kd,j,d);
      }
      /* Read in the rot PD values */
      {
         config_setting_t * pid_grp;
         pid_grp = config_setting_get_member( pids, "rot" );
         if ( !pid_grp
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "p" ), &c->rot_p)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "d" ), &c->rot_d))
         {
            syslog(LOG_ERR,"%s: No p, i, and/or d value in rot.",__func__);
            bt_control_cartesian_xyz_q_destroy(c);
            return 0;
         }
      }
   }
   
   return c;
}

void bt_control_cartesian_xyz_q_destroy(struct bt_control_cartesian_xyz_q * c)
{
   /* Free base vectors */
   if (c->base.position) gsl_vector_free(c->base.position);
   if (c->base.reference) gsl_vector_free(c->base.reference);
   
   /* Free vector views */
   if (c->pos_xyz)  free(c->pos_xyz);
   if (c->pos_quat) free(c->pos_quat);
   if (c->ref_xyz)  free(c->ref_xyz);
   if (c->ref_quat) free(c->ref_quat);

   if (c->force)  gsl_vector_free(c->force);
   if (c->torque) gsl_vector_free(c->torque);

   if (c->Kp) gsl_vector_free(c->Kp);
   if (c->Ki) gsl_vector_free(c->Ki);
   if (c->Kd) gsl_vector_free(c->Kd);
   if (c->integrator) gsl_vector_free(c->integrator);
   if (c->temp1) gsl_vector_free(c->temp1);
   if (c->temp2) gsl_vector_free(c->temp2);

   if (c->temp4vec) gsl_vector_free(c->temp4vec);
   
   free(c);
   return;
}

static int idle(struct bt_control * base)
{
   struct bt_control_cartesian_xyz_q * c = (struct bt_control_cartesian_xyz_q *) base;
   /* Do we need to stop doing anything? */
   c->is_holding = 0;
   return 0;
}

static int hold(struct bt_control * base)
{
   struct bt_control_cartesian_xyz_q * c = (struct bt_control_cartesian_xyz_q *) base;
   /* Do we need to call get_position first? */
   gsl_vector_memcpy(c->base.reference,c->base.position);
   gsl_vector_set_zero(c->integrator);
   c->last_time_saved = 0;
   c->is_holding = 1;
   return 0;
}

static int is_holding(struct bt_control * base)
{
   struct bt_control_cartesian_xyz_q * c = (struct bt_control_cartesian_xyz_q *) base;
   return c->is_holding ? 1 : 0;
}

/* This already points directly! */
static int get_position(struct bt_control * base)
{
   struct bt_control_cartesian_xyz_q * c = (struct bt_control_cartesian_xyz_q *) base;
   /* Copy the xyz position */
   gsl_vector_memcpy(c->pos_xyz, c->kin->tool->origin_pos);
   /* Compute the quaterion */
   /* wam->kin->tool->rot_to_world - 3x3 rotation matrix */
   rot_to_q( c->kin->tool->rot_to_world, c->pos_quat );
   return 0;
}

/* RT - Evaluate */
static int eval(struct bt_control * base, gsl_vector * jtorque, double time)
{
   struct bt_control_cartesian_xyz_q * c = (struct bt_control_cartesian_xyz_q *) base;
   double len;
   
   /* Do PID position control with the current reference */
   if (c->is_holding)
   {
      if (!c->last_time_saved)
      {
         c->last_time = time;
         c->last_time_saved = 1;
      }
      
      gsl_vector_set_zero(c->force);

      /* Compute the XYZ error */
      gsl_vector_memcpy( c->temp1, c->pos_xyz );
      gsl_vector_sub( c->temp1, c->ref_xyz );
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
      gsl_vector_memcpy( c->temp2, c->kin->tool_velocity );
      gsl_vector_mul( c->temp2, c->Kd );
      gsl_vector_sub( c->force, c->temp2 );

      /* Multiply by the Jacobian-transpose at the tool (force) */
      gsl_blas_dgemv( CblasTrans, 1.0, c->kin->tool_jacobian_linear,
                      c->force,
                      1.0, jtorque );
      
      
      /* Now, torque stuff ... */
      gsl_vector_set_zero(c->torque);
      
      /* First, normalize the reference quaternion */
      len = gsl_blas_dnrm2( c->ref_quat );
      gsl_blas_dscal( 1.0/len, c->ref_quat );
      
      /* Compute rotation from world to endpoint (reference)
       * then back to world (position) */
      q_mult_conj( c->temp4vec, c->pos_quat, c->ref_quat );
      
      /* Convert to angle * axis (3-vec)
       * Note that this is AT THE TOOL */
      q_to_angle_axis( c->temp4vec, c->temp1 );
      gsl_blas_dscal( c->rot_p, c->temp1 ); /* P TERM */
      
      /* Next, multiply it by the to_world transform
       * to get it in the world frame */
      gsl_blas_dgemv( CblasNoTrans, 1.0, c->kin->tool->rot_to_world,
                      c->temp1,
                      0.0, c->torque );
      
      /* Also, add in the angular velocity (D term) */
      gsl_blas_daxpy( - c->rot_d, c->kin->tool_velocity_angular, c->torque ); /* D TERM */
      
      /* Multiply by the Jacobian-transpose at the tool (torque) */
      gsl_blas_dgemv( CblasTrans, 1.0, c->kin->tool_jacobian_angular,
                      c->torque,
                      1.0, jtorque );
   }


   return 0;
}
