/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_draw2d.c
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
#include "control_draw2dq.h"

#include <libbarrett/dynamics.h>
#include <libbarrett/gsl.h>

#include <libconfig.h>
#include <syslog.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h> /* For M_PI */

/* This is just so we know what to look for in the config file */
static char *str_dimension[] = {"x","y","z"};

/* Define the type */
static int idle(struct bt_control * base);
static int hold(struct bt_control * base);
static int is_holding(struct bt_control * base);
static int get_position(struct bt_control * base);
static int eval(struct bt_control * base, gsl_vector * jtorque, double time);
static const struct bt_control_type control_draw2dq_type = {
   "draw2dq-space",
   &idle,
   &hold,
   &is_holding,
   &get_position,
   &eval
};
const struct bt_control_type * control_draw2dq = &control_draw2dq_type;

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
static int rot_to_q( gsl_matrix * rot, gsl_vector * quat )
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
static int q_mult_conj( gsl_vector * qa,  gsl_vector * qb,  gsl_vector * qc )
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

static int q_to_angle_axis( gsl_vector * q, gsl_vector * angleaxis )
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

/* Local static functions */
static int set_transform_matrix(struct control_draw2dq * c);

/* Controller-specific functions */
struct control_draw2dq * control_draw2dq_create(config_setting_t * config,
   struct bt_kinematics * kin, struct bt_dynamics * dyn)
{
   int err;
   struct control_draw2dq * c;
   
   /* Create */
   c = (struct control_draw2dq *) malloc( sizeof(struct control_draw2dq) );
   if (!c)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Initialize */
   c->base.type = control_draw2dq;
   c->base.n = 4;
   c->base.position = 0;
   c->base.reference = 0;
   c->is_holding = 0;
   c->kin = kin;
   c->dyn = dyn;
   c->base_pos_xy = 0;
   c->base_pos_vxvy = 0;
   c->base_ref_xy = 0;
   c->base_ref_vxvy = 0;
   c->position = 0;
   c->reference = 0;
   c->pos_xy = 0;
   c->pos_xyz = 0;
   c->pos_quat = 0;
   c->ref_xy = 0;
   c->ref_xyz = 0;
   c->ref_quat = 0;
   c->p_topleft = 0;
   c->p_topright = 0;
   c->p_onpage = 0;
   c->page_to_world = 0;
   c->state = CONTROL_DRAW2DQ_STATE_HOVER;
   c->rot_align_done = 0;
   c->Kp = 0;
   c->Ki = 0;
   c->Kd = 0;
   c->integrator = 0;
   c->temp1 = 0;
   c->temp2 = 0;
   c->rot_p = 20.0;
   c->rot_d = 0.2;
   c->temp_set = 0;
   c->tool_to_page = 0;
   c->tool_to_page_y = 0;
   c->tool_to_page_z = 0;
   c->ref_tool_to_page = 0;
   c->ref_tool_to_page_x = 0;
   c->ref_tool_to_page_x_xy = 0;
   c->ref_tool_to_page_y = 0;
   c->ref_tool_to_page_z = 0;
   c->temp4vec = 0;
   c->last_time_saved = 0;
   c->page_force = 0;
   c->world_force = 0;
   c->world_torque = 0;
   c->pressure = 15.0; /* N */
   c->hover_distance = 0.05; /* m */
   
   /* Allocate */
   if (   !(c->base.position  = gsl_vector_calloc(4)  )
       || !(c->base.reference = gsl_vector_calloc(4)  )
       || !(c->base_pos_xy    = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->base_pos_vxvy  = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->base_ref_xy    = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->base_ref_vxvy  = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->position       = gsl_vector_calloc(7)  )
       || !(c->reference      = gsl_vector_calloc(7)  )
       || !(c->pos_xy         = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->pos_xyz        = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->pos_quat       = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->ref_xy         = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->ref_xyz        = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->ref_quat       = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->p_topleft      = gsl_vector_calloc(3)  )
       || !(c->p_topright     = gsl_vector_calloc(3)  )
       || !(c->p_onpage       = gsl_vector_calloc(3)  )
       || !(c->page_to_world  = gsl_matrix_calloc(3,3))
       || !(c->Kp             = gsl_vector_calloc(3)  )
       || !(c->Ki             = gsl_vector_calloc(3)  )
       || !(c->Kd             = gsl_vector_calloc(3)  )
       || !(c->integrator     = gsl_vector_calloc(3)  )
       || !(c->temp1          = gsl_vector_calloc(3)  )
       || !(c->temp2          = gsl_vector_calloc(3)  )
       || !(c->temp_set       = gsl_vector_calloc(3)  )
       || !(c->tool_to_page   = gsl_matrix_calloc(3,3))
       || !(c->tool_to_page_y = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->tool_to_page_z = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->ref_tool_to_page      = gsl_matrix_calloc(3,3))
       || !(c->ref_tool_to_page_x    = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->ref_tool_to_page_x_xy = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->ref_tool_to_page_y    = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->ref_tool_to_page_z    = (gsl_vector *) malloc(sizeof(gsl_vector)) )
       || !(c->temp4vec       = gsl_vector_calloc(4)  )
       || !(c->page_force     = gsl_vector_calloc(3)  )
       || !(c->world_force    = gsl_vector_calloc(3)  )
       || !(c->world_torque   = gsl_vector_calloc(3)  ))
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      control_draw2dq_destroy(c);
      return 0;
   }
   
   /* Set up vector views */
   {
      gsl_vector_view view;
      
      /* Views into c->base.position */
      view = gsl_vector_subvector(c->base.position,0,2);
      *(c->base_pos_xy) = view.vector;
      view = gsl_vector_subvector(c->base.position,2,2);
      *(c->base_pos_vxvy) = view.vector;
      
      /* Views into c->base.reference */
      view = gsl_vector_subvector(c->base.reference,0,2);
      *(c->base_ref_xy) = view.vector;
      view = gsl_vector_subvector(c->base.reference,2,2);
      *(c->base_ref_vxvy) = view.vector;
      
      /* Views into c->position */
      view = gsl_vector_subvector(c->position,0,2);
      *(c->pos_xy) = view.vector;
      view = gsl_vector_subvector(c->position,0,3);
      *(c->pos_xyz) = view.vector;
      view = gsl_vector_subvector(c->position,3,4);
      *(c->pos_quat) = view.vector;
      
      /* Views into c->reference */
      view = gsl_vector_subvector(c->reference,0,2);
      *(c->ref_xy) = view.vector;
      view = gsl_vector_subvector(c->reference,0,3);
      *(c->ref_xyz) = view.vector;
      view = gsl_vector_subvector(c->reference,3,4);
      *(c->ref_quat) = view.vector;
      
      /* Views into c->tool_to_page */
      view = gsl_matrix_column( c->tool_to_page, 1 );
      *(c->tool_to_page_y) = view.vector;
      view = gsl_matrix_column( c->tool_to_page, 2 );
      *(c->tool_to_page_z) = view.vector;
      
      /* Views into c->ref_tool_to_page */
      view = gsl_matrix_subcolumn( c->ref_tool_to_page, 0, 0, 2 );
      *(c->ref_tool_to_page_x_xy) = view.vector;
      view = gsl_matrix_column( c->ref_tool_to_page, 0 );
      *(c->ref_tool_to_page_x) = view.vector;
      view = gsl_matrix_column( c->ref_tool_to_page, 1 );
      *(c->ref_tool_to_page_y) = view.vector;
      view = gsl_matrix_column( c->ref_tool_to_page, 2 );
      *(c->ref_tool_to_page_z) = view.vector;
   }
   
   /* Set default page location
    * (the page defaults in the YZ plane, directly above the WAM) */
   
   gsl_vector_set(c->p_topleft,0,  0.00 );
   gsl_vector_set(c->p_topleft,1, +0.10 );
   gsl_vector_set(c->p_topleft,2,  0.75 );
   
   gsl_vector_set(c->p_topright,0,  0.00 );
   gsl_vector_set(c->p_topright,1, -0.10 );
   gsl_vector_set(c->p_topright,2,  0.75 );
   
   gsl_vector_set(c->p_onpage,0,  0.00 );
   gsl_vector_set(c->p_onpage,1,  0.00 );
   gsl_vector_set(c->p_onpage,2,  0.50 );
   
   /* Set the rotation matrix based on the above defaults */
   err = set_transform_matrix(c);
   if (err)
   {
      syslog(LOG_ERR,"%s: Bad default page location.",__func__);
      control_draw2dq_destroy(c);
      return 0;
   }
   
   /* Read in the PID values from the configuration */
   {
      int j;
      config_setting_t * pids;
      /* Make sure the configuration looks good */
      if ( !(pids = config_setting_get_member( config, "pids" ))
                || (config_setting_type(pids)   != CONFIG_TYPE_GROUP) 
                || (config_setting_length(pids) != 4) )
      {
         syslog(LOG_ERR,"%s: The 'pids' configuration is not a 4-element group.",__func__);
         control_draw2dq_destroy(c);
         return 0;
      }
      /* Read in the PID values */
      for (j=0; j<3; j++)
      {
         double p,i,d;
         config_setting_t * pid_grp;
         
         pid_grp = config_setting_get_member( pids, str_dimension[j] );
         if (!pid_grp)
         {
            syslog(LOG_ERR,"%s: No member of pids called %s.",__func__,str_dimension[j]);
            control_draw2dq_destroy(c);
            return 0;
         }
         if (   bt_gsl_config_get_double(config_setting_get_member( pid_grp, "p" ), &p)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "i" ), &i)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "d" ), &d))
         {
            syslog(LOG_ERR,"%s: No p, i, and/or d value",__func__);
            control_draw2dq_destroy(c);
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
            syslog(LOG_ERR,"%s: No p and/or d value in rot.",__func__);
            control_draw2dq_destroy(c);
            return 0;
         }
      }
   }
   
   return c;
}

void control_draw2dq_destroy(struct control_draw2dq * c)
{
   /* De-allocate */
   if (c->base.position)  gsl_vector_free(c->base.position);
   if (c->base.reference) gsl_vector_free(c->base.reference);
   if (c->base_pos_xy)    free(c->base_pos_xy);
   if (c->base_pos_vxvy)  free(c->base_pos_vxvy);
   if (c->base_ref_xy)    free(c->base_ref_xy);
   if (c->base_ref_vxvy)  free(c->base_ref_vxvy);
   if (c->position)       gsl_vector_free(c->position);
   if (c->reference)      gsl_vector_free(c->reference);
   if (c->pos_xy)         free(c->pos_xy);
   if (c->pos_xyz)        free(c->pos_xyz);
   if (c->pos_quat)       free(c->pos_quat);
   if (c->ref_xy)         free(c->ref_xy);
   if (c->ref_xyz)        free(c->ref_xyz);
   if (c->ref_quat)       free(c->ref_quat);
   if (c->p_topleft)      gsl_vector_free(c->p_topleft);
   if (c->p_topright)     gsl_vector_free(c->p_topright);
   if (c->p_onpage)       gsl_vector_free(c->p_onpage);
   if (c->page_to_world)  gsl_matrix_free(c->page_to_world);
   if (c->Kp)             gsl_vector_free(c->Kp);
   if (c->Ki)             gsl_vector_free(c->Ki);
   if (c->Kd)             gsl_vector_free(c->Kd);
   if (c->integrator)     gsl_vector_free(c->integrator);
   if (c->temp1)          gsl_vector_free(c->temp1);
   if (c->temp2)          gsl_vector_free(c->temp2);
   if (c->temp_set)       gsl_vector_free(c->temp_set);
   if (c->tool_to_page)   gsl_matrix_free(c->tool_to_page);
   if (c->tool_to_page_y) free(c->tool_to_page_y);
   if (c->tool_to_page_z) free(c->tool_to_page_z);
   if (c->ref_tool_to_page)      gsl_matrix_free(c->ref_tool_to_page);
   if (c->ref_tool_to_page_x)    free(c->ref_tool_to_page_x);
   if (c->ref_tool_to_page_x_xy) free(c->ref_tool_to_page_x_xy);
   if (c->ref_tool_to_page_y)    free(c->ref_tool_to_page_y);
   if (c->ref_tool_to_page_z)    free(c->ref_tool_to_page_z);
   if (c->temp4vec)       gsl_vector_free(c->temp4vec);
   if (c->page_force)     gsl_vector_free(c->page_force);
   if (c->world_force)    gsl_vector_free(c->world_force);
   if (c->world_torque)   gsl_vector_free(c->world_torque);
   
   /* Destroy */
   free(c);
   return;
}

static int set_transform_matrix(struct control_draw2dq * c)
{
   gsl_vector_view x, y, z;
   double len;
   
   /* Make vector views of the page_to_world rotation matrix */
   x = gsl_matrix_column( c->page_to_world, 0);
   y = gsl_matrix_column( c->page_to_world, 1);
   z = gsl_matrix_column( c->page_to_world, 2);
   
   /* x is the unit vector (p_topright - p_topleft) */
   gsl_blas_dcopy(c->p_topright, &x.vector);
   gsl_blas_daxpy(-1, c->p_topleft, &x.vector);
   len = gsl_blas_dnrm2( &x.vector );
   gsl_blas_dscal( 1.0 / len, &x.vector );
   
   /* z is the unit vector (x crossed into (p_onpage - p_topleft)) */
   gsl_blas_dcopy(c->p_onpage, c->temp_set);
   gsl_blas_daxpy(-1, c->p_topleft, c->temp_set);
   bt_gsl_cross( &x.vector, c->temp_set, &z.vector );
   len = gsl_blas_dnrm2( &z.vector );
   gsl_blas_dscal( 1.0 / len, &z.vector );
   
   /* y is (z crossed into x) */
   gsl_vector_set_zero(&y.vector);
   bt_gsl_cross( &z.vector, &x.vector, &y.vector );
   
   return 0;
}

int control_draw2dq_set_topleft(struct control_draw2dq * c)
{
   gsl_blas_dcopy( c->kin->tool->origin_pos, c->p_topleft );
   set_transform_matrix(c);
   return 0;
}

int control_draw2dq_set_topright(struct control_draw2dq * c)
{
   gsl_blas_dcopy( c->kin->tool->origin_pos, c->p_topright );
   set_transform_matrix(c);
   return 0;
}

int control_draw2dq_set_onpage(struct control_draw2dq * c)
{
   gsl_blas_dcopy( c->kin->tool->origin_pos, c->p_onpage );
   set_transform_matrix(c);
   return 0;
}

int control_draw2dq_set_pressure(struct control_draw2dq * c, double pressure)
{
   c->pressure = pressure;
   return 0;
}

int control_draw2dq_set_hover_distance(struct control_draw2dq * c, double distance)
{
   c->hover_distance = distance;
   return 0;
}

int control_draw2dq_press(struct control_draw2dq * c)
{
   switch (c->state)
   {
      case CONTROL_DRAW2DQ_STATE_PRESSURE:
         break;
      case CONTROL_DRAW2DQ_STATE_HOVER:
      case CONTROL_DRAW2DQ_STATE_MOVEIN:
      case CONTROL_DRAW2DQ_STATE_MOVEOUT:
         c->state = CONTROL_DRAW2DQ_STATE_MOVEIN;
         break;
   }
   return 0;
}

int control_draw2dq_hover(struct control_draw2dq * c)
{
   switch (c->state)
   {
      case CONTROL_DRAW2DQ_STATE_HOVER:
         break;
      case CONTROL_DRAW2DQ_STATE_PRESSURE:
      case CONTROL_DRAW2DQ_STATE_MOVEIN:
      case CONTROL_DRAW2DQ_STATE_MOVEOUT:
         gsl_vector_set(c->integrator,2,0.0);
         /* Set the hidden z value in the reference to the current position */
         gsl_vector_set(c->reference,2, gsl_vector_get(c->position,2) );
         c->state = CONTROL_DRAW2DQ_STATE_MOVEOUT;
         break;
   }
   return 0;
}




static int idle(struct bt_control * base)
{
   struct control_draw2dq * c = (struct control_draw2dq *) base;
   /* Do we need to stop doing anything? */
   c->is_holding = 0;
   return 0;
}

static int hold(struct bt_control * base)
{
   struct control_draw2dq * c = (struct control_draw2dq *) base;
   
   /* Already holding? */
   if (c->is_holding)
      return 1;
   
   /* Convert current 4d base position to 4d base reference */
   gsl_vector_memcpy(c->base.reference, c->base.position);
   
   /* Save current z reference value */
   gsl_vector_set(c->reference,2, gsl_vector_get(c->position,2) );
   
   /* Save current tool z axis */
   gsl_vector_memcpy(c->ref_tool_to_page_z, c->tool_to_page_z);
   
   /* Zero the xyz integrators! */
   gsl_vector_set_zero(c->integrator);
   
   /* We're now in hover mode */
   c->state = CONTROL_DRAW2DQ_STATE_HOVER;
   c->rot_align_done = 0;
   c->last_time_saved = 0;
   c->is_holding = 1;
   return 0;
}

static int is_holding(struct bt_control * base)
{
   struct control_draw2dq * c = (struct control_draw2dq *) base;
   return c->is_holding ? 1 : 0;
}

static int get_position(struct bt_control * base)
{
   double len;
   struct control_draw2dq * c = (struct control_draw2dq *) base;
   
   /* Get cartesian position into c->pos_xyz, in page frame */
   gsl_blas_dgemv( CblasTrans, -1.0, c->page_to_world,
                   c->p_topleft,
                   0.0, c->pos_xyz );
   gsl_blas_dgemv( CblasTrans, 1.0, c->page_to_world,
                   c->kin->tool->origin_pos,
                   1.0, c->pos_xyz );
   /* Copy x,y of page cartesian position into c->base.position (0,1) */
   gsl_vector_memcpy(c->base_pos_xy, c->pos_xy);
   
   /* Get rotation matrix of tool in page frame
    * page^R_tool = page^R_world * world^R_tool */
   gsl_blas_dgemm( CblasTrans, CblasNoTrans, 1.0,
                   c->page_to_world, c->kin->tool->rot_to_world,
                   0.0, c->tool_to_page );
   /* Convert to quaternion in cartesian position */
   rot_to_q( c->tool_to_page, c->pos_quat );
   /* compute c->base.position (2,3) = y_tool x z_page
    * where z_page is always (0,0,1);
    * this is to get the vector in the page xy plane
    * to be used to reconstruct the tool's frame */
   gsl_vector_set( c->base_pos_vxvy, 0,
                   gsl_vector_get(c->tool_to_page_y,1));
   gsl_vector_set( c->base_pos_vxvy, 1,
                   - gsl_vector_get(c->tool_to_page_y,0));
   /* Normalize */
   len = gsl_blas_dnrm2( c->base_pos_vxvy );
   gsl_blas_dscal( 1.0 / len, c->base_pos_vxvy );
   
   return 0;
}

/* RT - Evaluate */
static int eval(struct bt_control * base, gsl_vector * jtorque, double time)
{
   double len;
   struct control_draw2dq * c = (struct control_draw2dq *) base;
   
   /* Do PID position control with the current reference */
   if (c->is_holding)
   {
      /* For the xyz integrators */
      if (!c->last_time_saved)
      {
         c->last_time = time;
         c->last_time_saved = 1;
      }
      
      /* Compute c->reference (7d) from c->base.reference (4d) */
      gsl_vector_memcpy(c->ref_xy, c->base_ref_xy);
      /* c->reference[2] (z) is already set (i own) */
      /* c->ref_tool_to_page_z already set */
      /* Copy in (x,y) vector as tool x unit vector */
      gsl_vector_memcpy(c->ref_tool_to_page_x_xy, c->base_ref_vxvy);
      gsl_matrix_set(c->ref_tool_to_page, 2, 0, 0.0 );
      /* tool y = tool z cross tool x */
      gsl_vector_set_zero(c->ref_tool_to_page_y);
      bt_gsl_cross( c->ref_tool_to_page_z, c->ref_tool_to_page_x, c->ref_tool_to_page_y );
      /* Normalize */
      len = gsl_blas_dnrm2( c->ref_tool_to_page_y );
      gsl_blas_dscal( 1.0 / len, c->ref_tool_to_page_y );
      /* tool x = tool y cross tool z */
      gsl_vector_set_zero(c->ref_tool_to_page_x);
      bt_gsl_cross( c->ref_tool_to_page_y, c->ref_tool_to_page_z, c->ref_tool_to_page_x );
      /* No need to normalize */
      /* Convert to quaternion */
      rot_to_q( c->ref_tool_to_page, c->ref_quat );
      
      /* Simple reference control of z direction (i own)
       * for moves */
      switch (c->state)
      {
         case CONTROL_DRAW2DQ_STATE_HOVER:
         case CONTROL_DRAW2DQ_STATE_PRESSURE:
            break;
         case CONTROL_DRAW2DQ_STATE_MOVEIN:
            *(gsl_vector_ptr(c->reference,2)) += 0.1 * (time - c->last_time); /* m/s */
            break;
         case CONTROL_DRAW2DQ_STATE_MOVEOUT:
            *(gsl_vector_ptr(c->reference,2)) -= 0.1 * (time - c->last_time); /* m/s */
            if (gsl_vector_get(c->reference,2) < - c->hover_distance)
            {
               gsl_vector_set(c->reference,2, - c->hover_distance );
               c->state = CONTROL_DRAW2DQ_STATE_HOVER;
            }
            break;
      }
      
      /* Slerp the angle until it's aligned with z into page */
      if (!c->rot_align_done)
      {
         double ang_total; /* omega */
         double ang_diff;
         
         ang_total = acos( gsl_vector_get(c->ref_tool_to_page_z,2) );
         ang_diff = 0.3 * (time - c->last_time); /* rad/s */
         
         if (ang_diff >= ang_total)
         {
            gsl_vector_set_zero(c->ref_tool_to_page_z);
            gsl_vector_set(c->ref_tool_to_page_z,2, 1.0 );
            c->rot_align_done = 1;
         }
         else
         {
            gsl_blas_dscal( sin(ang_total-ang_diff) / sin(ang_total),
                            c->ref_tool_to_page_z );
            *(gsl_vector_ptr(c->ref_tool_to_page_z,2)) += 
               sin(ang_diff) / sin(ang_total);
         }
      }
      
      /* Get current position
       * (this is guarenteed to happen in the wam loop) */
      
      /* Compute page force */
      gsl_vector_set_zero(c->page_force);
      /* Compute the error */
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
      gsl_vector_sub( c->page_force, c->temp2 );
      /* Copy in I term */
      gsl_vector_memcpy( c->temp2, c->integrator);
      gsl_vector_mul( c->temp2, c->Ki );
      gsl_vector_sub( c->page_force, c->temp2 );
      /* Copy in D term */
      gsl_vector_memcpy( c->temp2, c->kin->tool_velocity );
      gsl_vector_mul( c->temp2, c->Kd );
      gsl_vector_sub( c->page_force, c->temp2 );
      
      /* Switch from STATE_MOVEIN to STATE_HOVER if necessary */
      switch (c->state)
      {
         case CONTROL_DRAW2DQ_STATE_HOVER:
         case CONTROL_DRAW2DQ_STATE_PRESSURE:
         case CONTROL_DRAW2DQ_STATE_MOVEOUT:
            break;
         case CONTROL_DRAW2DQ_STATE_MOVEIN:
            if (gsl_vector_get(c->page_force,2) > 2 * c->pressure)
               c->state = CONTROL_DRAW2DQ_STATE_PRESSURE;
      }
      
      /* If we're in pressure mode, turn the z force to c->pressure */
      if (c->state == CONTROL_DRAW2DQ_STATE_PRESSURE)
         gsl_vector_set(c->page_force,2, c->pressure);
      
      /* Convert page force to world force */
      gsl_blas_dgemv( CblasNoTrans, 1.0, c->page_to_world,
                      c->page_force,
                      0.0, c->world_force );

      /* Multiply by the Jacobian-transpose at the tool */
      gsl_blas_dgemv( CblasTrans, 1.0, c->kin->tool_jacobian_linear,
                      c->world_force,
                      1.0, jtorque );
      
      
      /* Now, the rotation stuff */
      
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
                      0.0, c->world_torque );
      
      /* Also, add in the angular velocity (D term) */
      gsl_blas_daxpy( - c->rot_d, c->kin->tool_velocity_angular, c->world_torque ); /* D TERM */
      
      /* Multiply by the Jacobian-transpose at the tool (torque) */
      gsl_blas_dgemv( CblasTrans, 1.0, c->kin->tool_jacobian_angular,
                      c->world_torque,
                      1.0, jtorque );
   }

   return 0;
}
