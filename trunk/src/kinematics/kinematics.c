/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... kinematics.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... Feb 18, 2005
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2005 Nov 07 - TH
 *      Minimal documentation in place.
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt
 *
 * ======================================================================== */

/* Read config files for the wam stuff */
#include <libconfig.h>
#include <syslog.h>

/* For fast matrix multiplication */
#include <gsl/gsl_blas.h>

/* For M_PI */
#include <gsl/gsl_math.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include "kinematics.h"

#include "gsl.h"

#include <math.h> /* For sin(), cos() */

/* Local function definitions */
static int eval_trans_to_prev( struct bt_kinematics_link * link );
static int eval_trans_to_world( struct bt_kinematics_link * link );

struct bt_kinematics * bt_kinematics_create( config_setting_t * kinconfig, int ndofs )
{
   int err;
   int i;
   config_setting_t * moving;
   struct bt_kinematics * kin;
   
   /* Check arguments */
   if (kinconfig == 0) return 0;
   
   kin = (struct bt_kinematics *) malloc( sizeof(struct bt_kinematics) );
   if (!kin)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Initialize */
   kin->link_array = 0;
   kin->base = 0;
   kin->link = 0;
   kin->toolplate = 0;
   kin->tool = 0;
   kin->tool_jacobian = 0;
   kin->tool_jacobian_linear = 0;
   kin->tool_velocity = 0;
   kin->temp_v3 = 0;
   
   /* Create the links array */
   kin->dof = ndofs;
   kin->nlinks = 1 + ndofs + 1 + 1; /* base, toolplate, and tool */
   kin->link_array = (struct bt_kinematics_link **)
      malloc(kin->nlinks*sizeof(struct bt_kinematics_link *));
   if (!kin->link_array)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_kinematics_destroy(kin);
      return 0;
   }
   for (i=0; i<kin->nlinks; i++)
      kin->link_array[i] = 0;
   for (i=0; i<kin->nlinks; i++)
   {
      struct bt_kinematics_link * link;
      link = (struct bt_kinematics_link *)
         malloc(sizeof(struct bt_kinematics_link));
      if (!link)
      {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         bt_kinematics_destroy(kin);
         return 0;
      }
      /* Initialize */
      link->next = 0;
      link->prev = 0;
      link->trans_to_prev = 0;
      link->trans_to_world = 0;
      link->rot_to_prev = 0;
      link->prev_axis_z = 0;
      link->prev_origin_pos = 0;
      link->rot_to_world = 0;
      link->axis_z = 0;
      link->origin_pos = 0;
      /* Save */
      kin->link_array[i] = link;
   }
   /* Set some pointers */
   kin->base = kin->link_array[0];
   kin->link = kin->link_array + 1;
   kin->toolplate = kin->link_array[kin->nlinks - 2];
   kin->tool = kin->link_array[kin->nlinks - 1];
   
   /* Make sure we have an appropriate configuration for moving links */
   if (   !(moving = config_setting_get_member(kinconfig,"moving"))
       ||  (config_setting_type(moving) != CONFIG_TYPE_LIST)
       ||  (config_setting_length(moving) != ndofs)
   ) {
      syslog(LOG_ERR,"%s: kin:moving not a list with %d elements.",__func__,ndofs);
      bt_kinematics_destroy(kin);
      return 0;
   }
   
   /* Initialize each link */
   for (i=0; i<kin->nlinks; i++)
   {
      struct bt_kinematics_link * link;
      link = kin->link_array[i];
      
      /* Set up next, prev ptrs */
      link->prev = (link == kin->base) ? 0 : kin->link_array[i-1];
      link->next = (link == kin->tool) ? 0 : kin->link_array[i+1];
      
      /* DH-Parameters from config file */
      if (link == kin->base)
      {         
         /* Set the base-to-world transform to the identity by default */
         link->trans_to_prev = gsl_matrix_calloc(4,4);
         if (!link->trans_to_prev)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         gsl_matrix_set_identity( link->trans_to_prev );
         
         /* Our transform to the previous frame is equivalent to
          * the transform to the world frame. */
         link->trans_to_world = link->trans_to_prev;
         
      }
      else if (link == kin->toolplate)
      {
         /* Set up from config file */
         config_setting_t * toolplate_grp;
         
         /* Get the toolplate group */
         if (   !(toolplate_grp = config_setting_get_member(kinconfig,"toolplate"))
             || (config_setting_type(toolplate_grp) != CONFIG_TYPE_GROUP)
         ) {
            syslog(LOG_ERR,"%s: kin:toolplate not a group.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         
         /* Grab each of the dh parameters */
         if (  ( err = bt_gsl_config_double_from_group(toolplate_grp,"alpha_pi",&(link->alpha)) )
            || ( err = bt_gsl_config_double_from_group(toolplate_grp,"theta_pi",&(link->theta)) )
            || ( err = bt_gsl_config_double_from_group(toolplate_grp,       "a",&(link->a)    ) )
            || ( err = bt_gsl_config_double_from_group(toolplate_grp,       "d",&(link->d)    ) )
            )
         {
            syslog(LOG_ERR,"%s: No alpha_pi, theta_pi, a, and/or d in toolplate, or not a number.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         link->alpha *= M_PI;
         link->theta *= M_PI;
         
         /* Evaluate a few cache things */
         link->cos_alpha = cos(link->alpha);
         link->sin_alpha = sin(link->alpha);
         
         /* Since the toolplate transform is static,
          * just compute trans_to_prev once (right now) */
         link->trans_to_prev = gsl_matrix_calloc(4,4);
         if (!link->trans_to_prev)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         gsl_matrix_set( link->trans_to_prev, 2,1, link->sin_alpha );
         gsl_matrix_set( link->trans_to_prev, 2,2, link->cos_alpha );
         gsl_matrix_set( link->trans_to_prev, 2,3, link->d );
         gsl_matrix_set( link->trans_to_prev, 3,3, 1.0 );
         eval_trans_to_prev( link );
         
         link->trans_to_world = gsl_matrix_calloc(4,4);
         if (!link->trans_to_world)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
      }
      else if (link == kin->tool)
      {
         /* By default, the tool is at the toolplate */
         link->trans_to_prev = gsl_matrix_calloc(4,4);
         if (!link->trans_to_prev)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         gsl_matrix_set_identity( link->trans_to_prev );
         
         link->trans_to_world = gsl_matrix_calloc(4,4);
         if (!link->trans_to_world)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
      }
      else
      {
         /* Set up from config file */
         int j;
         config_setting_t * link_grp;
         
         j = i-1; /* (There's one base frame) */
         
         /* Get the link #j moving group */
         link_grp = config_setting_get_elem( moving, j );
         if (config_setting_type(link_grp) != CONFIG_TYPE_GROUP)
         {
            syslog(LOG_ERR,"%s: kin:moving:#%d not a group.",__func__,j);
            bt_kinematics_destroy(kin);
            return 0;
         }
         
         /* Grab each of the dh parameters */
         if (  ( err = bt_gsl_config_double_from_group(link_grp,"alpha_pi",&(link->alpha)) )
            || ( err = bt_gsl_config_double_from_group(link_grp,       "a",&(link->a)    ) )
            || ( err = bt_gsl_config_double_from_group(link_grp,       "d",&(link->d)    ) )
            )
         {
            syslog(LOG_ERR,"%s: No alpha_pi, theta_pi, a, and/or d in link %d, or not a number.",__func__,j);
            bt_kinematics_destroy(kin);
            return 0;
         }
         link->alpha *= M_PI;
         
         /* Evaluate a few cache things */
         link->cos_alpha = cos(link->alpha);
         link->sin_alpha = sin(link->alpha);
         
         /* Initialize the matrices (the last two rows are const for revolute) */
         link->trans_to_prev = gsl_matrix_calloc(4,4);
         if (!link->trans_to_prev)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         gsl_matrix_set( link->trans_to_prev, 2,1, link->sin_alpha );
         gsl_matrix_set( link->trans_to_prev, 2,2, link->cos_alpha );
         gsl_matrix_set( link->trans_to_prev, 2,3, link->d );
         gsl_matrix_set( link->trans_to_prev, 3,3, 1.0 );
         
         link->trans_to_world = gsl_matrix_calloc(4,4);
         if (!link->trans_to_world)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
      }
      
      /* Set up the vector views */
      {
         gsl_vector_view view;
         
         link->prev_axis_z = (gsl_vector *) malloc(sizeof(gsl_vector));
         if (!link->prev_axis_z)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         view = gsl_matrix_subrow( link->trans_to_prev, 2, 0, 3);
         *(link->prev_axis_z) = view.vector;
         
         link->prev_origin_pos = (gsl_vector *) malloc(sizeof(gsl_vector));
         if (!link->prev_origin_pos)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         view = gsl_matrix_subcolumn( link->trans_to_prev, 3, 0, 3);
         *(link->prev_origin_pos) = view.vector;
         
         link->axis_z = (gsl_vector *) malloc(sizeof(gsl_vector));
         if (!link->axis_z)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         view = gsl_matrix_subcolumn( link->trans_to_world, 2, 0, 3);
         *(link->axis_z) = view.vector;
         
         link->origin_pos = (gsl_vector *) malloc(sizeof(gsl_vector));
         if (!link->origin_pos)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         view = gsl_matrix_subcolumn( link->trans_to_world, 3, 0, 3);
         *(link->origin_pos) = view.vector;
      }
      /* Set up the matrix views */
      {
         gsl_matrix_view view;
         
         link->rot_to_prev = (gsl_matrix *) malloc(sizeof(gsl_matrix));
         if (!link->rot_to_prev)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         view = gsl_matrix_submatrix( link->trans_to_prev, 0,0, 3,3 );
         *(link->rot_to_prev) = view.matrix;
         
         link->rot_to_world = (gsl_matrix *) malloc(sizeof(gsl_matrix));
         if (!link->rot_to_world)
         {
            syslog(LOG_ERR,"%s: Out of memory.",__func__);
            bt_kinematics_destroy(kin);
            return 0;
         }
         view = gsl_matrix_submatrix( link->trans_to_world, 0,0, 3,3 );
         *(link->rot_to_world) = view.matrix;
      }
   }
   
   /* Make the toolplate jacobian matrix, view, and tool velocity */
   kin->tool_jacobian = gsl_matrix_alloc( 6, ndofs );
   if (!kin->tool_jacobian)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_kinematics_destroy(kin);
      return 0;
   }
   {
      gsl_matrix_view view;
      
      kin->tool_jacobian_linear = (gsl_matrix *) malloc(sizeof(gsl_matrix));
      if (!kin->tool_jacobian_linear)
      {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         bt_kinematics_destroy(kin);
         return 0;
      }
      view = gsl_matrix_submatrix( kin->tool_jacobian, 0,0, 3,ndofs );
      *(kin->tool_jacobian_linear) = view.matrix;
   }
   kin->tool_velocity = gsl_vector_alloc( 3 );
   if (!kin->tool_velocity)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_kinematics_destroy(kin);
      return 0;
   }
   
   /* Make temporary vectors */
   kin->temp_v3 = gsl_vector_alloc(3);
   if (!kin->temp_v3)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_kinematics_destroy(kin);
      return 0;
   }

   return kin;
}

int bt_kinematics_destroy( struct bt_kinematics * kin )
{
   int i;
   
   if (kin->tool_jacobian)
      gsl_matrix_free(kin->tool_jacobian);
   if (kin->tool_jacobian_linear)
      free(kin->tool_jacobian_linear);
   if (kin->tool_velocity)
      gsl_vector_free(kin->tool_velocity);
   if (kin->temp_v3)
      gsl_vector_free(kin->temp_v3);
   
   for (i=0; i<kin->nlinks; i++)
   if (kin->link_array[i])
   {
      struct bt_kinematics_link * link;
      link = kin->link_array[i];
      
      /* Free the views */
      if (link->prev_axis_z)
         free(link->prev_axis_z);
      if (link->prev_origin_pos)
         free(link->prev_origin_pos);
      if (link->axis_z)
         free(link->axis_z);
      if (link->origin_pos)
         free(link->origin_pos);
      if (link->rot_to_prev)
         free(link->rot_to_prev);
      if (link->rot_to_world)
         free(link->rot_to_world);
      
      /* Free the matrices */
      if ((link != kin->base) && (link->trans_to_world))
         gsl_matrix_free(link->trans_to_world);
      if (link->trans_to_prev)
         gsl_matrix_free(link->trans_to_prev);
      
      free(link);
   }
   
   if (kin->link_array) free(kin->link_array);
   free(kin);
   return 0;
}

int bt_kinematics_eval( struct bt_kinematics * kin, gsl_vector * jposition, gsl_vector * jvelocity )
{
   int j;
   
   /* Evaluate transforms from the base to the tool */
   for (j=0; j<kin->dof; j++)
   {
      struct bt_kinematics_link * link;
      link = kin->link[j];
     
      /* Step 1: Copy current joint positions into kinematics struct
       *         (revolute only) */
      link->theta = gsl_vector_get( jposition, j );
      
      /* Step 2: Evaluate the to_prev transform */
      eval_trans_to_prev( link );
      
      /* Step 3: Evaluate the to_world transform */
      eval_trans_to_world( link );
   }
   
   /* Also evaluate to_world for the tool (static to_prev) */
   eval_trans_to_world( kin->toolplate );
   
   /* Allow user to change tool transform here? (callback?) */
   eval_trans_to_world( kin->tool );
   
   /* Calculate the tool jacobian 
    * (gives vel of tool in world coords) */
   bt_kinematics_eval_jacobian( kin,
      kin->dof, kin->tool->origin_pos, kin->tool_jacobian );
   /* Calculate the tool Cartesian velocity */
   gsl_blas_dgemv( CblasNoTrans, 1.0, kin->tool_jacobian_linear,
                   jvelocity,
                   0.0, kin->tool_velocity );
   
   return 0;
}

int bt_kinematics_eval_jacobian( struct bt_kinematics * kin,
   int jlimit, gsl_vector * point, gsl_matrix * jac)
{

   int j;
   gsl_vector_view Jvj; /* Linear Velocity Jacobian, Link j */
   gsl_vector_view Jwj; /* Angular Velocity Jacobian, Link j */
   
   for (j=0; j<kin->dof; j++)
   {
      Jvj = gsl_matrix_subcolumn( jac, j, 0, 3 );
      Jwj = gsl_matrix_subcolumn( jac, j, 3, 3 );
      
      if (j < jlimit)
      {
         /* Jvj = z_(j-1) x (point - o_(j-1)) */
         gsl_vector_memcpy( kin->temp_v3, point );
         gsl_vector_sub( kin->temp_v3, kin->link[j-1]->origin_pos );
         gsl_vector_set_zero( &Jvj.vector );
         bt_gsl_cross( kin->link[j-1]->axis_z, kin->temp_v3, &Jvj.vector );

         /* Jwj = z(j-1) */
         gsl_vector_memcpy( &Jwj.vector, kin->link[j-1]->axis_z );
      }
      else
      {
         /* Set the jth column to 0. */
         gsl_vector_set_zero( &Jvj.vector );
         gsl_vector_set_zero( &Jwj.vector );
      }
   }
   
   return 0;
}

/* Forward Kinematics - Homogeneous Transform Matrix A
 * (Spong p. 61) */
static int eval_trans_to_prev( struct bt_kinematics_link * link )
{   
   /* Right now this is only for revolute joints */
   double cos_theta;
   double sin_theta;
   
   cos_theta = cos(link->theta);
   sin_theta = sin(link->theta);
   
   gsl_matrix_set( link->trans_to_prev, 0,0,  cos_theta );
   gsl_matrix_set( link->trans_to_prev, 1,0,  sin_theta );
   gsl_matrix_set( link->trans_to_prev, 0,1, -sin_theta * link->cos_alpha );
   gsl_matrix_set( link->trans_to_prev, 1,1,  cos_theta * link->cos_alpha );
   gsl_matrix_set( link->trans_to_prev, 0,2,  sin_theta * link->sin_alpha );
   gsl_matrix_set( link->trans_to_prev, 1,2, -cos_theta * link->sin_alpha );
   gsl_matrix_set( link->trans_to_prev, 0,3,  cos_theta * link->a );
   gsl_matrix_set( link->trans_to_prev, 1,3,  sin_theta * link->a );
   
   return 0;
}

/* Note - assume this isn't the world link */
static int eval_trans_to_world( struct bt_kinematics_link * link )
{   
   /* Matrix Multiply */
   gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0,
                  link->prev->trans_to_world,
                  link->trans_to_prev,
                  0.0, link->trans_to_world);
   return 0;
}




