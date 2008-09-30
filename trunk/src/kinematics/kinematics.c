/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... kinematics.h
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

#include <math.h> /* For sin(), cos() */

/* Local function definitions */
static int eval_trans_to_prev( struct bt_kinematics_link * link );
static int eval_trans_to_inertial( struct bt_kinematics_link * link );

struct bt_kinematics * bt_kinematics_create( config_setting_t * kinconfig, int ndofs )
{
   int i;
   config_setting_t * moving;
   struct bt_kinematics * kin;
   kin = (struct bt_kinematics *) malloc( sizeof(struct bt_kinematics) );
   
   /* Save the passed wam ptr */
   /*kin->bot = bot;*/
   
   /* Create the links array */
   kin->dof = ndofs;
   kin->nlinks = 1 + ndofs + 1; /* world and tool! */
   kin->link_array = (struct bt_kinematics_link **)
      malloc(kin->nlinks*sizeof(struct bt_kinematics_link *));
   for (i=0; i<kin->nlinks; i++)
      kin->link_array[i] = (struct bt_kinematics_link *)
         malloc(sizeof(struct bt_kinematics_link));
   kin->world = kin->link_array[0];
   kin->link = kin->link_array + 1;
   kin->tool = kin->link_array[kin->nlinks - 1];
   
   /* Make sure we have an appropriate configuration for moving links */
   moving = config_setting_get_member( kinconfig, "moving" );
   if (moving == NULL) { printf("No kin:moving in wam.config\n"); return 0; }
   if (config_setting_type(moving) != CONFIG_TYPE_LIST)
      { printf("kin:moving not a list\n"); return 0; }
   if (config_setting_length(moving) != ndofs)
      { printf("Expected %d links, but found %d.\n",
               ndofs, config_setting_length(moving)); return 0; }
   
   /* Initialize each link */
   for (i=0; i<kin->nlinks; i++)
   {
      struct bt_kinematics_link * link;
      link = kin->link_array[i];
      
      /* Set up next, prev ptrs */
      link->prev = (link == kin->world) ? 0 : kin->link_array[i-1];
      link->next = (link == kin->tool)  ? 0 : kin->link_array[i+1];
      
      /* DH-Parameters from config file */
      if (link == kin->world)
      {
         /* Initialize world link */
         /* For now, don't read world stuff from the config file */
         link->alpha = 0.0;
         link->theta = 0.0;
         link->a = 0.0;
         link->d = 0.0;
         
         /* Evaluate a few cache things */
         link->cos_alpha = cos(link->alpha);
         link->sin_alpha = sin(link->alpha);
         
         link->trans_to_prev = gsl_matrix_calloc(4,4);
         gsl_matrix_set_identity( link->trans_to_prev );
         
         link->trans_to_inertial = gsl_matrix_calloc(4,4);
         gsl_matrix_set_identity( link->trans_to_inertial );
      }
      else if (link == kin->tool)
      {
         /* Who knows? */
         link->alpha = 0.0;
         link->theta = 0.0;
         link->a = 0.0;
         link->d = 0.35; /* 4-dof */
         
         /* Evaluate a few cache things */
         link->cos_alpha = cos(link->alpha);
         link->sin_alpha = sin(link->alpha);
         
         link->trans_to_prev = gsl_matrix_calloc(4,4);
         gsl_matrix_set_identity( link->trans_to_prev );
         gsl_matrix_set( link->trans_to_prev, 2,3, link->d );
         
         link->trans_to_inertial = gsl_matrix_calloc(4,4);
         gsl_matrix_set_identity( link->trans_to_inertial );
      }
      else
      {
         /* Set up from config file */
         int j;
         config_setting_t * moving_grp;
         config_setting_t * setting;
         
         j = i-1; /* (There's one world frame) */
         
         moving_grp = config_setting_get_elem( moving, j );
         if (config_setting_type(moving_grp) != CONFIG_TYPE_GROUP)
         { printf("kin:moving:#%d is bad format\n",j); return 0; }
         
         /* Grab each of the dh parameters */
         setting = config_setting_get_member( moving_grp, "alpha_pi" );
         if (setting == NULL) { printf("No alpha_pi in link %d\n",j); return 0; }
         switch (config_setting_type(setting))
         {
            case CONFIG_TYPE_INT:
               link->alpha = M_PI * config_setting_get_int(setting);
               break;
            case CONFIG_TYPE_FLOAT:
               link->alpha = M_PI * config_setting_get_float(setting);
               break;
            default:
               printf("kin:that's not a numer!\n");
               return 0;
         }
         setting = config_setting_get_member( moving_grp, "a" );
         if (setting == NULL) { printf("No a in link %d\n",j); return 0; }
         switch (config_setting_type(setting))
         {
            case CONFIG_TYPE_INT:
               link->a = config_setting_get_int(setting);
               break;
            case CONFIG_TYPE_FLOAT:
               link->a = config_setting_get_float(setting);
               break;
            default:
               printf("kin:that's not a numer!\n");
               return 0;
         }
         setting = config_setting_get_member( moving_grp, "d" );
         if (setting == NULL) { printf("No d in link %d\n",j); return 0; }
         switch (config_setting_type(setting))
         {
            case CONFIG_TYPE_INT:
               link->d = config_setting_get_int(setting);
               break;
            case CONFIG_TYPE_FLOAT:
               link->d = config_setting_get_float(setting);
               break;
            default:
               printf("kin:that's not a numer!\n");
               return 0;
         }
         
         /* Evaluate a few cache things */
         link->cos_alpha = cos(link->alpha);
         link->sin_alpha = sin(link->alpha);
         
         /* Initialize the matrices (the last two rows are const for revolute) */
         link->trans_to_prev = gsl_matrix_calloc(4,4);
         gsl_matrix_set( link->trans_to_prev, 2,1, link->sin_alpha );
         gsl_matrix_set( link->trans_to_prev, 2,2, link->cos_alpha );
         gsl_matrix_set( link->trans_to_prev, 2,3, link->d );
         gsl_matrix_set( link->trans_to_prev, 3,3, 1.0 );
         
         link->trans_to_inertial = gsl_matrix_calloc(4,4);
      }
      
      /* Set up the vector views */
      {
         gsl_vector_view view;
         link->origin_pos = (gsl_vector *) malloc(sizeof(gsl_vector));
         view = gsl_matrix_subcolumn( link->trans_to_inertial, 3, 0, 3);
         *(link->origin_pos) = view.vector;
      }
      /* Set up the matrix views */
      {
         gsl_matrix_view view;
         
         link->rot_to_prev = (gsl_matrix *) malloc(sizeof(gsl_matrix));
         view = gsl_matrix_submatrix( link->trans_to_prev, 0,0, 3,3 );
         *(link->rot_to_prev) = view.matrix;
         
         link->rot_to_inertial = (gsl_matrix *) malloc(sizeof(gsl_matrix));
         view = gsl_matrix_submatrix( link->trans_to_inertial, 0,0, 3,3 );
         *(link->rot_to_inertial) = view.matrix;
      }
   }

   return kin;
}

int bt_kinematics_destroy( struct bt_kinematics * kin )
{
   int i;
   for (i=0; i<kin->nlinks; i++)
   {
      struct bt_kinematics_link * link;
      link = kin->link_array[i];
      gsl_matrix_free(link->trans_to_prev);
      gsl_matrix_free(link->trans_to_inertial);
      free(link->origin_pos);
      free(link->rot_to_prev);
      free(link->rot_to_inertial);
      free(link);
   }
   free(kin->link_array);
   return 0;
}

int bt_kinematics_eval_forward( struct bt_kinematics * kin, gsl_vector * jposition )
{
   int j;
   
   for (j=0; j<kin->dof; j++)
   {
      struct bt_kinematics_link * link;
      link = kin->link[j];
     
      /* Step 1: Copy current joint positions into kinematics struct
       *         (revolute only) */
      link->theta = gsl_vector_get( jposition, j );
      
      /* Step 2: Evaluate the to_prev transform */
      eval_trans_to_prev( link );
      
      /* Step 3: Evaluate the to_inertial transform */
      eval_trans_to_inertial( link );
   }
   
   /* Also evaluate to_inertial for the tool (no theta) */
   eval_trans_to_inertial( kin->tool );
   
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
   gsl_matrix_set( link->trans_to_prev, 0,3,  sin_theta * link->a );
   
   return 0;
}

/* Note - assume this isn't the world link */
static int eval_trans_to_inertial( struct bt_kinematics_link * link )
{   
   /* Matrix Multiply */
   gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0,
                  link->prev->trans_to_inertial,
                  link->trans_to_prev,
                  0.0, link->trans_to_inertial);
   return 0;
}




