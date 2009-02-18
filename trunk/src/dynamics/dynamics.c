/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... dynamics.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... Feb 3, 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *
 * ======================================================================== */

#include <libconfig.h>
#include <syslog.h>

/* For fast matrix multiplication */
#include <gsl/gsl_blas.h>

#include "dynamics.h"

#include "gsl.h"

/* Private functions */
static int eval_inverse_forward( struct bt_dynamics * dyn,
                                 struct bt_dynamics_link * link,
                                 struct bt_kinematics_link * kin_link,
                                 double vel, double acc );

static int eval_inverse_backward( struct bt_dynamics * dyn,
                                  struct bt_dynamics_link * link,
                                  struct bt_kinematics_link * kin_link,
                                  double * torque );

/* These fixed versions assume vel and acc are zero,
 * and just translate the forces/torques back to the previous joint */
static int eval_inverse_forward_fixed( struct bt_dynamics * dyn,
                                       struct bt_dynamics_link * link,
                                       struct bt_kinematics_link * kin_link );
static int eval_inverse_backward_fixed( struct bt_dynamics * dyn,
                                        struct bt_dynamics_link * link,
                                        struct bt_kinematics_link * kin_link );

struct bt_dynamics * bt_dynamics_create( config_setting_t * dynconfig, int ndofs, struct bt_kinematics * kin )
{
   int err;
   int i;
   config_setting_t * moving;
   struct bt_dynamics * dyn;
   
   dyn = (struct bt_dynamics *) malloc( sizeof(struct bt_dynamics) );
   
   /* Save the kinematics link */
   dyn->kin = kin;
   
   /* Create the links array */
   dyn->dof = ndofs;
   dyn->nlinks = 1 + ndofs + 1; /* base, toolplate */
   dyn->link_array = (struct bt_dynamics_link **)
      malloc(dyn->nlinks*sizeof(struct bt_dynamics_link *));
   for (i=0; i<dyn->nlinks; i++)
      dyn->link_array[i] = (struct bt_dynamics_link *)
         malloc(sizeof(struct bt_dynamics_link));
   dyn->base = dyn->link_array[0];
   dyn->link = dyn->link_array + 1;
   dyn->toolplate = dyn->link_array[dyn->nlinks - 1];
   
   /* Make sure we have an appropriate configuration for moving links */
   moving = config_setting_get_member( dynconfig, "moving" );
   if (moving == NULL) { printf("No dyn:moving in wam.config\n"); return 0; }
   if (config_setting_type(moving) != CONFIG_TYPE_LIST)
      { printf("dyn:moving not a list\n"); return 0; }
   if (config_setting_length(moving) != ndofs)
      { printf("Expected %d links, but found %d.\n",
               ndofs, config_setting_length(moving)); return 0; }
   
   /* Initialize each link */
   for (i=0; i<dyn->nlinks; i++)
   {
      struct bt_dynamics_link * link;
      link = dyn->link_array[i];
      
      /* Set up next, prev ptrs */
      link->prev = (link == dyn->base)     ? 0 : dyn->link_array[i-1];
      link->next = (i == dyn->nlinks - 1 ) ? 0 : dyn->link_array[i+1];
      
      /* Read inertial parameters from config file */
      if (link == dyn->base)
      {
         /* For now, assume the base is inertial. */
         link->mass = 0.0;
         link->com = gsl_vector_calloc(3);
         link->I = gsl_matrix_calloc(3,3);
         
         /* Initialize the base accelerations, etc to 0
          * for starting-point of RNEA */
         link->omega = gsl_vector_calloc(3);
         link->alpha = gsl_vector_calloc(3);
         link->a = gsl_vector_calloc(3);
         /*gsl_vector_set( link->a, 2, 9.81 );*/
         
         link->omega_prev = gsl_vector_calloc(3);
         link->f_next = gsl_vector_calloc(3);
         
         /* These don't matter */
         link->fnet = gsl_vector_calloc(3);
         link->tnet = gsl_vector_calloc(3);
         link->f = gsl_vector_calloc(3);
         link->t = gsl_vector_calloc(3);
         
         /* This definitely doesn't matter */
         link->com_jacobian = 0;
         link->com_jacobian_linear = 0;
         link->com_jacobian_angular = 0;
      }
      else if (link == dyn->toolplate)
      {
         /* None of these are ever used ... */
         link->mass = 0.0;
         link->com = gsl_vector_calloc(3);
         link->I = gsl_matrix_calloc(3,3);
         
         /* Initialize the base accelerations, etc to 0
          * for starting-point of RNEA */
         link->omega = gsl_vector_calloc(3);
         link->alpha = gsl_vector_calloc(3);
         link->a = gsl_vector_calloc(3);
         /*gsl_vector_set( link->a, 2, 9.81 );*/
         
         link->omega_prev = gsl_vector_calloc(3);
         link->f_next = gsl_vector_calloc(3);
         
         /* These don't matter */
         link->fnet = gsl_vector_calloc(3);
         link->tnet = gsl_vector_calloc(3);
         link->f = gsl_vector_calloc(3);
         link->t = gsl_vector_calloc(3);
         
         /* This definitely doesn't matter */
         link->com_jacobian = 0;
         link->com_jacobian_linear = 0;
         link->com_jacobian_angular = 0;
      }
      else
      {
         int j;
         config_setting_t * link_grp;
         
         j = i-1; /* (There's one base frame) */
         
         /* Get the link #j group */
         link_grp = config_setting_get_elem( moving, j );
         if (config_setting_type(link_grp) != CONFIG_TYPE_GROUP)
         { printf("kin:moving:#%d is bad format\n",j); return 0; }
         
         /* Read the mass */
         err = bt_gsl_config_double_from_group( link_grp, "mass", &(link->mass) );
         if (err)
         {
            printf("dyn: No mass in link, or not a number.\n");
            return 0;
         }
         
         /* Read the center of mass vector */
         link->com = gsl_vector_alloc(3);
         err = bt_gsl_fill_vector(link->com, link_grp, "com");
         if (err)
         {
            printf("dyn: No com in link, or not a 3-element vector.\n");
            return 0;
         }
         
         /* Read the inertia matrix */
         link->I = gsl_matrix_alloc(3,3);
         err = bt_gsl_fill_matrix(link->I, link_grp, "I");
         if (err)
         {
            printf("dyn: No I in link, or not a 3x3-element matrix.\n");
            return 0;
         }
         
         /* Allocate space for ac, omega, alpha */
         link->omega = gsl_vector_calloc(3);
         link->alpha = gsl_vector_calloc(3);
         link->a = gsl_vector_calloc(3);
         
         link->omega_prev = gsl_vector_calloc(3);
         link->f_next = gsl_vector_calloc(3);
         
         /* Allocate space for b, g, f, t */
         link->fnet = gsl_vector_calloc(3);
         link->tnet = gsl_vector_calloc(3);
         link->f = gsl_vector_calloc(3);
         link->t = gsl_vector_calloc(3);
         
         /* Allocate space for the COM-jacobian */
         link->com_jacobian = gsl_matrix_alloc(6,dyn->dof);
         
         /* Allocate matrix views into COM jacobian */
         {
            gsl_matrix_view view;
            
            link->com_jacobian_linear = (gsl_matrix *) malloc(sizeof(gsl_matrix));
            view = gsl_matrix_submatrix( link->com_jacobian, 0,0, 3,dyn->dof );
            *(link->com_jacobian_linear) = view.matrix;
            
            link->com_jacobian_angular = (gsl_matrix *) malloc(sizeof(gsl_matrix));
            view = gsl_matrix_submatrix( link->com_jacobian, 3,0, 3,dyn->dof );
            *(link->com_jacobian_angular) = view.matrix;
         }
      }   
   }
   
   /* Make temporary vectors */
   dyn->temp1_v3 = gsl_vector_alloc(3);
   dyn->temp2_v3 = gsl_vector_alloc(3);
   
   /* Make temporary matrices */
   dyn->temp3x3_1 = gsl_matrix_alloc(3,3);
   dyn->temp3x3_2 = gsl_matrix_alloc(3,3);
   dyn->temp3xn_1 = gsl_matrix_alloc(3,dyn->dof);
   
   return dyn;
}

int bt_dynamics_destroy( struct bt_dynamics * dyn )
{
   int i;
   for (i=0; i<dyn->nlinks; i++)
   {
      struct bt_dynamics_link * link;
      link = dyn->link_array[i];
      
      gsl_vector_free(link->com);
      gsl_matrix_free(link->I);
      
      gsl_vector_free(link->omega);
      gsl_vector_free(link->alpha);
      gsl_vector_free(link->a);
      
      gsl_vector_free(link->omega_prev);
      gsl_vector_free(link->f_next);
      
      gsl_vector_free(link->fnet);
      gsl_vector_free(link->tnet);
      gsl_vector_free(link->f);
      gsl_vector_free(link->t);
      
      if (link->com_jacobian)
      {
         gsl_matrix_free(link->com_jacobian);
         free(link->com_jacobian_linear);
         free(link->com_jacobian_angular);
      }
      
      free(link);
   }
   free(dyn->link_array);
   
   free(dyn->temp1_v3);
   free(dyn->temp2_v3);
   free(dyn->temp3x3_1);
   free(dyn->temp3x3_2);
   free(dyn->temp3xn_1);
   
   free(dyn);
   return 0;
}

/* NOTE: This takes ~ 152us on PC104 right now. */
int bt_dynamics_eval_inverse( struct bt_dynamics * dyn,
   gsl_vector * jvel, gsl_vector * jacc, gsl_vector * jtor )
{
   int j;
   
   /* Assume the base velocities, accelerations are already set
    * in dyn->base (set to zero on startup, i.e. assumed inertial) */
   
   /* Iterate from the base to the toolplate */
   for (j=0; j<dyn->dof; j++)
   {
      eval_inverse_forward( dyn, dyn->link[j], dyn->kin->link[j],
                            gsl_vector_get(jvel,j),
                            gsl_vector_get(jacc,j) );      
   }
   
   /* Evaluate the vels/accs of the fixed toolplate frame */
   eval_inverse_forward_fixed( dyn, dyn->toolplate, dyn->kin->toolplate );
   
   /* OK, now we have omega, alpha, and a calculated for all moving links,
    * and the toolplate frame, but not for the tool frame. */
   /* Potentially call some callback to calculate the tool inverse dynamics */
   
   /* Evaluate the forces of the massless fixed toolplate frame
    * (this produces 0 since the toolplate is the last frame ATM) */
   eval_inverse_backward_fixed( dyn, dyn->toolplate, dyn->kin->toolplate );
   
   for (j=dyn->dof-1; j>=0; j--)
   {
      eval_inverse_backward( dyn, dyn->link[j], dyn->kin->link[j],
                             gsl_vector_ptr(jtor,j) );
   }
   
   return 0;
}

static int eval_inverse_forward( struct bt_dynamics * dyn,
                                 struct bt_dynamics_link * link,
                                 struct bt_kinematics_link * kin_link,
                                 double vel, double acc )
{
   /* STEP 1: Calculate omega (this link's angular velocity) */
   
   /* First, bring the previous link's omega into our frame
    * omega = (R^(j-1)_j)^T omega_(j-1) */
   gsl_blas_dgemv( CblasTrans, 1.0, kin_link->rot_to_prev,
                   link->prev->omega,
                   0.0, link->omega );
   /* Make a copy for the cache */
   gsl_vector_memcpy( link->omega_prev, link->omega );
   
   /* Last, add in my joint's velocity
    * omega += qdot_j axis */
   gsl_blas_daxpy( vel, kin_link->prev_axis_z,
                   link->omega );
   
   /* STEP 2: Calculate alpha (this link's angular acceleration) */
   
   /* First, bring the previous link's alpha into our frame
    * alpha = (R^(j-1)_j)^T alpha_(j-1) */
   gsl_blas_dgemv( CblasTrans, 1.0, kin_link->rot_to_prev,
                   link->prev->alpha,
                   0.0, link->alpha );
   
   /* Next, add in my joint's acceleration
    * alpha += qdotdot_j axis */
   gsl_blas_daxpy( acc, kin_link->prev_axis_z,
                   link->alpha );
   
   /* Last, add in the weird cross-product
    * between the previous link's angular velocity
    * and my net angular velocity
    * alpha += (omega_prev) x (qdot_j axis) */
   gsl_vector_set_zero( dyn->temp2_v3 );
   gsl_blas_daxpy( vel, kin_link->prev_axis_z,
                   dyn->temp2_v3 );
   bt_gsl_cross( link->omega_prev, dyn->temp2_v3,
                 link->alpha );
   
   /* STEP 3: Calculate a (linear acceleration of origin of frame) */
   
   /* First, copy the previous link's acceleration
    * a^(j-1) = a_(j-1) */
   gsl_vector_memcpy( dyn->temp1_v3, link->prev->a );
   
   /* Next, copy in the acc due to the previous angular acceleration
    * a^(j-1) += alpha_(j-1) x r^(j-1)_j */
   bt_gsl_cross( link->prev->alpha, kin_link->prev_origin_pos,
                 dyn->temp1_v3 );
   
   /* Next, copy in the weird velocity components
    * t2 = omega_(j-1) x r^(j-1)_j */
   gsl_vector_set_zero( dyn->temp2_v3 );
   bt_gsl_cross( link->prev->omega, kin_link->prev_origin_pos,
                 dyn->temp2_v3 ); 
   /* a^(j-1) += omega_(j-1) x t2 */
   bt_gsl_cross( link->prev->omega, dyn->temp2_v3,
                 dyn->temp1_v3 );
   
   /* Last, bring it into my coordimate frame
    * a^j = (R^(j-1)_j)^T a^(j-1) */
   gsl_blas_dgemv( CblasTrans, 1.0, kin_link->rot_to_prev,
                   dyn->temp1_v3,
                   0.0, link->a );
   
   return 0;
}


static int eval_inverse_backward( struct bt_dynamics * dyn,
                                  struct bt_dynamics_link * link,
                                  struct bt_kinematics_link * kin_link,
                                  double * torque )
{
   /* STEP 1: Calculate the net force on the link
    * (note, we sum all the accelerations first) */
   
   /* First, copy in the acceleration
    * fnet/m = a */
   gsl_vector_memcpy( link->fnet, link->a );
   
   /* Next, add in the angular acceleration at the com
    * fnet/m += alpha x com */
   bt_gsl_cross( link->alpha, link->com,
                 link->fnet );
   
   /* Next, add in the weird velocity terms
    * fnet/m += omega x (omega x com) */
   gsl_vector_set_zero( dyn->temp2_v3 );
   bt_gsl_cross( link->omega, link->com,
                 dyn->temp2_v3 );
   bt_gsl_cross( link->omega, dyn->temp2_v3,
                 link->fnet );
   
   /* Last, scale by the mass
    * fnet = fnet/m * m */
   gsl_blas_dscal( link->mass, link->fnet );
   
   /* STEP 2: Calculate the net torque (moment) on the link */
   
   /* First, copy in the moment of inertia * angular acceleration
    * tnet = I alpha */
   gsl_blas_dgemv( CblasNoTrans, 1.0, link->I,
                   link->alpha,
                   0.0, link->tnet );
   
   /* Last, copy in the weird velocity terms
    * tnet += omega x (I omega) */
   gsl_blas_dgemv( CblasNoTrans, 1.0, link->I,
                   link->omega,
                   0.0, dyn->temp2_v3 );
   bt_gsl_cross( link->omega, dyn->temp2_v3,
                 link->tnet );
   
   /* STEP 3: Calculate the force exerted on link j through joint j */
   gsl_vector_memcpy( link->f, link->fnet );
   if (link->next)
   {
      /* Add in the next link's force into our frame
       * (put in cache first) */
      gsl_blas_dgemv( CblasNoTrans, 1.0, kin_link->next->rot_to_prev,
                      link->next->f,
                      0.0, link->f_next );
      gsl_blas_daxpy( 1.0, link->f_next,
                      link->f );
   }
      
   /* STEP 4: Calculate the moment exerted in link j through joint j */
   gsl_vector_memcpy( link->t, link->tnet );
   /* Add in the torque from the force at the com */
   bt_gsl_cross( link->com, link->fnet,
                 link->t );
   if (link->next)
   {
      /* Add in the next link's moment into our frame  */
      gsl_blas_dgemv( CblasNoTrans, 1.0, kin_link->next->rot_to_prev,
                      link->next->t,
                      1.0, link->t );
      /* Add in the next link's force at the endpoint */
      bt_gsl_cross( kin_link->next->prev_origin_pos, link->f_next,
                    link->t );
   }
   
   /* Get the component of the torque in the axis direction */
   gsl_blas_ddot( kin_link->prev_axis_z, link->t,
                  torque );
   
   return 0;
}



static int eval_inverse_forward_fixed( struct bt_dynamics * dyn,
                                       struct bt_dynamics_link * link,
                                       struct bt_kinematics_link * kin_link )
{
   /* STEP 1: Calculate omega (this link's angular velocity) */
   
   /* First, bring the previous link's omega into our frame
    * omega = (R^(j-1)_j)^T omega_(j-1) */
   gsl_blas_dgemv( CblasTrans, 1.0, kin_link->rot_to_prev,
                   link->prev->omega,
                   0.0, link->omega );
   /* Make a copy for the cache */
   gsl_vector_memcpy( link->omega_prev, link->omega );
   
   /* Last, add in my joint's velocity
    * omega += qdot_j axis = 0 (fixed) */
   
   /* STEP 2: Calculate alpha (this link's angular acceleration) */
   
   /* First, bring the previous link's alpha into our frame
    * alpha = (R^(j-1)_j)^T alpha_(j-1) */
   gsl_blas_dgemv( CblasTrans, 1.0, kin_link->rot_to_prev,
                   link->prev->alpha,
                   0.0, link->alpha );
   
   /* Next, add in my joint's acceleration
    * alpha += qdotdot_j axis = 0 (fixed) */
   
   /* Last, add in the weird cross-product
    * between the previous link's angular velocity
    * and my net angular velocity
    * alpha += (omega_prev) x (qdot_j axis) = 0 (fixed) */
   
   /* STEP 3: Calculate a (linear acceleration of origin of frame) */
   
   /* First, copy the previous link's acceleration
    * a^(j-1) = a_(j-1) */
   gsl_vector_memcpy( dyn->temp1_v3, link->prev->a );
   
   /* Next, copy in the acc due to the previous angular acceleration
    * a^(j-1) += alpha_(j-1) x r^(j-1)_j */
   bt_gsl_cross( link->prev->alpha, kin_link->prev_origin_pos,
                 dyn->temp1_v3 );
   
   /* Next, copy in the weird velocity components
    * t2 = omega_(j-1) x r^(j-1)_j */
   gsl_vector_set_zero( dyn->temp2_v3 );
   bt_gsl_cross( link->prev->omega, kin_link->prev_origin_pos,
                 dyn->temp2_v3 );
   /* a^(j-1) += omega_(j-1) x t2 */
   bt_gsl_cross( link->prev->omega, dyn->temp2_v3,
                 dyn->temp1_v3 );
   
   /* Last, bring it into my coordimate frame
    * a^j = (R^(j-1)_j)^T a^(j-1) */
   gsl_blas_dgemv( CblasTrans, 1.0, kin_link->rot_to_prev,
                   dyn->temp1_v3,
                   0.0, link->a );
   
   return 0;
}

static int eval_inverse_backward_fixed( struct bt_dynamics * dyn,
                                        struct bt_dynamics_link * link,
                                        struct bt_kinematics_link * kin_link )
{
   /* STEP 1: Calculate the net force on the link
    * (note, we sum all the accelerations first)
    * fnet = 0 (fixed, 0 mass) */
   
   /* STEP 2: Calculate the net torque (moment) on the link
    * tnet = 0 (fixed, 0 mass) */
   
   /* STEP 3: Calculate the force exerted on link j through joint j */
   gsl_vector_set_zero( link->f );
   if (link->next)
   {
      /* Add in the next link's force into our frame
       * (put in cache first) */
      gsl_blas_dgemv( CblasNoTrans, 1.0, kin_link->next->rot_to_prev,
                      link->next->f,
                      0.0, link->f_next );
      gsl_blas_daxpy( 1.0, link->f_next,
                      link->f );
   }
      
   /* STEP 4: Calculate the moment exerted in link j through joint j */
   gsl_vector_set_zero( link->t );
   /* Add in the torque from the force at the com
    * = 0 (fixed, no mass )*/
   if (link->next)
   {
      /* Add in the next link's moment into our frame  */
      gsl_blas_dgemv( CblasNoTrans, 1.0, kin_link->next->rot_to_prev,
                      link->next->t,
                      1.0, link->t );
      /* Add in the next link's force at the endpoint */
      bt_gsl_cross( kin_link->next->prev_origin_pos, link->f_next,
                    link->t );
   }
   
   return 0;
}


int bt_dynamics_eval_jsim( struct bt_dynamics * dyn )
{
   int j;
   
   /* Zero the JSIM */
   gsl_matrix_set_zero( dyn->jsim );
   
   for (j=0; j<dyn->dof; j++)
   {
      struct bt_dynamics_link * link;
      struct bt_kinematics_link * kin_link;
      
      link = dyn->link[j];
      kin_link = dyn->kin->link[j];
      
      /* First, calculate each moving link's Jacobian at the COM */
      
      /* Get the COM in world coords */
      gsl_blas_dgemv( CblasNoTrans, 1.0, kin_link->rot_to_world,
                      link->com,
                      0.0, dyn->temp1_v3 );
      
      /* Evaluate the jacobian at the link's COM point */
      bt_kinematics_eval_jacobian( dyn->kin, j+1, dyn->temp1_v3,
                                   link->com_jacobian );
      
      /* Add in the linear velocity component */
      gsl_blas_dgemm( CblasTrans, CblasNoTrans,
                      link->mass,
                      link->com_jacobian_linear, link->com_jacobian_linear,
                      1.0, dyn->jsim );
      
      /* The angular velocity component needs some work ... */
      
      /* Do I x R^T */
      gsl_blas_dgemm( CblasNoTrans, CblasTrans,
                      1.0,
                      link->I, kin_link->rot_to_world,
                      0.0, dyn->temp3x3_1 );
      /* Do R x (IxR^T) (AAH OVERLAP!) */
      gsl_blas_dgemm( CblasNoTrans, CblasNoTrans,
                      1.0,
                      kin_link->rot_to_world, dyn->temp3x3_1,
                      0.0, dyn->temp3x3_2 );
      /* Do (RxIxR^T) x Jw */
      gsl_blas_dgemm( CblasNoTrans, CblasNoTrans,
                      1.0,
                      dyn->temp3x3_2, link->com_jacobian_angular,
                      0.0, dyn->temp3xn_1 );
      /* Do Jw^T x (RxIxR^TxJw) */
      gsl_blas_dgemm( CblasTrans, CblasNoTrans,
                      1.0,
                      link->com_jacobian_angular, dyn->temp3xn_1,
                      1.0, dyn->jsim );
   }
   
   return 0;
}
