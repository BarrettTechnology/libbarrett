/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... gravity.c
 *  Author ............. Christopher Dellin
 *  Creation Date ...... Sept 15, 2008
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *
 * ======================================================================== */

/* Read config files for the wam stuff */
#include <libconfig.h>
#include <syslog.h>

/* For fast matrix multiplication */
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>

#include "kinematics.h"
#include "gravity.h"

#define PI (3.141592653589793238462643383)

/* Vector cross product.
 * Cannot be performed in-place! */
int cross( gsl_vector * a, gsl_vector * b, gsl_vector * res )
{
   gsl_vector_set( res, 0, gsl_vector_get(a,1)*gsl_vector_get(b,2)
                         - gsl_vector_get(a,2)*gsl_vector_get(b,1) );
   gsl_vector_set( res, 1, gsl_vector_get(a,2)*gsl_vector_get(b,0)
                         - gsl_vector_get(a,0)*gsl_vector_get(b,2) );
   gsl_vector_set( res, 2, gsl_vector_get(a,0)*gsl_vector_get(b,1)
                         - gsl_vector_get(a,1)*gsl_vector_get(b,0) );
   return 0;
}

struct bt_gravity * bt_gravity_create( config_setting_t * gravconfig, struct bt_kinematics * kin )
{
   int j;
   config_setting_t * mus;
   struct bt_gravity * grav;
   grav = (struct bt_gravity *) malloc(sizeof(struct bt_gravity));
   
   /* Save deps */
   grav->kin = kin;
   
   /* Initialize the world_g link */
   grav->world_g = gsl_vector_calloc(3);
   gsl_vector_set( grav->world_g, 2, -9.805 );
   
   /* Initialize vectors for each moving link (dof) */
   grav->g  = (gsl_vector **) malloc((kin->dof)*sizeof(gsl_vector *));
   grav->mu = (gsl_vector **) malloc((kin->dof)*sizeof(gsl_vector *));
   grav->t  = (gsl_vector **) malloc((kin->dof)*sizeof(gsl_vector *));
   grav->pt = (gsl_vector **) malloc((kin->dof)*sizeof(gsl_vector *));
   
   mus = config_setting_get_member( gravconfig, "mus" );
   if (mus == NULL) { syslog(LOG_ERR,"No gravity:mus in wam.config\n"); return 0; }
   if (config_setting_type(mus) != CONFIG_TYPE_LIST)
      { syslog(LOG_ERR,"gravity:mus must be list in wam.config\n"); return 0; }
   if (config_setting_length(mus) != kin->dof)
      { syslog(LOG_ERR,"Expected %d links, but found %d.\n",
               kin->dof, config_setting_length(mus)); return 0; }
   
   for (j=0; j<kin->dof; j++)
   {
      int i;
      config_setting_t * mu;
      
      /* Initialize this link's vectors */
      grav->g[j]  = gsl_vector_calloc(3);
      grav->mu[j] = gsl_vector_calloc(3);
      grav->t[j]  = gsl_vector_calloc(3);
      grav->pt[j] = gsl_vector_calloc(3);
      
      /* Grab the mu value from the configuration */
      mu = config_setting_get_elem( mus, j );
      if (config_setting_type(mu) != CONFIG_TYPE_LIST)
         { syslog(LOG_ERR,"gravity:mus is bad format\n"); return 0; }
      if (config_setting_length(mu) != 3)
         { syslog(LOG_ERR,"mu must be a 3-vector!\n"); return 0; }
      for (i=0; i<3; i++)
      {
         config_setting_t * val;
         val = config_setting_get_elem( mu, i );
         switch (config_setting_type(val))
         {
            case CONFIG_TYPE_INT:
               gsl_vector_set( grav->mu[j], i, config_setting_get_int(val) );
               break;
            case CONFIG_TYPE_FLOAT:
               gsl_vector_set( grav->mu[j], i, config_setting_get_float(val) );
               break;
            default:
               printf("mus: that's not a numer!\n");
               return 0;
         }
      }
      
      syslog(LOG_ERR,"mu%d - < %f, %f, %f >", j,
             gsl_vector_get( grav->mu[j], 0 ),
             gsl_vector_get( grav->mu[j], 1 ),
             gsl_vector_get( grav->mu[j], 2 ));
   }
   
   return grav;
}

int bt_gravity_destroy( struct bt_gravity * grav )
{
   int j;
   for (j=0; j<grav->kin->dof; j++)
   {
      gsl_vector_free(grav->g[j] );
      gsl_vector_free(grav->mu[j]);
      gsl_vector_free(grav->t[j] );
      gsl_vector_free(grav->pt[j]);
   }
   free(grav->g );
   free(grav->mu);
   free(grav->t );
   free(grav->pt);
   gsl_vector_free(grav->world_g);
   free(grav);
   return 0;
}

/* Evaluate and add into jtorque */
int bt_gravity_eval( struct bt_gravity * grav, gsl_vector * jtorque )
{
   int j;
   /* For each moving link, backwards ... */
   for (j=grav->kin->dof-1; j>=0; j--)
   {
      struct bt_kinematics_link * link;
      link = grav->kin->link[j];
      /* Fill each link's gravity vector */
      gsl_blas_dgemv(CblasTrans, 1.0, link->rot_to_inertial,
                     grav->world_g, 0.0, grav->g[j]);
      /* Compute each link's torque (T = g x mu) */
      cross( grav->g[j], grav->mu[j], grav->t[j] );
      /* If it's not the last moving link, add in the next link's t */
      if (j < grav->kin->dof-1)
         gsl_blas_dgemv(CblasNoTrans, 1.0, link->next->rot_to_prev,
                        grav->t[j+1], 1.0, grav->t[j]);
      /* Move torque vector to the previous frame */
      gsl_blas_dgemv(CblasNoTrans, 1.0, link->rot_to_prev,
                     grav->t[j], 0.0, grav->pt[j]);
      /* Add into the torque vector the z component */
      *gsl_vector_ptr(jtorque,j) += gsl_vector_get(grav->pt[j],2);
   }
   return 0;
}


