/** Implementation of bt_calgrav, a calibrated gravity compensation module.
 *
 * \file calgrav.c
 * \author Christopher Dellin
 * \date 2008-2009
 */

/* Copyright 2008, 2009
 *           Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */

#include <syslog.h>

#include <libconfig.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>

#include <barrett/cdlbt/kinematics.h>
#include <barrett/cdlbt/calgrav.h>
#include <barrett/cdlbt/gsl.h>


int bt_calgrav_create(struct bt_calgrav ** gravptr,
                  config_setting_t * gravconfig, size_t dof)
{
   int j;
   config_setting_t * mus;
   struct bt_calgrav * grav;
   
   (*gravptr) = 0;

   /* Check arguments */
   if (!gravconfig) return -1;

   grav = (struct bt_calgrav *) malloc(sizeof(struct bt_calgrav));
   if (!grav)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return -1;
   }
   
   /* Initialize */
   /*grav->kin = kin;*/
   grav->dof = dof;
   grav->world_g = 0;
   grav->g = 0;
   grav->mu = 0;
   grav->t = 0;
   grav->pt = 0;
   
   /* Initialize the world_g link */
   grav->world_g = gsl_vector_calloc(3);
   if (!grav->world_g)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_calgrav_destroy(grav);
      return -1;
   }
   gsl_vector_set( grav->world_g, 2, -9.805 );
   
   /* Initialize vectors for each moving link (dof) */
   grav->g  = (gsl_vector **) malloc(dof*sizeof(gsl_vector *));
   if (!grav->g)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_calgrav_destroy(grav);
      return -1;
   }
   for (j=0; j<dof; j++) grav->g[j] = 0;
   
   grav->mu = (gsl_vector **) malloc((dof)*sizeof(gsl_vector *));
   if (!grav->mu)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_calgrav_destroy(grav);
      return -1;
   }
   for (j=0; j<dof; j++) grav->mu[j] = 0;
   
   grav->t  = (gsl_vector **) malloc((dof)*sizeof(gsl_vector *));
   if (!grav->t)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_calgrav_destroy(grav);
      return -1;
   }
   for (j=0; j<dof; j++) grav->t[j] = 0;
   
   grav->pt = (gsl_vector **) malloc((dof)*sizeof(gsl_vector *));
   if (!grav->pt)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_calgrav_destroy(grav);
      return -1;
   }
   for (j=0; j<dof; j++) grav->pt[j] = 0;
   
   /* Make sure we have an appropriate configuration for moving links */
   if (   !(mus = config_setting_get_member(gravconfig,"mus"))
       ||  (config_setting_type(mus) != CONFIG_TYPE_LIST)
       ||  (config_setting_length(mus) != dof)
   ) {
      syslog(LOG_ERR,"%s: grav:mus not a list with %u elements.",__func__, (unsigned int)dof);
      bt_calgrav_destroy(grav);
      return -1;
   }
   
   /* Create each vector */
   for (j=0; j<dof; j++)
   {
      int i;
      config_setting_t * mu;
      
      /* Initialize this link's vectors */
      grav->g[j]  = gsl_vector_calloc(3);
      grav->mu[j] = gsl_vector_calloc(3);
      grav->t[j]  = gsl_vector_calloc(3);
      grav->pt[j] = gsl_vector_calloc(3);
      /* Do memory checking */
      if (   !grav->g[j]
          || !grav->mu[j]
          || !grav->t[j]
          || !grav->pt[j]
      ) {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         bt_calgrav_destroy(grav);
         return -1;
      }
      
      /* Grab the mu value from the configuration */
      mu = config_setting_get_elem( mus, j );
      if (   (config_setting_type(mu) != CONFIG_TYPE_LIST)
          || (config_setting_length(mu) != 3)
      ) {
         syslog(LOG_ERR,"%s: grav:mu #%d not a 3-element list.",__func__,j);
         bt_calgrav_destroy(grav);
         return -1;
      }
      
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
               syslog(LOG_ERR,"%s: that's not a number!",__func__);
               bt_calgrav_destroy(grav);
               return -1;
         }
      }
   }

   (*gravptr) = grav;
   return 0;
}


int bt_calgrav_destroy( struct bt_calgrav * grav )
{
   int j;
   for (j=0; j<grav->dof; j++)
   {
      if (grav->g && grav->g[j])
         gsl_vector_free(grav->g[j] );
      if (grav->mu && grav->mu[j])
         gsl_vector_free(grav->mu[j]);
      if (grav->t && grav->t[j])
         gsl_vector_free(grav->t[j] );
      if (grav->pt && grav->pt[j])
         gsl_vector_free(grav->pt[j]);
   }
   if (grav->g)  free(grav->g );
   if (grav->mu) free(grav->mu);
   if (grav->t)  free(grav->t );
   if (grav->pt) free(grav->pt);
   
   if (grav->world_g)
      gsl_vector_free(grav->world_g);
   
   free(grav);
   return 0;
}


int bt_calgrav_eval(struct bt_calgrav * grav,
        struct bt_kinematics * kin, gsl_vector * jtorque)
{
   int j;
   /* For each moving link, backwards ... */
   for (j=grav->dof-1; j>=0; j--)
   {
      struct bt_kinematics_link * link;
      link = kin->link[j];
      /* Fill each link's gravity vector */
      gsl_blas_dgemv(CblasTrans, 1.0, link->rot_to_world,
                     grav->world_g, 0.0, grav->g[j]);
      /* Compute each link's torque (T = g x mu) */
      gsl_vector_set_zero(grav->t[j]);
      bt_gsl_cross( grav->g[j], grav->mu[j], grav->t[j] );
      /* If it's not the last moving link, add in the next link's t */
      if (j < grav->dof-1)
         gsl_blas_dgemv(CblasNoTrans, 1.0, link->next->rot_to_prev,
                        grav->t[j+1], 1.0, grav->t[j]);
      /* Move torque vector to the previous frame */
      gsl_blas_dgemv(CblasNoTrans, 1.0, link->rot_to_prev,
                     grav->t[j], 0.0, grav->pt[j]);
      /* Add into the torque vector the z component */
      if (jtorque)
         *gsl_vector_ptr(jtorque,j) = gsl_vector_get(grav->pt[j],2);
   }
   return 0;
}
