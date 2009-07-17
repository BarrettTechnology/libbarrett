/** Implementation of bt_wambot_phys, a physical WAM robot.
 *
 * \file wambot_phys.c
 * \author Christopher Dellin
 * \date 2008-2009
 */

/* Copyright 2008, 2009 Barrett Technology <support@barrett.com> */

/*
 * This file is part of libbarrett.
 *
 * libbarrett is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libbarrett is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libbarrett.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */

#include <math.h> /* For floor() */
#include <syslog.h>
#include <libconfig.h>
#include <gsl/gsl_linalg.h> /* For matrix inverse operations, etc */
#include <gsl/gsl_blas.h> /* For fast matrix multiplication */

#include "wambot.h"
#include "wambot_phys.h"
#include "bus.h"
#include "gsl.h"

/** Implementation of bt_wambot::update for bt_wambot_phys. */
static int update( struct bt_wambot * base );

/** Implementation of bt_wambot::setjtor for bt_wambot_phys. */
static int setjtor( struct bt_wambot * base );

/** Initialize the pucks using the given position. */
int define_pos(struct bt_wambot_phys * wambot, gsl_vector * jpos)
{
   int j;
   long pos;
   
   /* Convert from joint space to motor space */
   gsl_blas_dgemv(CblasNoTrans, 1.0, wambot->j2mp, jpos, 0.0, wambot->mposition);

   /* Tell the safety logic to ignore the next several faults
    * (position will appear to be changing rapidly) */
   bt_bus_set_property(wambot->bus, SAFETY_PUCK_ID, wambot->bus->p->IFAULT, 1, 8);
   
   /* Manually set the encoder values */
   for (j=0; j<jpos->size; j++)
   {
      pos = floor( 4096 / (2*3.14159265358979) * gsl_vector_get(wambot->mposition,j) );
      bt_bus_set_property(wambot->bus, j+1, wambot->bus->p->AP, 0, pos);
      bt_os_usleep(1000);
   }
   
   return 0;
}

/* NOW, THE GLOBAL FUNCTIONS ... */

struct bt_wambot_phys * bt_wambot_phys_create( config_setting_t * config )
{
   int err;
   struct bt_wambot_phys * wambot;
   int n;
   
   /* Check arguments */
   if (config == 0) return 0;
   
   /* Make a new wambot */
   wambot = (struct bt_wambot_phys *) malloc(sizeof(struct bt_wambot_phys));
   if (!wambot)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
    
   /* Initialize to zeros */
   wambot->base.jtorque = 0;
   wambot->base.jposition = 0;
   wambot->base.jvelocity = 0;
   wambot->base.home = 0;
   wambot->base.update = &update;
   wambot->base.setjtor = &setjtor;
   
   wambot->bus = 0;
   wambot->zeromag = 0;
   wambot->j2mp = 0;
   wambot->m2jp = 0;
   wambot->j2mt = 0;
   wambot->mposition = 0;
   wambot->mvelocity = 0;
   
   /* First, create the bus */
   {
      /* parse bus{}, dof from config file */
      config_setting_t * setting;
      
      setting = config_setting_get_member( config, "bus" );
      if (!setting)
      {
         syslog(LOG_ERR,"%s: No bus in configuration.",__func__);
         bt_wambot_phys_destroy(wambot);
         return 0;
      }
      
      /* Attempt to open the system */
      wambot->bus = bt_bus_create( setting, bt_bus_UPDATE_POS_DIFF );
      if (!wambot->bus)
      {
         syslog(LOG_ERR,"%s: Could not create the bus.",__func__);
         bt_wambot_phys_destroy(wambot);
         return 0;
      }

      /* Grab the degrees-of-freedom */
      if ( !(setting = config_setting_get_member(config,"dof"))
          || !(wambot->base.dof = config_setting_get_int(setting) ) )
      {
         syslog(LOG_ERR,"%s: No (or zero) dof in configuration.",__func__);
         bt_wambot_phys_destroy(wambot);
         return 0; 
      }
   }
   
   n = wambot->base.dof;
   
   /* Make sure we have pucks with the IDs we expect */
   
   /* Allocate vectors */
   if ( 0
       /* Communication with actuators */
       || !( wambot->base.jtorque       = gsl_vector_calloc(n) )
       || !( wambot->base.jposition     = gsl_vector_calloc(n) )
       || !( wambot->base.jvelocity     = gsl_vector_calloc(n) )
       /* Physical properties to be read from config file */
       || !( wambot->base.home          = gsl_vector_calloc(n) )
       || !( wambot->zeromag            = gsl_vector_calloc(n) )
       || !( wambot->j2mp             = gsl_matrix_calloc(n,n) )
       /* Constant cache stuff computed from config file stuff above */
       || !( wambot->m2jp             = gsl_matrix_calloc(n,n) )
       || !( wambot->j2mt             = gsl_matrix_calloc(n,n) )
       /* temporary storage locations */
       || !( wambot->mposition          = gsl_vector_calloc(n) ) /*rad*/
       || !( wambot->mvelocity          = gsl_vector_calloc(n) ) /*rad/s*/
       || !( wambot->mtorque            = gsl_vector_calloc(n) ) /*Nm*/
   ) {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      bt_wambot_phys_destroy(wambot);
      return 0;
   }
   
   /* Parse values from config file */
   /* NOTE - DO BETTER ERROR CHECKING HERE! */
   err = bt_gsl_fill_vector(wambot->base.home, config, "home");
   if (err) { bt_wambot_phys_destroy(wambot); return 0; }
   err = bt_gsl_fill_vector(wambot->zeromag, config, "zeromag");
   if (err) { printf("No zeromag entry found.\n"); }
   err = bt_gsl_fill_matrix(wambot->j2mp, config, "j2mp");
   if (err) { bt_wambot_phys_destroy(wambot); return 0; }
     
   /* Calculate the constant cache elements (matrix inverses) */
   {
      gsl_matrix * lu;
      gsl_permutation * p;
      int signum;
      
      lu = gsl_matrix_alloc(n, n);
      if (!lu)
      {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         bt_wambot_phys_destroy(wambot);
         return 0;
      }
      p = gsl_permutation_alloc(n);
      if (!p)
      {
         syslog(LOG_ERR,"%s: Out of memory.",__func__);
         gsl_matrix_free(lu);
         bt_wambot_phys_destroy(wambot);
         return 0;
      }
      
      /* m2jp = inv(j2mp) */
      gsl_matrix_memcpy(lu,wambot->j2mp);
      gsl_linalg_LU_decomp( lu, p, &signum );
      gsl_linalg_LU_invert( lu, p, wambot->m2jp );
      
      /* j2mt = inv(j2mp^T) */
      gsl_matrix_transpose_memcpy(lu,wambot->j2mp);
      gsl_linalg_LU_decomp( lu, p, &signum );
      gsl_linalg_LU_invert( lu, p, wambot->j2mt );
      
      gsl_matrix_free(lu);
      gsl_permutation_free(p);
   }
    
   /* note - SetEngrUnits() defaults to 1; we shouldn't touch it here. */
   
   {
      /* Zero the WAM */
      long reply;
      /*btsys_getProperty(bot->bus, SAFETY_MODULE, ZERO, &reply);*/
      bt_bus_get_property(wambot->bus, SAFETY_PUCK_ID, wambot->bus->p->ZERO, &reply);
      if(reply) {
         syslog(LOG_ERR, "WAM was already zeroed");
      } else {
         define_pos(wambot, wambot->base.home); /* Assume we're exactly home */
         bt_bus_set_property(wambot->bus, SAFETY_PUCK_ID, wambot->bus->p->ZERO, 1, 1);
         /*SetByID(wam->bus, SAFETY_MODULE, ZERO, 1);*/ /* 0 = Joint velocity, 1 = Tip velocity */
         syslog(LOG_ERR, "WAM zeroed by application");
      }
   }
   
   return wambot;
}

int bt_wambot_phys_destroy( struct bt_wambot_phys * wambot )
{
   struct bt_wambot * base = (struct bt_wambot *) wambot;
   
   /* Free vectors */
   
   if (base->jtorque) gsl_vector_free( base->jtorque );
   if (base->jposition) gsl_vector_free( base->jposition );
   if (base->jvelocity) gsl_vector_free( base->jvelocity );
   if (base->home) gsl_vector_free( base->home );
    
   if (wambot->zeromag) gsl_vector_free( wambot->zeromag );
   if (wambot->j2mp) gsl_matrix_free( wambot->j2mp );
   
   if (wambot->m2jp) gsl_matrix_free( wambot->m2jp );
   if (wambot->j2mt) gsl_matrix_free( wambot->j2mt );
   
   if (wambot->mposition) gsl_vector_free( wambot->mposition );
   if (wambot->mvelocity) gsl_vector_free( wambot->mvelocity );
   if (wambot->mtorque) gsl_vector_free( wambot->mtorque );
   
   if (wambot->bus)
      bt_bus_destroy(wambot->bus);
   
   free(wambot);
   return 0;
}

/* OK, get positions and set torques! */
static int update( struct bt_wambot * base )
{
   struct bt_wambot_phys * wambot = (struct bt_wambot_phys *) base;
   int j;
   
   /* Grab the data into the btsys actuator structs
    * (note that this calls bt_bus_can_clearmsg() for us) */
   bt_bus_update( wambot->bus );
   
   /* Copy the data from the actuator structs to the mposition vector */
   for (j=0; j<wambot->mposition->size; j++)
   {
      gsl_vector_set( wambot->mposition, j, wambot->bus->puck[j+1]->position ); /* aah, fix this */
      gsl_vector_set( wambot->mvelocity, j, wambot->bus->puck[j+1]->velocity ); /* aah, fix this */
   }
   
   /* Convert from motor angles to joint angles */
   gsl_blas_dgemv(CblasNoTrans, 1.0, wambot->m2jp, wambot->mposition, 0.0, base->jposition);
   gsl_blas_dgemv(CblasNoTrans, 1.0, wambot->m2jp, wambot->mvelocity, 0.0, base->jvelocity);
   
   return 0;
}

static int setjtor( struct bt_wambot * base )
{
   struct bt_wambot_phys * wambot = (struct bt_wambot_phys *) base;
   int j;
   
   /* Convert from joint torques to motor torques */
   gsl_blas_dgemv(CblasNoTrans, 1.0, wambot->j2mt, base->jtorque, 0.0, wambot->mtorque);
   
   /* Copy the data from mtorque to the actuator structs */
   for (j=0; j<wambot->mposition->size; j++)
      wambot->bus->puck[j+1]->torque = gsl_vector_get( wambot->mtorque, j ); /* aah fix this too! */
   
   /* Send the data from the actuator structs to the bus */
   bt_bus_set_torques( wambot->bus );
   
   return 0;
}
