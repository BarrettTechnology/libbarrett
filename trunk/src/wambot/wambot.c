/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wambot.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2003 Feb 15
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2003-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *    wam-specific low-level functions
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt
 *
 * ======================================================================== */

#include "wambot.h"
#include "bus.h"

#include <math.h> /* For floor() */

/* For matrix inverse operations, etc */
#include <gsl/gsl_linalg.h>

/* For fasat matrix multiplication */
#include <gsl/gsl_blas.h>

/* Read config files for the wam stuff */
#include <libconfig.h>
#include <syslog.h>

/* A couple of convenience / glue functions from gsl to libconfig */
int glue_fill_vector(gsl_vector * vec, config_setting_t * parent, const char * name)
{
   int i;
   config_setting_t * child;
   child = config_setting_get_member( parent, name );
   if (child == NULL) return -1;
   if (   config_setting_type(child) != CONFIG_TYPE_ARRAY
       && config_setting_type(child) != CONFIG_TYPE_LIST ) return -1;
   if (config_setting_length(child) != vec->size) return -1;
   for (i=0; i<vec->size; i++)
   {
      config_setting_t * element;
      element = config_setting_get_elem(child,i);
      switch (config_setting_type(element))
      {
         case CONFIG_TYPE_INT:
            gsl_vector_set(vec,i,config_setting_get_int(element));
            break;
         case CONFIG_TYPE_INT64:
            gsl_vector_set(vec,i,config_setting_get_int64(element));
            break;
         case CONFIG_TYPE_FLOAT:
            gsl_vector_set(vec,i,config_setting_get_float(element));
            break;
         default:
            return -1;
      }
   }
   return 0;
}
int glue_fill_matrix(gsl_matrix * mat, config_setting_t * parent, const char * name)
{
   int i, j;
   config_setting_t * rows;
   rows = config_setting_get_member( parent, name );
   if (rows == NULL) return -1;
   if (config_setting_type(rows) != CONFIG_TYPE_LIST) return -1;
   if (config_setting_length(rows) != mat->size1) return -1;
   for (i=0; i<mat->size1; i++)
   {
      config_setting_t * row;
      row = config_setting_get_elem(rows,i);
      if (   config_setting_type(row) != CONFIG_TYPE_ARRAY
          && config_setting_type(row) != CONFIG_TYPE_LIST ) return -1;
      if (config_setting_length(row) != mat->size2) return -1;
      for (j=0; j<mat->size2; j++)
      {
         config_setting_t * element;
         element = config_setting_get_elem(row,j);
         switch (config_setting_type(element))
         {
            case CONFIG_TYPE_INT:
               gsl_matrix_set(mat,i,j,config_setting_get_int(element));
               break;
            case CONFIG_TYPE_INT64:
               gsl_matrix_set(mat,i,j,config_setting_get_int64(element));
               break;
            case CONFIG_TYPE_FLOAT:
               gsl_matrix_set(mat,i,j,config_setting_get_float(element));
               break;
            default:
               return -1;
         }
      }
   }
   return 0;
}

/* Zero the pucks */
int DefineWAMpos(struct bt_wambot * wambot, gsl_vector * jpos)
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

struct bt_wambot * bt_wambot_create( config_setting_t * botconfig )
{
   int ret;
   struct bt_wambot * wambot;
   wambot = (struct bt_wambot *) malloc(sizeof(struct bt_wambot));
   
   if (botconfig == 0) return 0;
   
   /* First, create the bus */
   {
      /* parse bus{}, dof from config file */
      config_setting_t * setting;
      
      setting = config_setting_get_member( botconfig, "bus" );
      if (setting == 0) { printf("No bus in wam.config\n"); return 0; }
      
      /* Attempt to open the system */
      wambot->bus = bt_bus_create( setting, bt_bus_UPDATE_ACCPOS );
      if (!(wambot->bus)) exit(-1);

      setting = config_setting_get_member( botconfig, "dof" );
      if (setting == 0) { printf("aaa\n"); return 0; }
      wambot->dof = config_setting_get_int(setting);
      if (wambot->dof == 0) { printf("aab\n"); return 0; }
   }
   
   /* Make sure we have pucks with the IDs we expect */

   /* Communication with actuators */
   wambot->jtorque = gsl_vector_calloc( wambot->dof );
   wambot->jposition = gsl_vector_calloc( wambot->dof );
   wambot->jvelocity = gsl_vector_calloc( wambot->dof );
   wambot->jacceleration = gsl_vector_calloc( wambot->dof );
   
   /* Physical properties to be read from config file */
   wambot->home = gsl_vector_calloc( wambot->dof );
   wambot->zeromag = gsl_vector_calloc( wambot->dof );
   wambot->j2mp = gsl_matrix_calloc(wambot->dof, wambot->dof);
   
   /* Constant cache stuff computed from config file stuff above */
   wambot->m2jp = gsl_matrix_calloc(wambot->dof, wambot->dof);
   wambot->j2mt = gsl_matrix_calloc(wambot->dof, wambot->dof);
   
   /* temporary storage locations */
   wambot->mposition = gsl_vector_calloc( wambot->dof ); /*rad*/
   wambot->mvelocity = gsl_vector_calloc( wambot->dof ); /*rad/s*/
   wambot->macceleration = gsl_vector_calloc( wambot->dof ); /*rad/s/s*/
   wambot->mtorque = gsl_vector_calloc( wambot->dof ); /*Nm*/
     
   /* parse from config file */
   ret = glue_fill_vector(wambot->home, botconfig, "home");
   if (ret) { bt_wambot_destroy(wambot); return 0; }
   ret = glue_fill_vector(wambot->zeromag, botconfig, "zeromag");
   if (ret) { printf("No zeromag entry found.\n"); }
   ret = glue_fill_matrix(wambot->j2mp, botconfig, "j2mp");
   if (ret) { bt_wambot_destroy(wambot); return 0; }
     
   /* Calculate the constant cache elements (matrix inverses) */
   {
      gsl_matrix * lu;
      gsl_permutation * p;
      int signum;
      
      lu = gsl_matrix_alloc(wambot->dof, wambot->dof);
      p = gsl_permutation_alloc(wambot->dof);
      
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
         DefineWAMpos(wambot, wambot->home); /* Assume we're exactly home */
         bt_bus_set_property(wambot->bus, SAFETY_PUCK_ID, wambot->bus->p->ZERO, 1, 1);
         /*SetByID(wam->bus, SAFETY_MODULE, ZERO, 1);*/ /* 0 = Joint velocity, 1 = Tip velocity */
         syslog(LOG_ERR, "WAM zeroed by application");
      }
   }
   
   return wambot;
}

int bt_wambot_destroy( struct bt_wambot * wambot )
{
   gsl_vector_free( wambot->jtorque );
   gsl_vector_free( wambot->jposition );
   gsl_vector_free( wambot->jvelocity );
   gsl_vector_free( wambot->jacceleration );
   
   gsl_vector_free( wambot->home );
   gsl_vector_free( wambot->zeromag );
   gsl_matrix_free( wambot->j2mp );
   
   gsl_matrix_free( wambot->m2jp );
   gsl_matrix_free( wambot->j2mt );
   
   gsl_vector_free( wambot->mposition );
   gsl_vector_free( wambot->mvelocity );
   gsl_vector_free( wambot->macceleration );
   gsl_vector_free( wambot->mtorque );
   
   free(wambot);
   
   return 0;
}

/* OK, get positions and set torques! */
int bt_wambot_update( struct bt_wambot * wambot )
{
   int j;
   
   /* Clear CAN bus of any unwanted messages */
   bt_bus_can_clearmsg( wambot->bus );
   
   /* Grab the data into the btsys actuator structs */
   bt_bus_update( wambot->bus );
   
   /* Copy the data from the actuator structs to the mposition vector */
   for (j=0; j<wambot->mposition->size; j++)
   {
      gsl_vector_set( wambot->mposition, j, wambot->bus->puck[j+1]->position ); /* aah, fix this */
      gsl_vector_set( wambot->mvelocity, j, wambot->bus->puck[j+1]->velocity ); /* aah, fix this */
      gsl_vector_set( wambot->macceleration, j, wambot->bus->puck[j+1]->acceleration ); /* aah, fix this */
   }
   
   /* Convert from motor angles to joint angles */
   gsl_blas_dgemv(CblasNoTrans, 1.0, wambot->m2jp, wambot->mposition, 0.0, wambot->jposition);
   gsl_blas_dgemv(CblasNoTrans, 1.0, wambot->m2jp, wambot->mvelocity, 0.0, wambot->jvelocity);
   gsl_blas_dgemv(CblasNoTrans, 1.0, wambot->m2jp, wambot->macceleration, 0.0, wambot->jacceleration);
   
   return 0;
}

int bt_wambot_setjtor( struct bt_wambot * wambot )
{
   int j;
   
   /* Convert from joint torques to motor torques */
   gsl_blas_dgemv(CblasNoTrans, 1.0, wambot->j2mt, wambot->jtorque, 0.0, wambot->mtorque);
   
   /* Copy the data from mtorque to the actuator structs */
   for (j=0; j<wambot->mposition->size; j++)
      wambot->bus->puck[j+1]->torque = gsl_vector_get( wambot->mtorque, j ); /* aah fix this too! */
   
   /* Send the data from the actuator structs to the bus */
   bt_bus_set_torques( wambot->bus );
   
   return 0;
}
