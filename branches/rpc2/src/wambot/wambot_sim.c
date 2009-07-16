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
#include "wambot_sim.h"

#include "os.h"

#include <gsl/gsl_vector.h>

/* Read config files for the wam stuff */
#include <libconfig.h>
#include <syslog.h>

#include <ode/ode.h>

static int update( struct bt_wambot * base );
static int setjtor( struct bt_wambot * base );

void sim_func(bt_os_thread * thread);

/* A setup helper for thread creation */
struct setup_helper
{
   struct bt_wambot_sim * wambot;
   config_setting_t * config;
   int is_setup;
   int setup_failed;
};
static struct setup_helper * helper_create(struct bt_wambot_sim * wambot, config_setting_t * config)
{
   struct setup_helper * helper;
   helper = (struct setup_helper *) malloc( sizeof(struct setup_helper) );
   if (!helper) return 0;
   helper->wambot = wambot;
   helper->config = config;
   helper->is_setup = 0;
   helper->setup_failed = 0;
   return helper;
}
static void helper_destroy(struct setup_helper * helper)
{
   free(helper);
   return;
}

/* A couple of convenience / glue functions from gsl to libconfig */
static int glue_fill_vector(gsl_vector * vec, config_setting_t * parent, const char * name)
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
         case CONFIG_TYPE_FLOAT:
            gsl_vector_set(vec,i,config_setting_get_float(element));
            break;
         default:
            return -1;
      }
   }
   return 0;
}

/* NOW, THE GLOBAL FUNCTIONS ... */
struct bt_wambot_sim * bt_wambot_sim_create( config_setting_t * config )
{
   int err;
   struct bt_wambot_sim * wambot;
   struct bt_wambot * base;
   struct setup_helper * helper;
   
   /* Check arguments */
   if (config == 0) return 0;
   
   /* Make a new wambot */
   wambot = (struct bt_wambot_sim *) malloc(sizeof(struct bt_wambot_sim));
   if (!wambot)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   base = (struct bt_wambot *) wambot;
    
   /* Initialize to zeros */
   base->jtorque = 0;
   base->jposition = 0;
   base->jvelocity = 0;
   base->jacceleration = 0;
   base->home = 0;
   base->update = &update;
   base->setjtor = &setjtor;
   
   {
      config_setting_t * setting;

      /* Grab the degrees-of-freedom
       * For now, this has to be 4! */
      if ( !(setting = config_setting_get_member( config, "dof" ))
          || !(base->dof = config_setting_get_int(setting) ) )
      {
         syslog(LOG_ERR,"%s: No (or zero) dof in configuration.",__func__);
         bt_wambot_sim_destroy(wambot);
         return 0; 
      }
   }
   if (base->dof != 4)
   {
      syslog(LOG_ERR,"%s: We can only simulate a 4DOF WAM!",__func__);
      bt_wambot_sim_destroy(wambot);
      return 0;
   }
   
   /* Make sure we have pucks with the IDs we expect */

   /* Communication with actuators */
   base->jtorque = gsl_vector_calloc( base->dof );
   base->jposition = gsl_vector_calloc( base->dof );
   base->jvelocity = gsl_vector_calloc( base->dof );
   base->jacceleration = gsl_vector_calloc( base->dof );
   
   /* Physical properties to be read from config file */
   base->home = gsl_vector_calloc( base->dof );
   
   /* parse from config file */
   /* NOTE - DO BETTER ERROR CHECKING HERE! */
   err = glue_fill_vector(base->home, config, "home");
   if (err) { bt_wambot_sim_destroy(wambot); return 0; }
   
   /* Target proportion of time spent actually calculating ... */
   wambot->duty_cycle = 0.1;
   
   /* Allocate simulation vectors */
   wambot->sim_jposition = gsl_vector_calloc( base->dof );
   wambot->sim_jvelocity = gsl_vector_calloc( base->dof );
   wambot->sim_jtorque = gsl_vector_calloc( base->dof );
   
   /* Spin off the simulation thread */
   helper = helper_create(wambot,config);
   if (!helper)
   {
      syslog(LOG_ERR,"%s: Could not create setup helper.",__func__);
      bt_wambot_sim_destroy(wambot);
      return 0;
   }
   wambot->sim_thread = bt_os_thread_create(BT_OS_RT, "SIM", 90, sim_func, (void *)helper);
   if (!wambot->sim_thread)
   {
      syslog(LOG_ERR,"%s: Could not create realtime simulation thread.",__func__);
      helper_destroy(helper);
      bt_wambot_sim_destroy(wambot);
      return 0;
   }
   
   /* Wait until the thread is done starting */
   while (!helper->is_setup)
      bt_os_usleep(10000);
   
   /* Check for setup failure */
   if (helper->setup_failed)
   {
      syslog(LOG_ERR,"%s: WAM simulation setup failed.",__func__);
      helper_destroy(helper);
      bt_wambot_sim_destroy(wambot);
      return 0;
   }
   
   /* Success! */
   helper_destroy(helper);
   return wambot;
}

int bt_wambot_sim_destroy( struct bt_wambot_sim * wambot )
{
   struct bt_wambot * base = (struct bt_wambot *) wambot;
   
   /* Tell the realtime thread to exit */
   if (wambot->sim_thread)
   {
      wambot->sim_thread->done = 1;
      bt_os_usleep(30000); /* We can actually check for this! */
   }
   
   if (base->jtorque) gsl_vector_free( base->jtorque );
   if (base->jposition) gsl_vector_free( base->jposition );
   if (base->jvelocity) gsl_vector_free( base->jvelocity );
   if (base->jacceleration) gsl_vector_free( base->jacceleration );
   if (base->home) gsl_vector_free( base->home );
   
   free(wambot);
   return 0;
}

/* OK, get positions and set torques! */
static int update( struct bt_wambot * base )
{
   struct bt_wambot_sim * wambot = (struct bt_wambot_sim *) base;
   
   gsl_vector_memcpy( base->jposition, wambot->sim_jposition);
   gsl_vector_memcpy( base->jvelocity, wambot->sim_jvelocity);
   gsl_vector_set_zero( base->jacceleration);
   
   return 0;
}

static int setjtor( struct bt_wambot * base )
{
   struct bt_wambot_sim * wambot = (struct bt_wambot_sim *) base;
   
   gsl_vector_memcpy( wambot->sim_jtorque, base->jtorque );
   return 0;
}

/* OK, here comes the realtime simulation thread ... */
void sim_func(bt_os_thread * thread)
{
   int j;
   struct setup_helper * helper = (struct setup_helper *) thread->data;
   struct bt_wambot_sim * wambot = helper->wambot;
   
   /* For now, we will ONLY simulate a 4DOF WAM! */
   dWorldID world;
   dBodyID body[4];
   dMass mass[4];
   dJointID joint[4];
   
   /* Set up the world */
   world = dWorldCreate();
   dWorldSetERP( world, 0.5 );
   dWorldSetGravity( world, 0.0, 0.0, -9.81 );
   
   /* Eventually, we'll get this info from the config file ... */
   /* Create bodies ... */
   for (j=0; j<4; j++)
       body[j] = dBodyCreate(world);
   /* Create masses (inertial matrices) (mass, direction, radius, height) */
   dMassSetCylinderTotal( mass + 0, 8.39, 3, 0.140, 0.37 ); /* 5kg, 28cm diam, 37cm high */
   dMassSetCylinderTotal( mass + 1, 4.85, 2, 0.090, 0.28 ); /* 1kg, y-axis, 18cm diam, 28cm high */
   dMassSetCylinderTotal( mass + 2, 1.72, 3, 0.045, 0.55 );
   dMassSetCylinderTotal( mass + 3, 1.09, 3, 0.045, 0.45 );
   for (j=0; j<4; j++)
       dBodySetMass( body[j], mass + j );
   /* Put each body in the zero position (relative to CM!) */
   dBodySetPosition( body[0], 0.0, 0.0, 0.37/2 ); /* base CM is halfway up */
   dBodySetPosition( body[1], 0.0, 0.0, 0.37 ); /* pitch CM is on top of base */
   dBodySetPosition( body[2], 0.0, 0.0, 0.37 + 0.55/2 ); /* twist CM is halfway up */
   dBodySetPosition( body[3], 0.0, 0.0, 0.37 + 0.55 + 0.45/2 ); /* elbow is halfway up */
   /* Create joints */
   for (j=0; j<4; j++)
   {
      joint[j] = dJointCreateHinge( world, 0 );
      if (j)
         dJointAttach( joint[j], body[j], body[j-1] );
      else
         dJointAttach( joint[j], body[j], 0 );
   }
   dJointSetHingeAnchor( joint[0], 0.0, 0.0, 0.0 );
   dJointSetHingeAxis( joint[0], 0.0, 0.0, 1.0 );
   dJointSetHingeAnchor( joint[1], 0.0, 0.0, 0.37 );
   dJointSetHingeAxis( joint[1], 0.0, 1.0, 0.0 );
   dJointSetHingeAnchor( joint[2], 0.0, 0.0, 0.37 );
   dJointSetHingeAxis( joint[2], 0.0, 0.0, 1.0 );
   dJointSetHingeAnchor( joint[3], 0.045, 0.0, 0.37 + 0.55 );
   dJointSetHingeAxis( joint[3], 0.0, 1.0, 0.0 );
   /* Set joint stops */
   dJointSetHingeParam( joint[0], dParamLoStop, -3.1);
   dJointSetHingeParam( joint[0], dParamHiStop, 3.1);
   dJointSetHingeParam( joint[1], dParamLoStop, -1.9);
   dJointSetHingeParam( joint[1], dParamHiStop, 1.9);
   dJointSetHingeParam( joint[2], dParamLoStop, -3.1);
   dJointSetHingeParam( joint[2], dParamHiStop, 3.1);
   dJointSetHingeParam( joint[3], dParamLoStop, -0.78);
   dJointSetHingeParam( joint[3], dParamHiStop, 3.14);
   /* Put the thing in the home position */
   for (j=0; j<4; j++)
   {
      /* Rotate all joints ... */
      dVector3 axis;
      dQuaternion quatChange;
      const dReal * quatCur;
      dQuaternion quatRes;
      
      dVector3 anchor1;
      dVector3 anchor2;
      dReal * pos;
      
      int i;
      
      dJointGetHingeAxis( joint[j], axis );
      syslog(LOG_ERR,"Joint %d Axis: < %f, %f, %f >",j,axis[0],axis[1],axis[2]);
      dQFromAxisAndAngle( quatChange, axis[0], axis[1], axis[2],
         gsl_vector_get(wambot->base.home,j) - dJointGetHingeAngle(joint[j]) );
      for (i=j; i<4; i++)
      {
         quatCur = dBodyGetQuaternion( body[i] );
         dQMultiply0( quatRes, quatChange, quatCur );
         dBodySetQuaternion( body[i], quatRes );
      }
      for (i=j; i<4; i++)
      {
         dJointGetHingeAnchor( joint[i], anchor1 );
         dJointGetHingeAnchor2( joint[i], anchor2 );
         pos = (dReal *) dBodyGetPosition( body[i] );
         pos[0] -= anchor1[0] - anchor2[0];
         pos[1] -= anchor1[1] - anchor2[1];
         pos[2] -= anchor1[2] - anchor2[2];
         dBodySetPosition( body[i], pos[0], pos[1], pos[2] );
      }
   }
   
   /* Setup is complete! */
   helper->is_setup = 1;
   
   /* Start the simulation at the current time */
   wambot->sim_time = bt_os_rt_get_time();
   
   /* Loop until the thread is done. */
   while (!bt_os_thread_done(thread))
   {
      bt_os_rtime sim_start;
      bt_os_rtime sim_end;
      
      /* Measure the simulation duration ... */
      sim_start = bt_os_rt_get_time();
      
      /* Copy in the current torques */
      for (j=0; j<4; j++)
         dJointAddHingeTorque(joint[j], gsl_vector_get(wambot->sim_jtorque,j) );
       
      /* Apply a (LARGE) friction torque */
      dJointAddHingeTorque(joint[0],
         - 20.0 * dJointGetHingeAngleRate(joint[0]) );
      dJointAddHingeTorque(joint[1],
         - 20.0 * dJointGetHingeAngleRate(joint[1]) );
      dJointAddHingeTorque(joint[2],
         - 10.0 * dJointGetHingeAngleRate(joint[2]) );
      dJointAddHingeTorque(joint[3],
         - 10.0 * dJointGetHingeAngleRate(joint[3]) );

      /* Simulate up until the start time  */
      dWorldStep( world, 1e-9 * (sim_start - wambot->sim_time) );
      wambot->sim_period_avg = 1e-9 * (sim_start - wambot->sim_time);
      wambot->sim_time = sim_start;
      
      /* Update positions (and velocities, etc) */
      for (j=0; j<4; j++)
      {
         gsl_vector_set( wambot->sim_jposition, j, dJointGetHingeAngle(joint[j]) );
         gsl_vector_set( wambot->sim_jvelocity, j, dJointGetHingeAngleRate(joint[j]) );
      }
      
      /* Update averages */
      wambot->start_time = sim_start;
      
      /* End measuring the simulation duration */
      sim_end = bt_os_rt_get_time();
      
      /* Sleep, to obey duty_cycle  */
      bt_os_usleep( 1e-3 * (sim_end - sim_start) * (1.0 - wambot->duty_cycle) / wambot->duty_cycle );
   }
   
   dWorldDestroy(world);
   
   /* Remove this thread from the realtime scheduler */
   bt_os_thread_exit( thread );
   
   return;
}
