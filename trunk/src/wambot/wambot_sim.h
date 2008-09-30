/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wambot.h
 *  Author ............. Sam Clanton
 *                       Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2004 Q3
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2004-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *    wam-specific low-level functions
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt, split from btwam into wam and wambot
 *
 * ======================================================================== */

#ifndef BT_WAMBOT_SIM_H
#define BT_WAMBOT_SIM_H

#include "wambot.h"
#include "os.h"

/* bt_wambot_sim uses gsl :-) */
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/* bt_wambot_phys uses libconfig */
#include <libconfig.h>

#include <ode/ode.h>

struct bt_wambot_sim
{
   /* The base is queried from the WAM control loop */
   struct bt_wambot base;
   
   /* We should have a "base update" mutex.
    * Should that be wambot-global, anyway? */

   /* We have a thread ... */
   bt_os_thread * sim_thread;
   
   /* The current simulation time */
   bt_os_rtime sim_time;
   
   /* Calculation Duty Cycle
    * (0.0 -> 1.0) */
   double duty_cycle;
   
   /* For updating (actually used in sim each step) */
   gsl_vector * sim_jposition;
   gsl_vector * sim_jvelocity;
   gsl_vector * sim_jtorque;
   /* mutex? */
   
   /* Some statistics, like:
    * average time step
    * time step variance, etc */
   bt_os_rtime start_time;
   double sim_period_avg;
   double rest_period_avg;
   
};

struct bt_wambot_sim * bt_wambot_sim_create( config_setting_t * config );
int bt_wambot_sim_destroy( struct bt_wambot_sim * wambot );

#endif /* BT_WAMBOT_SIM_H */
