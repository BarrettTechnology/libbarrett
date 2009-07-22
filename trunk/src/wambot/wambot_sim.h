/** Definition of bt_wambot_sim, a simulated WAM robot.
 *
 * \file wambot_sim.h
 * \author Christopher Dellin
 * \date 2008-2009
 */

/* Copyright 2008, 2009 Barrett Technology <support@barrett.com> */

/*
 * This file is part of libbarrett.
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

#ifndef BT_WAMBOT_SIM_H
#define BT_WAMBOT_SIM_H

#include <libconfig.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "wambot.h"
#include "os.h"

/** A simlulated WAM robot, which consists of a series links connected by
 *  revolute joints.
 *
 * A "wambot_sim" is a low-level abstraction of a simulated Barrett WAM
 * robot using the ODE physics simulator.  Currently, this is not guarenteed
 * to work.
 */
struct bt_wambot_sim
{
   /** We "inherit" from a bt_wambot. */
   struct bt_wambot base;
   
   /* We should have a "base update" mutex.
    * Should that be wambot-global, anyway? */

   /** We have a thread ... */
   bt_os_thread * sim_thread;
   
   /** The current simulation time */
   bt_os_rtime sim_time;
   
   /** Calculation Duty Cycle
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

/** bt_wambot_sim creation function, given a configuration group.
 *
 * This function creates a new wambot_sim object, given a libconfig
 * configuration group.
 */
struct bt_wambot_sim * bt_wambot_sim_create( config_setting_t * config );

/** bt_wambot_sim destroy function.
 *
 * This function destroys a wambot_sim.
 */
int bt_wambot_sim_destroy( struct bt_wambot_sim * wambot );

#endif /* BT_WAMBOT_SIM_H */
