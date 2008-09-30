/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_joint.h
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Sam Clanton
 *                       Christopher Dellin
 *  Creation Date ...... Nov 24, 2002
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2002 Nov 24 - TH
 *      File created.
 *    2004 Dec 16 - BZ, SC, TH
 *      Initial port to linux + RTAI
 *    2008 Sept 16 - CD
 *      Ported from btsystem to libbt; btstatecontrol and btcontrol merged
 *
 * ======================================================================== */

#include "control.h"

#include <libconfig.h>

/* Woo basic independent-PID joint controller! */
struct bt_control_joint
{
   /* Include the base function pointers */
   struct bt_control base;
   
   enum bt_control_mode mode;
   
   /* Pointers to external vectors */
   gsl_vector * jposition;
   gsl_vector * jvelocity;
   
   /* Owned by me: */
   gsl_vector * reference;
   gsl_vector * Kp;
   gsl_vector * Ki;
   gsl_vector * Kd;
   gsl_vector * integrator;
   gsl_vector * temp1;
   gsl_vector * temp2;

   struct bt_trajectory_spline * move_spline; /* The current spline, if there is one */
   struct bt_trajectory_profile * move_profile; /* The current profile, if there is one */
   
   struct bt_trajectory_spline * traj_spline; /* The current spline, if there is one */
   struct bt_trajectory_profile * traj_profile; /* The current profile, if there is one */
   
   /* When we start trajectories,
    * this is the start time! */
   double start_time;
   double last_time;
   int start_time_saved;
   
   /* Skip the TRAJ_READY state when starting a trajectory */
   int skip_ready;
};

struct bt_control_joint * bt_control_joint_create(config_setting_t * config, gsl_vector * jposition, gsl_vector * jvelocity);
void bt_control_joint_destroy(struct bt_control_joint *);
