/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wam.h
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
 *    a high-level asynchronous non-realtime interface to the WAM
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt, split from btwam into wam and wambot
 *
 * ======================================================================== */

#ifndef BT_WAM_H
#define BT_WAM_H

/* Include the bt libraries */
#include "os.h"
#include "wambot.h"
#include "log.h"
#include "gravity.h"
#include "kinematics.h"
#include "refgen.h"
#include "refgen_move.h"
#include "refgen_teachplay.h"
#include "control.h"
#include "control_joint.h"

/* A bt_wam_refgen_list represents the currently loaded refgen;
 * it keeps track of ownership and persistance */
struct bt_wam_refgen_list
{   
   struct bt_wam_refgen_list * next;
   
   int iown;
   int idelete;
   
   /* The refgen itself */
   struct bt_refgen * refgen;
   
};

struct bt_wam
{
   /* The WAM control stuff is in a separate realtime thread;
    * this is for synchronization. */
   bt_os_thread * rt_thread;
   
   /* We also have a non-realtime thread,
    * which just cleans up log files and the like */
   bt_os_thread * nonrt_thread;
   
   struct bt_os_timestat * ts; /* For timing things */
   
   int gcomp;
   int count;

   /* realtime WAM stuff */
   struct bt_wambot * wambot; /* wambot has dof */
   struct bt_kinematics * kin;
   struct bt_gravity * grav;
   struct bt_log * log; /* woo datalogger! */
   struct bt_log * ts_log; /* logger for timing statistics */

   /* Some pointers for easy access */
   gsl_vector * jposition; /* From wambot */
   gsl_vector * jvelocity; /* From wambot */
   gsl_vector * jtorque; /* From wambot */
   gsl_vector * cposition; /* From kinematics (tool) */
   gsl_matrix * crotation; /* 3x3 rotation matrix, From kinematics (tool) */
   
   /* Controllers */
   struct bt_control * con_active;
   struct bt_control_joint * con_joint;
   
   /* For moves ( rad/s(/s) in joint control mode, m/s(/s) in cartesian control mode )*/
   double vel, acc;
   
   /* This is used for both moves and teaches */
   double start_time;
   double elapsed_time;
   
   /* A WAM has a linked list of trajectories */
   struct bt_wam_refgen_list * refgen_list;
   struct bt_wam_refgen_list * refgen_current;
   
   /* Are we currently teaching into the first trajectory in the list? */
   int teach;
};


/* This function sets up a new WAM,
 * and spins off a wam_thread to control it.
 *
 * return values:
 *   0 (null pointer) - config or memory failure */
struct bt_wam * bt_wam_create(config_setting_t * wamconfig);

/* Close communication with a WAM */
void bt_wam_destroy(struct bt_wam * wam);

int bt_wam_isgcomp(struct bt_wam * wam);
int bt_wam_setgcomp(struct bt_wam * wam, int onoff);

/* These are simple wrappers for the active controller */
int bt_wam_idle(struct bt_wam * wam);
int bt_wam_hold(struct bt_wam * wam);
int bt_wam_is_holding(struct bt_wam * wam);

/* For moves */
int bt_wam_set_velocity(struct bt_wam * wam, double vel);
int bt_wam_set_acceleration(struct bt_wam * wam, double acc);

int bt_wam_moveto(struct bt_wam * wam, gsl_vector * dest);
int bt_wam_movehome(struct bt_wam * wam);
int bt_wam_moveisdone(struct bt_wam * wam);

int bt_wam_teach_start(struct bt_wam * wam);
int bt_wam_teach_end(struct bt_wam * wam);
int bt_wam_playback(struct bt_wam * wam);


#endif /* BT_WAM_H */

