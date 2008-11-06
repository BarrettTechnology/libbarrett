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
#include "trajectory.h"
#include "control.h"
#include "control_joint.h"

struct bt_wam_path
{
   char * name;
   struct bt_wam_path * next;
   struct bt_wam_path * prev;
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

   /* Controllers */
   struct bt_control * con_active;
   struct bt_control_joint * con_joint;
   
   /* Some pointers for easy access */
   gsl_vector * jposition; /* From wambot */
   gsl_vector * jvelocity; /* From wambot */
   gsl_vector * jtorque; /* From wambot */
   gsl_vector * cposition; /* From kinematics (tool) */
   gsl_matrix * crotation; /* 3x3 rotation matrix, From kinematics (tool) */
   
   /* A WAM has a list of named paths (linked list) */
   struct bt_wam_path * path_list;
   struct bt_wam_path * path_editing;
   
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

int bt_wam_path_new(struct bt_wam * const wam, const char * const name);
int bt_wam_path_delete(struct bt_wam * const wam, const char * const name);
int bt_wam_path_delete_all(struct bt_wam * const wam);
int bt_wam_path_get_number(const struct bt_wam * const wam, int * const num);
int bt_wam_path_get_name(const struct bt_wam * const wam, const int num, char ** nameptr);
int bt_wam_path_set_editing(struct bt_wam * const wam, char * name);
int bt_wam_path_get_editing(const struct bt_wam * const wam, char ** nameptr);



#endif /* BT_WAM_H */

