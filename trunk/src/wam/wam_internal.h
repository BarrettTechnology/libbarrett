/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wam_internal.h
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

#ifndef BT_WAM_INTERNAL_H
#define BT_WAM_INTERNAL_H

/* Include the bt libraries */
#include "os.h"
#include "wambot.h"
#include "log.h"
#include "kinematics.h"
#include "dynamics.h"
#include "gravity.h"
#include "refgen.h"
#include "refgen_move.h"
#include "refgen_teachplay.h"
#include "control.h"
#include "control_joint.h"
#include "control_cartesian_xyz.h"
#include "control_joint_legacy.h"
#include "rpc.h"
#include "wam_rpc.h"

#define WAMCONFIGDIR "/etc/wam/"
#define WAMCONFIGDIRLEN (70)
#define WAMNAMELEN (30)

/* Shortcut for proxy stuff */
#define bt_wam_proxy_handle(f,w,...) \
   bt_rpc_caller_handle(((struct bt_wam_proxy *)(w))->caller,bt_wam_rpc, \
                        (f),((struct bt_wam_proxy *)(w))->obj,__VA_ARGS__)

enum bt_wam_type
{
   BT_WAM_LOCAL,
   BT_WAM_PROXY
};

struct bt_wam_proxy
{
   /* This tells us if it's a local or RPC proxy wam */
   enum bt_wam_type type;
   
   struct bt_rpc_caller * caller;
   void * obj;
};

/* A bt_wam_refgen_list represents the currently loaded refgen;
 * it keeps track of ownership and persistance */
struct bt_wam_refgen_list
{   
   struct bt_wam_refgen_list * next;
   
   int iown;
   
   /* The refgen itself */
   struct bt_refgen * refgen;
   
};

struct bt_wam
{
   /* This tells us if it's a local or RPC proxy wam */
   enum bt_wam_type type;
   
   char name[WAMNAMELEN+1]; /* Do we even need this? */

   /* The WAM control stuff is in a separate realtime thread;
    * this is for synchronization. */
   struct bt_os_thread * rt_thread;
   
   /* We also have a non-realtime thread,
    * which just cleans up log files and the like */
   struct bt_os_thread * nonrt_thread;
   
   struct bt_os_timestat * ts; /* For timing things */
   
   int gcomp;
   int count;

   /* realtime WAM stuff */
   struct bt_wambot * wambot; /* wambot has dof */
   struct bt_kinematics * kin;
   struct bt_dynamics * dyn;
   struct bt_gravity * grav;
   struct bt_log * log; /* woo datalogger! */
   struct bt_log * ts_log; /* logger for timing statistics */

   /* Some pointers for easy access */
   gsl_vector * jposition; /* From wambot */
   gsl_vector * jvelocity; /* From wambot */
   gsl_vector * jacceleration; /* From wambot */
   gsl_vector * jtorque; /* From wambot */
   gsl_vector * cposition; /* From kinematics (tool) */
   gsl_vector * cvelocity;
   gsl_matrix * crotation; /* 3x3 rotation matrix, From kinematics (tool) */
   
   /* Controllers */
   struct bt_control * con_active;
   struct bt_control ** con_list;
   int con_num;
   /* We also keep a list of all the controllers we have */
   struct bt_control_joint * con_joint;
   struct bt_control_joint_legacy * con_joint_legacy;
   struct bt_control_cartesian_xyz * con_cartesian_xyz;
   
   /* For moves ( rad/s(/s) in joint control mode, m/s(/s) in cartesian control mode )*/
   double vel, acc;
   
   /* This is used for both moves and teaches */
   double start_time;
   double elapsed_time;
   
   /* A WAM has a linked list of refgens */
   struct bt_wam_refgen_list * refgen_list;
   struct bt_wam_refgen_list * refgen_current;
   
   /* Are we currently teaching into the first trajectory in the list? */
   int teach;
};

#endif /* BT_WAM_INTERNAL_H */
