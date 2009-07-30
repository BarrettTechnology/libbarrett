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

#ifndef BT_WAM_LOCAL_H
#define BT_WAM_LOCAL_H

/* Include the bt libraries */
#include "os.h"
#include "wambot.h"
#include "log.h"
#include "kinematics.h"
#include "dynamics.h"
#include "calgrav.h"
#include "refgen.h"
#include "refgen_move.h"
#include "refgen_teachplay.h"
#include "control.h"
#include "control_joint.h"
#include "control_joint_legacy.h"
#include "control_cartesian_xyz.h"
#include "control_cartesian_xyz_q.h"
#include "rpc.h"

#include "wam.h"
#include "wam_thread.h"

#define WAMCONFIGDIR "/etc/wam/"
#define WAMLOCKDIR "/var/lock/"
#define WAMCONFIGDIRLEN (70)
#define WAMLOCKDIRLEN (70)
#define WAMNAMELEN (30)


struct bt_wam_local
{
   char name[WAMNAMELEN+1]; /* Do we even need this? */
   
   int loop_go;

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
   struct bt_calgrav * grav;
   struct bt_log * log; /* woo datalogger! */
   struct bt_log * ts_log; /* logger for timing statistics */

   /* Some pointers for easy access */
   gsl_vector * jposition; /* From wambot */
   gsl_vector * jvelocity; /* From wambot */
   gsl_vector * jtorque; /* From wambot */
   gsl_vector * cposition; /* From kinematics (tool) */
   gsl_vector * cvelocity;
   gsl_matrix * crotation; /* 3x3 rotation matrix, From kinematics (tool) */
   
   /* Controllers */
   struct bt_control * con_active;
   struct bt_control ** con_list;
   int con_num;
   /* We also keep a list of all the controllers we own */
   struct bt_control_joint * con_joint;
   struct bt_control_joint_legacy * con_joint_legacy;
   struct bt_control_cartesian_xyz * con_cartesian_xyz;
   struct bt_control_cartesian_xyz_q * con_cartesian_xyz_q;

   /* The WAM Callback */
   int (*callback)(struct bt_wam_local * wam);
   
   /* For moves ( rad/s(/s) in joint control mode, m/s(/s) in cartesian control mode )*/
   double vel, acc;
   
   /* This is used for both moves and teaches */
   double start_time;
   double elapsed_time;

   const struct bt_refgen_type ** refgen_types;
   int refgen_types_num;
   
   /* A WAM has a current refgens (and maybe a move refgen also) */
   struct bt_refgen * refgen_tempmove;
   struct bt_refgen * refgen_loaded;
   struct bt_refgen * refgen_active; /* Used by sync */
   int refgen_loaded_idestroy;
   
   /* Are we currently teaching into the current refgen? */
   int teaching;
};

struct bt_wam_local * bt_wam_local_create(char * wamname, enum bt_wam_opt opts);
int bt_wam_local_destroy(struct bt_wam_local * wam);

int bt_wam_local_loop_start(struct bt_wam_local * wam);
int bt_wam_local_loop_stop(struct bt_wam_local * wam);

char * bt_wam_local_str_jposition(struct bt_wam_local * wam, char * buf);
char * bt_wam_local_str_jvelocity(struct bt_wam_local * wam, char * buf);
char * bt_wam_local_str_jtorque(struct bt_wam_local * wam, char * buf);
char * bt_wam_local_str_cposition(struct bt_wam_local * wam, char * buf);
char * bt_wam_local_str_crotation_r1(struct bt_wam_local * wam, char * buf);
char * bt_wam_local_str_crotation_r2(struct bt_wam_local * wam, char * buf);
char * bt_wam_local_str_crotation_r3(struct bt_wam_local * wam, char * buf);

int bt_wam_local_isgcomp(struct bt_wam_local * wam);
int bt_wam_local_setgcomp(struct bt_wam_local * wam, int onoff);

int bt_wam_local_idle(struct bt_wam_local * wam);
int bt_wam_local_hold(struct bt_wam_local * wam);
int bt_wam_local_is_holding(struct bt_wam_local * wam);
char * bt_wam_local_str_con_position(struct bt_wam_local * wam, char * buf);
char * bt_wam_local_get_current_controller_name(struct bt_wam_local * wam, char * buf);
char * bt_wam_local_get_current_controller_space(struct bt_wam_local * wam, char * buf);
int bt_wam_local_controller_toggle(struct bt_wam_local * wam);

/* local only */
int bt_wam_local_control_use(struct bt_wam_local * wam, struct bt_control * control);


int bt_wam_local_refgen_clear(struct bt_wam_local * wam);
char * bt_wam_local_refgen_active_name(struct bt_wam_local * wam, char * buf);
char * bt_wam_local_refgen_loaded_name(struct bt_wam_local * wam, char * buf);
int bt_wam_local_refgen_save(struct bt_wam_local * wam, char * filename);
int bt_wam_local_refgen_load(struct bt_wam_local * wam, char * filename);

/* local only */
int bt_wam_local_refgen_addtype(struct bt_wam_local * wam, const struct bt_refgen_type * type);
int bt_wam_local_refgen_use(struct bt_wam_local * wam, struct bt_refgen * refgen);


int bt_wam_local_set_velocity(struct bt_wam_local * wam, double vel);
int bt_wam_local_set_acceleration(struct bt_wam_local * wam, double acc);
int bt_wam_local_movehome(struct bt_wam_local * wam);
int bt_wam_local_moveisdone(struct bt_wam_local * wam);

/* local only */
int bt_wam_local_moveto(struct bt_wam_local * wam, gsl_vector * dest);

int bt_wam_local_is_teaching(struct bt_wam_local * wam);
int bt_wam_local_teach_start(struct bt_wam_local * wam);
int bt_wam_local_teach_end(struct bt_wam_local * wam);
int bt_wam_local_run(struct bt_wam_local * wam);

/* local only */
int bt_wam_local_set_callback(struct bt_wam_local * wam,
                              int (*callback)(struct bt_wam_local * wam));

#endif /* BT_WAM_LOCAL_H */
