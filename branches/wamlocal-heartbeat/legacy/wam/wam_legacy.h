/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wam_legacy.h
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

#ifndef BT_WAM_LEGACY_H
#define BT_WAM_LEGACY_H

#include <unistd.h> /* For usleep() */

/* For internal use: */
#include "wam.h"
#include "wam_local.h"

#define TRUE 1
#define FALSE 0

/* A quick forward declaration for back-references ... */
typedef struct btwam_struct wam_struct;

/* A random struct for typedefs */
struct bt_wam_legacy_unimplemented
{
   int a;
};

/* #################### MATH STUFF #################### */

typedef double btreal;

enum bt_wam_legacy_math_type
{
   BT_WAM_LEGACY_MATH_TYPE_VECT,
   BT_WAM_LEGACY_MATH_TYPE_MATR
};

struct bt_wam_legacy_math
{
   enum bt_wam_legacy_math_type type;
   void * gsl_ptr;
};
typedef struct bt_wam_legacy_math vect_n;
typedef struct bt_wam_legacy_math vect_3;
typedef struct bt_wam_legacy_math matr_h;
typedef struct bt_wam_legacy_math matr_n;

vect_n * new_vn(int size);
int destroy_vn(vect_n **p);
int len_vn(vect_n* src);
char * sprint_vn(char * buf, vect_n * vec);
double getval_vn(vect_n * vec, int i);

vect_3 * new_v3();
vect_3 * const_v3(vect_3 * vec, double a, double b, double c);

matr_n * new_mh();

struct bt_wam_legacy_math_vect_3_q
{
   double q[3];
};

/* #################### HAPTICS STUFF #################### */

typedef struct {
   int state;
} bthaptic_scene;

typedef struct {
   struct bt_wam_legacy_math_vect_3_q * vel;
} btgeom_state;

struct bthaptic_object_struct
{
   int a;
};

typedef struct bt_wam_legacy_unimplemented bthaptic_object;
typedef struct bt_wam_legacy_unimplemented btgeom_plane;
typedef struct bt_wam_legacy_unimplemented btgeom_sphere;
typedef struct bt_wam_legacy_unimplemented btgeom_box;
typedef struct bt_wam_legacy_unimplemented bteffect_wall;
typedef struct bt_wam_legacy_unimplemented bteffect_wickedwall;
typedef struct bt_wam_legacy_unimplemented bteffect_bulletproofwall;

int new_bthaptic_scene(bthaptic_scene *bth, int size);

void init_bulletproofwall(bteffect_bulletproofwall *wall, double Boffset, double K2, double K2offset, double K1, double Bin, double Bout);
int bulletproofwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force);

int wickedwall_nf(struct bthaptic_object_struct *obj, double depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force);

void init_state_btg(btgeom_state *bts, double samplerate, double cutoffHz);
int init_bx_btg( btgeom_box *box,vect_3 *pt1, vect_3 *pt2, vect_3 *pt3, double thk, double dir1, double dir2,int inside);

/* #################### OS / THREAD STUFF #################### */

typedef struct bt_wam_legacy_unimplemented btthread;

typedef bt_os_rtime RTIME;

typedef struct
{
   bt_os_mutex * mutex;
} btrt_mutex;

int btrt_mutex_init(btrt_mutex * m);

int test_and_log(int return_val, const char *str);

typedef struct
{
   struct bt_os_thread * thread;
   int done; /* This is here so people can kill the thread w/o calling stop() */
   double period;
} btrt_thread_struct;

int btrt_thread_create(btrt_thread_struct * thd, char * thdname, int priority, void (* thd_func)(void *), void * data);
int btrt_thread_done(btrt_thread_struct * thd);
int btrt_thread_exit(btrt_thread_struct * thd);

/* #################### SYSTEM STUFF #################### */

#define SAFETY_MODULE 10
#define TL1 1
#define TL2 2
#define MT 3

int InitializeSystem();
int CloseSystem();
int ReadSystemFromConfig(char * conffile, int * buscount);

int setSafetyLimits(int bus, double a, double b, double c);
int setProperty(int bus, int id, int prop, int check, long value);

/* #################### CONTROL STUFF #################### */

enum scstate {
  SCMODE_IDLE=0, //!< Evaluation alway returns 0.0 (or a vector filled with 0.0)
  SCMODE_TORQUE, // depreciated
  SCMODE_POS, //!< The position constraint code is evaluated.
  SCMODE_TRJ //!< A trajectory is active and updating the reference point automatically.
};

enum trjstate {
  BTTRAJ_OFF = -1, //!< This mode is set on an error
  BTTRAJ_STOPPED = 0, //!< The trajectory is not running
  BTTRAJ_INPREP, //!< We are in motion from the initial position to the start position of the trajectory
  BTTRAJ_READY, //!< We are at the start position; waiting for the trajectory to be started.
  BTTRAJ_RUN, //!< The trajectory is running.
  BTTRAJ_DONE, //!< The reference point is at the end and no longer moving. The trajectory is done.
  BTTRAJ_PAUSING, //!< Time is being stretched to "pause" the trajectory.
  BTTRAJ_UNPAUSING, //!< Time is being compressed toward real-time to "unpause" the trajectory.
  BTTRAJ_PAUSED //!< Time has been stopped.
};

struct bts_btt
{
   vect_n * (* reset)(struct bts_btt * btt); /* I think this returns the start of the current traj */
};

typedef struct
{
   wam_struct * my_wam;
   struct bt_control * control;
   struct bts_btt btt; /* I don't know what this is */
} btstatecontrol;

int setmode_bts(btstatecontrol *sc, enum scstate mode);
enum scstate getmode_bts(btstatecontrol *sc);
int moveparm_bts(btstatecontrol *sc, double vel, double acc);
int start_trj_bts(btstatecontrol *sc);
int stop_trj_bts(btstatecontrol *sc);
enum trjstate get_trjstate_bts(btstatecontrol *sc);
int pause_trj_bts(btstatecontrol *sc,double period);
int unpause_trj_bts(btstatecontrol *sc,double period);

vect_n * btt_reset(struct bts_btt * btt);

/* #################### TRAJECTORY STUFF #################### */

typedef struct
{
   int a;
   int b;
} via_trj_array;

via_trj_array * new_vta(int a, int b);
int register_vta(btstatecontrol *sc, via_trj_array * vta);

/* #################### ROBOT STUFF #################### */

typedef struct bt_wam_legacy_unimplemented btrobot;

int apply_tool_force_bot(btrobot * robot, vect_3 * point, vect_3 * force, vect_3 * torque);

/* #################### WAM STUFF #################### */

/* Note that this is named twice ... */
struct btwam_struct
{
   char * name;
   int dof;
   vect_n * Jpos;
   vect_n * Jtrq;
   vect_n * Gtrq; /* We should support this! */
   vect_n * torq_limit;
   vect_n * Cpos;
   matr_h * HMpos;
   
   vect_3 * Cpoint;
   vect_3 * Cforce;
   vect_3 * Ctrq;
   
   btrobot robot;
   
   btstatecontrol Jsc;
   btstatecontrol Csc;
   
   /* For internal use: */
   struct bt_wam_local * local;
   
};

wam_struct * OpenWAM(char * conffile, int which);
void WAMControlThread(struct bt_os_thread * thd);
int registerWAMcallback(wam_struct * wam, int (* thd_func)(wam_struct *));

int MoveSetup(wam_struct * wam, double vel, double acc);
int MoveWAM(wam_struct * wam, vect_n * dest);
int SetGravityComp(wam_struct * wam, double gcompval);

/* #################### SERIAL STUFF #################### */

#define PORT int

int serialOpen(PORT * port, char * device);
int serialSetBaud(PORT * port, int rate);

#endif /* BT_WAM_LEGACY_H */
