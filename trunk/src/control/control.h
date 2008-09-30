/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control.h
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

/* A controller takes care of:
 *   - reference tracking
 *   - trajectories
 *      - creating
 *      - starting/stopping
 *      - pausing/unpausing
 *   - teach & play */

#ifndef BT_CONTROL_H
#define BT_CONTROL_H

#include <gsl/gsl_vector.h>

enum bt_control_mode {
   BT_CONTROL_IDLE, /* Not affecting torque */
   BT_CONTROL_IDLE_TEACH_DISCRETE,
   BT_CONTROL_IDLE_TEACH_CONTINUOUS,
   BT_CONTROL_HOLD, /* Holding current reference position */
   BT_CONTROL_TRAJ_PREP, /* Moving to start of trajectory */
   BT_CONTROL_TRAJ_READY, /* At start, waiting for go-ahead */
   BT_CONTROL_TRAJ_MOVING, /* Moving through the trajectory */
   BT_CONTROL_TRAJ_PAUSING, /* Stretching time */
   BT_CONTROL_TRAJ_PAUSED, /* Stretching time */
   BT_CONTROL_TRAJ_UNPAUSING, /* Re-condensing time */
   BT_CONTROL_OTHER /* Unknown - used to add more states */
};
char * bt_control_mode_name(int mode);

/* "Base Class" function pointers */
struct bt_control
{
   const char * name;
   
   /* Getting mode */
   enum bt_control_mode(*get_mode)(struct bt_control * base);
   
   /* Simple state switching */
   int (*idle)(struct bt_control * base);
   int (*hold)(struct bt_control * base);
   
   /* Teaching new discrete trajectories */
   int (*teach_discrete)(struct bt_control * base);
   int (*teach_continuous)(struct bt_control * base);
   int (*teach_add)(struct bt_control * base); /* add a new discrete point */
   int (*teach_done)(struct bt_control * base); /* wrap-up and get ready to start */   
   
   /* Interacting with the current trajectory */
   int (*traj_start)(struct bt_control * base, int skip_ready); /* Move to the start of the traj if necessary, then go! */
   
   /* RT - Evaluate, put resulting torque in torque */
   int (*eval)(struct bt_control * base, gsl_vector * jtorque, double time);
};

/* Everyone has their own:
 *   - create() and destroy() methods
 *   - Reference vector(s) and/or matrices
 *   - */

#endif /* BT_CONTROL_H */
