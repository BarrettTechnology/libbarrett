/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... trajectory.h
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2005 Mar 30
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt; merged from btstatecontrol and btpath
 *
 * ======================================================================== */

#ifndef BT_TRAJECTORY_H
#define BT_TRAJECTORY_H

#include <gsl/gsl_vector.h>

/* Note: trajectories are time-invariant */

/* Bound methods */
#define bt_trajectory_destroy(t) (t->type->destroy(t))
#define bt_trajectory_get_num_points(t) (t->type->get_num_points(t))
#define bt_trajectory_get_total_time(t,time) (t->type->get_total_time(t,time))
#define bt_trajectory_get_start(t,s) (t->type->get_start(t,s))
#define bt_trajectory_get_reference(t,ref,time) (t->type->get_reference(t,ref,time))

struct bt_trajectory;

struct bt_trajectory_type
{
   char name[20];
   
   int (*destroy)(struct bt_trajectory * t);
   
   /* Define the asynchronous interface */
   int (*get_num_points)(struct bt_trajectory * t);
   int (*get_total_time)(struct bt_trajectory * t, double * time);
   int (*get_start)(struct bt_trajectory * t, gsl_vector ** start);

   /* Define the synchronous interface */
   int (*get_reference)(struct bt_trajectory * t, gsl_vector * ref, double time);
};

/* A path */
struct bt_trajectory {
   const struct bt_trajectory_type * type;
};

#endif /* BT_TRAJECTORY_H */
