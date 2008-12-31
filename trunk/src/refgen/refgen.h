/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... refgen.h
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

#ifndef BT_REFGEN_H
#define BT_REFGEN_H

#include <gsl/gsl_vector.h>

/* "refgen" - a reference generator
 *
 * A refgen is the first half of the simple libbt control architecture,
 * illustrated below:
 *
 * /------------------------\                       /---------\
 * |         refgen         | ->    reference    -> | control | -> jtorque
 * \------------------------/    (in some space)    \---------/
 *    |         |        |
 *    v         v        v
 * elapsed   current    other
 *  time    controller  info
 *           position
 *
 * Each refgen type has its own custom creation function, which allows it
 * to reference any necessary data (such as that in the WAM structure),
 * such as elapsed time, current end-effector position, or other info.
 *
 * During the WAM control loop, the active refgen is evaluated, which
 * generates a reference vector from its internal process.  This reference
 * vector is then fed into the current controller.  The refgen can
 * determine the current controller by examining the WAM structure.
 *
 * Common refgen references include:
 *  - elapsed time (double, in seconds)
 *  - current end-effector position (Cartesian vector)
 *  - current end-effector rotation (euler-angle vector, quaternion?)
 */

/* Note: refgens are time-invariant */

/* Bound methods */
#define bt_refgen_destroy(t) (t->type->destroy(t))
#define bt_refgen_get_start(t,s) (t->type->get_start(t,s))
#define bt_refgen_get_total_time(t,time) (t->type->get_total_time(t,time))
#define bt_refgen_get_num_points(t,p) (t->type->get_num_points(t,p))
#define bt_refgen_start(t) (t->type->start(t))
#define bt_refgen_eval(t,ref) (t->type->eval(t,ref))

/* Forward declaration of instance */
struct bt_refgen;

/* A type object, one for each type of refgen.
 * Here we describe the interface to a refgen, which has two parts,
 * asynchronous and synchronous.
 *
 * The synchronous interface consists solely of the
 * start() and eval() functions. */
struct bt_refgen_type
{
   char name[20];
   
   /* Destroy the refgen; self-explanatory. */
   int (*destroy)(struct bt_refgen * t);
   
   /* == Define the asynchronous interface == */
   
   /* get_start()
    *
    * Get the starting location of the refgen.
    * If this function is not defined, then the refgen may be started
    * from any point.  It is expected that the robot be moved automatically
    * to the start position before the refgen is started. */
   int (*get_start)(struct bt_refgen * t, gsl_vector ** start);
   
   /* get_total_time()
    *
    * Get the refgen's total time to completion.
    * If the refgen does not have a sense of completion (or a time estimate),
    * do not define this function. */
   int (*get_total_time)(struct bt_refgen * t, double * time);
   
   /* get_num_points()
    *
    * Get the refgen's total number of points.
    * If the refgen is not defined in terms of discrete elements,
    * do not define this function. */
   int (*get_num_points)(struct bt_refgen * t, int * points);
   
   /* == Define the synchronous interface == */
   
   /* start()
    *
    * Start the refgen. This function is called in the real-time loop
    * before the first eval(), and allows the refgen to initialize itself.
    * It is assumed that the robot is near the start position when this
    * is called.
    */
   int (*start)(struct bt_refgen * t);
   
   /* This function evaluates the refgen; that is, the refgen is expected
    * to generate a reference value for the active controller.
    * This call is time-sensitive, and should not break the real-time loop.
    *
    * Return 1 for "finished"
    */
   int (*eval)(struct bt_refgen * t, gsl_vector * ref);
};

/* A refgen instance */
struct bt_refgen {
   const struct bt_refgen_type * type;
};

#endif /* BT_REFGEN_H */
