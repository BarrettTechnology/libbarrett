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

#include <libconfig.h>
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
#define bt_refgen_has_create(type) (type->create)
#define bt_refgen_create(type,n) (type->create(n))
#define bt_refgen_destroy(r) ((r)->type->destroy(r))

#define bt_refgen_has_teach(r) ((r)->type->teach_init)
#define bt_refgen_teach_init(r) ((r)->type->teach_init(r))
#define bt_refgen_teach_flush(r) ((r)->type->teach_flush(r))
#define bt_refgen_teach_end(r) ((r)->type->teach_end(r))
#define bt_refgen_teach_start(r) ((r)->type->teach_start(r))
#define bt_refgen_teach_trigger(r,time,pos) \
                               ((r)->type->teach_trigger((r),(time),(pos)))

#define bt_refgen_has_loadsave(r) ((r)->type->load)
#define bt_refgen_load(r,s) ((r)->type->load((r),(s)))
#define bt_refgen_save(r,s) ((r)->type->save((r),(s)))

#define bt_refgen_get_start(r,s) ((r)->type->get_start((r),(s)))
#define bt_refgen_get_total_time(r,time) ((r)->type->get_total_time((r),(time)))
#define bt_refgen_get_num_points(r,p) ((r)->type->get_num_points((r),(p)))

#define bt_refgen_start(r) ((r)->type->start(r))
#define bt_refgen_eval(r,time,ref) ((r)->type->eval((r),(time),(ref)))


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

   
   struct bt_refgen * (*create)(int n);
   
   /* Destroy the refgen; self-explanatory. */
   int (*destroy)(struct bt_refgen * r);
   

   /** \name Functions for teachable refgens
    *  \{ */
   
   int (*teach_init)(struct bt_refgen * r);

   int (*teach_flush)(struct bt_refgen * r);

   int (*teach_end)(struct bt_refgen * r);

   /* called just before the first trigger,
    * CURRENTLY SYNC OR ASYNC! */
   int (*teach_start)(struct bt_refgen * r);

   /* Synchronous */
   /* For triggering teachable refgens */
   int (*teach_trigger)(struct bt_refgen * r, double time, gsl_vector * pos);

   /*  \} */
   
   
   /** \name Functions for saveable/loadable refgens
    *  \{ */

   int (*load)(struct bt_refgen * r, config_setting_t * setting);

   int (*save)(struct bt_refgen * r, config_setting_t * setting);

   /*  \} */


   
   
   /* == Define the asynchronous interface == */
   
   /* get_start()
    *
    * Get the starting location of the refgen.
    * If this function is not defined, then the refgen may be started
    * from any point.  It is expected that the robot be moved automatically
    * to the start position before the refgen is started. */
   int (*get_start)(struct bt_refgen * r, gsl_vector ** start);
   
   /* get_total_time()
    *
    * Get the refgen's total time to completion.
    * If the refgen does not have a sense of completion (or a time estimate),
    * do not define this function. */
   int (*get_total_time)(struct bt_refgen * r, double * time);
   
   /* get_num_points()
    *
    * Get the refgen's total number of points.
    * If the refgen is not defined in terms of discrete elements,
    * do not define this function. */
   int (*get_num_points)(struct bt_refgen * r, int * points);

   
   /* == Define the synchronous interface == */
   
   /* start()
    *
    * Start the refgen. This function is called in the real-time loop
    * before the first eval(), and allows the refgen to initialize itself.
    * It is assumed that the robot is near the start position when this
    * is called.
    */
   int (*start)(struct bt_refgen * r);
   
   /* This function evaluates the refgen; that is, the refgen is expected
    * to generate a reference value for the active controller.
    * This call is time-sensitive, and should not break the real-time loop.
    *
    * Return 1 for "finished"
    */
   int (*eval)(struct bt_refgen * r, double time, gsl_vector * ref);
   
   
};

/* A refgen instance */
struct bt_refgen {
   const struct bt_refgen_type * type;
};

#endif /* BT_REFGEN_H */
