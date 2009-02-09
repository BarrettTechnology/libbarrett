/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... kinematics.h
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... Mar 24, 2005
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2005 Nov 07 - TH
 *      Minimal documentation in place.
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt
 *
 * ======================================================================== */

#ifndef BT_KINEMATICS_H
#define BT_KINEMATICS_H

#include <libconfig.h>

/* bt_kinematics uses gsl :-) */
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/* Notes about the kinematics module, for now:
 *
 *  - There are (ndofs) link frames attached to the (ndofs) moving links,
 *    indexed by the link[] array.
 *
 *  - There is an implicit "world" frame;
 *    this is assumed to be an inertial frame for dynamics
 *
 *  - There is one base frame, attached to the base of the robot;
 *    this is used as a base for recursion.
 *
 *  - There is one toolplate frame, attached to the robot's end tool plate;
 *    this is assumed to be rigidly attached to the last moving frame.
 */

/* Links are things with attached frames */
struct bt_kinematics_link {

   /* Doubly-linked for convenience */
   struct bt_kinematics_link * next;
   struct bt_kinematics_link * prev;

   /* Denavit-Hartenberg Parameters */
   double alpha;
   double theta;
   double a;
   double d;
   
   /* Some caches */
   double cos_alpha;
   double sin_alpha;
   
   /* The Transforms */
   gsl_matrix * trans_to_prev; /* homogeneous transform matrix */
   gsl_matrix * trans_to_world; /* homogeneous transform matrix */
   
   /* Vector "views" */
   gsl_matrix * rot_to_prev;
   gsl_vector * prev_axis_z; /* My joint's axis of rotation, in my coords */
   gsl_vector * prev_origin_pos; /* My origin position, in prev coords */
   
   gsl_matrix * rot_to_world;
   gsl_vector * axis_z; /* z axis unit vector, in base coords */
   gsl_vector * origin_pos; /* Origin Position, in base coords */   
};

/* kinematics knows about d-h parameters,
 * endpoint positions and velocities,
 * etc. */
struct bt_kinematics {
   
   int dof;
   int nlinks;
   struct bt_kinematics_link ** link_array;
   
   struct bt_kinematics_link * base;
   struct bt_kinematics_link ** link; /* Moving links array */
   struct bt_kinematics_link * toolplate;
   struct bt_kinematics_link * tool;
   
   /* Toolplate Jacobian */
   gsl_matrix * tool_jacobian;
   
   /* Temp vectors */
   gsl_vector * temp_v3;
};


struct bt_kinematics * bt_kinematics_create( config_setting_t * kinconfig, int ndofs );
int bt_kinematics_destroy( struct bt_kinematics * kin );

/* Evaluate all link transforms, including the toolplate jacobian */
int bt_kinematics_eval( struct bt_kinematics * kin, gsl_vector * jposition );

/* Evaluate the jacobian, on link jlimit (ndofs for tool), at base-point point
 * NOTE: eval_forward must have already been computed!
 * NOTE: jac should be a 6xN matrix */
int bt_kinematics_eval_jacobian( struct bt_kinematics * kin,
   int jlimit, gsl_vector * point, gsl_matrix * jac);

#endif /* BT_KINEMATICS_H */
