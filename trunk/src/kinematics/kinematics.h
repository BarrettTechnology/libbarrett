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

/* btkinematics uses gsl :-) */
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/* Notes about the kinematics module, for now:
 *  - There are (ndofs) link frames attached to the (ndofs) moving links,
 *    indexed by the link[] array.
 *  - There is one base frame, attached to the base of the robot.
 *  - There is one toolplate frame, attached to the robot's end tool plate.
 *
 *  - FOR NOW, we asssume that the base frame is an inertial frame.
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
   
   gsl_matrix * trans_to_prev; /* homogeneous transform matrix */
   gsl_matrix * trans_to_base; /* homogeneous transform matrix */
   
   /* Vector "views" */
   gsl_matrix * rot_to_prev;
   gsl_matrix * rot_to_base;
   gsl_vector * axis_z; /* z axis unit vector, in base coords */
   gsl_vector * origin_pos; /* Origin Position, in base coords */
   
};

/* kinematics knows about d-h parameters,
 * endpoint positions and velocities,
 * etc. */
struct bt_kinematics {
   /*struct wam_bot * bot;*/
   /*gsl_matrix * jacobian;*/ /* Some sort of jacobian? */
   
   int dof;
   int nlinks;
   struct bt_kinematics_link ** link_array;
   
   struct bt_kinematics_link * base;
   struct bt_kinematics_link ** link; /* Moving links array */
   struct bt_kinematics_link * toolplate;
   
   /* Toolplate Jacobian */
   gsl_matrix * toolplate_jacobian;
   
   /* Temp vectors */
   gsl_vector * temp_v3;
};


struct bt_kinematics * bt_kinematics_create( config_setting_t * kinconfig, int ndofs );
int bt_kinematics_destroy( struct bt_kinematics * kin );

/* Evaluate all link transforms, including the toolplate jacobian */
int bt_kinematics_eval( struct bt_kinematics * kin, gsl_vector * jposition );

/* Evaluate the jacobian, on link jmax (ndofs for tool), at base-point point
 * NOTE: eval_forward must have already been computed!
 * NOTE: jac should be a 6xN matrix */
int bt_kinematics_eval_jacobian( struct bt_kinematics * kin,
   int jlimit, gsl_vector * point, gsl_matrix * jac);

#endif /* BT_KINEMATICS_H */
