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

/* btkinematics uses gsl :-) */
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

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
   gsl_matrix * trans_to_inertial; /* homogeneous transform matrix */
   
   /* Vector "views" */
   gsl_matrix * rot_to_prev;
   gsl_matrix * rot_to_inertial;
   gsl_vector * origin_pos; /* Origin Position, in inertial coords */
   
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
   
   struct bt_kinematics_link * world;
   struct bt_kinematics_link ** link; /* Moving links array */
   struct bt_kinematics_link * tool;
};


struct bt_kinematics * bt_kinematics_create( config_setting_t * kinconfig, int ndofs );
int bt_kinematics_destroy( struct bt_kinematics * kin );

int bt_kinematics_eval_forward( struct bt_kinematics * kin, gsl_vector * jposition );

#endif /* BT_KINEMATICS_H */
