/** Definition of bt_kinematics, a simple forward kinematics library for
 *  single-chain revolute robots.
 *
 * \file kinematics.h
 * \author Christopher Dellin
 * \date 2008-2009
 */

/* Copyright 2008, 2009
 *           Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */

/** \file kinematics.h
 *
 * \section sec_intro Introduction
 *
 * bt_kinematics is a module that implements simple forward kinematics for
 * single-chain robot manipulators consisting entirely of revolute joints.
 * Moving links are defined using Denavit-Hartenberg parameters, as defined
 * in <em>Spong, Hutchinson, and Vidyasagar: Robot Modeling and Control,
 * 2006</em>.
 *
 * The library uses 4x4 homogeneous transform matrices to represent
 * coordinate transformations from one link's frame to the next.
 *
 * This is an example diagram of the frames associated with a 3-DOF robot.
 * The joints and links are each labeled 0-2.
 * \dot
 *    digraph modules {
 *       node [shape=box, fontname=FreeSans, fontsize=9];
 *       rankdir=LR;
 *       world -> base [style=dashed];
 *       subgraph cluster0 {
 *          base -> link1 [label="J1"];
 *          link1 -> link2 [label="J2"];
 *          link2 -> link3 [label="J3"];
 *          link3 -> toolplate [style=bold];
 *       }
 *       toolplate -> tool [style=dashed];
 *    }
 * \enddot
 *
 * Notes about the kinematics module, for now:
 *
 *  - There are (ndofs) link frames attached to the (ndofs) moving links,
 *    indexed by the link[] array.
 *
 *  - There is an implicit "world" frame;
 *    this is assumed to be an inertial frame for dynamics
 *
 *  - There is one base frame (link 0), attached to the base of the robot;
 *    this is used as a base for recursion.
 *
 *  - There is one toolplate frame, attached to the robot's end tool plate;
 *    this is assumed to be rigidly attached to the last moving frame.
 */

#ifndef BT_KINEMATICS_H
#define BT_KINEMATICS_H

#include <libconfig.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


/** Link-specific data, including geometric parameters, transform matrices,
 *  and pointers to adjacent links.  For details about the kinematics module,
 *  see bt_kinematics.
 */
struct bt_kinematics_link {

   /** \name Doubly-linked for convenience
    *  \{ */
   struct bt_kinematics_link * next; /**< The next link in the chain */
   struct bt_kinematics_link * prev; /**< The previous link in the chain */
   /** \} */

   /** \name Denavit-Hartenberg Parameters
    *  \{ */
   double alpha;
   double theta;
   double a;
   double d;
   /** \} */
   
   /** \name Cached D-H values
    *  \{ */
   double cos_alpha;
   double sin_alpha;
   /** \} */
   
   /** \name 4x4 Homogeneous Transform Matrices
    *  \{ */
   gsl_matrix * trans_to_prev; /**< transform matrix to previous frame */
   gsl_matrix * trans_to_world; /**< transform matrix to world frame */
   /** \} */
   
   /** \name Vector views into the trans_to_prev matrix
    *  \{ */
   gsl_matrix * rot_to_prev; /**< Rotation matrix to the previous frame */
   gsl_vector * prev_axis_z; /**< My joint's rotation axis, in my coords */
   gsl_vector * prev_origin_pos; /**< My origin position, in prev coords */
   /** \} */

   /** \name Vector views into the trans_to_world matrix
    *  \{ */
   gsl_matrix * rot_to_world; /**< Rotation matrix to the world frame */
   gsl_vector * axis_z; /**< z axis unit vector, in base coords */
   gsl_vector * origin_pos; /**< Origin Position, in base coords */   
   /** \} */
};


/** Robot kinematics data, holding an array of links, and a set of tool
 *  jacobians.
 */
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
   gsl_matrix * tool_jacobian_linear; /* matrix view */
   gsl_matrix * tool_jacobian_angular; /* matrix view */
   gsl_vector * tool_velocity;
   gsl_vector * tool_velocity_angular;
   
   /* Temp vector */
   gsl_vector * temp_v3;
};



struct bt_kinematics * bt_kinematics_create( config_setting_t * kinconfig, int ndofs );
int bt_kinematics_destroy( struct bt_kinematics * kin );

/* Evaluate all link transforms, including the toolplate jacobian */
int bt_kinematics_eval( struct bt_kinematics * kin, gsl_vector * jposition, gsl_vector * jvelocity );

/* Evaluate the jacobian, on link jlimit (ndofs for tool), at base-point point
 * NOTE: eval_forward must have already been computed!
 * NOTE: jac should be a 6xN matrix */
int bt_kinematics_eval_jacobian( struct bt_kinematics * kin,
   int jlimit, gsl_vector * point, gsl_matrix * jac);

#endif /* BT_KINEMATICS_H */
