/** Definition of bt_kinematics, a simple forward kinematics library for
 *  single-chain revolute robots.
 *
 * \file cdlbt/kinematics.h
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

/** \file cdlbt/kinematics.h
 *
 * \section sec_intro Introduction
 *
 * bt_kinematics is a module that implements simple forward kinematics for
 * single-chain robot manipulators consisting entirely of revolute joints.
 * Moving links are defined using Denavit-Hartenberg parameters, as defined
 * in <em>Spong, Hutchinson, and Vidyasagar: Robot Modeling and Control,
 * 2006</em> page 76.
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
 *
 * \section sec_config Configuration Syntax
 *
 * The creation function, bt_kinematics_create(), takes a configuration
 * argument.  Here is an example configuration, for a 7-DOF WAM:
\verbatim
kinematics:
{
   # For a 7-DOF WAM
   moving:
   (
      # Note: alpha_pi = alpha / pi
      { alpha_pi = -0.5; a =      0; d =      0; }, # Base Yaw
      { alpha_pi =  0.5; a =      0; d =      0; }, # Base Pitch
      { alpha_pi = -0.5; a =  0.045; d = 0.5500; }, # Twist
      { alpha_pi =  0.5; a = -0.045; d =      0; }, # Elbow
      { alpha_pi = -0.5; a =      0; d = 0.3000; }, # Wrist Yaw
      { alpha_pi =  0.5; a =      0; d =      0; }, # Wrist Pitch
      { alpha_pi = -0.5; a =      0; d = 0.0609; }  # Wrist Twist
   );
   toolplate = { alpha_pi = 0.5; theta_pi = 0; a = 0; d = 0; };
};
\endverbatim
 */

#ifndef BARRETT_CDLBT_KINEMATICS_H_
#define BARRETT_CDLBT_KINEMATICS_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <libconfig.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


/** Link-specific data, including geometric parameters, transform matrices,
 *  and pointers to adjacent links.  For details about the kinematics module,
 *  see bt_kinematics.
 */
struct bt_kinematics_link
{
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
struct bt_kinematics
{
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


/** Create a bt_kinematics object from a given configuration.
 *
 * This function creates a new kinematics object, reading the DH parameters
 * and toolplate information from the configuration given by kinconfig.
 * Currently, the number of moving links (ndofs) is also passed, and the
 * creating fails if the number of moving links read from the configuration
 * file does not match the expected number.
 *
 * \param[out] kinptr The bt_kinematics object on success, or 0 on failure
 * \param[in] kinconfig Kinematics configuration, from libconfig
 * \param[in] ndofs Expected number of moving links
 * \retval 0 Success
 */
int bt_kinematics_create(struct bt_kinematics ** kinptr,
                         config_setting_t * kinconfig, int ndofs);


/** Destroy a bt_kinematics object.
 *
 * This function destroys a bt_kinematics object created by
 * bt_kinematics_create().
 *
 * \param[in] kin bt_kinematics object to destroy
 * \retval 0 Success
 */
int bt_kinematics_destroy(struct bt_kinematics * kin);


/** Evaluate all link transforms, including the toolplate jacobian.
 *
 * This function is used in a control loop to update all link transform
 * matrices (and associated matrix and vector views) given a vector
 * of joint positions and velocities.
 *
 * \param[in] kin bt_kinematics object
 * \param[in] jposition Joint position vector
 * \param[in] jvelocity Joint velocity vector
 * \retval 0 Success
 */
int bt_kinematics_eval(struct bt_kinematics * kin, const gsl_vector * jposition,
                       const gsl_vector * jvelocity);


/** Evalulate the Jacobian matrix at a paticular point on a particular link.
 *
 * This function evaluates the Jacobian matrix for a given point on a given
 * moving link.
 *
 * \param[in] kin bt_kinematics object
 * \param[in] jlimit The number of the moving link on which the point lies,
 *                   (ndofs for the toolplate)
 * \param[in] point The point at which the Jacobian should be calculate, in
 *                  world coordinates
 * \param[out] jac The Jacobian matrix into which to calculate; it should be
 *                 an alread-allocated 6-N matrix.
 * \retval 0 Success
 */
int bt_kinematics_eval_jacobian(struct bt_kinematics * kin, int jlimit,
                                gsl_vector * point, gsl_matrix * jac);

#ifdef __cplusplus
}
#endif
#endif /* BARRETT_CDLBT_KINEMATICS_H_ */
