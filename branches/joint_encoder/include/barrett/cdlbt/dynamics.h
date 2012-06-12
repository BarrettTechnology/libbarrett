/** Definition of bt_dynamics, a simple dynamics library which uses
 *  the Recursive Newton-Euler Algorithm for single-chain revolute robots.
 *
 * \file dynamics.h
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

/** \file dynamics.h
 *
 * \section sec_intro Introduction
 *
 * bt_dynamics is a simple module that uses the Recusive Newton-Euler
 * Algorithm (RNEA) to calculate the inverse dynamics of a single-chain
 * revolute robot defined with the bt_kinematics kinematics library.
 * See the notes on that library, located in kinematics.h, for details
 * about the set-up of the world, base, and moving links.
 *
 * \section sec_config Configuration Syntax
 *
 * The creation function, bt_dynamics_create(), takes a configuration
 * argument.  Here is an example configuration, for a 4-DOF WAM:
\verbatim
dynamics:
{
   moving:
   (
      {
         # Link 1
         mass = 10.7677;
         com = ( -4.43e-3, 121.89e-3, -0.66e-3 );
         I = (( 134880.33e-6,  -2130.41e-6,  -124.85e-6 ),
              (  -2130.41e-6, 113283.69e-6,   685.55e-6 ),
              (   -124.85e-6,    685.55e-6, 90463.30e-6 ));
      },
      {
         # Link 2
         mass = 3.8749;
         com = ( -2.37e-3, 31.06e-3, 15.42e-3 );
         I = (( 21409.58e-6,   271.72e-6,    24.61e-6 ),
              (   271.72e-6, 13778.75e-6, -1819.20e-6 ),
              (    24.61e-6, -1819.20e-6, 15589.06e-6 ));
      },
      {
         # Link 3
         mass = 1.8023;
         com = ( -38.26e-3, 207.51e-3, 0.03e-3 );
         I = (( 59110.77e-6, -2496.12e-6,     7.38e-6 ),
              ( -2496.12e-6,  3245.50e-6,   -17.67e-6 ),
              (     7.38e-6,   -17.67e-6, 59270.43e-6 ));
      },
      {
         # Link 4
         mass = 1.0651;
         com = ( 10.95e-3, -0.03e-3, 140.54e-3 );
         I = (( 18485.77e-6,     2.19e-6, -1608.68e-6 ),
              (     2.19e-6, 18916.58e-6,     5.15e-6 ),
              ( -1608.68e-6,     5.15e-6,  1975.17e-6 ));
      }

      # Tool?
   );
};
\endverbatim
 */

#ifndef BARRETT_CDLBT_DYNAMICS_H_
#define BARRETT_CDLBT_DYNAMICS_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <libconfig.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <barrett/cdlbt/kinematics.h>


/** Link-specific data, including mass and inertial parameters,
 *  the center-of-mass, and vectors/matrices to hold the results of
 *  dynamics calculations.  For an overview of the dynamics module,
 *  see dynamics.h.
 */
struct bt_dynamics_link
{
   /** \name Doubly-linked for convenience
    *  \{ */
   struct bt_dynamics_link * next;
   struct bt_dynamics_link * prev;
   /** \} */

   /** \name Geometrical and inertial parameters
    *  \{ */
   double mass;      /**< Mass of link (kg) */
   gsl_vector * com; /**< Center of mass of link (m), in DH link frame */
   gsl_matrix * I;   /**< Inertia matrix of link,
                      *   around link center-of-mass */
   /** \} */
   
   /* NOTE:
    * Do we need rotor inertia here?? */
   
   /* Next, a place to hold the results from calculations */
   
   /** \name Forward-calculated vectors (in local link frame)
    *  \{ */
   gsl_vector * omega;      /**< angular velocity of local frame
                             *   w.r.t. base frame */
   gsl_vector * omega_prev; /**< Previous frame's ang vel in my frame */
   gsl_vector * alpha;      /**< angular acceleration of local frame
                             *   w.r.t. base frame */
   gsl_vector * a;          /**< linear acceleration of frame origin */
   /** \} */
   
   /* A couple of caches, also expressed in local link frame */

   /** \name Backward-calculated vectors (in local link frame)
    *  \{ */
   gsl_vector * fnet;
   gsl_vector * tnet;
   gsl_vector * f;      /**< force exerted on this link by previous link */
   gsl_vector * f_next; /**< Next frame's force in my frame */
   gsl_vector * t;      /**< torque exerted on this link by previous link */
   /** \} */
   
   /** \name Jacobian computed at the center-of-mass (used for JSIM calc)
    *  \{ */
   gsl_matrix * com_jacobian;
   gsl_matrix * com_jacobian_linear;  /**< Matrix view of linear jacobian
                                       *   (upper half) */
   gsl_matrix * com_jacobian_angular; /**< Matrix view of angular jacobian
                                       *   (lower half) */
   /** \} */
   
};


/** Robot dynamics data, holding an array of links (mirroring the structure
 *  of the bt_kinematics module described in kinematics.h), along with
 *  a place to calculate the JSIM and some temporary vectors and matrices.
 */
struct bt_dynamics
{
   int dof;
   int nlinks;
   struct bt_dynamics_link ** link_array;
   
   struct bt_dynamics_link * base;
   struct bt_dynamics_link ** link; /* Moving links array */
   struct bt_dynamics_link * toolplate;
   
   /* Yay JSIM! */
   gsl_matrix * jsim;
   
   /* Temporary Vectors */
   gsl_vector * temp1_v3;
   gsl_vector * temp2_v3;
   
   /* Temporary Matrices */
   gsl_matrix * temp3x3_1;
   gsl_matrix * temp3x3_2;
   gsl_matrix * temp3xn_1;
};


/** Create a bt_dynamics object from a given configuration.
 *
 * This function creates a new dynamics object, center-of-mass and inertial
 * information from the configuration given by dynconfig.
 * Currently, the number of moving links (ndofs) is also passed, and the
 * creating fails if the number of moving links read from the configuration
 * file does not match the expected number.  It is also necessary to pass a
 * previously-created bt_kinematics object for kinematics information.
 *
 * \param[out] dynptr The bt_dynamics object on success, or 0 on failure
 * \param[in] dynconfig Dynamics configuration, from libconfig
 * \param[in] ndofs Expected number of moving links
 * \param[in] kin Previously-created bt_kinematics object describing robot
 * \retval 0 Success
 */
int bt_dynamics_create(struct bt_dynamics ** dynptr,
                       config_setting_t * dynconfig, int ndofs);


/** Destroy a bt_dynamics object.
 *
 * This function destroys a bt_dynamics object created by
 * bt_dynamics_create().
 *
 * \param[in] dyn bt_dynamics object to destroy
 * \retval 0 Success
 */
int bt_dynamics_destroy(struct bt_dynamics * dyn);


/** Evaluate inverse dynamics using the RNEA, both forward and backward.
 *
 * This function is used in a control loop to calculate the inverse dynamics
 * of the robot, given a particular configuration (calculated in the
 * kinematics object), set of joint velocities, and set of desired joint
 * accelerations.  The resulting joint torques are stored in jtor.
 *
 * \note Throw a switch in there for gravity on/off?
 * \note How to account for base acceleration (even gravity?)
 * \note This takes ~ 152us on PC104 right now.
 *
 * \param[in] dyn bt_dynamics object
 * \param[in] jvel Present joint velocity vector
 * \param[in] jacc Desired joint acceleration vector
 * \param[out] jtor Computed joint torque vector
 * \retval 0 Success
 */
int bt_dynamics_eval_inverse(struct bt_dynamics * dyn,
                             const struct bt_kinematics * kin,
                             const gsl_vector * jvel, const gsl_vector * jacc,
                             gsl_vector * jtor);


/** Calculate the Joint-Space Inertia Matrix (JSIM).
 *
 * The formulation used is the explicit formulation given in <em>Spong,
 * Hutchinson, and Vidyasagar: Robot Modeling and Control, 2006</em>
 * page 254.
 *
 * \param[in] dyn bt_dynamics object
 */
int bt_dynamics_eval_jsim(struct bt_dynamics * dyn, struct bt_kinematics * kin);


#ifdef __cplusplus
}
#endif
#endif /* BARRETT_CDLBT_DYNAMICS_H_ */
