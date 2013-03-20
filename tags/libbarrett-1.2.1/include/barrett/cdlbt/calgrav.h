/** Definition of bt_calgrav, a calibrated gravity compensation module.
 *
 * \file calgrav.h
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

/** \file calgrav.h
 *
 * \section sec_intro Introduction
 *
 * bt_calgrav is a module that implements gravity compensation for
 * single-chain revolute robotic manipulators using a set of first-moment
 * vectors calculated during a calibration process.
 *
 * \section sec_config Configuration Syntax
 *
 * The creation function, bt_calgrav_create(), takes a configuration
 * argument.  Here is an example configuration, for a 7-DOF WAM:
\verbatim
calgrav:
{
   # Example, for a 7-DOF WAM with hand
   mus = (( -0.694708,  0.000003, -0.078244 ),
          (  0.116866, -0.188539,  3.083559 ),
          (  0.175406,  0.208125,  0.097993 ),
          ( -0.146153, -0.105645,  0.899497 ),
          ( -0.002888,  0.082166,  0.032478 ),
          ( -0.006367, -0.030155,  0.149331 ),
          (  0.007856, -0.000520,  0.000888 ));
};
\endverbatim
 */

#ifndef BARRETT_CDLBT_CALGRAV_H_
#define BARRETT_CDLBT_CALGRAV_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <gsl/gsl_vector.h>
   
#include <barrett/cdlbt/kinematics.h>


/* gravity compensation
 * only needs a couple of additional
 * parameters in each link*/

/** Robot calibrated gravity data, holding the world gravity vector,
 *  along with arrays of vectors for each moving link.
 */
struct bt_calgrav
{
   /** We need to know about each link's kinematics */
   /*struct bt_kinematics * kin;*/
	size_t dof;

   /** Gravity vector in the world frame. This is initialized to 
    *  < 0, 0, -9.805 > in bt_calgrav_create(). */
   gsl_vector * world_g;
   
   gsl_vector ** g;  /**< Gravity vector in each frame */
   gsl_vector ** mu; /**< First-moment vector in each frame */
   gsl_vector ** t;  /**< My torque vector in my frame */
   gsl_vector ** pt; /**< My torque vector in previous frame */
};


/** Create a bt_calgrav object from a given configuration.
 *
 * This function creates a new calgrav object, reading the mu vectors
 * from the configuration given by gravconfig. It is also necessary to pass a
 * previously-created bt_kinematics object for kinematics information.
 *
 * \param[out] gravptr The bt_calgrav object on success, or 0 on failure
 * \param[in] gravconfig Calgrav configuration, from libconfig
 * \param[in] dof Degrees of freedom
 * \retval 0 Success 
 */
int bt_calgrav_create(struct bt_calgrav ** gravptr,
                      config_setting_t * gravconfig, size_t dof);


/** Destroy a bt_calgrav object.
 *
 * This function destroys a bt_calgrav object created by
 * bt_calgrav_create().
 *
 * \param[in] grav bt_calgrav object to destroy
 * \retval 0 Success
 */
int bt_calgrav_destroy(struct bt_calgrav * grav);


/** Evaluate first-moment gravity compensation torques.
 *
 * This function is used in a control loop to calculate the gravity
 * compensation torques of the robot, given a particular configuration
 * (calculated in the kinematics object).  The torques are stored into the
 * jtorque vector.
 *
 * \param[in] grav bt_calgrav object
 * \param[in] kin Previously-created bt_kinematics object describing robot
 * \param[out] jtorque Computed joint torque vector
 * \retval 0 Success
 */
int bt_calgrav_eval(struct bt_calgrav * grav,
        struct bt_kinematics * kin, gsl_vector * jtorque);


#ifdef __cplusplus
}
#endif
#endif /* BARRETT_CDLBT_CALGRAV_H_ */
