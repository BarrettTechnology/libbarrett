/** Defines the unit-full descendants of barrett::math::Vector.
 *
 * @file units.h
 * @date Oct 16, 2009
 * @author Dan Cody
 * @see barrett::units
 */

/* Copyright 2009 Barrett Technology <support@barrett.com> */

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


// TODO(dc): this documentation needs updating
/** @namespace barrett::units
 *
 * Contains the unit-full descendants of barrett::math::Vector.
 *
 * These classes use type information to give meaning to what would otherwise be an anonymous Vector of \c doubles.
 *
 * For instance, a barrett::Wam has two outputs: one for joint positions and one for joint velocities. Though both of these might be (depending on the
 * particular WAM) 7-element Vectors of doubles,
 * they represent two different quantities. They are not interchangeable. They have different %units. A user might want to design a joint-space position
 * controller and a joint-space velocity controller for the WAM. If the joint position output of the barrett::Wam were to be connected to the velocity
 * controller's feedback input, it would almost certainly be a programmer error. If such a program were run, the robot would not behave as intended. The
 * programmer might spend a long time chasing down the bug.
 *
 * Adding thin type-wrappers around our Vectors (units::JointPositions, units::JointVelocities, etc.) allows us to:
 *   - be explicit about our intents for any given input or parameter
 *   - more closely mimic the mathematical rules that govern our engineering discipline
 *   - help the compiler to help us catch such errors.
 *   .
 * This comes without a runtime performance penalty.
 *
 * Users can easily create their own barrett::units classes in the proper form using the \ref DECLARE_UNITS and \ref DECLARE_UNITS_WITH_ACTUATOR macros.
 *
 * @see DECLARE_UNITS
 * @see DECLARE_UNITS_WITH_ACTUATOR
 */


#ifndef BARRETT_UNITS_H_
#define BARRETT_UNITS_H_


#define BARRETT_UNITS_TYPEDEFS  \
typedef ::barrett::units::jt_type jt_type;  \
typedef ::barrett::units::jp_type jp_type;  \
typedef ::barrett::units::jv_type jv_type;  \
typedef ::barrett::units::cf_type cf_type;  \
typedef ::barrett::units::cp_type cp_type


#include <libconfig.h++>
#include "./math/matrix.h"


namespace barrett {
namespace units {


template<int R> struct JointTorques {
	typedef typename math::Vector<R, JointTorques<R> >::type type;
};
template<int R> struct JointPositions {
	typedef typename math::Vector<R, JointPositions<R> >::type type;
};
template<int R> struct JointVelocities {
	typedef typename math::Vector<R, JointVelocities<R> >::type type;
};


struct CartesianForce {
	typedef math::Vector<3, CartesianForce>::type type;
};
struct CartesianPosition {
	typedef math::Vector<3, CartesianPosition>::type type;
};


typedef JointTorques<Eigen::Dynamic>::type jt_type;
typedef JointPositions<Eigen::Dynamic>::type jp_type;
typedef JointVelocities<Eigen::Dynamic>::type jv_type;

typedef CartesianForce::type cf_type;
typedef CartesianPosition::type cp_type;

//namespace typedefs {
//
//typedef JointTorques<Eigen::Dynamic>::type jt_type;
//typedef JointPositions<Eigen::Dynamic>::type jp_type;
//typedef JointVelocities<Eigen::Dynamic>::type jv_type;
//
//typedef CartesianForce::type cf_type;
//typedef CartesianPosition::type cp_type;
//
//}
//using namespace typedefs;


}
}


#endif /* BARRETT_UNITS_H_ */
