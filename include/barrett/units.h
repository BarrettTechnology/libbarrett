/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/** Master header for the barrett::units module.
 *
 * @file units.h
 * @date Oct 16, 2009
 * @author Dan Cody
 * @see barrett::units
 */

/** @namespace barrett::units
 *
 * Defines "unit types" that are actually specializations of math::Vector or math::Matrix.
 *
 * These classes use type information to give meaning to what would otherwise be an anonymous math::Vector of \c doubles.
 *
 * For instance, a barrett::Wam has two outputs: one for joint positions and one for joint velocities. Though both of these might be (depending on the
 * particular WAM) 7-element arrays of \c doubles, they represent two different quantities. They are not interchangeable. They have different %units. A
 * user might want to design a joint-space position
 * controller and a joint-space velocity controller for the WAM. If the joint position output of the barrett::Wam were to be connected to the velocity
 * controller's feedback input, it would almost certainly be a programmer error. If such a program were run, the robot would not behave as intended. The
 * programmer might spend a long time chasing down the bug.
 *
 * Adding a notion of %units to our vectors (units::JointPositions::type, units::JointVelocities::type, etc.) allows us to:
 *   - be explicit about our intents for any given input or parameter
 *   - more closely mimic the mathematical rules that govern our engineering discipline
 *   - help the compiler to help us catch such errors.
 *   .
 * This comes without a runtime performance penalty.
 *
 * Users can easily create their own barrett::units classes by passing any C++ type name (such as the name of a \c class or \c struct) as the \c Units template parameter
 * to math::Vector or math::Matrix. The type \c void is the default and lifts all compile-time restrictions regarding %units.
 *
 * @see math::Vector
 * @see math::Matrix
 * @see BARRETT_UNITS_TYPEDEFS
 * @see BARRETT_UNITS_TEMPLATE_TYPEDEFS
 */


#ifndef BARRETT_UNITS_H_
#define BARRETT_UNITS_H_


/** Creates a standard set of \c typedefs in the local scope for all built-in barrett::units.
 *
 * Many classes use this macro to define internal short-hand names for the barrett::units they make frequent use of.
 *
 * \param dimension This positive integer is used in the \c typedefs below as the template parameter for those types that require one.
 *
 * The following \c typedefs are defined:
 *   - \c sqm_type The unitless square barrett::math::Matrix of the given \c dimension
 *   - \c v_type   The unitless barrett::math::Vector of the given \c dimension
 *   - \c jt_type  The barrett::units::JointTorques::type of the given \c dimension
 *   - \c jp_type  The barrett::units::JointPositions::type of the given \c dimension
 *   - \c jv_type  The barrett::units::JointVelocities::type of the given \c dimension
 *   - \c cf_type  The barrett::units::CartesianForce::type
 *   - \c cp_type  The barrett::units::CartesianPosition::type
 */
#define BARRETT_UNITS_TYPEDEFS(dimension)  \
	typedef ::barrett::math::Matrix<dimension,dimension> sqm_type;  \
	typedef ::barrett::math::Vector<dimension>::type v_type;  \
	typedef ::barrett::units::JointTorques<dimension>::type jt_type;  \
	typedef ::barrett::units::JointPositions<dimension>::type jp_type;  \
	typedef ::barrett::units::JointVelocities<dimension>::type jv_type;  \
	typedef ::barrett::units::CartesianForce::type cf_type;  \
	typedef ::barrett::units::CartesianPosition::type cp_type

/** Used in place of #BARRETT_UNITS_TYPEDEFS when \c dimension is dependent on a template parameter of the containing class.
 *
 * C++ requires the use of the \c typename keyword under these conditions.
 *
 * @see BARRETT_UNITS_TYPEDEFS
 */
#define BARRETT_UNITS_TEMPLATE_TYPEDEFS(dimension)  \
	typedef typename ::barrett::math::Matrix<dimension,dimension> sqm_type;  \
	typedef typename ::barrett::math::Vector<dimension>::type v_type;  \
	typedef typename ::barrett::units::JointTorques<dimension>::type jt_type;  \
	typedef typename ::barrett::units::JointPositions<dimension>::type jp_type;  \
	typedef typename ::barrett::units::JointVelocities<dimension>::type jv_type;  \
	typedef ::barrett::units::CartesianForce::type cf_type;  \
	typedef ::barrett::units::CartesianPosition::type cp_type


#include <libconfig.h++>
#include <barrett/math/matrix.h>


namespace barrett {
namespace units {

/** Template metafunction yielding the R-element math::Vector used to represent joint torques.\ Result available in the nested #type \c typedef.
 * @tparam R The number of rows in the resulting math::Vector
 */
template<int R> struct JointTorques {
	typedef typename math::Vector<R, JointTorques<R> >::type type;
};

/** Template metafunction yielding the R-element math::Vector used to represent joint positions.\ Result available in the nested #type \c typedef.
 * @tparam R The number of rows in the resulting math::Vector
 */
template<int R> struct JointPositions {
	typedef typename math::Vector<R, JointPositions<R> >::type type;
};

/** Template metafunction yielding the R-element math::Vector used to represent joint velocities.\ Result available in the nested #type \c typedef.
 * @tparam R The number of rows in the resulting math::Vector
 */
template<int R> struct JointVelocities {
	typedef typename math::Vector<R, JointVelocities<R> >::type type;
};


/// Template metafunction yielding the 3-element math::Vector used to represent a Cartesian force.\ Result available in the nested #type \c typedef.
struct CartesianForce {
	typedef math::Vector<3, CartesianForce>::type type;
};

/// Template metafunction yielding the 3-element math::Vector used to represent a Cartesian position.\ Result available in the nested #type \c typedef.
struct CartesianPosition {
	typedef math::Vector<3, CartesianPosition>::type type;
};


}
}


#endif /* BARRETT_UNITS_H_ */
