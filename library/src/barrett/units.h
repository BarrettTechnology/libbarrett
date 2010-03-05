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


#include <libconfig.h++>
//#include "./math/vector.h"
#include "./math/matrix.h"


/// @cond DETAIL
#define DECLARE_UNITS_IMPL_H(ClassName, N)  \
	class ClassName : public ::barrett::math::Vector<N> {  \
	public:

#define DECLARE_UNITS_IMPL_F(ClassName, N)  \
		ClassName() :  \
			::barrett::math::Vector<N>() {}  \
/*		ClassName(double x, double y) :  \
			::barrett::math::Vector<N>(x, y) {}  \
		ClassName(double x, double y, double z) :  \
			::barrett::math::Vector<N>(x, y, z) {}  \
		ClassName(double x, double y, double z, double w) :  \
			::barrett::math::Vector<N>(x, y, z, w) {}  \
*/		explicit ClassName(const double* data) :  \
			::barrett::math::Vector<N>(data) {}  \
		template<typename OtherDerived>  \
		ClassName(const Eigen::MatrixBase<OtherDerived>& other) :  \
			::barrett::math::Vector<N>(other) {}  \
		template<typename OtherDerived>  \
		explicit ClassName(const Eigen::RotationBase<OtherDerived,  \
						::barrett::math::Vector<N>::Base::ColsAtCompileTime>& r) :  \
			::barrett::math::Vector<N>(r) {}  \
		\
		explicit ClassName(double d) :  \
			::barrett::math::Vector<N>(d) {}  \
		explicit ClassName(const gsl_vector* vec) :  \
			::barrett::math::Vector<N>(vec) {}  \
		ClassName(const ::libconfig::Setting& setting) :  /* deliberately non-explicit */  \
			::barrett::math::Vector<N>(setting) {}  \
		ClassName(const ::barrett::math::Vector<N>& a) :  /* NOLINT: ctor deliberately non explicit */  \
			::barrett::math::Vector<N>(a) {}  \
		using ::barrett::math::Vector<N>::operator=;  \
	}

//
//	using ::barrett::math::Traits;
//	template<size_t N> struct Traits<ClassName<N> > :
//			public ::barrett::math::Traits< ::barrett::math::Vector<N> > {}
/// @endcond


#define DECLARE_FIXED_SIZE_UNITS(ClassName, N)  \
	DECLARE_UNITS_IMPL_H(ClassName, N)  \
	DECLARE_UNITS_IMPL_F(ClassName, N)

#define DECLARE_FIXED_SIZE_UNITS_WITH_ACTUATOR(ClassName, ActuatorType, N)  \
	DECLARE_UNITS_IMPL_H(ClassName, N)  \
		typedef ActuatorType actuator_type;  \
	DECLARE_UNITS_IMPL_F(ClassName, N)


/** Declares and defines a new barrett::units type.
 *
 * The new type is a class that descends from barrett::math::Vector. It has a
 * template parameter \c size_t \c N indicating how many \c doubles it holds.
 * It can be used as an argument to any of the arithmetic operators in vector.h
 * and any of the applicable math utilities in math/utils.h.
 *
 * The generated class is of the form:
 * \code
 * template<size_t N>
 * class ClassName : public ::barrett::math::Vector<N> {
 * public:
 * 	explicit ClassName(double d = 0.0) :
 * 		::barrett::math::Vector<N>(d) {}
 * 	explicit ClassName(const gsl_vector* vec) :
 * 		::barrett::math::Vector<N>(vec) {}
 * 	ClassName(const ::barrett::math::Vector<N>& a) :
 * 		::barrett::math::Vector<N>(a) {}
 * 	using ::barrett::math::Vector<N>::operator=;
 * };
 * \endcode
 *
 * @param ClassName The name of the new barrett::units type.
 * @see DECLARE_UNITS_WITH_ACTUATOR
 */
#define DECLARE_UNITS(ClassName)  \
	template<size_t N>  \
	DECLARE_FIXED_SIZE_UNITS(ClassName, N)

/** @copybrief DECLARE_UNITS
 *
 * This macro is identical to \ref DECLARE_UNITS, except that the generated
 * class has an additional \c public \c typedef called \c actuator_type which
 * allows other components (such as barrett::systems::Controller types) to set
 * intelligent defaults for certain template parameters.
 *
 * An \c actuator_type is a barrett::units type representing the units used to
 * communicate with your particular set of actuators. If your actuators receive
 * joint torque commands (as in the WAM), the appropriate \c actuator_type is
 * probably barrett::units::JointTorques. If you have linear actuators (or can
 * emulate them using some sort of transform, also as in the WAM), it might be
 * Cartesian forces. If you are working with a thermal system, the
 * \c actuator_type might represent heat flow. Look at the other barrett::units
 * classes for examples.
 *
 * The concept of an actuator type is helpful because it allows (in certain
 * cases) our syntax to more closely resemble the way we talk and think about a
 * problem. For example, if I said "Make me a PID controller to control joint
 * positions," I wouldn't need to tell you that the output of that controller
 * should be joint torques. So, when describing the problem to the C++
 * compiler, I shouldn't have to say:
 * @code
 * systems::PIDController<units::JointPositions, units::JointTorques> jpPID;
 * @endcode
 * I should be able to, instead, say:
 * @code
 * systems::PIDController<units::JointPositions> jpPID;
 * @endcode
 * The \c actuator_type \c typedef allows
 * \ref barrett::systems::PIDController "systems::PIDController" to look inside
 * the \ref barrett::units::JointPositions "units::JointPositions" class and
 * discover that the proper default for its \c OutputType template parameter is
 * \ref barrett::units::JointTorques "units::JointTorques". (Of course, if this
 * default behavior doesn't fit your usage, specifying the second template
 * parameter explicitly would get you a
 * \ref barrett::systems::PIDController "systems::PIDController" with whatever
 * \c OutputType you want.)
 *
 * In conclusion, if the units being described have naturally associated
 * "actuator" units, use this macro; otherwise, use \ref DECLARE_UNITS instead.
 * If in doubt, use \ref DECLARE_UNITS; the worst that will happen is
 * occasionally you'll have to type a little bit more.
 *
 * The generated class is of the form:
 * \code
 * template<size_t N>
 * class ClassName : public ::barrett::math::Vector<N> {
 * public:
 * 	typedef ActuatorType<N> actuator_type;
 *
 * 	explicit ClassName(double d = 0.0) :
 * 		::barrett::math::Vector<N>(d) {}
 * 	explicit ClassName(const gsl_vector* vec) :
 * 		::barrett::math::Vector<N>(vec) {}
 * 	ClassName(const ::barrett::math::Vector<N>& a) :
 * 		::barrett::math::Vector<N>(a) {}
 * 	using ::barrett::math::Vector<N>::operator=;
 * };
 * \endcode
 *
 * @param ClassName The name of the new barrett::units type.
 * @param ActuatorType The barrett::units type representing the associated
 *        actuator units.
 * @see DECLARE_UNITS
 */
#define DECLARE_UNITS_WITH_ACTUATOR(ClassName, ActuatorType)  \
	template<size_t N>  \
	DECLARE_FIXED_SIZE_UNITS_WITH_ACTUATOR(ClassName, ActuatorType<N>, N)


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

//DECLARE_UNITS(JointTorques);
//DECLARE_UNITS_WITH_ACTUATOR(JointPositions, JointTorques);
//DECLARE_UNITS_WITH_ACTUATOR(JointVelocities, JointTorques);
//
//DECLARE_FIXED_SIZE_UNITS(CartesianForce, 3);
//DECLARE_FIXED_SIZE_UNITS_WITH_ACTUATOR(CartesianPosition, CartesianForce, 3);


}
}


#endif /* BARRETT_UNITS_H_ */
