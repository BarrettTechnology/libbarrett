/** Defines barrett::units::Array and its descendants.
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


/** @namespace barrett::units
 *
 * Contains barrett::units::Array and its unit-full descendants.
 *
 * These classes use type information to give meaning to what would otherwise be an anonymous array of \c doubles.
 *
 * For instance, a barrett::Wam has two outputs: one for joint positions and one for joint velocities. Though both of these might be (depending on the
 * particular WAM) 7-element arrays of doubles,
 * they represent two different quantities. They are not interchangeable. They have different %units. A user might want to design a joint-space position
 * controller and a joint-space velocity controller for the WAM. If the joint position output of the barrett::Wam were to be connected to the velocity
 * controller's feedback input, it would almost certainly be a programmer error. If such a program were run, the robot would not behave as intended. The
 * programmer might spend a long time chasing down the bug.
 *
 * Adding thin type-wrappers around our arrays (units::JointPositions, units::JointVelocities, etc.) allows us to:
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


// TODO(dc): is there somewhere more central that this can go?
/** @file array.hpp
 *
 * Included from the Boost C++ library (http://www.boost.org) for completeness.
 *
 * @see http://www.boost.org/doc/libs/1_41_0/doc/html/array.html
 */

/** @class boost::array
 *
 * @copybrief array.hpp
 * @copydetails array.hpp
 */


#ifndef BARRETT_UNITS_H_
#define BARRETT_UNITS_H_


#include <iostream>
#include <stdexcept>

#include <boost/array.hpp>
#include <gsl/gsl_vector.h>


/// @cond DETAIL
#define DECLARE_UNITS_IMPL_H(ClassName)  \
	template<size_t N>  \
	class ClassName : public ::barrett::units::Array<N> {  \
	public:

#define DECLARE_UNITS_IMPL_F(ClassName)  \
		explicit ClassName(double d = 0.0) :  \
			::barrett::units::Array<N>(d) {}  \
		explicit ClassName(const gsl_vector* vec) :  \
			::barrett::units::Array<N>(vec) {}  \
		ClassName(const ::barrett::units::Array<N>& a) :  /* NOLINT: ctor deliberately non explicit */  \
			::barrett::units::Array<N>(a) {}  \
		using ::barrett::units::Array<N>::operator=;  \
	}
/// @endcond


/** Declares and defines a new barrett::units type.
 *
 * The new type is a class that descends from barrett::units::Array. It has a
 * template parameter \c size_t \c N indicating how many \c doubles it holds.
 * It can be used as an argument to any of the arithmetic operators in units.h
 * and any of the applicable math utilities in math/utils.h.
 *
 * The generated class is of the form:
 * \code
 * template<size_t N>
 * class ClassName : public ::barrett::units::Array<N> {
 * public:
 * 	explicit ClassName(double d = 0.0) :
 * 		::barrett::units::Array<N>(d) {}
 * 	explicit ClassName(const gsl_vector* vec) :
 * 		::barrett::units::Array<N>(vec) {}
 * 	ClassName(const ::barrett::units::Array<N>& a) :
 * 		::barrett::units::Array<N>(a) {}
 * 	using ::barrett::units::Array<N>::operator=;
 * };
 * \endcode
 *
 * @param ClassName The name of the new barrett::units type.
 * @see DECLARE_UNITS_WITH_ACTUATOR
 */
#define DECLARE_UNITS(ClassName)  \
	DECLARE_UNITS_IMPL_H(ClassName)  \
	DECLARE_UNITS_IMPL_F(ClassName)

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
 * class ClassName : public ::barrett::units::Array<N> {
 * public:
 * 	typedef ActuatorType<N> actuator_type;
 *
 * 	explicit ClassName(double d = 0.0) :
 * 		::barrett::units::Array<N>(d) {}
 * 	explicit ClassName(const gsl_vector* vec) :
 * 		::barrett::units::Array<N>(vec) {}
 * 	ClassName(const ::barrett::units::Array<N>& a) :
 * 		::barrett::units::Array<N>(a) {}
 * 	using ::barrett::units::Array<N>::operator=;
 * };
 * \endcode
 *
 * @param ClassName The name of the new barrett::units type.
 * @param ActuatorType The barrett::units type representing the associated
 *        actuator units.
 * @see DECLARE_UNITS
 */
#define DECLARE_UNITS_WITH_ACTUATOR(ClassName, ActuatorType)  \
	DECLARE_UNITS_IMPL_H(ClassName)  \
		typedef ActuatorType<N> actuator_type;  \
	DECLARE_UNITS_IMPL_F(ClassName)


namespace barrett {
namespace units {


template<size_t N> class Array;

DECLARE_UNITS(JointTorques);
DECLARE_UNITS_WITH_ACTUATOR(JointPositions, JointTorques);
DECLARE_UNITS_WITH_ACTUATOR(JointVelocities, JointTorques);


/** A fixed-size array of \c doubles.\ Parent of all barrett::units.
 *
 * This class supports explicit assignment and element-wise arithmetic using
 * overloaded operators. It inherits from boost::array.
 *
 * Having a compile-time constant length encoded in the type allows the
 * compiler's type checking system to better ensure code correctness.
 *
 * @tparam N Length of the array.
 * @see barrett::units
 */
template<size_t N>
class Array : public boost::array<double, N> {
public:
	static const size_t SIZE = N;  ///< Length of the array.

	/** Used by clients of child classes to loose type info when necessary.
	 *
	 * Sometimes it is useful to cast a specific barrett::units into a generic
	 * Array of the appropriate size. \c array_type gives easy access to the
	 * the correct type.
	 */
	typedef Array<N> array_type;


	/** Default and initial value ctor.
	 *
	 * Initializes all elements of the Array to a given value.
	 *
	 * @param[in] d The initial value of the Array's elements.
	 */
	explicit Array(double d = 0.0);
	explicit Array(const gsl_vector* vec);
	Array(const Array& a);
	~Array();


	void copyTo(gsl_vector* vec) const throw(std::logic_error);
	void copyFrom(const gsl_vector* vec) throw(std::logic_error);

	gsl_vector* asGslVector();
	const gsl_vector* asGslVector() const;


	/** Tests for equality with the zero vector.
	 *
	 * @retval true if all Array elements are equal to 0.0
	 * @retval false otherwise.
	 */
	bool isZero() const;


	/// @name Explicit assignment
	//@{

	/** Enables explicit assignment to the Array.
	 *
	 * The operator<<() and operator,() pair allow the convenient (if somewhat
	 * magical) syntax:
	 * \code
	 * barrett::units::Array<5> a;
	 * a << 5, 42.8, 37, -12, 1.4;
	 * \endcode
	 *
	 * If fewer elements are given than the Array can hold, the remaining
	 * elements are assigned a value of 0.
	 *
	 * @param[in] d The value of the first element of the Array.
	 * @throws std::out_of_range if too many elements are given.
	 * @see operator,()
	 */
	Array& operator<< (double d);

	/** @copybrief operator<<()
	 *
	 * The other half of the explicit assignment mechanism.
	 *
	 * @param[in] d The value of the <tt>n</tt>th element of the Array.
	 * @throws std::out_of_range if too many elements are given.
	 * @see operator<<()
	 */
	Array& operator, (double d);

	//@}

protected:
	void initGslVector();
	gsl_vector gslVector;

private:
	mutable size_t explicitAssignmentIndex;
};


/// @name Element-wise vector arithmetic
//@{
template<size_t N>
const Array<N> operator+ (const Array<N>& lhs, const Array<N>& rhs);
template<size_t N>
const Array<N> operator- (const Array<N>& lhs, const Array<N>& rhs);
template<size_t N>
const Array<N> operator* (const Array<N>& lhs, const Array<N>& rhs);
template<size_t N>
const Array<N> operator/ (const Array<N>& lhs, const Array<N>& rhs);

template<size_t N>
const Array<N> operator- (const Array<N>& a);
//@}


/// @name Vector-scaler arithmetic
//@{
template<size_t N>
const Array<N> operator+ (double lhs, const Array<N>& rhs);
template<size_t N>
const Array<N> operator+ (const Array<N>& lhs, double rhs);
template<size_t N>
const Array<N> operator- (double lhs, const Array<N>& rhs);
template<size_t N>
const Array<N> operator- (const Array<N>& lhs, double rhs);
template<size_t N>
const Array<N> operator* (double lhs, const Array<N>& rhs);
template<size_t N>
const Array<N> operator* (const Array<N>& lhs, double rhs);
template<size_t N>
const Array<N> operator/ (double lhs, const Array<N>& rhs);
template<size_t N>
const Array<N> operator/ (const Array<N>& lhs, double rhs);
//@}

template<size_t N>
std::ostream& operator<< (std::ostream& os, const Array<N>& a);


}
}


// include template definitions
#include "./detail/units-inl.h"


#endif /* BARRETT_UNITS_H_ */
