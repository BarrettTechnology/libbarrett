/** Defines barrett::units::Array and its descendants.
 *
 * @file units.h
 * @date Oct 16, 2009
 * @author Dan Cody
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


// TODO(dc) finish documenting this.
/** @file units.h
 *
 * \section sec_intro Introduction
 *
 * These classes use type information to give meaning to what would otherwise be an anonymous array of \c doubles. For instance, a barrett::Wam has
 */


#ifndef UNITS_H_
#define UNITS_H_


#include <iostream>
#include <boost/array.hpp>


/// @cond DETAIL
#define DECLARE_UNITS_IMPL_H(ClassName)  \
	template<size_t N>  \
	class ClassName : public ::barrett::units::Array<N> {  \
	public:

#define DECLARE_UNITS_IMPL_F(ClassName)  \
		explicit ClassName(double d = 0.0) :  \
				::barrett::units::Array<N>(d) {}  \
		ClassName(const Array<N>& a) :  /* NOLINT: ctor deliberately non explicit */  \
				::barrett::units::Array<N>(a) {}  \
		using ::barrett::units::Array<N>::operator=;  \
	}
/// @endcond


/** Declares and defines a new barrett::units type.
 *
 * The new type is a class that descends from barrett::units::Array. It has a
 * template parameter \c size_t \c N indicating how many \c doubles it holds.
 * It can be used as an argument to any of the arithmetic operators in units.h
 * and any of the applicable math utilities in math_utils.h.
 *
 * The generated class is of the form:
 * \code
 * template<size_t N>
 * class ClassName : public ::barrett::units::Array<N> {
 * public:
 * 	explicit ClassName(double d = 0.0) :
 * 			::barrett::units::Array<N>(d) {}
 * 	ClassName(const Array<N>& a) :
 * 			::barrett::units::Array<N>(a) {}
 * 	using ::barrett::units::Array<N>::operator=;
 * }
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
 * The new type is a class that descends from barrett::units::Array. It has a
 * template parameter \c size_t \c N indicating how many \c doubles it holds.
 * It can be used as an argument to any of the arithmetic operators in units.h
 * and any of the applicable math utilities in math_utils.h.
 *
 * The class has an additional \c public \c typedef called \c actuator_type
 * which allows barrett::systems::Controller's and other components to set
 * intelligent defaults for certain template parameters. If the units being
 * described have a naturally associated actuator units, use this macro;
 * otherwise, use \ref DECLARE_UNITS instead.
 *
 * The generated class is of the form:
 * \code
 * template<size_t N>
 * class ClassName : public ::barrett::units::Array<N> {
 * public:
 * 	typedef ActuatorType<N> actuator_type;
 *
 * 	explicit ClassName(double d = 0.0) :
 * 			::barrett::units::Array<N>(d) {}
 * 	ClassName(const Array<N>& a) :
 * 			::barrett::units::Array<N>(a) {}
 * 	using ::barrett::units::Array<N>::operator=;
 * }
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


template<size_t N>
class Array : public boost::array<double, N> {
public:
	static const size_t SIZE = N;
	typedef Array<N> array_type;

	explicit Array(double d = 0.0);

	bool isZero();

	// this pair of operators enables the convenient, if somewhat magical,
	// syntax:
	//		units::Array<5> a;
	//		a << 5, 42.8, 37, -12, 1.4;
	Array& operator<< (double d);
	Array& operator, (double d);

private:
	mutable size_t explicitAssignmentIndex;
};


// element-wise vector arithmetic
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

// vector-scaler arithmetic
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

template<size_t N>
std::ostream& operator<< (std::ostream& os, const Array<N>& a);


}
}


// include template definitions
#include "./detail/units-inl.h"


#endif /* UNITS_H_ */
