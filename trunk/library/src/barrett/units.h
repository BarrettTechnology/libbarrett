/*
 * units.h
 *
 *  Created on: Oct 16, 2009
 *      Author: dc
 */

#ifndef UNITS_H_
#define UNITS_H_


#include <iostream>
#include <boost/array.hpp>


#define DECLARE_UNITS(ClassName)  \
template<size_t N>  \
class ClassName : public Array<N> {  \
public:  \
	explicit ClassName(double d = 0.0) :  \
			Array<N>(d) {}  \
	ClassName(const Array<N>& a) :  \
			Array<N>(a) {}  \
	using Array<N>::operator=;  \
}

#define DECLARE_UNITS_WITH_ACTUATOR(ClassName, ActuatorType)  \
template<size_t N>  \
class ClassName : public Array<N> {  \
public:  \
	typedef ActuatorType<N> actuator_type;  \
	explicit ClassName(double d = 0.0) :  \
			Array<N>(d) {}  \
	ClassName(const Array<N>& a) :  \
			Array<N>(a) {}  \
	using Array<N>::operator=;  \
}


namespace barrett {
namespace units {


template<size_t N> class Array;

DECLARE_UNITS(JointTorques);
DECLARE_UNITS_WITH_ACTUATOR(JointAngles, JointTorques);


template<size_t N>
class Array : public boost::array<double, N> {
public:
	static const size_t SIZE = N;

	explicit Array(double d = 0.0);

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
