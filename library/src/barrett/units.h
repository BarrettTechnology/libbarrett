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


#define DECLARE_UNITS_IMPL_H(ClassName)  \
	template<size_t N>  \
	class ClassName : public Array<N> {  \
	public:

#define DECLARE_UNITS_IMPL_F(ClassName)  \
		explicit ClassName(double d = 0.0) :  \
				Array<N>(d) {}  \
		ClassName(const Array<N>& a) :  /* NOLINT: ctor deliberately non explicit */  \
				Array<N>(a) {}  \
		using Array<N>::operator=;  \
	}


#define DECLARE_UNITS(ClassName)  \
	DECLARE_UNITS_IMPL_H(ClassName)  \
	DECLARE_UNITS_IMPL_F(ClassName)

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
