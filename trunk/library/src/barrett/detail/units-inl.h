/*
 * units-inl.h
 *
 *  Created on: Oct 28, 2009
 *      Author: dc
 */


#include <algorithm>
#include <functional>


namespace barrett {
namespace units {


template<size_t N>
inline Array<N>::Array(double d) :
	explicitAssignmentIndex()
{
	this->assign(d);
}

template<size_t N>
inline bool Array<N>::isZero()
{
	for (size_t i = 0; i < N; ++i) {
		if (this->operator[](i) != 0.0) {
			return false;
		}
	}

	return true;
}

// this pair of operators enables the convenient, if somewhat magical,
// syntax:
//		units::Array<5> a;
//		a << 5, 42.8, 37, -12, 1.4;
template<size_t N>
inline Array<N>& Array<N>::operator<< (double d)
{
	explicitAssignmentIndex = 0;

	// in case fewer than N elements are specified, put the Array in a
	// consistent state
	this->assign(0.0);

	// use at() for range checking
	this->at(explicitAssignmentIndex) = d;
	return *this;
}

template<size_t N>
inline Array<N>& Array<N>::operator, (double d)
{
	// use at() for range checking
	this->at(++explicitAssignmentIndex) = d;
	return *this;
}


template<size_t N, typename BinaryFunction>
inline const Array<N> binaryArrayTransform(const Array<N>& a,
		const Array<N>& b, BinaryFunction binaryOp)
{
	Array<N> result;
	std::transform(a.begin(), a.end(), b.begin(), result.begin(), binaryOp);
	return result;
}

// element-wise vector arithmetic
template<size_t N>
inline const Array<N> operator+ (const Array<N>& lhs, const Array<N>& rhs)
{
	return binaryArrayTransform(lhs, rhs, std::plus<double>());
}
template<size_t N>
inline const Array<N> operator- (const Array<N>& lhs, const Array<N>& rhs)
{
	return binaryArrayTransform(lhs, rhs, std::minus<double>());
}
template<size_t N>
inline const Array<N> operator* (const Array<N>& lhs, const Array<N>& rhs)
{
	return binaryArrayTransform(lhs, rhs, std::multiplies<double>());
}
template<size_t N>
inline const Array<N> operator/ (const Array<N>& lhs, const Array<N>& rhs)
{
	return binaryArrayTransform(lhs, rhs, std::divides<double>());
}

template<size_t N>
inline const Array<N> operator- (const Array<N>& a)
{
	Array<N> result;
	for (size_t i = 0; i< N; ++i) {
		result[i] = -a[i];
	}
	return result;
}



// vector-scaler arithmetic
template<size_t N>
inline const Array<N> operator+ (double lhs, const Array<N>& rhs)
{
	return Array<N>(lhs) + rhs;
}
template<size_t N>
inline const Array<N> operator+ (const Array<N>& lhs, double rhs)
{
	return lhs + Array<N>(rhs);
}
template<size_t N>
inline const Array<N> operator- (double lhs, const Array<N>& rhs)
{
	return Array<N>(lhs) - rhs;
}
template<size_t N>
inline const Array<N> operator- (const Array<N>& lhs, double rhs)
{
	return lhs - Array<N>(rhs);
}
template<size_t N>
inline const Array<N> operator* (double lhs, const Array<N>& rhs)
{
	return Array<N>(lhs) * rhs;
}
template<size_t N>
inline const Array<N> operator* (const Array<N>& lhs, double rhs)
{
	return lhs * Array<N>(rhs);
}
template<size_t N>
inline const Array<N> operator/ (double lhs, const Array<N>& rhs)
{
	return Array<N>(lhs) / rhs;
}
template<size_t N>
inline const Array<N> operator/ (const Array<N>& lhs, double rhs)
{
	return lhs / Array<N>(rhs);
}


template<size_t N>
std::ostream& operator<< (std::ostream& os, const Array<N>& a) {
	os << "[";

	for (size_t i = 0; i< N; ++i) {
		os << a[i];
		if (i != N-1) {
			os << ", ";
		}
	}

	os << "]";
	return os;
}


}
}
