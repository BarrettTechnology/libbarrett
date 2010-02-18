/** <b> Implementation file: do not include.\ </b> Defines
 * barrett::units::Vector and its descendants.
 *
 * @file vector-inl.h
 * @date Oct 28, 2009
 * @author Dan Cody
 *
 * @warning
 * This file is located in a \c detail directory. It is part of the
 * implementation and should not be directly included by the user.
 * @see array.h
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


#include <stdexcept>
#include <cstring>
#include <algorithm>
#include <functional>
#include <sstream>

#include <libconfig.h++>
#include <gsl/gsl_vector.h>

#include "../../detail/libconfig_utils.h"


namespace barrett {
namespace math {


template<size_t N>
inline Vector<N>::Vector() :
	Base(), gslVector()
{
	initGslVector();
}

template<size_t N>
inline Vector<N>::Vector(double x, double y) :
	Base(x, y), gslVector()
{
	initGslVector();
}

template<size_t N>
inline Vector<N>::Vector(double x, double y, double z) :
	Base(x, y, z), gslVector()
{
	initGslVector();
}

template<size_t N>
inline Vector<N>::Vector(double x, double y, double z, double w) :
	Base(x, y, z, w), gslVector()
{
	initGslVector();
}

template<size_t N>
inline Vector<N>::Vector(const double* data) :
	Base(data), gslVector()
{
	initGslVector();
}

template<size_t N>
template<typename OtherDerived>
inline Vector<N>::Vector(const Eigen::MatrixBase<OtherDerived>& other) :
	Base(other), gslVector()
{
	initGslVector();
}

template<size_t N>
template<typename OtherDerived>
inline Vector<N>::Vector(const Eigen::RotationBase<OtherDerived,Base::ColsAtCompileTime>& r) :
	Base(r), gslVector()
{
	initGslVector();
}

//template<size_t N>
//template<typename OtherDerived>
//inline Vector& Vector<N>::operator=(const Eigen::RotationBase<OtherDerived,Base::ColsAtCompileTime>& r)
//{
//	Base::operator=(r);
//	return *this;
//}


template<size_t N>
inline Vector<N>::Vector(double d) :
	gslVector()
{
	initGslVector();
	this->setConstant(d);
}

template<size_t N>
inline Vector<N>::Vector(const gsl_vector* vec) :
	gslVector()
{
	initGslVector();
	copyFrom(vec);
}

template<size_t N>
inline Vector<N>::Vector(const libconfig::Setting& setting) :
	gslVector()
{
	initGslVector();
	copyFrom(setting);
}



template<size_t N>
inline Vector<N>::Vector(const Vector& a) :
	Base(a), gslVector(a.gslVector)
{
	initGslVector();
}

template<size_t N>
inline Vector<N>::~Vector()
{
}


template<size_t N>
inline size_t Vector<N>::serializedLength()
{
	return sizeof(double) * N;
}

template<size_t N>
inline void Vector<N>::serialize(char* dest) const
{
	std::memcpy(dest, this->data(), serializedLength());
}

template<size_t N>
inline Vector<N> Vector<N>::unserialize(char* source)
{
	Vector<N> a;
	std::memcpy(a.data(), source, serializedLength());
	return a;
}


template<size_t N>
inline void Vector<N>::copyTo(gsl_vector* vec) const
throw(std::logic_error)
{
	if (vec->size != N) {
		std::stringstream ss;
		ss << "(math::Vector<>::copyTo(gsl_vector*)): The size of the "
				"gsl_vector must match the size of the Vector. Expected size "
				<< N << ", got size " << vec->size;
		throw std::logic_error(ss.str());
	}

	for (size_t i = 0; i < N; ++i) {
		gsl_vector_set(vec, i, (*this)[i]);
	}
}

template<size_t N>
inline void Vector<N>::copyFrom(const gsl_vector* vec)
throw(std::logic_error)
{
	if (vec->size != N) {
		std::stringstream ss;
		ss << "(math::Vector<>::copyFrom(gsl_vector*)): The size of the "
				"gsl_vector must match the size of the Vector. Expected size "
				<< N << ", got size " << vec->size;
		throw std::logic_error(ss.str());
	}

	for (size_t i = 0; i < N; ++i) {
		(*this)[i] = gsl_vector_get(vec, i);
	}
}

template<size_t N>
inline void Vector<N>::copyFrom(const libconfig::Setting& setting)
{
	if (setting.getLength() != N) {
		std::stringstream ss;
		ss << "(math::Vector<>::copyFrom(libconfig::Setting)): The size of "
				"the configuration list must match the size of the Vector. "
				"Expected size " << N << ", got size " << setting.getLength() <<
				". Path: \"" << setting.getPath() << "\", Line: " <<
				setting.getSourceLine();
		throw std::runtime_error(ss.str());
	}

	for (size_t i = 0; i < N; ++i) {
		(*this)[i] = detail::numericToDouble(setting[i]);
	}
}

template<size_t N>
inline gsl_vector* Vector<N>::asGslVector()
{
	return &(this->gslVector);
}

template<size_t N>
inline const gsl_vector* Vector<N>::asGslVector() const
{
	return &(this->gslVector);
}

// init the gsl_vector representation of the Vector
template<size_t N>
inline void Vector<N>::initGslVector()
{
	gslVector.size = N;
	gslVector.stride = 1;
	gslVector.data = this->data();
	gslVector.block = 0;
	gslVector.owner = 0;
}



//namespace detail {
//
//template<size_t N, typename BinaryFunction>
//inline const Vector<N> binaryVectorTransform(const Vector<N>& a,
//		const Vector<N>& b, BinaryFunction binaryOp)
//{
//	Vector<N> result;
//	std::transform(a.begin(), a.end(), b.begin(), result.begin(), binaryOp);
//	return result;
//}
//
//}


//// coefficient-wise vector arithmetic
////template<typename LDerived, typename RDerived>
//////inline const typename Eigen::ProductReturnType<LDerived,RDerived>::Type operator* (const Eigen::MatrixBase<LDerived>& lhs, const Eigen::MatrixBase<RDerived>& rhs)
////inline const bool operator* (const Eigen::MatrixBase<LDerived>& lhs, const Eigen::MatrixBase<RDerived>& rhs)
////{
////	enum {
////		AreVectors = LDerived::IsVectorAtCompileTime && RDerived::IsVectorAtCompileTime,
////		SameSizes = EIGEN_PREDICATE_SAME_MATRIX_SIZE(LDerived,RDerived)
////	};
////
////	lhs.thisdns();
//////	EIGEN_STATIC_ASSERT(AreVectors && SameSizes, YOU_MIXED_VECTORS_OF_DIFFERENT_SIZES);
////
//////	return lhs * rhs;
////	return true;
////}
//template<size_t N>
//inline const Vector<N> operator* (const Vector<N>& lhs, const Vector<N>& rhs)
//{
//	return lhs.cwise() * rhs;
//}
//template<size_t N>
//inline const Vector<N> operator/ (const Vector<N>& lhs, const Vector<N>& rhs)
//{
//	return lhs.cwise() / rhs;
//}
//
////template<size_t N>
////inline const Vector<N> operator- (const Vector<N>& a)
////{
////	Vector<N> result;
////	for (size_t i = 0; i< N; ++i) {
////		result[i] = -a[i];
////	}
////	return result;
////}
//
//
//
//// vector-scalar arithmetic
//template<size_t N>
//inline const Vector<N> operator+ (double lhs, const Vector<N>& rhs)
//{
//	return lhs + rhs.cwise();
//}
//template<size_t N>
//inline const Vector<N> operator+ (const Vector<N>& lhs, double rhs)
//{
//	return lhs.cwise() + rhs;
//}
//template<size_t N>
//inline const Vector<N> operator- (double lhs, const Vector<N>& rhs)
//{
//	return lhs + (-rhs).cwise();
//}
//template<size_t N>
//inline const Vector<N> operator- (const Vector<N>& lhs, double rhs)
//{
//	return lhs.cwise() - rhs;
//}
//template<size_t N>
//inline const Vector<N> operator/ (double lhs, const Vector<N>& rhs)
//{
//	return lhs * rhs.cwise().inverse();
//}


template<size_t N>
std::ostream& operator<< (std::ostream& os, const Vector<N>& a) {
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
