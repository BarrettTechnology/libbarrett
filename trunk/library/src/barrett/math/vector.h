/*
 * vector.h
 *
 *  Created on: Jan 13, 2010
 *      Author: dc
 */

// TODO(dc): this documentation needs updating

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


#ifndef BARRETT_VECTOR_H_
#define BARRETT_VECTOR_H_


#include <iostream>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Array>

#include <libconfig.h++>
#include <gsl/gsl_vector.h>

#include "./traits.h"


namespace barrett {
namespace math {


/** A fixed-size array of \c doubles.\ Parent of all barrett::units.
 *
 * This class supports explicit assignment and coefficient-wise arithmetic using
 * overloaded operators. It inherits from boost::array.
 *
 * Having a compile-time constant length encoded in the type allows the
 * compiler's type checking system to better ensure code correctness.
 *
 * @tparam N Length of the array.
 * @see barrett::units
 */
template<size_t N>
class Vector : public Eigen::Matrix<double, N,1, Eigen::RowMajorBit> {
public:
	typedef Eigen::Matrix<double, N,1, Eigen::RowMajorBit> Base;
	static const size_t SIZE = N;  ///< Length of the array.

	/** Used by clients of child classes to loose type info when necessary.
	 *
	 * Sometimes it is useful to cast a specific barrett::units into a generic
	 * Vector of the appropriate size. \c vector_type gives easy access to the
	 * the correct type.
	 */
	typedef Vector<N> vector_type;

	// Duplicate the non-inherited parts of Eigen's interface.
	Vector();
	Vector(double x, double y);
	Vector(double x, double y, double z);
	Vector(double x, double y, double z, double w);
	explicit Vector(const double* data);
	template<typename OtherDerived>
	Vector(const Eigen::MatrixBase<OtherDerived>& other);

//	EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Vector);
	using Base::operator=;

	// the default operator= does it wrong.
	inline Vector& operator=(const Vector& other) {
		if (this != &other) {
			// gslVector stuff doen't need to be altered
			this->Base::operator=(other);
		}
		return *this;
	}


	template<typename OtherDerived>
	explicit Vector(const Eigen::RotationBase<OtherDerived,Base::ColsAtCompileTime>& r);
//	template<typename OtherDerived>
//	Vector& operator=(const Eigen::RotationBase<OtherDerived,Base::ColsAtCompileTime>& r);

	// Additional ctors

	/** Default and initial value ctor.
	 *
	 * Initializes all coefficients of the Vector to a given value.
	 *
	 * @param[in] d The initial value of the Vector's coefficients.
	 */
	explicit Vector(double d);
	explicit Vector(const gsl_vector* vec);
	Vector(const libconfig::Setting& setting);  // deliberately non-explicit
	Vector(const Vector& a);
	~Vector();

	static size_t serializedLength();
	void serialize(char* dest) const;
	static Vector<N> unserialize(char* source);

	void copyTo(gsl_vector* vec) const throw(std::logic_error);
	void copyFrom(const gsl_vector* vec) throw(std::logic_error);

	void copyFrom(const libconfig::Setting& setting);

	gsl_vector* asGslVector();
	const gsl_vector* asGslVector() const;

protected:
	void initGslVector();
	gsl_vector gslVector;
};


///// @name Coefficient-wise vector arithmetic
////@{
//template<size_t N>
//const Vector<N> operator* (const Vector<N>& lhs, const Vector<N>& rhs);
//template<size_t N>
//const Vector<N> operator/ (const Vector<N>& lhs, const Vector<N>& rhs);
////@}
//
//
///// @name Vector-scalar arithmetic
////@{
//template<size_t N>
//const Vector<N> operator+ (double lhs, const Vector<N>& rhs);
//template<size_t N>
//const Vector<N> operator+ (const Vector<N>& lhs, double rhs);
//template<size_t N>
//const Vector<N> operator- (double lhs, const Vector<N>& rhs);
//template<size_t N>
//const Vector<N> operator- (const Vector<N>& lhs, double rhs);
//template<size_t N>
//const Vector<N> operator/ (double lhs, const Vector<N>& rhs);
////@}

template<size_t N>
std::ostream& operator<< (std::ostream& os, const Vector<N>& a);


template<size_t N> struct Traits<Vector<N> > {
	static Vector<N> zero() {
		return Vector<N>::Zero();
	}

	static void zero(Vector<N>& t) {
		t.setZero();
	}

	// vector-vector
	template<typename LDerived, typename RDerived> static
	const Eigen::CwiseBinaryOp<
		Eigen::ei_scalar_sum_op<
			typename Eigen::ei_traits<LDerived>::Scalar>,
		LDerived,
		RDerived
	>
	add(const Eigen::MatrixBase<LDerived>& l, const Eigen::MatrixBase<RDerived>& r) {
		return l + r;
	}

	template<typename LDerived, typename RDerived> static
	const Eigen::CwiseBinaryOp<
		Eigen::ei_scalar_difference_op<
			typename Eigen::ei_traits<LDerived>::Scalar>,
		LDerived,
		RDerived
	>
	sub(const Eigen::MatrixBase<LDerived>& l, const Eigen::MatrixBase<RDerived>& r) {
		return l - r;
	}

	template<typename LDerived, typename RDerived> static
	Eigen::CwiseBinaryOp<
		Eigen::ei_scalar_product_op<
			typename Eigen::ei_scalar_product_traits<
				typename Eigen::ei_traits<LDerived>::Scalar,
				typename Eigen::ei_traits<RDerived>::Scalar
				>::ReturnType
		>,
		LDerived,
		RDerived
	>
	mult(const Eigen::MatrixBase<LDerived>& l, const Eigen::MatrixBase<RDerived>& r) {
		return l.cwise() * r;
	}

	template<typename LDerived, typename RDerived> static
	Eigen::CwiseBinaryOp<
		Eigen::ei_scalar_quotient_op<typename Eigen::ei_traits<LDerived>::Scalar>,
		LDerived,
		RDerived
	>
	div(const Eigen::MatrixBase<LDerived>& l, const Eigen::MatrixBase<RDerived>& r) {
		return l.cwise() / r;
	}


	// vector-scalar
	template<typename Derived> static
	const Eigen::CwiseUnaryOp<
		Eigen::ei_scalar_add_op<typename Eigen::ei_traits<Derived>::Scalar>,
		Derived
	>
	add(const Eigen::MatrixBase<Derived>& l, double r) {
		return l.cwise() + r;
	}

	template<typename Derived> static
	const Eigen::CwiseUnaryOp<
		Eigen::ei_scalar_add_op<typename Eigen::ei_traits<Derived>::Scalar>,
		Derived
	>
	add(double l, const Eigen::MatrixBase<Derived>& r) {
		return l + r.cwise();
	}

	template<typename Derived> static
	const Eigen::CwiseUnaryOp<
		Eigen::ei_scalar_add_op<typename Eigen::ei_traits<Derived>::Scalar>,
		Derived
	>
	sub(const Eigen::MatrixBase<Derived>& l, double r) {
		return l.cwise() - r;
	}

	template<typename Derived> static
// TODO(dc): this method returns random (uninitialized? deallocated?) values when it has the commented-out return type. i don't know why.
//	const Eigen::CwiseUnaryOp<
//		Eigen::ei_scalar_add_op<typename Eigen::ei_traits<Eigen::CwiseUnaryOp<
//			Eigen::ei_scalar_opposite_op<typename Eigen::ei_traits<Derived>::Scalar>,
//			Derived
//		> >::Scalar>,
//		Eigen::CwiseUnaryOp<
//			Eigen::ei_scalar_opposite_op<typename Eigen::ei_traits<Derived>::Scalar>,
//			Derived
//		>
//	>
	Vector<N>
	sub(double l, const Eigen::MatrixBase<Derived>& r) {
		return l + (-r).cwise();
	}

	template<typename Derived> static
	const Eigen::CwiseUnaryOp<
		Eigen::ei_scalar_multiple_op<typename Eigen::ei_traits<Derived>::Scalar>,
		Derived
	>
	mult(const Eigen::MatrixBase<Derived>& l, double r) {
		return l * r;
	}

	template<typename Derived> static
	const Eigen::CwiseUnaryOp<
		Eigen::ei_scalar_multiple_op<typename Eigen::ei_traits<Derived>::Scalar>,
		Derived
	>
	mult(double l, const Eigen::MatrixBase<Derived>& r) {
		return l * r;
	}

	template<typename Derived> static
	const Eigen::CwiseUnaryOp<
		Eigen::ei_scalar_quotient1_op<typename Eigen::ei_traits<Derived>::Scalar>,
		Derived
	>
	div(const Eigen::MatrixBase<Derived>& l, double r) {
		return l / r;
	}

	template<typename Derived> static
// TODO(dc): this method returns random (uninitialized? deallocated?) values when it has the commented-out return type. i don't know why.
//	const Eigen::CwiseUnaryOp<
//		Eigen::ei_scalar_multiple_op<typename Eigen::ei_traits<Eigen::CwiseUnaryOp<
//			Eigen::ei_scalar_inverse_op<typename Eigen::ei_traits<Derived>::Scalar>,
//			Derived
//		> >::Scalar>,
//		Eigen::CwiseUnaryOp<
//			Eigen::ei_scalar_inverse_op<typename Eigen::ei_traits<Derived>::Scalar>,
//			Derived
//		>
//	>
	Vector<N>
	div(double l, const Eigen::MatrixBase<Derived>& r) {
		return l * r.cwise().inverse();
	}

};


}
}


// include template definitions
#include "./detail/vector-inl.h"


#endif /* BARRETT_VECTOR_H_ */
