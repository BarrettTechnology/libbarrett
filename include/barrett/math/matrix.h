/*
 * matrix.h
 *
 *  Created on: Jan 13, 2010
 *      Author: dc
 */

#ifndef BARRETT_MATH_MATRIX_H_
#define BARRETT_MATH_MATRIX_H_


#include <iostream>
#include <stdexcept>

#include <boost/type_traits/is_same.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/or.hpp>

#include <libconfig.h++>

#include <Eigen/Core>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <barrett/math/traits.h>


namespace barrett {
namespace math {


template<int R, int C, typename Units = void> class Matrix;

template<int R, typename Units = void>
struct Vector {
	typedef math::Matrix<R,1, Units> type;
};


template<int R, int C, typename Units>
class Matrix : public Eigen::Matrix<double, R,C, (C==1 && R>1) ? Eigen::ColMajor : Eigen::RowMajor> {
public:
	typedef Eigen::Matrix<double, R,C, (C==1 && R>1) ? Eigen::ColMajor : Eigen::RowMajor> Base;
	typedef Matrix<R,C, Units> type;

	typedef typename boost::mpl::if_c<
		Base::IsVectorAtCompileTime,
		gsl_vector,
		gsl_matrix
	>::type gsl_type;

	// TODO(dc): disable SIZE somehow for dynamic Matrices?
	static const size_t SIZE = R*C;  ///< Length of the array. Avoid using this if possible in case dynamic sizing is supported in the future.

	/** Used by clients of child classes to loose type info when necessary.
	 *
	 * Sometimes it is useful to cast a specific barrett::units into a generic
	 * Matrix of the appropriate size. \c unitless_type gives easy access to the
	 * the correct type.
	 */
	typedef Matrix<R,C> unitless_type;


	// Duplicate the non-inherited parts of Eigen's interface.
//	Matrix();
//	explicit Matrix(int dim);
//	Matrix(int r, int c);
	Matrix(double x, double y);
	Matrix(double x, double y, double z);
	Matrix(double x, double y, double z, double w);
	explicit Matrix(const double* data);
	template<typename OtherDerived>
	Matrix(const Eigen::MatrixBase<OtherDerived>& other);

//	EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Matrix);
	using Base::operator=;

  inline bool isVector() const {
    return this->rows() == 1 || this->cols() == 1;
  }

	// the default operator= does it wrong.
	inline Matrix& operator=(const Matrix& other) {
		if (this != &other) {
			// GSL stuff doen't need to be altered
			this->Base::operator=(other);
		}
		return *this;
	}

	// make sure units match
	template<int OtherR, int OtherC, typename OtherUnits>
	inline Matrix<R,C, Units>& operator=(const Matrix<OtherR,OtherC, OtherUnits>& other) {
		BOOST_MPL_ASSERT((	boost::mpl::or_<
								boost::is_same<Units,	void>,
								boost::is_same<void,	OtherUnits>,
								boost::is_same<Units,	OtherUnits>
							> ));

		// GSL stuff doen't need to be altered
		this->Base::operator=(other);
		return *this;
	}


	template<typename OtherDerived>
	explicit Matrix(const Eigen::RotationBase<OtherDerived,Base::ColsAtCompileTime>& r);
//	template<typename OtherDerived>
//	Matrix& operator=(const Eigen::RotationBase<OtherDerived,Base::ColsAtCompileTime>& r);

	// Additional ctors

	/** Default and initial value ctor.
	 *
	 * Initializes all coefficients of the Matrix to a given value.
	 *
	 * @param[in] d The initial value of the Matrix's coefficients.
	 */
	explicit Matrix(double d = 0.0);
	explicit Matrix(int r, double d = 0.0);
	Matrix(int r, int c, double d = 0.0);
	explicit Matrix(const gsl_type* gslType);
	Matrix(const libconfig::Setting& setting);  // deliberately non-explicit
	Matrix(const Matrix& a);  // TODO(dc): make sure units match in a copy construction
	~Matrix();

	// TODO(dc): add a ctor that doesn't initialize the data?

	// TODO(dc): How does this need to change to support dynamically sized Matrices?
	static size_t serializedLength();
	void serialize(char* dest) const;
	static Matrix<R,C, Units> unserialize(char* source);

	void copyTo(gsl_type* gslType) const;
	void copyFrom(const gsl_type* gslType);

	void copyFrom(const libconfig::Setting& setting);

	gsl_type* asGslType();
	const gsl_type* asGslType() const;

protected:
	void resizeIfDynamic(int r, int c = 1);

	void initGslType(gsl_vector* g);
	void initGslType(gsl_matrix* g);

	void resizeToMatchIfDynamic(const gsl_vector* g);
	void resizeToMatchIfDynamic(const gsl_matrix* g);

	void checkSize(const gsl_vector* g) const;
	void checkSize(const gsl_matrix* g) const;

	void copyToHelper(gsl_vector* g) const;
	void copyToHelper(gsl_matrix* g) const;

	void copyFromHelper(const gsl_vector* g);
	void copyFromHelper(const gsl_matrix* g);

	gsl_type gsl;
};


template<int R, int C, typename Units>
std::ostream& operator<< (std::ostream& os, const Matrix<R,C, Units>& a);


template<typename TraitsDerived> struct Traits<Eigen::MatrixBase<TraitsDerived> > {
	typedef Eigen::MatrixBase<TraitsDerived> MatrixBaseType;
	typedef typename MatrixBaseType::ConstantReturnType ConstantReturnType;


	static const bool IsDynamic = (MatrixBaseType::RowsAtCompileTime == Eigen::Dynamic  ||  MatrixBaseType::ColsAtCompileTime == Eigen::Dynamic);
	static const bool RequiresAlignment = !IsDynamic;


	static const ConstantReturnType zero() {
		return MatrixBaseType::Zero();
	}

	static const ConstantReturnType zero(int r) {
		return MatrixBaseType::Zero(r);
	}

	static const ConstantReturnType zero(int r, int c) {
		return MatrixBaseType::Zero(r,c);
	}

	template<typename Derived>
	static void zero(Eigen::MatrixBase<Derived>& t) {
		t.setZero();
	}

	// matrix-matrix
	template<typename LDerived, typename RDerived> static
	const Eigen::CwiseBinaryOp<
		Eigen::internal::scalar_sum_op<
			typename Eigen::internal::traits<LDerived>::Scalar>,
		const LDerived,
		const RDerived
	>
	add(const Eigen::MatrixBase<LDerived>& l, const Eigen::MatrixBase<RDerived>& r) {
		return (l.derived() + r.derived());
	}

	template<typename LDerived, typename RDerived> static
	const Eigen::CwiseBinaryOp<
		Eigen::internal::scalar_difference_op<
			typename Eigen::internal::traits<LDerived>::Scalar>,
		const LDerived,
		const RDerived
	>
	sub(const Eigen::MatrixBase<LDerived>& l, const Eigen::MatrixBase<RDerived>& r) {
		return l - r;
	}

	template<typename LDerived, typename RDerived> static
	const typename MatrixBaseType::PlainObject
	mult(const Eigen::MatrixBase<LDerived>& l, const Eigen::MatrixBase<RDerived>& r) {
    	return l.cwiseProduct(r);
	}

	template<typename LDerived, typename RDerived> static
	const Eigen::CwiseBinaryOp<
		Eigen::internal::scalar_quotient_op<typename Eigen::internal::traits<LDerived>::Scalar>,
		const LDerived,
		const RDerived
	>
	div(const Eigen::MatrixBase<LDerived>& l, const Eigen::MatrixBase<RDerived>& r) {
    	return l.cwiseQuotient(r);
	}


	// matrix-scalar
	template<typename Derived> static
	const typename MatrixBaseType::PlainObject
	add(const Eigen::MatrixBase<Derived>& l, double r) {
		return l.array() + r;
	}

	template<typename Derived> static
	const typename MatrixBaseType::PlainObject
	add(double l, const Eigen::MatrixBase<Derived>& r) {
    	return l + r.array();
	}

	template<typename Derived> static
	const typename MatrixBaseType::PlainObject
	sub(const Eigen::MatrixBase<Derived>& l, double r) {
		return l.array() - r;
	}

	template<typename Derived> static
	const typename MatrixBaseType::PlainObject
	neg(const Eigen::MatrixBase<Derived>& t) {
		return -t;
	}

	template<typename Derived> static
// TODO(dc): this method returns random (uninitialized? deallocated?) values when it has the commented-out return type. i don't know why.
//	const Eigen::CwiseUnaryOp<
//		Eigen::internal::scalar_add_op<typename Eigen::internal::traits<Eigen::CwiseUnaryOp<
//			Eigen::internal::scalar_opposite_op<typename Eigen::internal::traits<Derived>::Scalar>,
//			Derived
//		> >::Scalar>,
//		Eigen::CwiseUnaryOp<
//			Eigen::internal::scalar_opposite_op<typename Eigen::internal::traits<Derived>::Scalar>,
//			Derived
//		>
//	>
	const typename MatrixBaseType::PlainObject
	sub(double l, const Eigen::MatrixBase<Derived>& r) {
		return l - r.array();
	}

	template<typename Derived> static
	const typename MatrixBaseType::PlainObject
	mult(const Eigen::MatrixBase<Derived>& l, double r) {
		return l.array() * r;
	}

	template<typename Derived> static
	const typename MatrixBaseType::PlainObject
	mult(double l, const Eigen::MatrixBase<Derived>& r) {
		return l * r.array();
	}

	template<typename Derived> static
	const typename MatrixBaseType::PlainObject
	div(const Eigen::MatrixBase<Derived>& l, double r) {
		return l.array() / r;
	}

	template<typename Derived> static
// TODO(dc): this method returns random (uninitialized? deallocated?) values when it has the commented-out return type. i don't know why.
//	const Eigen::CwiseUnaryOp<
//		Eigen::internal::scalar_multiple_op<typename Eigen::internal::traits<Eigen::CwiseUnaryOp<
//			Eigen::internal::scalar_inverse_op<typename Eigen::internal::traits<Derived>::Scalar>,
//			Derived
//		> >::Scalar>,
//		Eigen::CwiseUnaryOp<
//			Eigen::internal::scalar_inverse_op<typename Eigen::internal::traits<Derived>::Scalar>,
//			Derived
//		>
//	>
	const typename MatrixBaseType::PlainObject
	div(double l, const Eigen::MatrixBase<Derived>& r) {
		return l * r.cwiseInverse();
	}
};


template<int R, int C, typename Units> struct Traits<Matrix<R,C, Units> > :
		public Traits<Eigen::MatrixBase<typename Matrix<R,C, Units>::Base> > {
	typedef typename Matrix<R,C, Units>::unitless_type unitless_type;
};


}
}


// include template definitions
#include <barrett/math/detail/matrix-inl.h>


#endif /* BARRETT_MATH_MATRIX_H_ */
