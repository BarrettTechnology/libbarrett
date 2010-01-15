/*
 * array.h
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


#ifndef ARRAY_H_
#define ARRAY_H_


#include <iostream>
#include <stdexcept>

#include <boost/array.hpp>
#include <gsl/gsl_vector.h>


namespace barrett {
namespace math {


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
	Array<N>& operator= (const Array<N>& a);
	~Array();

	static size_t serializedLength();
	void serialize(char* dest) const;
	static Array<N> unserialize(char* source);

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
#include "./detail/array-inl.h"


#endif /* ARRAY_H_ */
