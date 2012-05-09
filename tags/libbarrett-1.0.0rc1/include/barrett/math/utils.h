/** Math utilities and operators for \c double and descendants of
 * barrett::math::Vector.
 *
 * @file math/utils.h
 * @date Oct 26, 2009
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


#ifndef BARRETT_MATH_UTILS_H_
#define BARRETT_MATH_UTILS_H_


#include <Eigen/Core>


namespace barrett {
namespace math {


namespace detail {
template<typename Scalar> struct CwiseSignOp;
template<typename Scalar> struct CwiseUnarySaturateOp;
template<typename Scalar> struct CwiseBinarySaturateOp;
template<typename Scalar> struct CwiseUnaryDeadbandOp;
template<typename Scalar> struct CwiseBinaryDeadbandOp;
}

/** Computes the sign (positive, zero, or negative) of its input.
 *
 * Sometimes referred to as the \c signum function. The function operates
 * coefficient-wise for barrett::math::Vector inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Vector.
 * @param[in] x
 * @retval 1 if <tt> x > 0 </tt>
 * @retval 0 if <tt> x == 0 </tt>
 * @retval -1 if <tt> x < 0 </tt>
 */
template<typename Derived>
const Eigen::CwiseUnaryOp<
	detail::CwiseSignOp<typename Eigen::ei_traits<Derived>::Scalar>,
	Derived
> sign(const Eigen::MatrixBase<Derived>& x);

double sign(double x);

/** Computes the absolute value of its input.
 *
 * The function operates coefficient-wise for barrett::math::Vector inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Vector.
 * @param[in] x
 * @retval x if <tt> x >= 0 </tt>
 * @retval -x if <tt> x < 0 </tt>
 */
template<typename Derived>
const Eigen::CwiseUnaryOp<
	Eigen::ei_scalar_abs_op<typename Eigen::ei_traits<Derived>::Scalar>,
	Derived
> abs(const Eigen::MatrixBase<Derived>& x);

/** Returns the minimum of its two inputs.
 *
 * The function operates coefficient-wise for barrett::math::Vector inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Vector.
 * @param[in] a
 * @param[in] b
 * @retval a if <tt> a < b </tt>
 * @retval b otherwise.
 */
template<typename Derived1, typename Derived2>
const Eigen::CwiseBinaryOp<
	Eigen::ei_scalar_min_op<typename Eigen::ei_traits<Derived1>::Scalar>,
	Derived1,
	Derived2
> min(const Eigen::MatrixBase<Derived1>& a, const Eigen::MatrixBase<Derived2>& b);

double min(double a, double b);

/** Returns the maximum of its two inputs.
 *
 * The function operates coefficient-wise for barrett::math::Vector inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Vector.
 * @param[in] a
 * @param[in] b
 * @retval a if <tt> a > b </tt>
 * @retval b otherwise.
 */
template<typename Derived1, typename Derived2>
const Eigen::CwiseBinaryOp<
	Eigen::ei_scalar_max_op<typename Eigen::ei_traits<Derived1>::Scalar>,
	Derived1,
	Derived2
> max(const Eigen::MatrixBase<Derived1>& a, const Eigen::MatrixBase<Derived2>& b);

double max(double a, double b);

//template<typename T> T max(const T& a, const T& b);  //NOLINT: this is not the max() from <algorithm>

/** Ensures the input does not exceed the given limits.
 *
 * The function operates coefficient-wise for barrett::math::Vector inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Vector.
 * @param[in] x Input value.
 * @param[in] limit The maximum absolute value of the return value.
 * @retval x if <tt> abs(x) < limit </tt>
 * @retval sign(x)*limit otherwise.
 */
template<typename Derived>
const Eigen::CwiseUnaryOp<
	detail::CwiseUnarySaturateOp<typename Eigen::ei_traits<Derived>::Scalar>,
	Derived
> saturate(const Eigen::MatrixBase<Derived>& x, double limit);

template<typename Derived1, typename Derived2>
inline const Eigen::CwiseBinaryOp<
	detail::CwiseBinarySaturateOp<typename Eigen::ei_traits<Derived1>::Scalar>,
	Derived1,
	Derived2
> saturate(const Eigen::MatrixBase<Derived1>& x, const Eigen::MatrixBase<Derived2>& limit);

double saturate(double x, double limit);


template<typename Derived>
const Eigen::CwiseUnaryOp<
	detail::CwiseUnarySaturateOp<typename Eigen::ei_traits<Derived>::Scalar>,
	Derived
> saturate(const Eigen::MatrixBase<Derived>& x, double lowerLimit, double upperLimit);

double saturate(double x, double lowerLimit, double upperLimit);


/** Maps input values smaller than the given cutoff to zero.
 *
 * The function operates coefficient-wise for barrett::math::Vector inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Vector.
 * @param[in] x Input value.
 * @param[in] cutoff The smallest input value that produces a non-zero return
 *            value.
 * @retval x-sign(x)*cutoff if <tt> abs(x) > cutoff </tt>
 * @retval 0 otherwise.
 */
template<typename Derived>
const Eigen::CwiseUnaryOp<
	detail::CwiseUnaryDeadbandOp<typename Eigen::ei_traits<Derived>::Scalar>,
	Derived
> deadband(const Eigen::MatrixBase<Derived>& x, double cutoff);

template<typename Derived1, typename Derived2>
const Eigen::CwiseBinaryOp<
	detail::CwiseBinaryDeadbandOp<typename Eigen::ei_traits<Derived1>::Scalar>,
	Derived1,
	Derived2
> deadband(const Eigen::MatrixBase<Derived1>& x, const Eigen::MatrixBase<Derived2>& cutoff);

double deadband(double x, double cutoff);

//template<typename T> T deadBand(const T& x,
//		const T& lowerCutoff, const T& upperCutoff);


}
}


// include template definitions
#include <barrett/math/detail/utils-inl.h>


#endif /* BARRETT_MATH_UTILS_H_ */
