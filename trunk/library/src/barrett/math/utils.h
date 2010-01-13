/** Math utilities and operators for \c double and descendants of
 * barrett::math::Array.
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


namespace barrett {
namespace math {


/** Computes the sign (positive, zero, or negative) of its input.
 *
 * Sometimes referred to as the \c signum function. The function operates
 * element-wise for barrett::math::Array inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Array.
 * @param[in] x
 * @retval 1 if <tt> x > 0 </tt>
 * @retval 0 if <tt> x == 0 </tt>
 * @retval -1 if <tt> x < 0 </tt>
 */
template<typename T> T sign(const T& x);

/** Computes the absolute value of its input.
 *
 * The function operates element-wise for barrett::math::Array inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Array.
 * @param[in] x
 * @retval x if <tt> x >= 0 </tt>
 * @retval -x if <tt> x < 0 </tt>
 */
template<typename T> T abs(const T& x);

/** Returns the minimum of its two inputs.
 *
 * The function operates element-wise for barrett::math::Array inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Array.
 * @param[in] a
 * @param[in] b
 * @retval a if <tt> a < b </tt>
 * @retval b otherwise.
 */
template<typename T> T min(const T& a, const T& b);

/** Returns the maximum of its two inputs.
 *
 * The function operates element-wise for barrett::math::Array inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Array.
 * @param[in] a
 * @param[in] b
 * @retval a if <tt> a > b </tt>
 * @retval b otherwise.
 */
template<typename T> T max(const T& a, const T& b);  //NOLINT: this is not the max() from <algorithm>

/** Ensures the input does not exceed the given limits.
 *
 * The function operates element-wise for barrett::math::Array inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Array.
 * @param[in] x Input value.
 * @param[in] limit The maximum absolute value of the return value.
 * @retval x if <tt> abs(x) < limit </tt>
 * @retval sign(x)*limit otherwise.
 */
template<typename T> T saturate(const T& x, const T& limit);
//template<typename T> T saturate(const T& x,
//		const T& lowerLimit, const T& upperLimit);

/** Maps input values smaller than the given cutoff to zero.
 *
 * The function operates element-wise for barrett::math::Array inputs.
 *
 * @tparam Designed to operate on a \c double or barrett::math::Array.
 * @param[in] x Input value.
 * @param[in] cutoff The smallest input value that produces a non-zero return
 *            value.
 * @retval x-sign(x)*cutoff if <tt> abs(x) > cutoff </tt>
 * @retval 0 otherwise.
 */
template<typename T> T deadband(const T& x, const T& cutoff);
//template<typename T> T deadBand(const T& x,
//		const T& lowerCutoff, const T& upperCutoff);


}
}


// include template definitions
#include "./detail/utils-inl.h"


#endif /* BARRETT_MATH_UTILS_H_ */
