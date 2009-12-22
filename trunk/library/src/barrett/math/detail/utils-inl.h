/** <b> Implementation file: do not include.\ </b> Math utilities and operators
 * for \c double and descendants of barrett::units::Array.
 *
 * @file math/utils-inl.h
 * @date Nov 11, 2009
 * @author Dan Cody
 *
 * @warning
 * This file is located in a \c detail directory. It is part of the
 * implementation and should not be directly included by the user.
 * @see math/utils.h
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


#include <algorithm>
#include <cmath>


namespace barrett {
namespace math {
using std::abs;


template<typename T>
inline T sign(const T& x)
{
	T res;
	for (size_t i = 0; i < T::SIZE; ++i) {
		res[i] = sign(x[i]);
	}
	return res;
}

/// @cond SPECIALIZATIONS
template<>
inline double sign(const double& x)
{
	if (x > 0.0) {
		return 1.0;
	} else if (x == 0.0) {
		return 0.0;
	} else {
		return -1.0;
	}
}
/// @endcond


template<typename T>
inline T abs(const T& x)
{
	T res;
	for (size_t i = 0; i < T::SIZE; ++i) {
		res[i] = abs(x[i]);
	}
	return res;
}


template<typename T>
inline T min(const T& a, const T& b)
{
	T res;
	for (size_t i = 0; i < T::SIZE; ++i) {
		res[i] = min(a[i], b[i]);
	}
	return res;
}

/// @cond SPECIALIZATIONS
template<>
inline double min(const double& a, const double& b)
{
	return std::min(a, b);
}
/// @endcond


template<typename T>
inline T max(const T& a, const T& b)
{
	T res;
	for (size_t i = 0; i < T::SIZE; ++i) {
		res[i] = max(a[i], b[i]);
	}
	return res;
}

/// @cond SPECIALIZATIONS
template<>
inline double max(const double& a, const double& b)
{
	return std::max(a, b);
}
/// @endcond


template<typename T>
inline T saturate(const T& x, const T& limit)
{
	return sign(x) * min(abs(x), limit);
}


// TODO(dc): test!
template<typename T>
inline T deadband(const T& x, const T& cutoff)
{
	T res;
	for (size_t i = 0; i < T::SIZE; ++i) {
		res[i] = deadBand(x[i], cutoff[i]);
	}
	return res;
}

/// @cond SPECIALIZATIONS
template<>
inline double deadband(const double& x, const double& cutoff)
{
	return (abs(x) > cutoff) ? (x - cutoff * sign(x)) : 0.0;
}
/// @endcond


}
}
