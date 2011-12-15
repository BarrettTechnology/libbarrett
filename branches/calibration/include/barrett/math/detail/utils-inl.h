/*
 * utils-inl.h
 *
 *  Created on: Nov 11, 2009
 *      Author: dc
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
#include <cassert>

namespace barrett {
namespace math {


namespace detail {
template<typename Scalar> struct CwiseSignOp {
	inline const Scalar operator() (const Scalar& x) const {
		return sign(x);
	}
};
}

template<typename Derived>
inline const Eigen::CwiseUnaryOp<
	detail::CwiseSignOp<typename Eigen::ei_traits<Derived>::Scalar>,
	Derived
> sign(const Eigen::MatrixBase<Derived>& x)
{
	return x.unaryExpr(detail::CwiseSignOp<typename Eigen::ei_traits<Derived>::Scalar>());
}

inline double sign(double x)
{
	if (x > 0.0) {
		return 1.0;
	} else if (x == 0.0) {
		return 0.0;
	} else {
		return -1.0;
	}
}


using std::abs;

template<typename Derived>
inline const Eigen::CwiseUnaryOp<
	Eigen::ei_scalar_abs_op<typename Eigen::ei_traits<Derived>::Scalar>,
	Derived
> abs(const Eigen::MatrixBase<Derived>& x)
{
	return x.cwise().abs();
}


template<typename Derived1, typename Derived2>
inline const Eigen::CwiseBinaryOp<
	Eigen::ei_scalar_min_op<typename Eigen::ei_traits<Derived1>::Scalar>,
	Derived1,
	Derived2
> min(const Eigen::MatrixBase<Derived1>& a, const Eigen::MatrixBase<Derived2>& b)
{
	return a.cwise().min(b);
}

inline double min(double a, double b)
{
	return std::min(a, b);
}


template<typename Derived1, typename Derived2>
inline const Eigen::CwiseBinaryOp<
	Eigen::ei_scalar_max_op<typename Eigen::ei_traits<Derived1>::Scalar>,
	Derived1,
	Derived2
> max(const Eigen::MatrixBase<Derived1>& a, const Eigen::MatrixBase<Derived2>& b)
{
	return a.cwise().max(b);
}

inline double max(double a, double b)
{
	return std::max(a, b);
}


namespace detail {
template<typename Scalar> struct CwiseUnarySaturateOp {
	CwiseUnarySaturateOp(Scalar lowerLimit, Scalar upperLimit) : lowerLimit(lowerLimit), upperLimit(upperLimit) {}

	inline const Scalar operator() (const Scalar& x) const {
		return saturate(x, lowerLimit, upperLimit);
	}

	Scalar lowerLimit, upperLimit;
};

template<typename Scalar> struct CwiseBinarySaturateOp {
	inline const Scalar operator() (const Scalar& x, const Scalar& limit) const {
		return saturate(x, limit);
	}
};
}

template<typename Derived>
inline const Eigen::CwiseUnaryOp<
	detail::CwiseUnarySaturateOp<typename Eigen::ei_traits<Derived>::Scalar>,
	Derived
> saturate(const Eigen::MatrixBase<Derived>& x, double limit)
{
	return x.unaryExpr(detail::CwiseUnarySaturateOp<typename Eigen::ei_traits<Derived>::Scalar>(-limit, limit));
}

template<typename Derived1, typename Derived2>
inline const Eigen::CwiseBinaryOp<
	detail::CwiseBinarySaturateOp<typename Eigen::ei_traits<Derived1>::Scalar>,
	Derived1,
	Derived2
> saturate(const Eigen::MatrixBase<Derived1>& x, const Eigen::MatrixBase<Derived2>& limit)
{
	return x.binaryExpr(limit, detail::CwiseBinarySaturateOp<typename Eigen::ei_traits<Derived1>::Scalar>());
}

inline double saturate(double x, double limit)
{
	return saturate(x, -limit, limit);
}


template<typename Derived>
inline const Eigen::CwiseUnaryOp<
	detail::CwiseUnarySaturateOp<typename Eigen::ei_traits<Derived>::Scalar>,
	Derived
> saturate(const Eigen::MatrixBase<Derived>& x, double lowerLimit, double upperLimit)
{
	return x.unaryExpr(detail::CwiseUnarySaturateOp<typename Eigen::ei_traits<Derived>::Scalar>(lowerLimit, upperLimit));
}

inline double saturate(double x, double lowerLimit, double upperLimit)
{
	assert(lowerLimit < upperLimit);

	if (x > upperLimit) {
		return upperLimit;
	} else if (x < lowerLimit) {
		return lowerLimit;
	} else {
		return x;
	}
}


namespace detail {
template<typename Scalar> struct CwiseUnaryDeadbandOp {
	CwiseUnaryDeadbandOp(Scalar cutoff) : cutoff(cutoff) {}

	inline const Scalar operator() (const Scalar& x) const {
		return deadband(x, cutoff);
	}

	Scalar cutoff;
};

template<typename Scalar> struct CwiseBinaryDeadbandOp {
	inline const Scalar operator() (const Scalar& x, const Scalar& cutoff) const {
		return deadband(x, cutoff);
	}
};
}

template<typename Derived>
inline const Eigen::CwiseUnaryOp<
	detail::CwiseUnaryDeadbandOp<typename Eigen::ei_traits<Derived>::Scalar>,
	Derived
> deadband(const Eigen::MatrixBase<Derived>& x, double cutoff)
{
	return x.unaryExpr(detail::CwiseUnaryDeadbandOp<typename Eigen::ei_traits<Derived>::Scalar>(cutoff));
}

template<typename Derived1, typename Derived2>
inline const Eigen::CwiseBinaryOp<
	detail::CwiseBinaryDeadbandOp<typename Eigen::ei_traits<Derived1>::Scalar>,
	Derived1,
	Derived2
> deadband(const Eigen::MatrixBase<Derived1>& x, const Eigen::MatrixBase<Derived2>& cutoff)
{
	return x.binaryExpr(cutoff, detail::CwiseBinaryDeadbandOp<typename Eigen::ei_traits<Derived1>::Scalar>());
}

inline double deadband(double x, double cutoff)
{
	if (x > cutoff) {
		return x - cutoff;
	} else if (x < -cutoff) {
		return x + cutoff;
	} else {
		return 0.0;
	}
}


}
}
