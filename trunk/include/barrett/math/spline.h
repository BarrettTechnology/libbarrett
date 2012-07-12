/*
	Copyright 2009, 2010, 2011, 2012 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * spline.h
 *
 *  Created on: Dec 17, 2009
 *      Author: dc
 */


#ifndef BARRETT_MATH_SPLINE_H_
#define BARRETT_MATH_SPLINE_H_


#include <vector>

#include <boost/tuple/tuple.hpp>

#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/detail/spline-helper.h>


// forward declaration from <barrett/spline/spline.h>
struct bt_spline;


namespace barrett {
namespace math {


template<typename T>
class Spline {
public:
	typedef T data_type;
	typedef boost::tuple<double, T> tuple_type;

	template<template<typename, typename> class Container, typename Allocator>
	Spline(const Container<tuple_type, Allocator>& samples, bool saturateS = true);

	// initialDirection will be normalized internally
	template<template<typename, typename> class Container, typename Allocator>
	Spline(const Container<T, Allocator>& points, /*const typename T::unitless_type& initialDirection = typename T::unitless_type(0.0),*/ bool saturateS = true);

	~Spline();

	double initialS() const { return s_0; }
	double finalS() const { return s_f; }
	double changeInS() const;

	T eval(double s) const;
	T evalDerivative(double s) const;

	typedef T result_type;  ///< For use with boost::bind().
	result_type operator() (double s) const {
		return eval(s);
	}

	// Access the low-level spline implementation. Please avoid using this.
	struct bt_spline* getImplementation() { return impl; }

protected:
	struct bt_spline* impl;
	bool sat;
	double s_0, s_f;

private:
	// TODO(dc): write a real copy constructor and assignment operator?
	DISALLOW_COPY_AND_ASSIGN(Spline);
};


// Specialization for Eigen::Quaternion<> types
template<typename Scalar>
class Spline<Eigen::Quaternion<Scalar> > {
public:
	typedef Eigen::Quaternion<Scalar> data_type;
	typedef boost::tuple<double, data_type> tuple_type;

	template<template<typename, typename> class Container, typename Allocator>
	Spline(const Container<tuple_type, Allocator>& samples, bool saturateS = true);

	// initialDirection is ignored for quaternion types
	template<template<typename, typename> class Container, typename Allocator>
	Spline(const Container<data_type, Allocator>& points, bool saturateS = true);

	double initialS() const { return boost::get<0>(data.front()); }
	double finalS() const { return boost::get<0>(data.back()); }
	double changeInS() const { return finalS() - initialS(); }

	data_type eval(double s) const;

	typedef data_type result_type;  ///< For use with boost::bind().
	result_type operator() (double s) const {
		return eval(s);
	}

protected:
	std::vector<tuple_type, Eigen::aligned_allocator<tuple_type> > data;
	bool sat;

	mutable size_t index;
	mutable double rate;

private:
	// TODO(dc): write a real copy constructor and assignment operator?
	DISALLOW_COPY_AND_ASSIGN(Spline);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


// Specialization for boost::tuple<> types
template <
	typename T0, typename T1, typename T2, typename T3, typename T4,
	typename T5, typename T6, typename T7, typename T8, typename T9>
class Spline<boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> > {
public:
	typedef boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> data_type;
	typedef boost::tuple<double, data_type> tuple_type;

	static const size_t TUPLE_LEN = boost::tuples::length<data_type>::value;
	typedef detail::TupleSplineHolder<TUPLE_LEN, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> holder_type;

	template<template<typename, typename> class Container, typename Allocator>
	Spline(const Container<tuple_type, Allocator>& samples, bool saturateS = true) :
		holder(samples, saturateS) {}

	// initialDirection is ignored for boost::tuple types
	template<template<typename, typename> class Container, typename Allocator>
	Spline(const Container<data_type, Allocator>& points, bool saturateS = true) :
		holder(points, saturateS) {}

	double initialS() const { return holder.initialS; }
	double finalS() const { return initialS() + changeInS(); }
	double changeInS() const { return holder.maxChangeInS; }

	data_type eval(double s) const {
		holder.collectValues(s);
		return holder.data;
	}

	typedef data_type result_type;  ///< For use with boost::bind().
	result_type operator() (double s) const {
		return eval(s);
	}

protected:
	holder_type holder;

private:
	// TODO(dc): write a real copy constructor and assignment operator?
	DISALLOW_COPY_AND_ASSIGN(Spline);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


}
}


// include template definitions
#include <barrett/math/detail/spline-inl.h>


#endif /* BARRETT_MATH_SPLINE_H_ */
