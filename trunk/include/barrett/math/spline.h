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
#include<Eigen/StdVector>
#include <Eigen/Geometry>

#include <barrett/detail/ca_macro.h>


// forward declaration from <barrett/spline/spline.h>
struct bt_spline;


namespace barrett {
namespace math {


template<typename T>
class Spline {
public:
	typedef boost::tuple<double, T> tuple_type;

	template<template<typename, typename> class Container, typename Allocator>
	Spline(const Container<tuple_type, Allocator>& samples, bool saturateS = true);

	// initialDirection will be normalized internally
	template<template<typename, typename> class Container, typename Allocator>
	Spline(const Container<T, Allocator>& points, const typename T::unitless_type& initialDirection = typename T::unitless_type(0.0), bool saturateS = true);

	~Spline();

	double initialS() const { return s_0; }
	double finalS() const { return s_f; }
	double changeInS() const;

	T eval(double s) const;

	typedef T result_type;  ///< For use with boost::bind().
	result_type operator() (double s) const {
		return eval(s);
	}

protected:
	struct bt_spline* impl;
	bool sat;
	double s_0, s_f;

private:
	// TODO(dc): write a real copy constructor and assignment operator?
	DISALLOW_COPY_AND_ASSIGN(Spline);
};


// Specialization for Eigen::Quaternion<>  types
template<typename Scalar>
class Spline<Eigen::Quaternion<Scalar> > {
public:
	typedef Eigen::Quaternion<Scalar> T;
	typedef boost::tuple<double, T> tuple_type;

	template<template<typename, typename> class Container, typename Allocator>
	Spline(const Container<tuple_type, Allocator>& samples, bool saturateS = true);

	// initialDirection is ignored for quaternion types
	template<template<typename, typename> class Container, typename Allocator>
	Spline(const Container<T, Allocator>& points, bool saturateS = true);

	double initialS() const { return boost::get<0>(data.front()); }
	double finalS() const { return boost::get<0>(data.back()); }
	double changeInS() const { return finalS() - initialS(); }

	T eval(double s) const;

	typedef T result_type;  ///< For use with boost::bind().
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


}
}


// include template definitions
#include <barrett/math/detail/spline-inl.h>


#endif /* BARRETT_MATH_SPLINE_H_ */
