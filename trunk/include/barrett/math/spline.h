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

#include <barrett/detail/ca_macro.h>


// forward declaration from <barrett/spline/spline.h>
struct bt_spline;


namespace barrett {
namespace math {


// TODO(dc): add an option to saturate x if it is outside the range of [initialX, finalX].
template<typename T>
class Spline {
public:
	typedef boost::tuple<double, T> tuple_type;

	template<template<typename U, typename = std::allocator<U> > class Container>
	Spline(const Container<tuple_type>& samples);

	// initialDirection will be normalized internally
	template<template<typename U, typename = std::allocator<U> > class Container>
	Spline(const Container<T>& points, const typename T::unitless_type& initialDirection = typename T::unitless_type(0.0));

	~Spline();

	double initialX() const;
	double finalX() const;
	double changeInX() const;

	T eval(double x) const;

	typedef T result_type;  ///< For use with boost::bind().
	result_type operator() (double x) const {
		return eval(x);
	}

protected:
	struct bt_spline* impl;
	double x_0;

private:
	// TODO(dc): write a real copy constructor and assignment operator?
	DISALLOW_COPY_AND_ASSIGN(Spline);
};


}
}


// include template definitions
#include <barrett/math/detail/spline-inl.h>


#endif /* BARRETT_MATH_SPLINE_H_ */
