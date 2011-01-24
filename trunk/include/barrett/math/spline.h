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


// TODO(dc): add an option to saturate s if it is outside the range of [initialS, finalS].
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

	double initialS() const;
	double finalS() const;
	double changeInS() const;

	T eval(double s) const;

	typedef T result_type;  ///< For use with boost::bind().
	result_type operator() (double s) const {
		return eval(s);
	}

protected:
	struct bt_spline* impl;
	double s_0;

private:
	// TODO(dc): write a real copy constructor and assignment operator?
	DISALLOW_COPY_AND_ASSIGN(Spline);
};


}
}


// include template definitions
#include <barrett/math/detail/spline-inl.h>


#endif /* BARRETT_MATH_SPLINE_H_ */
