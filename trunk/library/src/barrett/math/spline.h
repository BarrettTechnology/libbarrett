/*
 * spline.h
 *
 *  Created on: Dec 17, 2009
 *      Author: dc
 */


#ifndef SPLINE_H_
#define SPLINE_H_


#include <vector>
#include "../detail/ca_macro.h"
#include <barrett/spline/spline.h>


// forward declaration from <barrett/spline/spline.h>
struct bt_spline;


namespace barrett {
namespace math {


template<typename T>
class Spline {
public:
	struct Sample {
		double x;
		T point;
	};

	template<template<typename U, typename = std::allocator<U> > class Container>
	Spline(const Container<Sample>& samples);
	template<template<typename U, typename = std::allocator<U> > class Container>
	Spline(const Container<T>& points);

	~Spline();

	double initialX() const;
	double finalX() const;
	double changeInX() const;

	T eval(double x) const;

	typedef T result_type;  ///< For use with boost::bind().
	T operator () (double x) const {
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
#include "./detail/spline-inl.h"


#endif /* SPLINE_H_ */
