/*
 * spline-inl.h
 *
 *  Created on: Dec 22, 2009
 *      Author: dc
 */


#include <iostream>
#include <barrett/spline/spline.h>


namespace barrett {
namespace math {


template<typename T>
template<template<typename U, typename = std::allocator<U> > class Container>
Spline<T>::Spline(const Container<Spline<T>::Sample>& samples) :
	impl(NULL), x_0(0.0)
{
	x_0 = samples[0].x;

	bt_spline_create(&impl, samples[0].point.asGslVector(), BT_SPLINE_MODE_EXTERNAL);

	typename Container<typename Spline<T>::Sample>::const_iterator i;
	for (i = ++(samples.begin()); i != samples.end(); ++i) {
		bt_spline_add(impl, (*i).point.asGslVector(), (*i).x - x_0);
	}

	bt_spline_init(impl, NULL, NULL);
//	bt_spline_init(impl, NULL, gsl_vector * direction);
}

template<typename T>
template<template<typename U, typename = std::allocator<U> > class Container>
Spline<T>::Spline(const Container<T>& points) :
	impl(NULL), x_0(0.0)
{
	bt_spline_create(&impl, points[0].asGslVector(), BT_SPLINE_MODE_ARCLEN);

	typename Container<T>::const_iterator i;
	for (i = ++(points.begin()); i != points.end(); ++i) {
		bt_spline_add(impl, (*i).asGslVector(), 0);
	}

	bt_spline_init(impl, NULL, NULL);
}

template<typename T>
Spline<T>::~Spline()
{
	bt_spline_destroy(impl);
	impl = NULL;
}


template<typename T>
inline double Spline<T>::initialX() const
{
	return x_0;
}

template<typename T>
inline double Spline<T>::finalX() const
{
	return x_0 + impl->length;
}

template<typename T>
inline double Spline<T>::changeInX() const
{
	return impl->length;
}


template<typename T>
inline T Spline<T>::eval(double x) const
{
	T result;
	bt_spline_get(impl, result.asGslVector(), x - x_0);
	return result;
}


}
}
