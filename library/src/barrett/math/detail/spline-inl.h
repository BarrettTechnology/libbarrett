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

	typename Container<Spline<T>::Sample>::const_iterator i;
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
//	std::cout << gsl_vector_get(points[0].asGslVector(), 0) << std::endl;
	bt_spline_create(&impl, points[0].asGslVector(), BT_SPLINE_MODE_ARCLEN);

	typename Container<T>::const_iterator i;
	for (i = ++(points.begin()); i != points.end(); ++i) {
//		std::cout << gsl_vector_get(i->asGslVector(), 0) << std::endl;
		bt_spline_add(impl, (*i).asGslVector(), 0);
	}

	bt_spline_init(impl, NULL, NULL);

//	for (int i = 0; i < impl->npoints; ++i) {
//		std::cout << impl->ss[i] << ", ";
//	}
//	std::cout << std::endl;
//
//	std::cout << impl->length << std::endl;
//	for (int i = 0; i < impl->npoints; ++i) {
//		std::cout << impl->points[0][i] << ", ";
//	}
//	std::cout << std::endl;
}

template<typename T>
Spline<T>::~Spline()
{
	bt_spline_destroy(impl);
	impl = NULL;
}


template<typename T>
inline double Spline<T>::initialX()
{
	return x_0;
}

template<typename T>
inline double Spline<T>::finalX()
{
	return x_0 + impl->length;
}

template<typename T>
inline double Spline<T>::changeInX()
{
	return impl->length;
}


template<typename T>
inline T Spline<T>::eval(double x)
{
	T result;
	bt_spline_get(impl, result.asGslVector(), x - x_0);
	return result;
}


}
}
