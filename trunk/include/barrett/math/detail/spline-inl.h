/*
 * spline-inl.h
 *
 *  Created on: Dec 22, 2009
 *      Author: dc
 */


#include <iostream>

#include <barrett/math/utils.h>
#include <barrett/cdlbt/spline.h>


namespace barrett {
namespace math {


template<typename T>
template<template<typename U, typename = std::allocator<U> > class Container>
Spline<T>::Spline(const Container<tuple_type>& samples, bool saturateS) :
	impl(NULL), sat(saturateS), s_0(0.0), s_f(0.0)
{
	s_0 = boost::get<0>(samples[0]);

	bt_spline_create(&impl, boost::get<1>(samples[0]).asGslType(), BT_SPLINE_MODE_EXTERNAL);

	typename Container<tuple_type>::const_iterator i;
	for (i = ++(samples.begin()); i != samples.end(); ++i) {  // start with the 2nd sample
		bt_spline_add(impl, boost::get<1>(*i).asGslType(), boost::get<0>(*i) - s_0);
	}

	bt_spline_init(impl, NULL, NULL);
	s_f = s_0 + changeInS();
}

template<typename T>
template<template<typename U, typename = std::allocator<U> > class Container>
Spline<T>::Spline(const Container<T>& points, const typename T::unitless_type& initialDirection, bool saturateS) :
	impl(NULL), sat(saturateS), s_0(0.0), s_f(0.0)
{
	bt_spline_create(&impl, points[0].asGslType(), BT_SPLINE_MODE_ARCLEN);

	typename Container<T>::const_iterator i;
	for (i = ++(points.begin()); i != points.end(); ++i) {  // start with the 2nd sample
		bt_spline_add(impl, (*i).asGslType(), 0);
	}

	// local copy because init modifies its 3rd parameter
	typename T::unitless_type id(initialDirection);
	bt_spline_init(impl, NULL, id.asGslType());
	s_f = s_0 + changeInS();
}

template<typename T>
Spline<T>::~Spline()
{
	bt_spline_destroy(impl);
	impl = NULL;
}


template<typename T>
inline double Spline<T>::initialS() const
{
	return s_0;
}

template<typename T>
inline double Spline<T>::finalS() const
{
	return s_f;
}

template<typename T>
inline double Spline<T>::changeInS() const
{
	return impl->length;
}


template<typename T>
inline T Spline<T>::eval(double s) const
{
	if (sat) {
		s = saturate(s, s_0, s_f);
	}

	T result;
	bt_spline_get(impl, result.asGslType(), s - s_0);
	return result;
}


}
}
