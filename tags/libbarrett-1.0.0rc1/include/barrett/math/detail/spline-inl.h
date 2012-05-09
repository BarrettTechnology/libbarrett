/*
 * spline-inl.h
 *
 *  Created on: Dec 22, 2009
 *      Author: dc
 */


#include <iostream>
#include <cassert>

#include <barrett/math/utils.h>
#include <barrett/cdlbt/spline.h>


namespace barrett {
namespace math {


template<typename T>
template<template<typename, typename> class Container, typename Allocator>
Spline<T>::Spline(const Container<tuple_type, Allocator>& samples, bool saturateS) :
	impl(NULL), sat(saturateS), s_0(0.0), s_f(0.0)
{
	s_0 = boost::get<0>(samples[0]);

	bt_spline_create(&impl, boost::get<1>(samples[0]).asGslType(), BT_SPLINE_MODE_EXTERNAL);

	typename Container<tuple_type, Allocator>::const_iterator i;
	for (i = ++(samples.begin()); i != samples.end(); ++i) {  // start with the 2nd sample
		bt_spline_add(impl, boost::get<1>(*i).asGslType(), boost::get<0>(*i) - s_0);
	}

	bt_spline_init(impl, NULL, NULL);
	s_f = s_0 + changeInS();
}

template<typename T>
template<template<typename, typename> class Container, typename Allocator>
Spline<T>::Spline(const Container<T, Allocator>& points, const typename T::unitless_type& initialDirection, bool saturateS) :
	impl(NULL), sat(saturateS), s_0(0.0), s_f(0.0)
{
	bt_spline_create(&impl, points[0].asGslType(), BT_SPLINE_MODE_ARCLEN);

	typename Container<T, Allocator>::const_iterator i;
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


// Specialization for Eigen::Quaternion  types
template<typename Scalar>
template<template<typename, typename> class Container, typename Allocator>
Spline<Eigen::Quaternion<Scalar> >::Spline(const Container<tuple_type, Allocator>& samples, bool saturateS) :
	data(samples.begin(), samples.end()), sat(saturateS), index(0), rate(-1.0)
{
	// Make sure s is monotonic.
	for (size_t i = 0; i < data.size() - 1; ++i) {
		assert(boost::get<0>(data[i]) < boost::get<0>(data[i+1]));
	}
}

template<typename Scalar>
template<template<typename, typename> class Container, typename Allocator>
Spline<Eigen::Quaternion<Scalar> >::Spline(const Container<T, Allocator>& points, bool saturateS) :
	data(points.size()), sat(saturateS), index(0), rate(-1.0)
{
	double s = 0.0;
	for (size_t i = 0; i < data.size(); ++i) {
		if (i > 0) {
			s += points.at(i).angularDistance(points.at(i-1));
		}

		boost::get<0>(data[i]) = s;
		boost::get<1>(data[i]) = points.at(i);
	}

	// Make sure s is monotonic.
	for (size_t i = 0; i < data.size() - 1; ++i) {
		assert(boost::get<0>(data[i]) < boost::get<0>(data[i+1]));
	}
}

template<typename Scalar>
typename Spline<Eigen::Quaternion<Scalar> >::T Spline<Eigen::Quaternion<Scalar> >::eval(double s) const
{
	s = saturate(s, initialS(), finalS());

	while (index > 0  &&  s < boost::get<0>(data[index])) {
		--index;
		rate = -1.0;
	}
	while (index < data.size()-1  &&  s >= boost::get<0>(data[index+1])) {
		++index;
		rate = -1.0;
	}
	assert(index < data.size());

	if (index == data.size()-1) {
		return boost::get<1>(data[index]);
	} else {
		if (rate < 0.0) {
			rate = 1.0 / (boost::get<0>(data[index+1]) - boost::get<0>(data[index]));
		}

		return boost::get<1>(data[index]).slerp(rate * (s - boost::get<0>(data[index])), boost::get<1>(data[index+1]));
	}
}


}
}
