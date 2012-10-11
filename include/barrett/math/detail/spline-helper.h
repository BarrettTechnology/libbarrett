/*
	Copyright 2012 Barrett Technology <support@barrett.com>

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
 * spline-helper.h
 *
 *  Created on: Jul 11, 2012
 *      Author: dc
 */


#include <vector>
#include <cassert>

#include <boost/tuple/tuple.hpp>

#include <barrett/math/utils.h>


namespace barrett {
namespace math {


template<typename T> class Spline;


// doxygen can't handle TupleSplineHolder's recursive inheritance.
#ifndef BARRETT_PARSED_BY_DOXYGEN
namespace detail {


template<size_t N,
	typename T0, typename T1, typename T2, typename T3, typename T4,
	typename T5, typename T6, typename T7, typename T8, typename T9>
struct TupleSplineHolder :
		public TupleSplineHolder<N-1, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> {

	typedef TupleSplineHolder<N-1, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> inherited_type;
	typedef boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> tuple_type;
	typedef Spline<tuple_type> parent_spline_type;
	typedef typename boost::tuples::element<N-1, tuple_type>::type current_data_type;
	typedef Spline<current_data_type> current_spline_type;

	template<template<typename, typename> class Container, typename Allocator>
	TupleSplineHolder(const Container<typename parent_spline_type::tuple_type, Allocator>& samples, bool saturateS) :
		inherited_type(samples, saturateS), spline(NULL), rateAdjustment(1.0)
	{
		std::vector<typename current_spline_type::tuple_type> currentSamples;
		currentSamples.reserve(samples.size());

		typename Container<typename parent_spline_type::tuple_type, Allocator>::const_iterator i;
		for (i = samples.begin(); i != samples.end(); ++i) {
			currentSamples.push_back(boost::make_tuple(boost::get<0>(*i), boost::get<N-1>(boost::get<1>(*i))));
		}

		spline = new current_spline_type(currentSamples, saturateS);
		this->initialS = spline->initialS();
		this->maxChangeInS = spline->changeInS();
	}

	template<template<typename, typename> class Container, typename Allocator>
	TupleSplineHolder(const Container<typename parent_spline_type::data_type, Allocator>& points, bool saturateS) :
		inherited_type(points, saturateS), spline(NULL), rateAdjustment(0.0)
	{
		std::vector<current_data_type> currentPoints;
		currentPoints.reserve(points.size());

		typename Container<typename parent_spline_type::data_type, Allocator>::const_iterator i;
		for (i = points.begin(); i != points.end(); ++i) {
			currentPoints.push_back(boost::get<N-1>(*i));
		}

		spline = new current_spline_type(currentPoints, saturateS);
		this->initialS = spline->initialS();
		updateRateAdjustments();
	}

	~TupleSplineHolder() {
		delete spline;
		spline = NULL;
	}

	void updateRateAdjustments() {
		double changeInS = spline->changeInS();
		this->maxChangeInS = math::max(changeInS, this->maxChangeInS);

		inherited_type::updateRateAdjustments();

		rateAdjustment = changeInS / this->maxChangeInS;
		assert(rateAdjustment <= 1.0);
	}

	void collectValues(double s) const {
		inherited_type::collectValues(s);

		// Scale s so that all of the component Splines have the same final value.
		boost::get<N-1>(this->data) = spline->eval(s * rateAdjustment);
	}

	current_spline_type* spline;
	double rateAdjustment;
};

// Specialization for the inheritance base case:
template<
	typename T0, typename T1, typename T2, typename T3, typename T4,
	typename T5, typename T6, typename T7, typename T8, typename T9>
struct TupleSplineHolder<0, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> {

	typedef boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> tuple_type;
	typedef Spline<tuple_type> parent_spline_type;

	template<template<typename, typename> class Container, typename Allocator>
	TupleSplineHolder(const Container<typename parent_spline_type::tuple_type, Allocator>& samples, bool saturateS) :
		initialS(0.0), maxChangeInS(0.0) {}

	template<template<typename, typename> class Container, typename Allocator>
	TupleSplineHolder(const Container<typename parent_spline_type::data_type, Allocator>& points, bool saturateS) :
		initialS(0.0), maxChangeInS(0.0) {}

	void updateRateAdjustments() {}
	void collectValues(double s) const {}

	double initialS;
	double maxChangeInS;
	mutable tuple_type data;
};


}
#endif // BARRETT_PARSED_BY_DOXYGEN
}
}
