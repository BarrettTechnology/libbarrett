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


#include <boost/tuple/tuple.hpp>


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
	typedef typename boost::tuples::element<N-1, tuple_type>::type current_data_type;
	typedef Spline<current_data_type> spline_type;

	template<template<typename, typename> class Container, typename Allocator>
	TupleSplineHolder(const Container<typename spline_type::tuple_type, Allocator>& samples, bool saturateS = true);

	// initialDirection is ignored for boost::tuple types
	template<template<typename, typename> class Container, typename Allocator>
	TupleSplineHolder(const Container<typename spline_type::T, Allocator>& points, bool saturateS = true);

	spline_type* spline;
};

// Specialization for the inheritance base case:
template<
	typename T0, typename T1, typename T2, typename T3, typename T4,
	typename T5, typename T6, typename T7, typename T8, typename T9>
struct TupleSplineHolder<0, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> {

};


}
#endif // BARRETT_PARSED_BY_DOXYGEN
}
}
