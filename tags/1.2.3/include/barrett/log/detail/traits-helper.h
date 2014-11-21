/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 */

/**
 * @file traits-helper.h
 * @date 1/5/2010
 * @author Dan Cody
 *  
 */

#ifndef BARRETT_LOG_DETAIL_TRAITS_HELPER_H_
#define BARRETT_LOG_DETAIL_TRAITS_HELPER_H_


#include <ostream>
#include <boost/tuple/tuple.hpp>


namespace barrett {
namespace log {


// forward declaration from "../traits.h"
template<typename T> struct Traits;


namespace detail {


template<typename T>
inline void arrayAsCSV(std::ostream& os, const T& array, const size_t size)
{
	for (size_t i = 0; i < (size - 1); ++i) {
		os << array[i] << ",";
	}
	os << array[size - 1];
}


template<size_t N, typename TraitsType>
struct TupleTraitsHelper {

	typedef typename TraitsType::tuple_type tuple_type;
	typedef typename TraitsType::parameter_type parameter_type;

	static const size_t INDEX = TraitsType::NUM_INPUTS - N;
	typedef Traits<typename boost::tuples::element<INDEX, tuple_type>::type> element_traits;
	typedef TupleTraitsHelper<(N - 1), TraitsType> next_helper;


	static size_t serializedLength() {
		return element_traits::serializedLength() + next_helper::serializedLength();
	}

	static void serialize(parameter_type source, char* dest) {
		element_traits::serialize(boost::get<INDEX>(source), dest);
		next_helper::serialize(source, dest + element_traits::serializedLength());
	}

	static void unserialize(char* source, tuple_type* t) {
		boost::get<INDEX>(*t) = element_traits::unserialize(source);
		next_helper::unserialize(source + element_traits::serializedLength(), t);
	}

	static void asCSV(parameter_type source, std::ostream& os) {
		if (INDEX != 0) {
			os << ",";
		}

		element_traits::asCSV(boost::get<INDEX>(source), os);
		next_helper::asCSV(source, os);
	}
};

// base-case specialization (N == 0)
template<typename TraitsType>
struct TupleTraitsHelper<0, TraitsType> {

	typedef typename TraitsType::tuple_type tuple_type;
	typedef typename TraitsType::parameter_type parameter_type;

	static size_t serializedLength() {  return 0;  }
	static void serialize(parameter_type source, char* dest) {}
	static void unserialize(char* source, tuple_type* t) {}
	static void asCSV(parameter_type source, std::ostream& os) {}
};


}
}
}


#endif /* BARRETT_LOG_DETAIL_TRAITS_HELPER_H_ */
