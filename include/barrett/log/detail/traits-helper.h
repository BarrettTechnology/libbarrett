/*
 * traits-helper.h
 *
 *  Created on: Jan 5, 2010
 *      Author: dc
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
