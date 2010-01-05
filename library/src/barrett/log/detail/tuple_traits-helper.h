/*
 * tuple_traits-helper.h
 *
 *  Created on: Jan 5, 2010
 *      Author: dc
 */

#ifndef TUPLE_TRAITS_HELPER_H_
#define TUPLE_TRAITS_HELPER_H_


#include <boost/tuple/tuple.hpp>


namespace barrett {
namespace log {


// forward declaration from "../traits.h"
template<typename T> struct Traits;


namespace detail {


template<
	typename T0, typename T1, typename T2, typename T3, typename T4,
	typename T5, typename T6, typename T7, typename T8, typename T9,
	size_t N, typename TraitsType>
struct TupleTraitsHelper {

	typedef typename TraitsType::tuple_type tuple_type;
	typedef typename TraitsType::parameter_type parameter_type;

	static const size_t INDEX = TraitsType::NUM_INPUTS - N;

	typedef TupleTraitsHelper<T1, T2, T3, T4, T5, T6, T7, T8, T9, boost::tuples::null_type,
							  (N - 1), TraitsType> next_helper_type;


	static size_t serializedLength() {
		return Traits<T0>::serializedLength() + next_helper_type::serializedLength();
	}

	static void serialize(parameter_type source, char* dest) {
		Traits<T0>::serialize(source.template get<INDEX>(), dest);
		next_helper_type::serialize(source, dest + Traits<T0>::serializedLength());
	}

	static void unserialize(char* source, tuple_type* t) {
		t->template get<INDEX>() = Traits<T0>::unserialize(source);
		next_helper_type::unserialize(source + Traits<T0>::serializedLength(), t);
	}
};

// base-case specialization (N == 0)
template<
	typename T0, typename T1, typename T2, typename T3, typename T4,
	typename T5, typename T6, typename T7, typename T8, typename T9,
	typename TraitsType>
struct TupleTraitsHelper<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, 0, TraitsType> {

	typedef typename TraitsType::tuple_type tuple_type;
	typedef typename TraitsType::parameter_type parameter_type;

	static size_t serializedLength() {  return 0;  }
	static void serialize(parameter_type source, char* dest) {}
	static void unserialize(char* source, tuple_type* t) {}
};


}
}
}


#endif /* TUPLE_TRAITS_HELPER_H_ */
