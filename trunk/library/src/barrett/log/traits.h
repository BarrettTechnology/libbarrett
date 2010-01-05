/*
 * traits.h
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */

#ifndef TRAITS_H_
#define TRAITS_H_


#include <boost/tuple/tuple.hpp>


namespace barrett {
namespace log {


// default traits delegate to the type in question
template<typename T> struct Traits {
	typedef const T& parameter_type;

	static size_t serializedLength() {
		return T::serializedLength();
	}

	static void serialize(parameter_type source, char* dest) {
		source.serialize(dest);
	}

	static T unserialize(char* source) {
		return T::unserialize(source);
	}
};


template<> struct Traits<double> {
	typedef double parameter_type;

	static size_t serializedLength() {
		return sizeof(double);
	}

	static void serialize(parameter_type source, char* dest) {
		*reinterpret_cast<double*>(dest) = source;
	}

	static double unserialize(char* source) {
		return *reinterpret_cast<double*>(source);
	}
};


}
}


#include "detail/tuple_traits.h"


#endif /* TRAITS_H_ */
