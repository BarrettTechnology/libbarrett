/*
 * traits.h
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */

#ifndef TRAITS_H_
#define TRAITS_H_


#include "../../units.h"


namespace barrett {
namespace log {
namespace detail {


// default traits delegate to the type in question
template<typename T> struct Traits {
	typedef const T& parameter_type;
	typedef char* pointer_type;

	static size_t serializedLength() {
		return T::serializedLength();
	}

	static void serialize(parameter_type source, pointer_type dest) {
		source.serialize(dest);
	}

	static T unserialize(pointer_type source) {
		return T::unserialize(source);
	}
};


template<> struct Traits<double> {
	typedef double parameter_type;
	typedef double* pointer_type;

	static size_t serializedLength() {
		return sizeof(double);
	}

	static void serialize(parameter_type source, pointer_type dest) {
		*dest = source;
	}

	static double unserialize(pointer_type source) {
		return *source;
	}
};


}
}
}


#endif /* TRAITS_H_ */
