/*
 * traits.h
 *
 *  Created on: Feb 17, 2010
 *      Author: dc
 */

#ifndef BARRETT_MATH_TRAITS_H_
#define BARRETT_MATH_TRAITS_H_


namespace barrett {
namespace math {


// TODO(dc): test!

// Default designed for built-in arithmetic types.
template<typename T> struct Traits {
	typedef T unitless_type;
	static const bool RequiresAlignment = false;


	static T zero() {
		return T();
	}

	static void zero(T& t) {
		t = 0;
	}

	static T add(T l, T r) {
		return l + r;
	}

	static T sub(T l, T r) {
		return l - r;
	}

	static T neg(T t) {
		return -t;
	}

	static T mult(T l, T r) {
		return l * r;
	}

	static T div(T l, T r) {
		return l / r;
	}
};


}
}


#endif /* BARRETT_MATH_TRAITS_H_ */
