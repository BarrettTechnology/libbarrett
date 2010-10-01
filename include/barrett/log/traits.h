/*
 * traits.h
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */

#ifndef BARRETT_LOG_TRAITS_H_
#define BARRETT_LOG_TRAITS_H_


#include <ostream>

#include <boost/tuple/tuple.hpp>

#include "../math/matrix.h"
#include "./detail/tuple_traits-helper.h"


namespace barrett {
namespace log {


// default traits delegate to the type in question
template<typename T> struct DefaultTraits {
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

	static void asCSV(parameter_type source, std::ostream& os) {
		os << source;
	}
};


template<typename T> struct Traits : public DefaultTraits<T> {};

template<> struct Traits<double> : public DefaultTraits<double> {
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

//template<typename TraitsDerived> struct Traits<Eigen::MatrixBase<TraitsDerived> > :
//		public DefaultTraits<Eigen::MatrixBase<TraitsDerived> > {
//	typedef typename DefaultTraits<Eigen::MatrixBase<TraitsDerived> >::parameter_type parameter_type;
//	static void asCSV(parameter_type source, std::ostream& os) {
//		os << source;
//	}
//};
//
//template<int R, int C, typename Units> struct Traits<math::Matrix<R,C, Units> > :
//		public Traits<Eigen::MatrixBase<typename math::Matrix<R,C, Units>::Base> > {};

template<int R, int C, typename Units> struct Traits<math::Matrix<R,C, Units> > :
		public DefaultTraits<math::Matrix<R,C, Units> > {

	typedef typename DefaultTraits<math::Matrix<R,C, Units> >::parameter_type parameter_type;

	static void asCSV(parameter_type source, std::ostream& os) {
		for (int i = 0; i < (source.size() - 1); ++i) {
			os << source[i] << ",";
		}
		os << source[source.size() - 1];
	}
};

template<
	typename T0, typename T1, typename T2, typename T3, typename T4,
	typename T5, typename T6, typename T7, typename T8, typename T9>
struct Traits<boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> > {

	typedef boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> tuple_type;
	static const size_t NUM_INPUTS = boost::tuples::length<tuple_type>::value;

	typedef detail::TupleTraitsHelper<NUM_INPUTS, Traits<tuple_type> > tuple_traits_helper;

	typedef const tuple_type& parameter_type;

	static size_t serializedLength() {
		return tuple_traits_helper::serializedLength();
	}

	static void serialize(parameter_type source, char* dest) {
		tuple_traits_helper::serialize(source, dest);
	}

	static tuple_type unserialize(char* source) {
		tuple_type t;
		tuple_traits_helper::unserialize(source, &t);
		return t;
	}

	static void asCSV(parameter_type source, std::ostream& os) {
		tuple_traits_helper::asCSV(source, os);
	}
};


}
}


#endif /* BARRETT_LOG_TRAITS_H_ */
