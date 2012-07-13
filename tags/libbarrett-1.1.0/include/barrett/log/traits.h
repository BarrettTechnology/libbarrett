/*
 * traits.h
 *
 *  Created on: Dec 29, 2009
 *      Author: dc
 */

#ifndef BARRETT_LOG_TRAITS_H_
#define BARRETT_LOG_TRAITS_H_


#include <ostream>
#include <cstring>

#include <boost/tuple/tuple.hpp>
#include <boost/array.hpp>
#include <Eigen/Geometry>

#include <barrett/math/matrix.h>
#include <barrett/log/detail/traits-helper.h>


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


template<typename T> struct PODTraits {
	typedef const T& parameter_type;

	static size_t serializedLength() {
		return sizeof(T);
	}

	static void serialize(parameter_type source, char* dest) {
		std::memcpy(dest, &source, serializedLength());
	}

	static T unserialize(char* source) {
		return *reinterpret_cast<T*>(source);
	}

	static void asCSV(parameter_type source, std::ostream& os) {
		os << source;
	}
};

template<> struct Traits<bool>					: public PODTraits<bool> {};
template<> struct Traits<float>					: public PODTraits<float> {};
template<> struct Traits<double>				: public PODTraits<double> {};
template<> struct Traits<signed char>			: public PODTraits<signed char> {};
template<> struct Traits<short>					: public PODTraits<short> {};
template<> struct Traits<int>					: public PODTraits<int> {};
template<> struct Traits<long>					: public PODTraits<long> {};
template<> struct Traits<long long>				: public PODTraits<long long> {};
template<> struct Traits<unsigned char>			: public PODTraits<unsigned char> {};
template<> struct Traits<unsigned short>		: public PODTraits<unsigned short> {};
template<> struct Traits<unsigned int>			: public PODTraits<unsigned int> {};
template<> struct Traits<unsigned long>			: public PODTraits<unsigned long> {};
template<> struct Traits<unsigned long long>	: public PODTraits<unsigned long long> {};


template<typename T, size_t N> struct Traits< ::boost::array<T,N> > {
	typedef const ::boost::array<T,N>& parameter_type;

	static size_t serializedLength() {
		return N * Traits<T>::serializedLength();
	}

	static void serialize(parameter_type source, char* dest) {
		for (size_t i = 0; i < N; ++i) {
			Traits<T>::serialize(source[i], dest);
			dest += Traits<T>::serializedLength();
		}
	}

	static ::boost::array<T,N> unserialize(char* source) {
		::boost::array<T,N> dest;
		for (size_t i = 0; i < N; ++i) {
			dest[i] = Traits<T>::unserialize(source);
			source += Traits<T>::serializedLength();
		}
		return dest;
	}

	static void asCSV(parameter_type source, std::ostream& os) {
		detail::arrayAsCSV(os, source, N);
	}
};


template<typename Scalar> struct Traits<Eigen::Quaternion<Scalar> > {
	typedef Eigen::Quaternion<Scalar> T;
	typedef const T& parameter_type;

	static size_t serializedLength() {
		return sizeof(Scalar) * 4;
	}

	static void serialize(parameter_type source, char* dest) {
		std::memcpy(dest, source.coeffs().data(), serializedLength());
	}

	static T unserialize(char* source) {
		T q;
		std::memcpy(q.coeffs().data(), source, serializedLength());
		return q;
	}

	static void asCSV(parameter_type source, std::ostream& os) {
		os << source.w() << "," << source.x() << "," << source.y() << "," << source.z();
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
		detail::arrayAsCSV(os, source, source.size());
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
