/*
 * math_utils-inl.h
 *
 *  Created on: Nov 11, 2009
 *      Author: dc
 */


#include <algorithm>
#include <cmath>


namespace barrett {
using std::abs;


template<typename T>
inline T sign(const T& x)
{
	T res;
	for (size_t i = 0; i < T::SIZE; ++i) {
		res[i] = sign(x[i]);
	}
	return res;
}

template<>
inline double sign(const double& x)
{
	if (x > 0.0) {
		return 1.0;
	} else if (x == 0.0) {
		return 0.0;
	} else {
		return -1.0;
	}
}


template<typename T>
inline T abs(const T& x)
{
	T res;
	for (size_t i = 0; i < T::SIZE; ++i) {
		res[i] = abs(x[i]);
	}
	return res;
}


template<typename T>
inline T min(const T& a, const T& b)
{
	T res;
	for (size_t i = 0; i < T::SIZE; ++i) {
		res[i] = min(a[i], b[i]);
	}
	return res;
}

template<>
inline double min(const double& a, const double& b)
{
	return std::min(a, b);
}


template<typename T>
inline T max(const T& a, const T& b)
{
	T res;
	for (size_t i = 0; i < T::SIZE; ++i) {
		res[i] = max(a[i], b[i]);
	}
	return res;
}

template<>
inline double max(const double& a, const double& b)
{
	return std::max(a, b);
}


template<typename T>
inline T symLimit(const T x, const T limit)
{
	return sign(x) * min(abs(x), limit);
}


}
