/*
 * math_utils.h
 *
 *  Created on: Oct 26, 2009
 *      Author: dc
 */

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_


namespace barrett {


template<typename T> T sign(const T& x);
template<> double sign(const double& x);

template<typename T> T abs(const T& x);

template<typename T> T min(const T& a, const T& b);
template<> double min(const double& a, const double& b);

template<typename T> T max(const T& a, const T& b);
template<> double max(const double& a, const double& b);  //NOLINT: irrelevant

template<typename T> T saturate(const T& x, const T& limit);
//template<typename T> T saturate(const T& x,
//		const T& lowerLimit, const T& upperLimit);
template<typename T> T deadband(const T& x, const T& cutoff);
//template<> double deadBand(const double& x, const double& cutoff);
//template<typename T> T deadBand(const T& x,
//		const T& lowerCutoff, const T& upperCutoff);


}


// include template definitions
#include "./detail/math_utils-inl.h"


#endif /* MATH_UTILS_H_ */
