/*
 * math_utils.h
 *
 *  Created on: Oct 26, 2009
 *      Author: dc
 */

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_


#include <algorithm>
#include <cmath>


template<typename T>
inline T symLimit(const T x, const T limit)
{
	return sign(x) * std::min(std::abs(x), limit);
}

template<typename T>
inline int sign(const T x)
{
	if (x > 0) {
		return 1;
	} else if (x == 0) {
		return 0;
	} else {
		return -1;
	}
}

#endif /* MATH_UTILS_H_ */
