/*
 * stl_utils.h
 *
 *  Created on: Oct 5, 2009
 *      Author: dc
 */

#ifndef BARRETT_DETAIL_STL_UTILS_H_
#define BARRETT_DETAIL_STL_UTILS_H_


#include <algorithm>


namespace barrett {
namespace detail {


void waitForEnter();

template<typename Container>
inline void replaceWithNull(Container& container, typename Container::const_reference value)
{
	std::replace(container.begin(), container.end(),
			const_cast<typename Container::value_type&>(value),
			static_cast<typename Container::value_type>(NULL));
}


// Delete pointers in an STL sequence container.
//
// Code stolen from Thinking in C++, 2nd Ed., Vol. 2, p.534

template<class Seq> void purge(Seq& c) {  //NOLINT
	typename Seq::iterator i;
	for (i = c.begin(); i != c.end(); ++i) {
		delete *i;
		*i = 0;
	}
}

// Iterator version:
template<class InpIt> void purge(InpIt begin, InpIt end) {
	while (begin != end) {
		delete *begin;
		*begin = 0;
		++begin;
	}
}


}
}


#endif /* BARRETT_DETAIL_STL_UTILS_H_ */
