/*
 * purge.h
 *
 *  Created on: Oct 5, 2009
 *      Author: dc
 */

#ifndef PURGE_H_
#define PURGE_H_

// Delete pointers in an STL sequence container.
//
// Code stolen from Thinking in C++, 2nd Ed., Vol. 2, p.534

#include <algorithm>

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

#endif /* PURGE_H_ */
