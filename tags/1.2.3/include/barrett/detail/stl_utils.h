/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */

/**
 * @file stl_utils.h
 * @author Dan Cody
 * @date 10/05/2009
 *  
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
