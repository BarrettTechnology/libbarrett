/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * tuple_splitter-helper.h
 *
 *  Created on: Nov 16, 2009
 *      Author: dc
 */


#include <boost/static_assert.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


template<
	typename T0, typename T1, typename T2, typename T3, typename T4,
	typename T5, typename T6, typename T7, typename T8, typename T9>
class TupleSplitter;


// doxygen can't handle OutputHolder's recursive inheritance.
#ifndef BARRETT_PARSED_BY_DOXYGEN
namespace detail {


template<size_t N,
	typename T0, typename T1, typename T2, typename T3, typename T4,
	typename T5, typename T6, typename T7, typename T8, typename T9>
struct OutputHolder :
		public OutputHolder<N-1, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> {

	typedef OutputHolder<N-1, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9>
			inherited_type;
	typedef TupleSplitter<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> ts_type;
	typedef typename ts_type::tuple_type tuple_type;
	typedef typename boost::tuples::element<N-1, tuple_type>::type data_type;

	explicit OutputHolder(ts_type* parent) :
		inherited_type(parent), output(parent, &outputValue) {}

	template<size_t Index>
	System::Output<typename boost::tuples::element<Index, tuple_type>::type>&
	getOutput() {
		BOOST_STATIC_ASSERT(Index < N);
		return ( static_cast<OutputHolder<  //NOLINT: lint doesn't know that these are templates
			Index+1, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9>*>(this) )->output;
	}

	void setData(const tuple_type* t) {
		inherited_type::setData(t);
		outputValue->setData( &(boost::get<N-1>(*t)) );
	}

	System::Output<data_type> output;

protected:
	typename System::Output<data_type>::Value* outputValue;
};

template<
	typename T0, typename T1, typename T2, typename T3, typename T4,
	typename T5, typename T6, typename T7, typename T8, typename T9>
struct OutputHolder<1, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> {

	typedef TupleSplitter<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> ts_type;
	typedef typename ts_type::tuple_type tuple_type;
	typedef T0 data_type;

	explicit OutputHolder(ts_type* parent) :
		output(parent, &outputValue) {}

	template<size_t Index>
	System::Output<data_type>& getOutput() {
		return output;
	}

	void setData(const tuple_type* t) {
		outputValue->setData( &(boost::get<0>(*t)) );
	}

	System::Output<data_type> output;

protected:
	typename System::Output<data_type>::Value* outputValue;
};



}
#endif // BARRETT_PARSED_BY_DOXYGEN
}
}
