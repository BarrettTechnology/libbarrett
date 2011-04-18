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
 * tuple_grouper.h
 *
 *  Created on: Nov 16, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_TUPLE_GROUPER_H_
#define BARRETT_SYSTEMS_TUPLE_GROUPER_H_


#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>  // For aligned operator new

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/detail/tuple_grouper-helper.h>


namespace barrett {
namespace systems {


template <
	typename T0 = boost::tuples::null_type,
	typename T1 = boost::tuples::null_type,
	typename T2 = boost::tuples::null_type,
	typename T3 = boost::tuples::null_type,
	typename T4 = boost::tuples::null_type,
	typename T5 = boost::tuples::null_type,
	typename T6 = boost::tuples::null_type,
	typename T7 = boost::tuples::null_type,
	typename T8 = boost::tuples::null_type,
	typename T9 = boost::tuples::null_type>
class TupleGrouper : public System,
					 public SingleOutput<
						 boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> > {
public:
	typedef boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> tuple_type;
	static const size_t NUM_INPUTS = boost::tuples::length<tuple_type>::value;


// IO
private:	detail::InputHolder<
				NUM_INPUTS, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> inputs;


public:
	TupleGrouper(const std::string& sysName = "TupleGrouper") :
		System(sysName), SingleOutput<tuple_type>(this), inputs(this) {}
	virtual ~TupleGrouper() { mandatoryCleanUp(); }

	template<size_t N>
	Input<typename boost::tuples::element<N, tuple_type>::type >& getInput() {
		return inputs.getInput<N>();
	}

protected:
	virtual void operate() {
		this->outputValue->setData( &(inputs.getValues()) );
	}

private:
	DISALLOW_COPY_AND_ASSIGN(TupleGrouper);

public:
	// To be safe, assume that at least one of the input types needs to be aligned.
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_TUPLE_GROUPER_H_ */
