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
 * tuple_splitter.h
 *
 *  Created on: Feb 2, 2011
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_TUPLE_SPLITTER_H_
#define BARRETT_SYSTEMS_TUPLE_SPLITTER_H_


#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>

#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/detail/tuple_splitter-helper.h>


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
class TupleSplitter : public System,
					 public SingleInput<
						 boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> > {
public:
	typedef boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> tuple_type;
	static const size_t NUM_OUTPUTS = boost::tuples::length<tuple_type>::value;


// IO
private:	detail::OutputHolder<
				NUM_OUTPUTS, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> outputs;


public:
	TupleSplitter(const std::string& sysName = "TupleSplitter") :
		System(sysName), SingleInput<tuple_type>(this), outputs(this) {}
	virtual ~TupleSplitter() { mandatoryCleanUp(); }

	template<size_t N>
	Output<typename boost::tuples::element<N, tuple_type>::type >& getOutput() {
		return outputs.getOutput<N>();
	}

protected:
	virtual void operate() {
		outputs.setData( &(this->input.getValue()) );
	}

private:
	DISALLOW_COPY_AND_ASSIGN(TupleSplitter);

public:
	// To be safe, assume that at least one of the input types needs to be aligned.
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_TUPLE_SPLITTER_H_ */
