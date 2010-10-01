/*
 * tuple_grouper.h
 *
 *  Created on: Nov 16, 2009
 *      Author: dc
 */

#ifndef TUPLE_GROUPER_H_
#define TUPLE_GROUPER_H_


#include <boost/tuple/tuple.hpp>
#include "./abstract/system.h"
#include "./abstract/single_io.h"
#include "./detail/tuple_grouper-helper.h"


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
	TupleGrouper() :
		SingleOutput<tuple_type>(this), inputs(this) {}
	virtual ~TupleGrouper() {}

	template<size_t N>
	Input<typename boost::tuples::element<N, tuple_type>::type >& getInput() {
		return inputs.getInput<N>();
	}

protected:
	virtual void operate() {
		this->outputValue->setValue(inputs.getValues());
	}
};


}
}


#endif /* TUPLE_GROUPER_H_ */
