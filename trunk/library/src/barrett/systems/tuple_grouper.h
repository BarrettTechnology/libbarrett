/*
 * tuple_grouper.h
 *
 *  Created on: Nov 16, 2009
 *      Author: dc
 */

#ifndef TUPLE_GROUPER_H_
#define TUPLE_GROUPER_H_


//#include <boost/mpl/if.hpp>
#include <boost/tuple/tuple.hpp>
#include "abstract/system.h"
#include "detail/tuple_grouper-helper.h"


namespace barrett {
namespace systems {
//using boost::tuples::null_type;

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
class TupleGrouper : public System {
public:
	typedef boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> tuple_type;
	static const size_t NUM_INPUTS = boost::tuples::length<tuple_type>::value;
//	typedef boost::tuple<
//		boost::mpl::if_c< (0 < NUM_INPUTS), Input<T0>, boost::tuples::null_type>,
//		boost::mpl::if_c< (1 < NUM_INPUTS), Input<T1>, boost::tuples::null_type>,
//		boost::mpl::if_c< (2 < NUM_INPUTS), Input<T2>, boost::tuples::null_type>,
//		boost::mpl::if_c< (3 < NUM_INPUTS), Input<T3>, boost::tuples::null_type>,
//		boost::mpl::if_c< (4 < NUM_INPUTS), Input<T4>, boost::tuples::null_type>,
//		boost::mpl::if_c< (5 < NUM_INPUTS), Input<T5>, boost::tuples::null_type>,
//		boost::mpl::if_c< (6 < NUM_INPUTS), Input<T6>, boost::tuples::null_type>,
//		boost::mpl::if_c< (7 < NUM_INPUTS), Input<T7>, boost::tuples::null_type>,
//		boost::mpl::if_c< (8 < NUM_INPUTS), Input<T8>, boost::tuples::null_type>,
//		boost::mpl::if_c< (9 < NUM_INPUTS), Input<T9>, boost::tuples::null_type> >
//			input_tuple_type;


// IO
private:	detail::InputHolder<
				NUM_INPUTS, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> inputs;
//			input_tuple_type inputsTuple;

public:		Output<tuple_type> output;
protected:	typename Output<tuple_type>::Value* outputValue;


public:
	TupleGrouper() :
		inputs(this), output(&outputValue) {}
	explicit TupleGrouper(const tuple_type& initialOutputValue) :
		inputs(this), output(initialOutputValue, &outputValue) {}
	virtual ~TupleGrouper() {}

	template<size_t N>
	Input<typename boost::tuples::element<N, tuple_type>::type >& getInput() {
//		return ( static_cast<detail::InputHolder<
//			N+1, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9>* >(&inputs) )->input;
		return inputs.getInput<N>();
	}

protected:
	virtual void operate() {
		if (inputs.valuesDefined()) {
//			if (0 < NUM_INPUTS)
//				getInput<0>().getValue();
//			if (1 < NUM_INPUTS)
//				getInput<1>().getValue();
//			if (2 < NUM_INPUTS)
//				getInput<2>().getValue();
//			if (3 < NUM_INPUTS)
//				getInput<3>().getValue();
//			if (4 < NUM_INPUTS)
//				getInput<4>().getValue();
//			if (5 < NUM_INPUTS)
//				getInput<5>().getValue();
//			if (6 < NUM_INPUTS)
//				getInput<6>().getValue();
//			if (7 < NUM_INPUTS)
//				getInput<7>().getValue();
//			if (8 < NUM_INPUTS)
//				getInput<8>().getValue();
//			if (9 < NUM_INPUTS)
//				getInput<9>().getValue();
//			tuple_type()
			outputValue->setValue(inputs.getValues());
		} else {
			outputValue->setValueUndefined();
		}
	}
};


}
}


#endif /* TUPLE_GROUPER_H_ */
