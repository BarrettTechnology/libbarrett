/*
 * array_splitter.h
 *
 *  Created on: Nov 17, 2009
 *      Author: dc
 */

#ifndef ARRAY_SPLITTER_H_
#define ARRAY_SPLITTER_H_


#include <boost/array.hpp>

#include "../detail/ca_macro.h"
#include "./abstract/system.h"
#include "./abstract/single_io.h"


namespace barrett {
namespace systems {

// TODO(dc): test!

template <typename T>
class ArraySplitter : public System, public SingleInput<T> {
// IO
// protected because of variable number of outputs
protected:	boost::array<Output<double>*, T::SIZE> outputs;
protected:	boost::array<Output<double>::Value*, T::SIZE> outputValues;


public:
	ArraySplitter();
	virtual ~ArraySplitter();

	Output<double>& getOutput(const size_t i);

protected:
	void initOutputs();
	virtual void operate();

private:
	DISALLOW_COPY_AND_ASSIGN(ArraySplitter);
};


}
}


// include template definitions
#include "./detail/array_splitter-inl.h"



#endif /* ARRAY_SPLITTER_H_ */
