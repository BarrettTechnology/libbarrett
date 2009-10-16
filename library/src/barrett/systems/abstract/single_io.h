/*
 * SingleIO.h
 *
 *  Created on: Sep 12, 2009
 *      Author: dc
 */

#ifndef SINGLE_IO_H_
#define SINGLE_IO_H_


#include "./system.h"
#include "../../detail/ca_macro.h"


namespace Systems {


template<typename InputType, typename OutputType>
class SingleIO : public System {
// IO
public:		Input<InputType> input;
public:		Output<OutputType> output;
protected:	typename Output<OutputType>::Value* outputValue;


public:
	SingleIO() :
		input(this), output(&outputValue) {}
	explicit SingleIO(const OutputType& initialOutputValue) :
		input(this), output(initialOutputValue, &outputValue) {}
	virtual ~SingleIO() {}

private:
	DISALLOW_COPY_AND_ASSIGN(SingleIO);
};


}


#endif /* SINGLE_IO_H_ */
