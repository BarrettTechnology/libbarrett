/*
 * SingleIO.h
 *
 *  Created on: Sep 12, 2009
 *      Author: dc
 */

#ifndef SINGLE_IO_H_
#define SINGLE_IO_H_


#include "../../detail/ca_macro.h"
#include "./system.h"
#include "./conversion.h"


namespace barrett {
namespace systems {


template<typename InputType, typename OutputType>
class SingleIO : public System, public Conversion<OutputType> {
// IO
public:		Input<InputType> input;
public:		Output<OutputType> output;
protected:	typename Output<OutputType>::Value* outputValue;


public:
	explicit SingleIO(bool updateEveryExecutionCycle = false) :
		System(updateEveryExecutionCycle),
		input(this), output(this, &outputValue) {}
	virtual ~SingleIO() {}

	virtual System::Input<InputType>* getConversionInput() {
		return &input;
	}
	virtual System::Output<OutputType>& getConversionOutput() {
		return output;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(SingleIO);
};


}
}


#endif /* SINGLE_IO_H_ */
