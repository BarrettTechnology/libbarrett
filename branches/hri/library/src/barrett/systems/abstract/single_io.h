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


template<typename InputType>
class SingleInput {  // not a System in order to avoid diamond inheritance
// IO
public:	System::Input<InputType> input;


public:
	explicit SingleInput(System* parentSys) :
		input(parentSys) {}

private:
	DISALLOW_COPY_AND_ASSIGN(SingleInput);
};


template<typename OutputType>
class SingleOutput {  // not a System in order to avoid diamond inheritance
// IO
public:		System::Output<OutputType> output;
protected:	typename System::Output<OutputType>::Value* outputValue;


public:
	explicit SingleOutput(System* parentSys) :
		output(parentSys, &outputValue) {}

private:
	DISALLOW_COPY_AND_ASSIGN(SingleOutput);
};


template<typename InputType, typename OutputType>
class SingleIO :	public System, public SingleInput<InputType>,
					public SingleOutput<OutputType>,
					public Conversion<OutputType> {
public:
	explicit SingleIO(bool updateEveryExecutionCycle = false) :
		System(updateEveryExecutionCycle),
		SingleInput<InputType>(this), SingleOutput<OutputType>(this) {}
	virtual ~SingleIO() {}

	virtual System::Input<InputType>* getConversionInput() {
		return &(this->input);
	}
	virtual System::Output<OutputType>& getConversionOutput() {
		return this->output;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(SingleIO);
};


}
}


#endif /* SINGLE_IO_H_ */
