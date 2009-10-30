/*
 * controller.h
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_


#include <list>

#include "../../detail/ca_macro.h"
#include "./system.h"
#include "./conversion.h"


namespace barrett {
namespace systems {


template<typename InputType, typename OutputType = InputType>
class Controller : public System, public Conversion<OutputType> {
// IO
public:		Input<InputType> referenceInput;
public:		Input<InputType> feedbackInput;
public:		Output<OutputType> controlOutput;
protected:	typename Output<OutputType>::Value* controlOutputValue;


public:
	Controller() :
		referenceInput(this),
		feedbackInput(this),
		controlOutput(&controlOutputValue) {}
	explicit Controller(const OutputType& initialOutputValue) :
		referenceInput(this),
		feedbackInput(this),
		controlOutput(initialOutputValue, &controlOutputValue) {}

	virtual System::Input<InputType>* getConversionInput() {
		return &referenceInput;
	}
	virtual System::Output<OutputType>& getConversionOutput() {
		return controlOutput;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Controller);
};


}
}


#endif /* CONTROLLER_H_ */