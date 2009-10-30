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


namespace barrett {
namespace systems {


// FIXME(dc): do we need a superclass for Controller<>'s any more?
class AbstractController {
public:
	AbstractController() {}
	virtual ~AbstractController() {}

	virtual System::AbstractInput* getReferenceInput() = 0;
	virtual System::AbstractInput* getFeedbackInput() = 0;
	virtual System::AbstractOutput* getControlOutput() = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(AbstractController);
};


template<typename InputType, typename OutputType = InputType>
class Controller : public System, public AbstractController {
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

	virtual Input<InputType>* getReferenceInput();
	virtual Input<InputType>* getFeedbackInput();
	virtual Output<OutputType>* getControlOutput();

private:
	DISALLOW_COPY_AND_ASSIGN(Controller);
};


}
}


// include template definitions
#include "./detail/controller-inl.h"


#endif /* CONTROLLER_H_ */
