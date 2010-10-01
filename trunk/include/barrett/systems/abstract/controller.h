/*
 * controller.h
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_ABSTRACT_CONTROLLER_H_
#define BARRETT_SYSTEMS_ABSTRACT_CONTROLLER_H_


#include <list>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/conversion.h>


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
	explicit Controller(bool updateEveryExecutionCycle = false) :
		System(updateEveryExecutionCycle),
		referenceInput(this),
		feedbackInput(this),
		controlOutput(this, &controlOutputValue) {}
	virtual ~Controller() {}

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


#endif /* BARRETT_SYSTEMS_ABSTRACT_CONTROLLER_H_ */
