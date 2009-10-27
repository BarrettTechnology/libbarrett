/*
 * supervisory_controller.h
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#ifndef SUPERVISORY_CONTROLLER_H_
#define SUPERVISORY_CONTROLLER_H_


#include <list>
#include <stdexcept>

#include "../detail/ca_macro.h"
#include "../units.h"
#include "./abstract/system.h"
#include "./abstract/abstract_controller.h"
#include "./abstract/joint_torque_adapter.h"


namespace barrett {
namespace systems {


// TODO(dc): need a way to disambiguate Controllers with the same
// Input/Output types. Boost::Units and/or methods for specifying specific
// Controllers and JointTorqueAdapters?
class SupervisoryController : public System {
// IO
public:		Input<units::JointTorques> input;  // TODO(dc): ugly ugly! :(
protected:	Output<units::JointTorques>::Value* outputValue;
public:		Output<units::JointTorques> output;


public:
	SupervisoryController(
//			const std::list<AbstractController*>& additionalControllers =
//					std::list<AbstractController*>(),
			bool includeStandardControllers = true,
			bool includeStandardAdapters = true);
	~SupervisoryController();

	// TODO(dc): need sister functions single/lists of controllers, adapters
	void addFeedbackSignal(AbstractOutput* feedbackOutput);

	template<typename T>
	void trackReferenceSignal(Output<T>& referenceOutput)  //NOLINT: non-const reference for syntax
	throw(std::invalid_argument);

	// FIXME: should this be public?
	template<typename T>
	AbstractController& selectController(
			const Output<T>& referenceOutput) const
	throw(std::invalid_argument);

	// FIXME: should this be public?
	template<typename T>
	JointTorqueAdapter& selectAdapter(
			const Output<T>& controlOutput) const
	throw(std::invalid_argument);

	// FIXME: should this be public?
	template<typename T>
	Output<T>* selectFeedbackSignal(
			const Input<T>& feedbackInput) const
	throw(std::invalid_argument);

protected:
	// the SupervisoryController owns the objects pointed to by these lists
	std::list<AbstractController*> controllers;
	std::list<JointTorqueAdapter*> adapters;

	// the SupervisoryController does not own these objects
	std::list<AbstractOutput*> feedbackOutputs;

	// TODO(dc): ugly ugly :(
	virtual void operate() {
		if (input.valueDefined()) {
			outputValue->setValue(input.getValue());
		} else {
			outputValue->setValueUndefined();
		}
	}

private:
	DISALLOW_COPY_AND_ASSIGN(SupervisoryController);
};


}
}


// include inline definitions
#include "detail/supervisory_controller-inl.h"


#endif /* SUPERVISORY_CONTROLLER_H_ */
