/*
 * supervisory_controller-inl.h
 *
 *  Created on: Oct 6, 2009
 *      Author: dc
 */

#include <list>
#include <stdexcept>

#include "../../detail/purge.h"
#include "../abstract/system.h"
#include "../abstract/abstract_controller.h"
#include "../abstract/joint_torque_adapter.h"
#include "../helpers.h"


// I think RTTI is useful here to keep syntax clean and support flexibility and
// extensibility. Also, Visitor and multiple-dispatch don't really fit in this
// usage.
// FIXME(dc): Is there a clean way to get this behavior without RTTI?

namespace barrett {
namespace systems {


inline SupervisoryController::~SupervisoryController() {
	purge(controllers);
	purge(adapters);
}

template<typename T>
void SupervisoryController::trackReferenceSignal(
		Output<T>& referenceOutput)
throw(std::invalid_argument)
{
	Input<T>* referenceInput = NULL;
	Input<T>* feedbackInput = NULL;

	Output<T>* feedbackOutput = NULL;

	// will throw if no controller is found
	AbstractController& controller = selectController(referenceOutput);

	// selectController guarantees these downcasts won't fail
	referenceInput = dynamic_cast<Input<T>*>(  //NOLINT: see RTTI note above
			controller.getReferenceInput() );
	feedbackInput = dynamic_cast<Input<T>*>(  //NOLINT: see RTTI note above
			controller.getFeedbackInput() );

	// see if we need to connect the feedbackInput
	if ( !feedbackInput->isConnected() ) {
		feedbackOutput = selectFeedbackSignal(*feedbackInput);
	} // else: assume that it's already been appropriately hooked up


	// only start connecting things once we know all connections will succeed

	// double-dispatch to find and connect adapter
	// will throw if no adapter is found
	controller.selectAndConnectAdapter(*this);
	forceConnect(referenceOutput, *referenceInput);
	if (feedbackOutput != NULL) {
		forceConnect(*feedbackOutput, *feedbackInput);
	}
}

// doesn't actually use referenceOutput, just it's type...
template<typename T>
AbstractController& SupervisoryController::selectController(
		const Output<T>& referenceOutput) const
throw(std::invalid_argument)
{
	Input<T>* input = NULL;

	typename std::list<AbstractController*>::const_iterator controllerItr;
	for (controllerItr = controllers.begin();
		 controllerItr != controllers.end();
		 ++controllerItr)
	{
		input = dynamic_cast<Input<T>*>(  //NOLINT: see RTTI note above
				(*controllerItr)->getReferenceInput() );

		if (input != NULL) {  // if the downcast was successful
			return *(*controllerItr);
		}
	}

	// if we couldn't find a controller that accepts that type of input...
	throw std::invalid_argument(
			"(Systems::SupervisoryController::selectController): "
			"No AbstractController is registered that accepts that type of "
			"referenceOutput.");
}

// second half of double-dispatch from controller.selectAndConnectAdapter()
// doesn't actually use controlOutput, just it's type...
template<typename T>
JointTorqueAdapter& SupervisoryController::selectAdapter(
		const Output<T>& controlOutput) const
throw(std::invalid_argument)
{
	Input<T>* input = NULL;

	typename std::list<JointTorqueAdapter*>::const_iterator adapterItr;
	for (adapterItr = adapters.begin();
		 adapterItr != adapters.end();
		 ++adapterItr)
	{
		input = dynamic_cast<Input<T>*>(  //NOLINT: see RTTI note above
				(*adapterItr)->getControlInput() );

		if (input != NULL) {  // if the downcast was successful
			return *(*adapterItr);
		}
	}

	// if we couldn't find an adapter that accepts that type of input...
	throw std::invalid_argument(
			"(Systems::SupervisoryController::selectAdapter): "
			"No JointTorqueAdapter is registered that accepts that type of "
			"controlOutput.");
}

// doesn't actually use feedbackInput, just it's type...
template<typename T>
System::Output<T>* SupervisoryController::selectFeedbackSignal(
		const Input<T>& feedbackInput) const
throw(std::invalid_argument)
{
	Output<T>* output = NULL;

	typename std::list<AbstractOutput*>::const_iterator outputItr;
	for (outputItr = feedbackOutputs.begin();
			outputItr != feedbackOutputs.end();
		 ++outputItr)
	{
		output = dynamic_cast<Output<T>*>(*outputItr);  //NOLINT: see RTTI note above

		if (output != NULL) {  // if the downcast was successful
			return output;
		}
	}

	// if we couldn't find a feedbackOutput of that type...
	throw std::invalid_argument(
			"(Systems::SupervisoryController::selectFeedbackSignal): "
			"No feedbackOutput is registered that matches the selected "
			"Controller's feedbackInput type");
}


}
}
