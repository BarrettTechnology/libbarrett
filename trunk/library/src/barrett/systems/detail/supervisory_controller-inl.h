/*
 * supervisory_controller-inl.h
 *
 *  Created on: Oct 6, 2009
 *      Author: dc
 */

#include <list>
#include <stdexcept>

//#include "../../detail/purge.h"
#include "../abstract/system.h"
#include "../abstract/abstract_controller.h"
#include "../abstract/joint_torque_adapter.h"
#include "../helpers.h"


// I think RTTI is useful here to keep syntax clean and support flexibility and
// extensibility. Also, Visitor and multiple-dispatch don't really fit in this
// usage.
// FIXME(dc): Is there a clean way to get this behavior without RTTI?

namespace Systems {


inline SupervisoryController::~SupervisoryController() {
	controllers.clear();
//	purge(controllers);
}

template<typename T>
void SupervisoryController::trackReferenceSignal(
		System::Output<T>& referenceOutput)
throw(std::invalid_argument)
{
	System::Input<T>* referenceInput = NULL;
	System::Input<T>* feedbackInput = NULL;

	System::Output<T>* feedbackOutput = NULL;

	Systems::AbstractController& controller =   // will throw if no
			selectController(referenceOutput);  // controller is found

	// selectController guarantees this downcast won't fail
	referenceInput = dynamic_cast<System::Input<T>*>(  //NOLINT: see RTTI note above
			controller.getReferenceInput() );
	Systems::forceConnect(referenceOutput, *referenceInput);

	feedbackInput = dynamic_cast<System::Input<T>*>(  //NOLINT: see RTTI note above
			controller.getFeedbackInput() );
	if ( !feedbackInput->isConnected() ) {
		// find a feedback input, connect it
		feedbackOutput = selectFeedbackSignal(*feedbackInput);
		Systems::forceConnect(*feedbackOutput, *feedbackInput);
	} // else: assume that it's already been appropriately hooked up

	controller.selectAdapter(adapters);
}

// doesn't actually use referenceOutput, just it's type...
template<typename T>
AbstractController& SupervisoryController::selectController(
		const System::Output<T>& referenceOutput) const
throw(std::invalid_argument)
{
	System::Input<T>* input = NULL;

	typename std::list<AbstractController*>::iterator controllerItr;
	for (controllerItr = controllers.begin();
		 controllerItr != controllers.end();
		 ++controllerItr)
	{
		input = dynamic_cast<System::Input<T>*>(  //NOLINT: see RTTI note above
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

// doesn't actually use controlOutput, just it's type...
template<typename T>
JointTorqueAdapter& SupervisoryController::selectAdapter(
		const System::Output<T>& controlOutput) const
throw(std::invalid_argument)
{
	System::Input<T>* input = NULL;

	typename std::list<JointTorqueAdapter*>::iterator adapterItr;
	for (adapterItr = adapters.begin();
		 adapterItr != adapters.end();
		 ++adapterItr)
	{
		input = dynamic_cast<System::Input<T>*>(  //NOLINT: see RTTI note above
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

// TODO(dc): implement this method
template<typename T>
System::Output<T>* SupervisoryController::selectFeedbackSignal(
		const System::Input<T>& feedbackInput) const
throw(std::invalid_argument)
{
	throw std::invalid_argument(
			"(Systems::SupervisoryController::trackReferenceSignal): "
			"A Controller's feedbackInput must be connected before you "
			"can use it.");
}


}
