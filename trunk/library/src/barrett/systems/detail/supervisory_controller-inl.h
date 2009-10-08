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
#include "../helpers.h"


namespace Systems {


inline SupervisoryController::~SupervisoryController() {
	purge(controllers);
}

template<typename T>
void SupervisoryController::trackReferenceSignal(
		System::Output<T>& referenceSignal)
throw(std::invalid_argument)
{
	System::Input<T>* input = NULL;

	typename std::list<AbstractController*>::iterator controllerItr;
	for (controllerItr = controllers.begin();
		 controllerItr != controllers.end();
		 ++controllerItr)
	{
		// I think RTTI is useful here to keep syntax clean.
		// Also, Visitor and multiple-dispatch don't really fit.
		// FIXME(dc): Is there a way to do this without RTTI?
		input = dynamic_cast<System::Input<T>*>(  //NOLINT: see RTTI note above
				(*controllerItr)->getReferenceInput() );

		if (input != NULL) {  // if the downcast was successful
			Systems::AbstractController& controller = *(*controllerItr);
			Systems::forceConnect(referenceSignal, *input);

			if ( !controller.getFeedbackInput()->isConnected() ) {
				// find a feedback input, connect it
			}  // else: assume that it's already been appropriately hooked up
			return;
		}
	}

	// if we couldn't find a Controller that accepts that type of input...
	throw std::invalid_argument(
			"(Systems::SupervisoryController::trackReferenceSignal): "
			"No AbstractController is registered that accepts that type of "
			"Input.");
}


}
