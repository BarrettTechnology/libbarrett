/*
 * supervisory_controller-inl.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */


#include <utility>
#include <list>
#include <stdexcept>

#include "../../detail/purge.h"
#include "../abstract/system.h"
#include "../helpers.h"
#include "../abstract/supervisory_controllable.h"
#include "../supervisory_controller.h"


// I think RTTI is useful here to keep syntax clean and support flexibility and
// extensibility. Also, Visitor and multiple-dispatch don't really fit in this
// usage.
// FIXME(dc): Is there a clean way to get this behavior without RTTI?

namespace barrett {
namespace systems {

template<typename OutputType>
inline SupervisoryController<OutputType>::~SupervisoryController()
{
	purge(controllables);
}

template<typename OutputType>
inline void SupervisoryController<OutputType>::registerControllable(
		SupervisoryControllable<OutputType>* controllable)
{
	controllables.push_front(controllable);
}

template<typename OutputType>
template<typename T>
void SupervisoryController<OutputType>::trackReferenceSignal(
		System::Output<T>& referenceOutput)
throw(std::invalid_argument)
{
	typename std::list<SupervisoryControllable<OutputType>*>::iterator i;
	for (i = controllables.begin(); i != controllables.end(); ++i) {
		if (trackReferenceSignal(referenceOutput, *i)) {
			return;
		}
	}

	throw std::invalid_argument(
			"(systems::SupervisoryController::trackReferenceSignal): "
			"No SupervisoryControllable is registered that matches "
			"referenceOutput's type");
}

template<typename OutputType>
template<typename T>
bool SupervisoryController<OutputType>::trackReferenceSignal(
		System::Output<T>& referenceOutput,
		SupervisoryControllable<OutputType>* controllable)
//throw(std::invalid_argument)
{
	System::Input<T>* referenceInput = dynamic_cast<Input<T>*>(  //NOLINT: see RTTI note above
			controllable->getSupervisoryControllableInput() );

	if (referenceInput == NULL) {
		return false;
	}

	forceConnect(referenceOutput, *referenceInput);
	forceConnect(controllable->getSupervisoryControllableOutput(),
			shaddowInput);

	return true;
}


template<typename OutputType>
void SupervisoryController<OutputType>::operate()
{
	outputValue->setValue(shaddowInput.getValue());
//	this->outputValue->setValue(this->shaddowInput.getValue());
}


}
}
