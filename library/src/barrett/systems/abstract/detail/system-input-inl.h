/*
 * system-input-inl.h
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <vector>

#include "../../../detail/stl_utils.h"


namespace barrett {
namespace systems {


inline System::AbstractInput::AbstractInput(System* parentSys) :
	parentSystem(parentSys)
{
	lockExecutionManager();

	parentSystem->inputs.push_back(this);

	unlockExecutionManager();
}

inline System::AbstractInput::~AbstractInput()
{
	lockExecutionManager();

	if (parentSystem != NULL) {
		replaceWithNull(parentSystem->inputs, this);
	}

	unlockExecutionManager();
}

inline void System::AbstractInput::lockExecutionManager()
{
	if (parentSystem != NULL) {
		parentSystem->lockExecutionManager();
	}
}

inline void System::AbstractInput::unlockExecutionManager()
{
	if (parentSystem != NULL) {
		parentSystem->unlockExecutionManager();
	}
}


// TODO(dc): add tests for this
template<typename T>
inline bool System::Input<T>::isConnected() const
{
	return output != NULL;
}

template<typename T>
inline bool System::Input<T>::valueDefined() const
{
	if (isConnected()) {
		typename Output<T>::Value& outputValue = *(output->getValueObject());
		outputValue.updateValue();
		if (outputValue.value != NULL) {
			return true;
		}
	}

	return false;
}

template<typename T>
inline const T& System::Input<T>::getValue() const
throw(std::logic_error)
{
	if ( !isConnected() ) {
		throw std::logic_error("(systems::System::Input::getValue): "
		                       "Input is not connected to anything. "
		                       "Cannot retrieve value.");
	}

	// valueDefined() calls Output<T>::Value::updateValue() for us
	if ( !valueDefined() ) {
		throw std::logic_error("(systems::System::Input::getValue): "
		                          "The value of the associated output is "
		                          "undefined.");
	}

	return *(output->getValueObject()->value);
}


}
}
