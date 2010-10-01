/*
 * system-input-inl.h
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <vector>

#include <barrett/detail/stl_utils.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/systems/helpers.h>


namespace barrett {
namespace systems {


inline System::AbstractInput::AbstractInput(System* parentSys) :
	parentSystem(parentSys)
{
	SCOPED_LOCK(getEmMutex());

	parentSystem->inputs.push_back(this);
}

inline System::AbstractInput::~AbstractInput()
{
	SCOPED_LOCK(getEmMutex());

	if (parentSystem != NULL) {
		replaceWithNull(parentSystem->inputs, this);
	}
}

inline thread::Mutex& System::AbstractInput::getEmMutex()
{
	if (parentSystem != NULL) {
		return parentSystem->getEmMutex();
	} else {
		return thread::NullMutex::aNullMutex;
	}
}


template<typename T>
inline System::Input<T>::~Input()
{
	SCOPED_LOCK(getEmMutex());

	if (isConnected()) {
		disconnect(*this);
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
