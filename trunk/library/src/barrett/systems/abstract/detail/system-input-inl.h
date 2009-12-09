/*
 * system-input-inl.h
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <algorithm>
#include <vector>


namespace barrett {
namespace systems {


inline System::AbstractInput::AbstractInput(System* parentSys) :
	parentSystem(parentSys)
{
	parentSystem->inputs.push_back(this);
}

inline System::AbstractInput::~AbstractInput()
{
	if (parentSystem != NULL) {
		std::replace(parentSystem->inputs.begin(), parentSystem->inputs.end(),
				const_cast<AbstractInput*>(this),
				static_cast<AbstractInput*>(NULL));
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
		outputValue.refreshValue();
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

	// valueDefined() calls Output<T>::Value::refreshValue() for us
	if ( !valueDefined() ) {
		throw std::logic_error("(systems::System::Input::getValue): "
		                          "The value of the associated output is "
		                          "undefined.");
	}

	return *(output->getValueObject()->value);
}


}
}
