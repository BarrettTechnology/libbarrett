/*
 * system-input-inl.h
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */

#include <stdexcept>
#include <algorithm>

namespace barrett {
namespace systems {


inline System::AbstractInput::AbstractInput(System* parentSys) :
	parent(*parentSys)
{
	parent.inputs.push_back(this);
}

inline System::AbstractInput::~AbstractInput()
{
	std::replace(parent.inputs.begin(), parent.inputs.end(),
			const_cast<AbstractInput*>(this),
			static_cast<AbstractInput*>(NULL));
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
	if (output != NULL) {
		const typename Output<T>::Value* valueObj = output->getValueObject();
		if (valueObj->value != NULL) {
			return true;
		}
	}

	return false;
}

template<typename T>
inline const T& System::Input<T>::getValue() const
throw(std::logic_error, System::Input<T>::ValueUndefinedError)
{
	if (output == NULL) {
		throw std::logic_error("(systems::System::Input::getValue): "
		                       "Input is not connected to anything. "
		                       "Cannot retrieve value.");
	}

	const typename Output<T>::Value* valueObj = output->getValueObject();
	if (valueObj->value == NULL) {
		throw ValueUndefinedError("(systems::System::Input::getValue): "
		                          "The value of the associated output is "
		                          "undefined.");
	}

	return *(valueObj->value);
}

template<typename T>
inline void System::Input<T>::onValueChanged() const
{
	// TODO(dc): test!
	if (parent.inputsValid()) {
		parent.operate();
	}
}


}
}
