/*
 * system::input.tcc
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */

#include <stdexcept>

namespace barrett {
namespace systems {


// the compiler requires a definition for a dtor, even if it's pure virtual
inline System::AbstractInput::~AbstractInput() {}


// TODO(dc): add tests for this
template<typename T>
inline bool System::Input<T>::isConnected() const {
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
	parent.operate();
}


}
}
