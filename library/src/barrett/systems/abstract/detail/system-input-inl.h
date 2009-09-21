/*
 * system::input.tcc
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */

// #include <cassert>
#include <stdexcept>

namespace Systems {


template<typename T>
inline bool System::Input<T>::valueDefined() const
{
	if (output != NULL  &&  output->value.value != NULL) {
		return true;
	} else {
		return false;
	}
}

template<typename T>
inline const T& System::Input<T>::getValue() const
throw(std::logic_error, System::Input<T>::ValueUndefinedError)
{
	if (output == NULL) {
		throw std::logic_error("(Systems::System::Input::getValue): "
		                       "Input is not connected to anything. "
		                       "Cannot retrieve value.");
	}
	if (output->value.value == NULL) {
		throw ValueUndefinedError("(Systems::System::Input::getValue): "
		                          "The value of the associated output is "
		                          "undefined.");
	}

	return *(output->value.value);
}

template<typename T>
inline void System::Input<T>::onValueChanged() const
{
	parent.operate();
}


}
