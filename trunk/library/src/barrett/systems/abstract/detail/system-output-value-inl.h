/*
 * system-output-value.tcc
 *
 *  Created on: Sep 13, 2009
 *      Author: dc
 */


namespace barrett {
namespace systems {


template<typename T>
inline System::Output<T>::Value::~Value()
{
	delete value;  // delete NULL does nothing
}

template<typename T>
inline void System::Output<T>::Value::setValue(const T& newValue) {
	if (value == NULL) {
		value = new T(newValue);
	} else {
		(*value) = newValue;
	}

	parent.notifyInputs();
}

template<typename T>
inline void System::Output<T>::Value::setValueUndefined() {
	if (value != NULL) {
		delete value;
		value = NULL;
	}

	parent.notifyInputs();
}


}
}
