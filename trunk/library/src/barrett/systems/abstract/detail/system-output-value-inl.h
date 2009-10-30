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
//	undelegate();  // TODO(dc): this line breaks things :(
	delete value;  // delete NULL does nothing
}

template<typename T>
inline void System::Output<T>::Value::setValue(const T& newValue) {
	undelegate();

	if (value == NULL) {
		value = new T(newValue);
	} else {
		(*value) = newValue;
	}

	parent.notifyListeners();
}

template<typename T>
inline void System::Output<T>::Value::setValueUndefined() {
	undelegate();

	if (value != NULL) {
		delete value;
		value = NULL;
	}

	parent.notifyListeners();
}

template<typename T>
inline void
System::Output<T>::Value::delegateTo(const Output<T>& delegateOutput)
{
	undelegate();
	delegate = &(delegateOutput.value);
	delegate->parent.addDelegator(parent);
}

template<typename T>
inline void System::Output<T>::Value::undelegate()
{
	if (delegate != NULL) {
		delegate->parent.removeDelegator(parent);
		delegate = NULL;
	}
}


}
}
