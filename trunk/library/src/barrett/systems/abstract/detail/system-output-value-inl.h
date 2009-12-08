/*
 * system-output-value-inl.h
 *
 *  Created on: Sep 13, 2009
 *      Author: dc
 */


namespace barrett {
namespace systems {


inline System::AbstractOutput::AbstractValue::AbstractValue(System* parent) :
			parentSystem(*parent)
{
	parentSystem.outputValues.push_back(this);
}

inline System::AbstractOutput::AbstractValue::~AbstractValue()
{
	// TODO(dc): this needs to be handled more like delegators in ~Output()
	std::replace(parentSystem.outputValues.begin(), parentSystem.outputValues.end(),
			const_cast<AbstractValue*>(this),
			static_cast<AbstractValue*>(NULL));
}


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

	undelegate();
	parentOutput.notifyListeners();
}

template<typename T>
inline void System::Output<T>::Value::setValueUndefined() {
	if (value != NULL) {
		delete value;
		value = NULL;
	}

	undelegate();
	parentOutput.notifyListeners();
}

template<typename T>
inline void
System::Output<T>::Value::delegateTo(const Output<T>& delegateOutput)
{
	undelegate();
	delegate = &(delegateOutput.value);
	delegate->parentOutput.addDelegator(parentOutput);
	parentOutput.notifyListeners();
}

template<typename T>
inline void System::Output<T>::Value::undelegate()
{
	if (delegate != NULL) {
		delegate->parentOutput.removeDelegator(parentOutput);
		delegate = NULL;
	}
}


}
}
