/*
 * system-output-value-inl.h
 *
 *  Created on: Sep 13, 2009
 *      Author: dc
 */


#include <barrett/detail/stl_utils.h>
#include <barrett/thread/abstract/mutex.h>


namespace barrett {
namespace systems {


inline System::AbstractOutput::AbstractValue::AbstractValue(System* parentSys) :
	parentSystem(parentSys)
{
	SCOPED_LOCK(getEmMutex());

	parentSystem->outputValues.push_back(this);
}

inline System::AbstractOutput::AbstractValue::~AbstractValue()
{
	SCOPED_LOCK(getEmMutex());

	if (parentSystem != NULL) {
		detail::replaceWithNull(parentSystem->outputValues, this);
	}
}

inline thread::Mutex& System::AbstractOutput::AbstractValue::getEmMutex()
{
	if (parentSystem != NULL) {
		return parentSystem->getEmMutex();
	} else {
		return thread::NullMutex::aNullMutex;
	}
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
}

template<typename T>
inline void System::Output<T>::Value::setValueUndefined() {
	if (value != NULL) {
		delete value;
		value = NULL;
	}

	undelegate();
}

template<typename T>
inline void
System::Output<T>::Value::delegateTo(Output<T>& delegateOutput)
{
	SCOPED_LOCK(getEmMutex());

	undelegate();
	delegate = &(delegateOutput.value);
	delegate->parentOutput.delegators.push_back(&parentOutput);
}

template<typename T>
inline void System::Output<T>::Value::undelegate()
{
	SCOPED_LOCK(getEmMutex());

	if (delegate != NULL) {
		delegate->parentOutput.delegators.remove(&parentOutput);
		delegate = NULL;
	}
}


template<typename T>
inline void System::Output<T>::Value::updateValue()
{
	// TODO(dc): test!
	if (parentSystem != NULL) {
		parentSystem->update();
	}
}


}
}
