/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

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
	BARRETT_SCOPED_LOCK(getEmMutex());

	parentSystem->outputValues.push_back(this);
}

inline System::AbstractOutput::AbstractValue::~AbstractValue()
{
	BARRETT_SCOPED_LOCK(getEmMutex());

	if (parentSystem != NULL) {
		detail::replaceWithNull(parentSystem->outputValues, this);
	}
}

inline thread::Mutex& System::AbstractOutput::AbstractValue::getEmMutex() const
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
	BARRETT_SCOPED_LOCK(getEmMutex());

	undelegate();
	delegate = &(delegateOutput.value);
	delegate->parentOutput.delegators.push_back(&parentOutput);
}

template<typename T>
inline void System::Output<T>::Value::undelegate()
{
	BARRETT_SCOPED_LOCK(getEmMutex());

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
