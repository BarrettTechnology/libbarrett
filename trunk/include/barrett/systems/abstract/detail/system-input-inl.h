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
	BARRETT_SCOPED_LOCK(getEmMutex());

	parentSystem->inputs.push_back(this);
}

inline System::AbstractInput::~AbstractInput()
{
	BARRETT_SCOPED_LOCK(getEmMutex());

	if (parentSystem != NULL) {
		detail::replaceWithNull(parentSystem->inputs, this);
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
	BARRETT_SCOPED_LOCK(getEmMutex());

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
