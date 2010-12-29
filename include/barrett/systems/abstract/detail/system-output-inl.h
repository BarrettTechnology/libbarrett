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
 * system-output-inl.h
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */


#include <algorithm>
#include <vector>
#include <list>


namespace barrett {
namespace systems {


// the compiler requires a definition for a dtor, even if it's "pure" virtual
inline System::AbstractOutput::~AbstractOutput() { }


template<typename T>
System::Output<T>::Output(System* parentSystem, Value** valueHandle) :
	value(parentSystem, *this), inputs(), delegators()
{
	(*valueHandle) = &value;
}

template<typename T>
System::Output<T>::~Output()
{
	disconnect(*this);
	value.undelegate();

	// value.undelegate() removes elements from the delegators list
	while (delegators.size()) {
		delegators.front()->value.undelegate();
	}
}

// TODO(dc): test
template<typename T>
bool System::Output<T>::isConnected() const
{
	return !inputs.empty();
}

template<typename T>
typename System::Output<T>::Value*
System::Output<T>::getValueObject()
{
	Value* v = &value;
	while (v->delegate != NULL) {
		v = v->delegate;
	}

	return v;
}


}
}
