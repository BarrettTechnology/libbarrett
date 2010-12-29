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
 * array_editor-inl.h
 *
 *  Created on: Nov 12, 2009
 *      Author: dc
 */


#include <barrett/detail/stl_utils.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


template <typename T>
ArrayEditor<T>::ArrayEditor() :
	SingleIO<T, T>()
{
	initInputs();
}

template <typename T>
ArrayEditor<T>::ArrayEditor(const T& initialOutputValue) :
	SingleIO<T, T>(initialOutputValue)
{
	initInputs();
}

template <typename T>
inline ArrayEditor<T>::~ArrayEditor()
{
	barrett::detail::purge(elementInputs);
}


template <typename T>
inline System::Input<double>& ArrayEditor<T>::getElementInput(const size_t i)
{
	return *( elementInputs.at(i) );
}

template <typename T>
void ArrayEditor<T>::operate()
{
	T tmp = this->input.getValue();
	for (size_t i = 0; i < T::SIZE; ++i) {
		if (elementInputs[i]->valueDefined()) {
			tmp[i] = elementInputs[i]->getValue();
		}
	}

	this->outputValue->setValue(tmp);
}

template <typename T>
bool ArrayEditor<T>::inputsValid()
{
	return this->input.valueDefined();
}


template<typename T>
void ArrayEditor<T>::initInputs()
{
	for (size_t i = 0; i < T::SIZE; ++i) {
		elementInputs[i] = new System::Input<double>(this);
	}
}


}
}
