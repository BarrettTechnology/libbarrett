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
 * array_splitter-inl.h
 *
 *  Created on: Nov 17, 2009
 *      Author: dc
 */


#include <barrett/detail/stl_utils.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


template <typename T>
ArraySplitter<T>::ArraySplitter(const std::string& sysName) :
	System(sysName), SingleInput<T>(this)
{
	for (size_t i = 0; i < T::SIZE; ++i) {
		outputs[i] = new System::Output<double>(this, &outputValues[i]);
	}
}

template <typename T>
ArraySplitter<T>::~ArraySplitter()
{
	mandatoryCleanUp();
	barrett::detail::purge(outputs);
}


template <typename T>
inline System::Output<double>& ArraySplitter<T>::getOutput(const size_t i)
{
	return *( outputs.at(i) );
}

template <typename T>
void ArraySplitter<T>::operate()
{
	for (size_t i = 0; i < T::SIZE; ++i) {
		outputValues[i]->setData(this->input.getValue().data() + i);
	}
}


}
}
