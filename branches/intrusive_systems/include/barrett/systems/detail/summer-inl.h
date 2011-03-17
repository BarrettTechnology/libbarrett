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
 * summer-inl.h
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */

#include <boost/array.hpp>
#include <string>
#include <bitset>

#include <barrett/detail/stl_utils.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


template<typename T, size_t numInputs, bool RequiresAlignment>
Summer<T, numInputs, RequiresAlignment>::Summer(const Polarity& inputPolarity, bool undefinedIsZero, const std::string& sysName) :
	System(sysName), SingleOutput<T>(this), polarity(inputPolarity), strict(!undefinedIsZero), sum()
{
	initInputs();
}

template<typename T, size_t numInputs, bool RequiresAlignment>
Summer<T, numInputs, RequiresAlignment>::Summer(const std::string& inputPolarity, bool undefinedIsZero, const std::string& sysName) :
	System(sysName), SingleOutput<T>(this), polarity(inputPolarity), strict(!undefinedIsZero), sum()
{
	initInputs();
}

template<typename T, size_t numInputs, bool RequiresAlignment>
Summer<T, numInputs, RequiresAlignment>::Summer(const char* inputPolarity, bool undefinedIsZero, const std::string& sysName) :
	System(sysName), SingleOutput<T>(this), polarity(inputPolarity), strict(!undefinedIsZero), sum()
{
	initInputs();
}

template<typename T, size_t numInputs, bool RequiresAlignment>
Summer<T, numInputs, RequiresAlignment>::Summer(const std::bitset<numInputs>& inputPolarity, bool undefinedIsZero, const std::string& sysName) :
	System(sysName), SingleOutput<T>(this), polarity(inputPolarity), strict(!undefinedIsZero), sum()
{
	initInputs();
}

template<typename T, size_t numInputs, bool RequiresAlignment>
Summer<T, numInputs, RequiresAlignment>::Summer(bool undefinedIsZero, const std::string& sysName) :
	System(sysName), SingleOutput<T>(this), polarity(), strict(!undefinedIsZero), sum()
{
	initInputs();
}

template<typename T, size_t numInputs, bool RequiresAlignment>
Summer<T, numInputs, RequiresAlignment>::~Summer()
{
	mandatoryCleanUp();
	barrett::detail::purge(inputs);
}

template<typename T, size_t numInputs, bool RequiresAlignment>
inline System::Input<T>& Summer<T, numInputs, RequiresAlignment>::getInput(const size_t i) {
	return *( inputs.at(i) );
}

template<typename T, size_t numInputs, bool RequiresAlignment>
void Summer<T, numInputs, RequiresAlignment>::operate()
{
	typedef math::Traits<T> Traits;

	sum = Traits::zero();
	for (size_t i = 0; i < numInputs; ++i) {
		if (inputs[i]->valueDefined()) {
			sum = Traits::add(sum, Traits::mult(polarity[i], inputs[i]->getValue()));
		} else if (strict) {
			this->outputValue->setUndefined();
			return;
		}
	}

	this->outputValue->setData(&sum);
}

template<typename T, size_t numInputs, bool RequiresAlignment>
void Summer<T, numInputs, RequiresAlignment>::initInputs()
{
	for (size_t i = 0; i < numInputs; ++i) {
		inputs[i] = new Input<T>(this);
	}
}


}
}
