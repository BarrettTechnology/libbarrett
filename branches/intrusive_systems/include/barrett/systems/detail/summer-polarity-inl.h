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
 * summer-polarity-inl.h
 *
 *  Created on: Sep 12, 2009
 *      Author: dc
 */

#include <string>
#include <stdexcept>


namespace barrett {
namespace systems {


template<typename T, size_t numInputs, bool RequiresAlignment>
Summer<T, numInputs, RequiresAlignment>::Polarity::Polarity() :  // default: all positive
	polarity()
{
	polarity.set();
}

template<typename T, size_t numInputs, bool RequiresAlignment>
Summer<T, numInputs, RequiresAlignment>::Polarity::Polarity(std::string polarityStr)
throw(std::invalid_argument) :
	polarity()
{
	if (polarityStr.size() != numInputs) {
		throw std::invalid_argument("(systems::Summer::Polarity::Polarity): "
		                            "polarityStr must be of the same length "
		                            "as the number of inputs to the Summer.");
	}

	for (size_t i = 0; i < numInputs; ++i) {
		switch (polarityStr[i]) {
		case '+':
			polarity.set(i);
			break;
		case '-':
			polarity.reset(i);
			break;
		default:
			throw std::invalid_argument(
					"(systems::Summer::Polarity::Polarity): polarityStr must "
					"contain only '+' and '-' characters.");
			break;
		}
	}
}

template<typename T, size_t numInputs, bool RequiresAlignment>
inline int Summer<T, numInputs, RequiresAlignment>::Polarity::operator[] (const size_t i) const
{
	return polarity[i] ? 1 : -1;
}


}
}
