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
 * conversion.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_ABSTRACT_CONVERSION_H_
#define BARRETT_SYSTEMS_ABSTRACT_CONVERSION_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


// objects implementing this interface can be manipulated by a
// Converter
template<typename OutputType>
class Conversion {
public:
	Conversion() {}
	virtual ~Conversion() {}

	virtual System::AbstractInput* getConversionInput() = 0;
	virtual System::Output<OutputType>& getConversionOutput() = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(Conversion);
};


}
}


#endif /* BARRETT_SYSTEMS_ABSTRACT_CONVERSION_H_ */
