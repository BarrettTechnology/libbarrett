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
 * converter.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_CONVERTER_H_
#define BARRETT_SYSTEMS_CONVERTER_H_


#include <list>
#include <utility>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/abstract/conversion.h>


namespace barrett {
namespace systems {


template<typename OutputType>
class Converter : public System, public SingleOutput<OutputType> {
public:
	Converter(const std::string& sysName = "Converter") :
		System(sysName), SingleOutput<OutputType>(this), conversions() {}
	virtual ~Converter();

	void registerConversion(
			Conversion<OutputType>* conversion);

	template<typename T>
	void connectInputTo(System::Output<T>& output)  //NOLINT: non-const reference for syntax
	throw(std::invalid_argument);

	template<typename T>
	bool connectInputToNoThrow(System::Output<T>& output);  //NOLINT: non-const reference for syntax

	template<typename T>
	Conversion<OutputType>* getInput(System::Input<T>** input);
	void disconnectInput();

protected:
	virtual void operate() {  invalidateOutputs();  }

	std::list<Conversion<OutputType>*> conversions;

private:
	DISALLOW_COPY_AND_ASSIGN(Converter);
};


}
}


// include template definitions
#include <barrett/systems/detail/converter-inl.h>


#endif /* BARRETT_SYSTEMS_CONVERTER_H_ */
