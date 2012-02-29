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
 * single_io.h
 *
 *  Created on: Sep 12, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_ABSTRACT_SINGLE_IO_H_
#define BARRETT_SYSTEMS_ABSTRACT_SINGLE_IO_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/conversion.h>


namespace barrett {
namespace systems {


template<typename InputType>
class SingleInput {  // not a System in order to avoid diamond inheritance
// IO
public:	System::Input<InputType> input;


public:
	explicit SingleInput(System* parentSys) :
		input(parentSys) {}

private:
	DISALLOW_COPY_AND_ASSIGN(SingleInput);
};


template<typename OutputType>
class SingleOutput {  // not a System in order to avoid diamond inheritance
// IO
public:		System::Output<OutputType> output;
protected:	typename System::Output<OutputType>::Value* outputValue;


public:
	explicit SingleOutput(System* parentSys) :
		output(parentSys, &outputValue) {}

private:
	DISALLOW_COPY_AND_ASSIGN(SingleOutput);
};


template<typename InputType, typename OutputType>
class SingleIO :	public System, public SingleInput<InputType>,
					public SingleOutput<OutputType>,
					public Conversion<OutputType> {
public:
	explicit SingleIO(const std::string& sysName = "SingleIO") :
		System(sysName),
		SingleInput<InputType>(this), SingleOutput<OutputType>(this) {}
	virtual ~SingleIO() { mandatoryCleanUp(); }

	virtual System::Input<InputType>* getConversionInput() {
		return &(this->input);
	}
	virtual System::Output<OutputType>& getConversionOutput() {
		return this->output;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(SingleIO);
};


}
}


#endif /* BARRETT_SYSTEMS_ABSTRACT_SINGLE_IO_H_ */
