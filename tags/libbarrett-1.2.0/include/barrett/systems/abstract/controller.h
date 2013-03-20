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
 * controller.h
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_ABSTRACT_CONTROLLER_H_
#define BARRETT_SYSTEMS_ABSTRACT_CONTROLLER_H_


#include <list>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/conversion.h>


namespace barrett {
namespace systems {


template<typename InputType, typename OutputType = InputType>
class Controller : public System, public Conversion<OutputType> {
// IO
public:		Input<InputType> referenceInput;
public:		Input<InputType> feedbackInput;
public:		Output<OutputType> controlOutput;
protected:	typename Output<OutputType>::Value* controlOutputValue;


public:
	explicit Controller(const std::string& sysName = "Controller") :
		System(sysName),
		referenceInput(this),
		feedbackInput(this),
		controlOutput(this, &controlOutputValue) {}
	virtual ~Controller() { mandatoryCleanUp(); }

	virtual System::Input<InputType>* getConversionInput() {
		return &referenceInput;
	}
	virtual System::Output<OutputType>& getConversionOutput() {
		return controlOutput;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Controller);
};


}
}


#endif /* BARRETT_SYSTEMS_ABSTRACT_CONTROLLER_H_ */
