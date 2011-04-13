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
 * haptic_object.h
 *
 *  Created on: Feb 19, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_ABSTRACT_HAPTIC_OBJECT_H_
#define BARRETT_SYSTEMS_ABSTRACT_HAPTIC_OBJECT_H_


#include <barrett/detail/ca_macro.h>

#include <barrett/units.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


class HapticObject : public System, public SingleInput<units::CartesianPosition::type> {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

// IO
public:		Output<double> depthOutput;
protected:	Output<double>::Value* depthOutputValue;
public:		Output<cf_type> directionOutput;
protected:	Output<cf_type>::Value* directionOutputValue;

public:
	HapticObject(const std::string& sysName = "HapticObject") :
		System(sysName), SingleInput<cp_type>(this),
		depthOutput(this, &depthOutputValue),
		directionOutput(this, &directionOutputValue) {}
	virtual ~HapticObject() { mandatoryCleanUp(); }

private:
	DISALLOW_COPY_AND_ASSIGN(HapticObject);
};


}
}


#endif /* BARRETT_SYSTEMS_ABSTRACT_HAPTIC_OBJECT_H_ */
