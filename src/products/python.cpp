/*
	Copyright 2012 Barrett Technology <support@barrett.com>

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
 * python.cpp
 *
 *  Created on: Mar 21, 2012
 *      Author: dc
 */


#include <boost/python.hpp>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/products/puck.h>

using namespace barrett;
using namespace boost::python;


int getPropertyNoRT(const Puck& p, enum Puck::Property prop) {
	return p.getProperty(prop, false);
}

void pythonProductsInterface() {
	// Puck class
	{
		// The Puck class becomes the active scope until puckScope is destroyed.
		scope puckScope = class_<Puck>("Puck", init<bus::CommunicationsBus&, int>()[with_custodian_and_ward<1,2>()])
			.def("getPuckTypeStr", &Puck::getPuckTypeStr).staticmethod("getPuckTypeStr")
			.def("getPropertyStr", &Puck::getPropertyStr).staticmethod("getPropertyStr")
			.def("getPropertyEnum", &Puck::getPropertyEnum).staticmethod("getPropertyEnum")
			.def("getPropertyEnumNoThrow", &Puck::getPropertyEnumNoThrow).staticmethod("getPropertyEnumNoThrow")

//			.def("wake", (void(Puck::*)()) &Puck::wake)  // Cast to resolve the overload
//			.def("getProperty", getPropertyNoRT)
		;

		enum_<enum Puck::RoleOption>("RoleOption")
			.value("RO_MagEncOnSerial", Puck::RO_MagEncOnSerial)
			.value("RO_MagEncOnHall", Puck::RO_MagEncOnHall)
			.value("RO_MagEncOnEnc", Puck::RO_MagEncOnEnc)
			.value("RO_Strain", Puck::RO_Strain)
			.value("RO_Tact", Puck::RO_Tact)
			.value("RO_IMU", Puck::RO_IMU)
			.value("RO_OpticalEncOnEnc", Puck::RO_OpticalEncOnEnc)
			.export_values()
		;

		enum_<enum Puck::PuckType>("PuckType")
			.value("PT_Monitor", Puck::PT_Monitor)
			.value("PT_Safety", Puck::PT_Safety)
			.value("PT_Motor", Puck::PT_Motor)
			.value("PT_ForceTorque", Puck::PT_ForceTorque)
			.value("PT_Unknown", Puck::PT_Unknown)
			.export_values()
		;

		enum_<enum Puck::Property> propEnum("Property");
		for (int i = 0; i < Puck::NUM_PROPERTIES; ++i) {
			enum Puck::Property p = (enum Puck::Property)i;
			propEnum.value(Puck::getPropertyStr(p), p);
		}
		propEnum.export_values();
	}
}
