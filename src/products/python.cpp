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


int getProperty(const Puck& p, enum Puck::Property prop) {
	return p.getProperty(prop, false);
}

// TODO(dc): I'm not convinced that this feature has value.
//object tryGetProperty(const Puck& p, enum Puck::Property prop, double timeout_s = -1.0) {
//	int result;
//	int ret;
//
//	if (timeout_s < 0.0) {
//		ret = p.tryGetProperty(prop, &result);
//	} else {
//		ret = p.tryGetProperty(prop, &result, timeout_s * 1e9);
//	}
//
//	if (ret == 0) {
//		return object(result);
//	} else {
//		return object();
//	}
//}
//BOOST_PYTHON_FUNCTION_OVERLOADS(tryGetProperty_overloads, tryGetProperty, 2, 3)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Puck_setProperty_overloads, setProperty, 2, 3)

void pythonProductsInterface() {
	// Puck class
	{
		// The Puck class becomes the active scope until puckScope is destroyed.
		scope puckScope = class_<Puck>("Puck", init<bus::CommunicationsBus&, int>()[with_custodian_and_ward<1,2>()])
			.def("getPuckTypeStr", &Puck::getPuckTypeStr).staticmethod("getPuckTypeStr")
			.def("getPropertyStr", &Puck::getPropertyStr).staticmethod("getPropertyStr")
			.def("getPropertyEnum", &Puck::getPropertyEnum).staticmethod("getPropertyEnum")
			.def("getPropertyEnumNoThrow", &Puck::getPropertyEnumNoThrow).staticmethod("getPropertyEnumNoThrow")

			.def("wake", (void(Puck::*)()) &Puck::wake)  // Cast to resolve the overload
			.def("getProperty", &getProperty)
//			.def("tryGetProperty", &tryGetProperty,
//					tryGetProperty_overloads())
			.def("setProperty", (void (Puck::*)(enum Puck::Property, int, bool) const) &Puck::setProperty,
					Puck_setProperty_overloads())

			.def("saveProperty", &Puck::saveProperty)
			.def("resetProperty", &Puck::resetProperty)

			.def("respondsToProperty", (bool (Puck::*)(enum Puck::Property) const) &Puck::respondsToProperty)
			.def("getPropertyId", (int (Puck::*)(enum Puck::Property) const) &Puck::getPropertyId)
			.def("getPropertyIdNoThrow", (int (Puck::*)(enum Puck::Property) const) &Puck::getPropertyIdNoThrow)

			.def("updateRole", &Puck::updateRole)
			.def("updateStatus", &Puck::updateStatus)

			.def("getBus", &Puck::getBus, return_internal_reference<>())
			.def("getId", &Puck::getId)
			.def("getVers", &Puck::getVers)
			.def("getRole", &Puck::getRole)
			.def("hasOption", &Puck::hasOption)
			.def("getType", &Puck::getType)
			.def("getEffectiveType", &Puck::getEffectiveType)
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
