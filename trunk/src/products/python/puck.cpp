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
 * puck.cpp
 *
 *  Created on: Mar 21, 2012
 *      Author: dc
 */


#include <stdexcept>

#include <boost/python.hpp>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/products/puck.h>

#include "../../python.h"


using namespace barrett;
using namespace boost::python;


// Reserve storage for static constants.
const int NUM_PROPERTIES = Puck::NUM_PROPERTIES;
const int DEFAULT_IPNM = Puck::DEFAULT_IPNM;
const int MIN_ID = Puck::MIN_ID;
const int MAX_ID = Puck::MAX_ID;
const int HOST_ID = Puck::HOST_ID;
const int NODE_ID_WIDTH = Puck::NODE_ID_WIDTH;
const int NODE_ID_MASK = Puck::NODE_ID_MASK;
const int GROUP_MASK = Puck::GROUP_MASK;
const int FROM_MASK = Puck::FROM_MASK;
const int TO_MASK = Puck::TO_MASK;
const int SET_MASK = Puck::SET_MASK;
const int PROPERTY_MASK = Puck::PROPERTY_MASK;
const double WAKE_UP_TIME = Puck::WAKE_UP_TIME;
const double TURN_OFF_TIME = Puck::TURN_OFF_TIME;


int getProperty(const Puck& p, enum Puck::Property prop) {
	return p.getProperty(prop, false);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Puck_setProperty_overloads, setProperty, 2, 3)

void wakeList(list puckList) {
	std::vector<Puck*> pucks;
	for (int i = 0; i < len(puckList); ++i) {
		pucks.push_back(extract<Puck*>(puckList[i]));
	}

	Puck::wake(pucks);
}

int staticGetProperty(const bus::CommunicationsBus& bus, int id, int propId) {
	return Puck::getProperty(bus, id, propId, false);
}

tuple staticTryGetProperty(const bus::CommunicationsBus& bus, int id, int propId, double timeout_s = -1.0) {
	int ret;
	int result = 0;

	if (timeout_s <= 0) {
		ret = Puck::tryGetProperty(bus, id, propId, &result);
	} else {
		ret = Puck::tryGetProperty(bus, id, propId, &result, timeout_s);
	}

	return make_tuple(ret, result);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(staticTryGetProperty_overloads, staticTryGetProperty, 3, 4)

BOOST_PYTHON_FUNCTION_OVERLOADS(staticSetProperty_overloads, Puck::setProperty, 4, 5)

tuple receiveGetPropertyReply(const bus::CommunicationsBus& bus, int id, int propId, bool blocking) {
	int ret;
	int result = 0;
	ret = Puck::receiveGetPropertyReply(bus, id, propId, &result, blocking, false);
	return make_tuple(ret, result);
}

tuple decodeBusId(int busId) {
	int fromId, toId;
	Puck::decodeBusId(busId, &fromId, &toId);
	return make_tuple(fromId, toId);
}


void pythonProductsPuckInterface() {
	// The Puck class becomes the active scope until puckScope is destroyed.
	scope puckScope = class_<Puck>("Puck", init<const bus::CommunicationsBus&, int>()[with_custodian_and_ward<1,2>()])
		.def_readonly("NUM_PROPERTIES", NUM_PROPERTIES)
		.def("getPuckTypeStr", &Puck::getPuckTypeStr).staticmethod("getPuckTypeStr")
		.def("getPropertyStr", &Puck::getPropertyStr).staticmethod("getPropertyStr")
		.def("getPropertyEnum", &Puck::getPropertyEnum).staticmethod("getPropertyEnum")
		.def("getPropertyEnumNoThrow", &Puck::getPropertyEnumNoThrow).staticmethod("getPropertyEnumNoThrow")


		.def("wake", (void(Puck::*)()) &Puck::wake)  // Cast to resolve the overload
		// TODO(dc): Expose getProperty() with alternate parsers?
		.def("getProperty", &getProperty)
		.def("setProperty", (void (Puck::*)(enum Puck::Property, int, bool) const) &Puck::setProperty,
				Puck_setProperty_overloads())

		.def("saveProperty", &Puck::saveProperty)
		.def("saveAllProperties", &Puck::saveAllProperties)
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

		.def("wakeList", &wakeList).staticmethod("wakeList")  // Renamed because boost::puthon can't handle a static/non-static overload
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


	class_<Namespace, boost::noncopyable>("LowLevel", no_init)
		// TODO(dc): Expose getProperty(), etc. with alternate parsers?
		.def("getProperty", &staticGetProperty).staticmethod("getProperty")
		.def("tryGetProperty", &staticTryGetProperty, staticTryGetProperty_overloads()).staticmethod("tryGetProperty")
		.def("setProperty", (void (*)(const bus::CommunicationsBus&, int, int, int, bool))&Puck::setProperty, staticSetProperty_overloads()).staticmethod("setProperty")

		.def("sendGetPropertyRequest", &Puck::sendGetPropertyRequest).staticmethod("sendGetPropertyRequest")
		.def("receiveGetPropertyReply", &receiveGetPropertyReply).staticmethod("receiveGetPropertyReply")

		.def("respondsToProperty", (bool (*)(enum Puck::Property, enum Puck::PuckType, int))&Puck::respondsToProperty).staticmethod("respondsToProperty")
		.def("getPropertyId", (int (*)(enum Puck::Property, enum Puck::PuckType, int))&Puck::getPropertyId).staticmethod("getPropertyId")
		.def("getPropertyIdNoThrow", (int (*)(enum Puck::Property, enum Puck::PuckType, int))&Puck::getPropertyIdNoThrow).staticmethod("getPropertyIdNoThrow")


		.def_readonly("DEFAULT_IPNM", DEFAULT_IPNM)

		.def_readonly("MIN_ID", MIN_ID)
		.def_readonly("MAX_ID", MAX_ID)
		.def_readonly("HOST_ID", HOST_ID)
		.def_readonly("NODE_ID_WIDTH", NODE_ID_WIDTH)
		.def_readonly("NODE_ID_MASK", NODE_ID_MASK)

		.def("nodeId2BusId", &Puck::nodeId2BusId).staticmethod("nodeId2BusId")
		.def("busId2NodeId", &Puck::busId2NodeId).staticmethod("busId2NodeId")
		.def("encodeBusId", &Puck::encodeBusId).staticmethod("encodeBusId")
		.def("decodeBusId", &decodeBusId).staticmethod("decodeBusId")


		.def_readonly("GROUP_MASK", GROUP_MASK)
		.def_readonly("FROM_MASK", FROM_MASK)
		.def_readonly("TO_MASK", TO_MASK)

		.def_readonly("SET_MASK", SET_MASK)
		.def_readonly("PROPERTY_MASK", PROPERTY_MASK)

		.def_readonly("WAKE_UP_TIME", WAKE_UP_TIME)
		.def_readonly("TURN_OFF_TIME", TURN_OFF_TIME)
	;
}
