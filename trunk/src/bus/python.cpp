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
 *  Created on: Mar 13, 2012
 *      Author: dc
 */


#include <boost/python.hpp>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/can_socket.h>
#include <barrett/bus/bus_manager.h>


using namespace barrett;
using namespace bus;
using namespace boost::python;


// Reserve storage for static constants.
const size_t MAX_MESSAGE_LEN = CommunicationsBus::MAX_MESSAGE_LEN;
const size_t TIMEOUT = CommunicationsBus::TIMEOUT;


void pythonBusInterface() {
	class_<CommunicationsBus, boost::noncopyable>("CommunicationsBus", no_init)
		.def_readonly("MAX_MESSAGE_LEN", MAX_MESSAGE_LEN)
		.def_readonly("TIMEOUT", TIMEOUT)

//		.def("getMutex", &CommunicationsBus::getMutex, return_internal_reference<>())

		.def("open", &CommunicationsBus::open)
		.def("close", &CommunicationsBus::close)
		.def("isOpen", &CommunicationsBus::isOpen)

		.def("send", &CommunicationsBus::send)
		.def("receive", &CommunicationsBus::receive)
		.def("receiveRaw", &CommunicationsBus::receiveRaw)
	;

	class_<CANSocket, bases<CommunicationsBus>, boost::noncopyable>("CANSocket")
		.def(init<int>())
	;

	class_<BusManager, bases<CommunicationsBus>, boost::noncopyable>("BusManager")
		.def(init<CommunicationsBus*>()[with_custodian_and_ward<1,2>()])
		.def(init<int>())

		.def("getUnderlyingBus", &BusManager::getUnderlyingBus, return_internal_reference<>())
	;
}
