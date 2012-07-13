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


#include <stdexcept>
#include <boost/python.hpp>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/can_socket.h>
#include <barrett/bus/bus_manager.h>

#include "../python.h"


using namespace barrett;
using namespace bus;
using namespace boost::python;


// Reserve storage for static constants.
const size_t MAX_MESSAGE_LEN = CommunicationsBus::MAX_MESSAGE_LEN;
const size_t TIMEOUT = CommunicationsBus::TIMEOUT;


void send(const CommunicationsBus& cb, int busId, object pyData) {
	const size_t L = len(pyData);
	if (L > (int)cb.MAX_MESSAGE_LEN) {
		throw std::logic_error("data exceeds the maximum length");
	}

	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
	for (size_t i = 0; i < L; ++i) {
		data[i] = extract<unsigned char>(pyData[i]);
	}

	int ret = cb.send(busId, data, L);
	if (ret != 0) {
		throw std::runtime_error("bus::CommunicationsBus::send() failed. See /var/log/syslog for details.");
	}
}

list receive(const CommunicationsBus& cb, int expectedBusId, bool blocking = true) {
	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
	size_t l = 0;
	int ret = cb.receive(expectedBusId, data, l, blocking, false);  // No realtime for python!

	// If we failed because receive() would have blocked, return an empty list
	if (!blocking  &&  ret == 1) {
		return list();
	} else if (ret != 0) {
		throw std::runtime_error("bus::CommunicationsBus::receive() failed. See /var/log/syslog for details.");
	}

	list pyData;
	for (size_t i = 0; i < l; ++i) {
		pyData.append(data[i]);
	}
	return pyData;
}
BOOST_PYTHON_FUNCTION_OVERLOADS(receive_overloads, receive, 2, 3)

tuple receiveRaw(const CommunicationsBus& cb, bool blocking = true) {
	int busId = 0;
	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
	size_t l = 0;
	int ret = cb.receiveRaw(busId, data, l, blocking);

	// If we failed because receiveRaw() would have blocked, return an empty list
	if (!blocking  &&  ret == 1) {
		return make_tuple(0, list());
	} else if (ret != 0) {
		throw std::runtime_error("bus::CommunicationsBus::receiveRaw() failed. See /var/log/syslog for details.");
	}

	list pyData;
	for (size_t i = 0; i < l; ++i) {
		pyData.append(data[i]);
	}
	return make_tuple(busId, pyData);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(receiveRaw_overloads, receiveRaw, 1, 2)


void pythonBusInterface() {
	class_<CommunicationsBus, boost::noncopyable>("CommunicationsBus", no_init)
		.def_readonly("MAX_MESSAGE_LEN", MAX_MESSAGE_LEN)
		.def_readonly("TIMEOUT", TIMEOUT)

		// TODO(dc): Why is this broken?
		//.def("getMutex", &CommunicationsBus::getMutex, return_internal_reference<>())

		.def("open", &CommunicationsBus::open)
		.def("close", &CommunicationsBus::close)
		.def("isOpen", &CommunicationsBus::isOpen)

		.def("send", &send)
		.def("receive", &receive, receive_overloads())
		.def("receiveRaw", &receiveRaw, receiveRaw_overloads())
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
