/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */
/*
 * bus_manager.cpp
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <barrett/os.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/can_socket.h>
#include <barrett/bus/bus_manager.h>


namespace barrett {
namespace bus {


BusManager::BusManager(CommunicationsBus* _bus) :
	bus(_bus), deleteBus(false), messageBuffers()
{
	if (bus == NULL) {
		bus = new CANSocket;
		deleteBus = true;
	}
}

BusManager::BusManager(int port) :
	bus(NULL), deleteBus(true), messageBuffers()
{
	bus = new CANSocket(port);
}

BusManager::~BusManager()
{
	if (deleteBus) {
		delete bus;
	}
}

int BusManager::receive(int expectedBusId, unsigned char* data, size_t& len, bool blocking, bool realtime) const
{
	thread::Mutex& m = getMutex();
	m.lock();

	double start = highResolutionSystemTime();

	if (retrieveMessage(expectedBusId, data, len)) {
		m.unlock();
		return 0;
	}

	int ret;
	while (true) {
		ret = updateBuffers();
		if (ret != 0) {
			m.unlock();
			return ret;
		} else if (retrieveMessage(expectedBusId, data, len)) {
			m.unlock();
			return 0;
		} else if (!blocking) {
			m.unlock();
			return 1;
		}

		if ((highResolutionSystemTime() - start) > CommunicationsBus::TIMEOUT) {
			m.unlock();
			logMessage("BusManager::receive(): timed out", true);
			return 2;
		}

		if (!realtime) {
			int lc = m.fullUnlock();
			btsleepRT(0.0001);
			m.relock(lc);
		}
	}
}

int BusManager::updateBuffers() const
{
	BARRETT_SCOPED_LOCK(getMutex());

	int busId;
	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
	size_t len;
	int ret;

	// empty the bus' receive buffer
	while (true) {
		ret = receiveRaw(busId, data, len, false);  // non-blocking read
		if (ret == 0) {  // successfully received a message
			if (busId != 1344) storeMessage(busId, data, len); // disregard safetyboard broadcast message
		} else if (ret == 1) {  // would block
			return 0;
		} else {  // error
			return ret;
		}
	}
}

void BusManager::storeMessage(int busId, const unsigned char* data, size_t len) const
{
	if (messageBuffers[busId].full()) {
		(logMessage("BusManager::%s: Buffer overflow. ID = %d",true) %__func__ %busId).raise<std::runtime_error>();
	}
	messageBuffers[busId].push_back(Message(data, len));
}

bool BusManager::retrieveMessage(int busId, unsigned char* data, size_t& len) const
{
	if (messageBuffers[busId].empty()) {
		return false;
	}

	messageBuffers[busId].front().copyTo(data, len);
	messageBuffers[busId].pop_front();

	return true;
}


}
}
