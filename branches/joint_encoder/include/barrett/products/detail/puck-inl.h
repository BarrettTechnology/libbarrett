/*
	Copyright 2010, 2011, 2012 Barrett Technology <support@barrett.com>

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
 * puck-inl.h
 *
 *  Created on: Oct 5, 2010
 *      Author: dc
 */

#include <boost/thread/locks.hpp>

#include <barrett/os.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/products/puck_group.h>


namespace barrett {


inline int Puck::getProperty(enum Property prop, bool realtime) const {
	return getProperty(bus, id, getPropertyId(prop), realtime);
}
template<typename Parser>
inline void Puck::getProperty(enum Property prop, typename Parser::result_type* result, bool realtime) const {
	getProperty<Parser> (bus, id, getPropertyId(prop), result, realtime);
}


inline int Puck::getProperty(const bus::CommunicationsBus& bus, int id, int propId, bool realtime)
{
	int result;
	getProperty<StandardParser>(bus, id, propId, &result, realtime);
	return result;
}
// TODO(dc): throw exception if getProperty is called with a group ID?
template<typename Parser>
void Puck::getProperty(const bus::CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, bool realtime)
{
	int ret = getPropertyHelper<Parser>(bus, id, propId, result, true, realtime, 0.0);
	if (ret != 0) {
		(logMessage("Puck::%s(): Failed to receive reply. "
				"Puck::receiveGetPropertyReply() returned error %d.")
				% __func__ % ret).template raise<std::runtime_error>();
	}
}

inline int Puck::tryGetProperty(const bus::CommunicationsBus& bus, int id, int propId, int* result, double timeout_s)
{
	return tryGetProperty<StandardParser>(bus, id, propId, result, timeout_s);
}
template<typename Parser>
int Puck::tryGetProperty(const bus::CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, double timeout_s)
{
	int ret = getPropertyHelper<Parser>(bus, id, propId, result, false, false, timeout_s);
	if (ret != 0  &&  ret != 1) {  // some error other than "would block" occurred
		(logMessage("Puck::%s(): Receive error. "
				"Puck::receiveGetPropertyReply() returned error %d.")
				% __func__ % ret).template raise<std::runtime_error>();
	}
	return ret;
}

template<typename Parser>
int Puck::getPropertyHelper(const bus::CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, bool blocking, bool realtime, double timeout_s)
{
	boost::unique_lock<thread::Mutex> ul(bus.getMutex(), boost::defer_lock);
	if (realtime) {
		ul.lock();
	}

	int ret = sendGetPropertyRequest(bus, id, propId);
	if (ret != 0) {
		(logMessage("Puck::%s(): Failed to send request. "
				"Puck::sendGetPropertyRequest() returned error %d.")
				% __func__ % ret).template raise<std::runtime_error>();
	}

	if (timeout_s != 0.0) {
		btsleepRT(timeout_s);
	}

	return receiveGetPropertyReply<Parser>(bus, id, propId, result, blocking, realtime);
}

inline void Puck::setProperty(const bus::CommunicationsBus& bus, int id, int propId, int value, bool blocking)
{
	static const size_t MSG_LEN = 6;

	unsigned char data[MSG_LEN];
	data[0] = (propId & PROPERTY_MASK) | SET_MASK;
	data[1] = 0;
	data[2] = (value & 0x000000ff);
	data[3] = (value & 0x0000ff00) >>  8;
	data[4] = (value & 0x00ff0000) >> 16;
	data[5] = (value & 0xff000000) >> 24;

	int ret = bus.send(nodeId2BusId(id), data, MSG_LEN);
	if (ret != 0) {
		(logMessage("Puck::%s(): Failed to send SET message. "
				"bus::CommunicationsBus::send() returned error %d.")
				% __func__ % ret).raise<std::runtime_error>();
	}

	if (blocking) {
		// Make sure the command has completed so we don't overflow the Puck's receive buffer.
		getProperty(bus, id, getPropertyId(STAT, PT_Unknown, 0));
	}
}

inline int Puck::sendGetPropertyRequest(const bus::CommunicationsBus& bus, int id, int propId)
{
	static const size_t MSG_LEN = 1;

	unsigned char data[MSG_LEN];
	data[0] = propId & PROPERTY_MASK;

	return bus.send(nodeId2BusId(id), data, MSG_LEN);
}

inline int Puck::receiveGetPropertyReply(const bus::CommunicationsBus& bus, int id, int propId, int* result, bool blocking, bool realtime)
{
	return receiveGetPropertyReply<StandardParser>(bus, id, propId, result, blocking, realtime);
}
template<typename Parser>
int Puck::receiveGetPropertyReply(const bus::CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, bool blocking, bool realtime)
{
	unsigned char data[bus::CommunicationsBus::MAX_MESSAGE_LEN];
	size_t len;

	int ret = bus.receive(Parser::busId(id, propId), data, len, blocking, realtime);
	if (ret != 0) {
		return ret;
	}

	// Make return code negative to avoid colliding with bus.receive() return codes.
	return -Parser::parse(id, propId, result, data, len);
}

inline int Puck::getPropertyId(enum Property prop, enum PuckType pt, int fwVers)
		throw(std::runtime_error)
{
	int propId = getPropertyIdNoThrow(prop, pt, fwVers);
	if (propId == -1) {
		(logMessage("Puck::%s(): Invalid property. "
				"Pucks with type %s and firmware version %d do not respond to property %s.")
				% __func__ % getPuckTypeStr(pt) % fwVers % getPropertyStr(prop)).raise<std::runtime_error>();
	}
	return propId;
}


inline int Puck::StandardParser::busId(int id, int propId)
{
	return Puck::encodeBusId(id, PuckGroup::FGRP_OTHER);
}


}
