/*
 * puck-inl.h
 *
 *  Created on: Oct 5, 2010
 *      Author: dc
 */

#include <native/task.h>

#include <boost/thread/locks.hpp>

#include <barrett/thread/abstract/mutex.h>
#include <barrett/products/puck_group.h>


namespace barrett {


inline int Puck::getProperty(enum Property prop, bool realtime) const {
	return getProperty(bus, id, getPropertyId(prop), realtime);
}
inline int Puck::tryGetProperty(enum Property prop, int* result, int timeout_ns) const {
	return tryGetProperty(bus, id, getPropertyId(prop), result, timeout_ns);
}

template<typename Parser>
inline void Puck::getProperty(enum Property prop, typename Parser::result_type* result, bool realtime) const {
	getProperty<Parser> (bus, id, getPropertyId(prop), result, realtime);
}
template<typename Parser>
inline int Puck::tryGetProperty(enum Property prop, typename Parser::result_type* result, int timeout_ns) const {
	return tryGetProperty<Parser> (bus, id, getPropertyId(prop), result, timeout_ns);
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
	int ret = getPropertyHelper<Parser>(bus, id, propId, result, true, realtime, 0);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d.", __func__, ret);
		throw std::runtime_error("Puck::getProperty(): Failed to receive reply. Check /var/log/syslog for details.");
	}
}

inline int Puck::tryGetProperty(const bus::CommunicationsBus& bus, int id, int propId, int* result, int timeout_ns)
{
	return tryGetProperty<StandardParser>(bus, id, propId, result, timeout_ns);
}
template<typename Parser>
int Puck::tryGetProperty(const bus::CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, int timeout_ns)
{
	int ret = getPropertyHelper<Parser>(bus, id, propId, result, false, false, timeout_ns);
	if (ret != 0  &&  ret != 1) {  // some error other than "would block" occurred
			syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d.", __func__, ret);
			throw std::runtime_error("Puck::tryGetProperty(): Receive error. Check /var/log/syslog for details.");
	}
	return ret;
}

template<typename Parser>
int Puck::getPropertyHelper(const bus::CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, bool blocking, bool realtime, int timeout_ns)
{
	boost::unique_lock<thread::Mutex> ul(bus.getMutex(), boost::defer_lock);
	if (realtime) {
		ul.lock();
	}

	int ret = sendGetPropertyRequest(bus, id, propId);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
		throw std::runtime_error("Puck::getPropertyHelper(): Failed to send request. Check /var/log/syslog for details.");
	}

	if (timeout_ns != 0) {
		rt_task_sleep(timeout_ns);
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
		syslog(LOG_ERR, "%s: bus::CommunicationsBus::send() returned error %d.", __func__, ret);
		throw std::runtime_error("Puck::setProperty(): Failed to send SET message. Check /var/log/syslog for details.");
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
		syslog(LOG_ERR, "Puck::getPropertyId(): Pucks with type %s and firmware version %d do not respond to property %s.", getPuckTypeStr(pt), fwVers, getPropertyStr(prop));
		throw std::runtime_error("Puck::getPropertyId(): Invalid property. Check /var/log/syslog for details.");
	}
	return propId;
}


inline int Puck::StandardParser::busId(int id, int propId)
{
	return Puck::encodeBusId(id, PuckGroup::FGRP_OTHER);
}


}
