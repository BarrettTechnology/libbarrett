/*
 * puck-inl.h
 *
 *  Created on: Oct 5, 2010
 *      Author: dc
 */

#include <boost/thread/locks.hpp>

#include <barrett/thread/abstract/mutex.h>
#include <barrett/products/puck_group.h>


namespace barrett {


inline int Puck::getProperty(enum Property prop, bool realtime) const {
	return getProperty(bus, id, getPropertyId(prop), realtime);
}
inline int Puck::tryGetProperty(enum Property prop, int* result, int timeout_us) const {
	return tryGetProperty(bus, id, getPropertyId(prop), result, timeout_us);
}

template<typename Parser>
inline void Puck::getProperty(enum Property prop, typename Parser::result_type* result, bool realtime) const {
	getProperty<Parser> (bus, id, getPropertyId(prop), result, realtime);
}
template<typename Parser>
inline int Puck::tryGetProperty(enum Property prop, typename Parser::result_type* result, int timeout_us) const {
	return tryGetProperty<Parser> (bus, id, getPropertyId(prop), result, timeout_us);
}

template<template<typename U, typename = std::allocator<U> > class Container>
void Puck::wake(Container<Puck*> pucks)
{
	typename Container<Puck*>::iterator i;
	bool allPucksAwake;

	// Find the Pucks that to be woken up
	allPucksAwake = true;
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}
//		(*i)->updateStatus();
		if ((*i)->getEffectiveType() == PT_Monitor) {
			allPucksAwake = false;
		} else {
			*i = NULL;
		}
	}

	if (allPucksAwake) {
		return;
	}

	// Wake the Pucks. This is a separate step because talking on the CANbus
	// while the Pucks' transceivers come online can cause the host to go
	// bus-off.
	{
		// Prevent other threads from talking on the CANbus while Pucks are coming online.
		BARRETT_SCOPED_LOCK(pucks.front()->getBus().getMutex());

		for (i = pucks.begin(); i != pucks.end(); ++i) {
			if (*i == NULL) {
				continue;
			}
			(*i)->setProperty(STAT, STATUS_READY);

			// Talking on the CANbus when a transceiver goes offline can also cause
			// bus-off. Wait until this Puck drops off the bus before trying to
			// wake the next one.
			// TODO(dc): is there a more robust way of doing this?
			usleep(10000);
		}

		usleep(WAKE_UP_TIME);
	}

	int ret;
	int stat;
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}

		// Pucks can take longer to respond when in the process of waking up, so wait for 50ms.
		ret = (*i)->tryGetProperty(STAT, &stat, 50000);
		if (ret == 0  &&  stat == STATUS_READY) {
			(*i)->updateStatus();
		} else {
			if (ret == 0) {
				syslog(LOG_ERR, "%s: Failed to wake Puck ID=%d: STAT=%d", __func__, (*i)->getId(), stat);
			} else {
				syslog(LOG_ERR, "%s: Failed to wake Puck ID=%d: No response after waiting %.2fs.", __func__, (*i)->getId(), WAKE_UP_TIME/1e6);
			}
			throw std::runtime_error("Puck::wake(): Failed to wake Puck. Check /var/log/syslog for details.");
		}
	}
}

inline int Puck::getProperty(const CommunicationsBus& bus, int id, int propId, bool realtime)
{
	int result;
	getProperty<StandardParser>(bus, id, propId, &result, realtime);
	return result;
}
// TODO(dc): throw exception if getProperty is called with a group ID?
template<typename Parser>
void Puck::getProperty(const CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, bool realtime)
{
	int ret = getPropertyHelper<Parser>(bus, id, propId, result, true, realtime, 0);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d.", __func__, ret);
		throw std::runtime_error("Puck::getProperty(): Failed to receive reply. Check /var/log/syslog for details.");
	}
}

inline int Puck::tryGetProperty(const CommunicationsBus& bus, int id, int propId, int* result, int timeout_us)
{
	return tryGetProperty<StandardParser>(bus, id, propId, result, timeout_us);
}
template<typename Parser>
int Puck::tryGetProperty(const CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, int timeout_us)
{
	int ret = getPropertyHelper<Parser>(bus, id, propId, result, false, false, timeout_us);
	if (ret != 0  &&  ret != 1) {  // some error other than "would block" occurred
			syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d.", __func__, ret);
			throw std::runtime_error("Puck::tryGetProperty(): Receive error. Check /var/log/syslog for details.");
	}
	return ret;
}

template<typename Parser>
int Puck::getPropertyHelper(const CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, bool blocking, bool realtime, int timeout_us)
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

	if (timeout_us != 0) {
		usleep(timeout_us);
	}

	return receiveGetPropertyReply<Parser>(bus, id, propId, result, blocking, realtime);
}

inline void Puck::setProperty(const CommunicationsBus& bus, int id, int propId, int value)
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
		syslog(LOG_ERR, "%s: Puck::setProperty() returned error %d.", __func__, ret);
		throw std::runtime_error("Puck::setProperty(): Failed to send SET message. Check /var/log/syslog for details.");
	}
}

inline int Puck::sendGetPropertyRequest(const CommunicationsBus& bus, int id, int propId)
{
	static const size_t MSG_LEN = 1;

	unsigned char data[MSG_LEN];
	data[0] = propId & PROPERTY_MASK;

	return bus.send(nodeId2BusId(id), data, MSG_LEN);
}

inline int Puck::receiveGetPropertyReply(const CommunicationsBus& bus, int id, int propId, int* result, bool blocking, bool realtime)
{
	return receiveGetPropertyReply<StandardParser>(bus, id, propId, result, blocking, realtime);
}
template<typename Parser>
int Puck::receiveGetPropertyReply(const CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, bool blocking, bool realtime)
{
	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
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
