/*
 * puck-inl.h
 *
 *  Created on: Oct 5, 2010
 *      Author: dc
 */

#include <barrett/puck_group.h>


namespace barrett {


inline int Puck::getProperty(enum Property prop, bool realtime) const {
	return getProperty(bus, id, getPropertyId(prop), realtime);
}
inline int Puck::tryGetProperty(enum Property prop, bool* successful, int timeout_us) const {
	return tryGetProperty(bus, id, getPropertyId(prop), successful, timeout_us);
}

template<typename Parser>
inline typename Parser::result_type Puck::getProperty(enum Property prop, bool realtime) const {
	return getProperty<Parser> (bus, id, getPropertyId(prop), realtime);
}
template<typename Parser>
inline typename Parser::result_type Puck::tryGetProperty(enum Property prop, bool* successful, int timeout_us) const {
	return tryGetProperty<Parser> (bus, id, getPropertyId(prop), successful, timeout_us);
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
	// TODO(dc): lock the bus mutex when waking Pucks?
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}
		(*i)->setProperty(STAT, STATUS_READY);

		// Talking on the CANbus when a transceiver goes offline can also cause
		// bus-off. Wait until this Puck drops off the bus before trying to
		// wake the next one.
		// TODO(dc): is there a more robust way of doing this?
		usleep(2000);
	}

	usleep(WAKE_UP_TIME);

	int stat;
	bool successful;
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}

		// Pucks can take longer to respond when in the process of waking up, so wait for 50ms.
		stat = (*i)->tryGetProperty(STAT, &successful, 50000);
		if (successful  &&  stat == STATUS_READY) {
			(*i)->updateStatus();
		} else {
			if (successful) {
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
	return getProperty<StandardParser>(bus, id, propId, realtime);
}
// TODO(dc): throw exception if getProperty is called with a group ID?
template<typename Parser>
typename Parser::result_type Puck::getProperty(const CommunicationsBus& bus, int id, int propId, bool realtime)
{
	int retCode;
	typename Parser::result_type value = getPropertyHelper<Parser>(bus, id, propId, true, realtime, &retCode, 0);
	if (retCode != 0) {
		syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d.", __func__, retCode);
		throw std::runtime_error("Puck::getProperty(): Failed to receive reply. Check /var/log/syslog for details.");
	}
	return value;
}

inline int Puck::tryGetProperty(const CommunicationsBus& bus, int id, int propId, bool* successful, int timeout_us)
{
	return tryGetProperty<StandardParser>(bus, id, propId, successful, timeout_us);
}
template<typename Parser>
typename Parser::result_type Puck::tryGetProperty(const CommunicationsBus& bus, int id, int propId, bool* successful, int timeout_us)
{
	int retCode;
	typename Parser::result_type value = getPropertyHelper<Parser>(bus, id, propId, false, false, &retCode, timeout_us);
	if (retCode == 0) {
		*successful = true;
	} else {
		*successful = false;
		if (retCode != 1) {  // some error other than "would block" occurred
			syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d.", __func__, retCode);
			throw std::runtime_error("Puck::tryGetProperty(): Receive error. Check /var/log/syslog for details.");
		}
	}

	return value;
}

template<typename Parser>
typename Parser::result_type Puck::getPropertyHelper(const CommunicationsBus& bus, int id, int propId, bool blocking, bool realtime, int* retCode, int timeout_us)
{
	*retCode = sendGetPropertyRequest(bus, id, propId);
	if (*retCode != 0) {
		syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, *retCode);
		throw std::runtime_error("Puck::getPropertyHelper(): Failed to send request. Check /var/log/syslog for details.");
	}

	if (timeout_us != 0) {
		usleep(timeout_us);
	}

	return receiveGetPropertyReply<Parser>(bus, id, propId, blocking, realtime, retCode);
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

template<typename Parser>
typename Parser::result_type Puck::receiveGetPropertyReply(const CommunicationsBus& bus, int id, int propId, bool blocking, bool realtime, int* retCode)
{
	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
	size_t len;

	*retCode = bus.receive(Parser::busId(id, propId), data, len, blocking, realtime);
	if (*retCode) {
		return typename Parser::result_type();
	}

	return Parser::parse(id, propId, data, len);
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

inline Puck::StandardParser::result_type Puck::StandardParser::parse(int id,
		int propId, const unsigned char* data, size_t len)
{
	bool err = false;
	if (len != 4 && len != 6) {
		syslog(
				LOG_ERR,
				"%s: expected message length of 4 or 6, got message length of %d",
				__func__, len);
		err = true;
	}
	if (!(data[0] & Puck::SET_MASK)) {
		syslog(LOG_ERR, "%s: expected SET command, got GET request", __func__);
		err = true;
	}
	if ((propId & Puck::PROPERTY_MASK) != (data[0] & Puck::PROPERTY_MASK)) {
		syslog(LOG_ERR, "%s: expected property = %d, got property %d",
				__func__, (int) (propId & Puck::PROPERTY_MASK), (int) (data[0]
						& Puck::PROPERTY_MASK));
		err = true;
	}
	if (data[1]) {
		syslog(LOG_ERR,
				"%s: expected second data byte to be 0, got value of %d",
				__func__, (int) data[1]);
		err = true;
	}

	if (err) {
		throw std::runtime_error("Puck::StandardParser::parse(): Unexpected "
			"message. Check /var/log/syslog for details.");
	}

	int value = (data[len - 1] & 0x80) ? -1 : 0;
	for (int i = len - 1; i >= 2; --i) {
		value = (value << 8) | data[i];
	}
	return value;
}


}
