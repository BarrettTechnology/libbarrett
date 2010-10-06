/*
 * puck.cpp
 *
 *  Created on: Aug 23, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <vector>

#include <syslog.h>
#include <unistd.h>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/puck.h>


namespace barrett {


Puck::Puck(const CommunicationsBus& _bus, int _id, enum PuckType pt) :
	bus(_bus), id(_id), type(pt), effectiveType(PT_Unknown), vers(-1)
{
	updateStatus();
}

Puck::~Puck()
{
}

void Puck::wake()
{
	updateStatus();
	if (effectiveType == PT_Monitor) {
		setProperty(STAT, STATUS_READY);
		usleep(300000);

		int stat;
		bool successful;
		do {
			usleep(100000);
			stat = tryGetProperty(STAT, &successful);
		} while (!successful);

		if (stat != STATUS_READY) {
			syslog(LOG_ERR, "%s: Failed to wake Puck ID=%d: STAT=%d", __func__, id, stat);
			throw std::runtime_error("Puck::wake(): Failed to wake Puck. Check /var/log/syslog for details.");
		}
		updateStatus();
	}
}

void Puck::updateStatus() {
	vers = getProperty(VERS);
	int stat = getProperty(STAT);
	switch (stat) {
	case STATUS_RESET:
		effectiveType = PT_Monitor;
		break;
	case STATUS_READY:
		effectiveType = type;
		break;
	default:
		syslog(LOG_ERR, "%s: unexpected value from ID=%d: STAT=%d.", __func__, id, stat);
		throw std::runtime_error("Puck::updateStatus(): Bad STAT value. Check /var/log/syslog for details.");
	}
}

// TODO(dc): throw exception if getProperty is called with a group ID?
int Puck::getProperty(const CommunicationsBus& bus, int id, int propId)
{
	bool successful;
	int ret = getPropertyHelper(bus, id, propId, true, &successful, 0);
	if (!successful) {
		syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d.", __func__, ret);
		throw std::runtime_error("Puck::getProperty(): Failed to receive reply. Check /var/log/syslog for details.");
	}
	return ret;
}

int Puck::tryGetProperty(const CommunicationsBus& bus, int id, int propId, bool* successful, int timeout_us)
{
	int ret = getPropertyHelper(bus, id, propId, false, successful, timeout_us);
	if (!(*successful)  &&  ret != 1) {  // some error other than "would block" occurred
		syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d.", __func__, ret);
		throw std::runtime_error("Puck::tryGetProperty(): Receive error. Check /var/log/syslog for details.");
	}
	return ret;
}

int Puck::getPropertyHelper(const CommunicationsBus& bus, int id, int propId,	bool blocking, bool* successful, int timeout_us)
{
	int ret = sendGetPropertyRequest(bus, id, propId);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
		throw std::runtime_error("Puck::getPropertyHelper(): Failed to send request. Check /var/log/syslog for details.");
	}

	if (timeout_us != 0) {
		usleep(timeout_us);
	}

	return receiveGetPropertyReply(bus, id, propId, blocking, successful);
}

void Puck::setProperty(const CommunicationsBus& bus, int id, int propId, int value)
{
	static const size_t MSG_LEN = 6;

	unsigned char data[MSG_LEN];
	data[0] = (propId & PROPERTY_MASK) | SET_MASK;
	data[1] = 0;
	data[2] = (value & 0x000000ff);
	data[3] = (value & 0x0000ff00) >>  8;
	data[4] = (value & 0x00ff0000) >> 16;
	data[5] = (value & 0xff000000) >> 24;

	bus.send(nodeId2BusId(id), data, MSG_LEN);
}

int Puck::sendGetPropertyRequest(const CommunicationsBus& bus, int id, int propId)
{
	static const size_t MSG_LEN = 1;

	unsigned char data[MSG_LEN];
	data[0] = propId & PROPERTY_MASK;

	return bus.send(nodeId2BusId(id), data, MSG_LEN);
}

int Puck::receiveGetPropertyReply(const CommunicationsBus& bus, int id, int propId, bool blocking, bool* successful)
{
	unsigned char dataIn[CommunicationsBus::MAX_MESSAGE_LEN];
	size_t lenIn;
	int ret = bus.receive(encodeBusId(id, 0x406), dataIn, lenIn, blocking);  // TODO(dc): fix magic number and generalize
	if (ret) {
		*successful = false;
		return ret;
	} else {
		*successful = true;
	}

	bool err = false;
	if (lenIn != 4  &&  lenIn != 6) {
		syslog(LOG_ERR, "%s: expected message length of 4 or 6, got message length of %d", __func__, lenIn);
		err = true;
	}
	if ( !(dataIn[0] & SET_MASK) ) {
		syslog(LOG_ERR, "%s: expected SET command, got GET request", __func__);
		err = true;
	}
	if ((propId & PROPERTY_MASK) != (dataIn[0] & PROPERTY_MASK)) {
		syslog(LOG_ERR, "%s: expected property = %d, got property %d", __func__, (int)(propId & PROPERTY_MASK), (int)(dataIn[0] & PROPERTY_MASK));
		err = true;
	}
	if (dataIn[1]) {
		syslog(LOG_ERR, "%s: expected second data byte to be 0, got value of %d", __func__, (int)dataIn[1]);
		err = true;
	}

	if (err) {
		throw std::runtime_error("Puck::receiveGetPropertyReply(): Unexpected message. Check /var/log/syslog for details.");
	}


	int value = (dataIn[lenIn-1] & 0x80) ? -1 : 0;
	for (int i = lenIn-1; i >= 2; --i) {
		value = (value << 8) | dataIn[i];
	}
	return value;
}


}
