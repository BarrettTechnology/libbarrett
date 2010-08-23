/*
 * puck.cpp
 *
 *  Created on: Aug 23, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <syslog.h>

#include "../can_socket.h"
#include "../puck.h"


namespace barrett {


Puck::Puck(const CANSocket& bus, int id) :
	bus(bus), id(id)
{
}

Puck::~Puck()
{
}

// TODO(dc): throw exception if getProperty is called with a group ID?
int Puck::getProperty(const CANSocket& bus, int id, int property)
{
	static const size_t MSG_LEN = 1;

	unsigned char data[MSG_LEN];
	data[0] = property & PROPERTY_MASK;

	bus.send(nodeId2BusId(id), data, MSG_LEN);


	int canIdIn;
	unsigned char dataIn[CANSocket::MAX_MESSAGE_LEN];
	size_t lenIn;
	bus.receive(canIdIn, dataIn, lenIn);
	int idIn = busId2NodeId(canIdIn);

	bool err = false;
	if (id != idIn) {
		syslog(LOG_ERR, "%s: expected message from NodeID = %d, got message from NodeID = %d (CANID = %d (%#X))", __func__, id, idIn, canIdIn, canIdIn);
		err = true;
	}
	if (lenIn != 4  &&  lenIn != 6) {
		syslog(LOG_ERR, "%s: expected message length of 4 or 6, got message length of %d", __func__, lenIn);
		err = true;
	}
	if ( !(dataIn[0] & SET_MASK) ) {
		syslog(LOG_ERR, "%s: expected SET command, got GET request", __func__);
		err = true;
	}
	if (data[0] != (dataIn[0] & PROPERTY_MASK)) {
		syslog(LOG_ERR, "%s: expected property = %d, got property %d", __func__, (int)data[0], (int)(dataIn[0] & PROPERTY_MASK));
		err = true;
	}
	if (dataIn[1]) {
		syslog(LOG_ERR, "%s: expected second data byte to be 0, got value of %d", __func__, (int)data[1]);
		err = true;
	}

	if (err) {
		throw std::runtime_error("Puck::getProperty(): Unexpected message. Check /var/log/syslog for details.");
	}


	int value = (dataIn[lenIn-1] & 0x80) ? -1 : 0;
	for (int i = lenIn-1; i >= 2; --i) {
		value = (value << 8) | dataIn[i];
	}
	return value;
}

void Puck::setProperty(const CANSocket& bus, int id, int property, int value)
{
	static const size_t MSG_LEN = 6;

	unsigned char data[MSG_LEN];
	data[0] = (property & PROPERTY_MASK) | SET_MASK;
	data[1] = 0;
	data[2] = (value & 0x000000ff);
	data[3] = (value & 0x0000ff00) >>  8;
	data[4] = (value & 0x00ff0000) >> 16;
	data[5] = (value & 0xff000000) >> 24;

	bus.send(nodeId2BusId(id), data, MSG_LEN);
}


}
