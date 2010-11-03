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
#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>


namespace barrett {


Puck::Puck(const CommunicationsBus& _bus, int _id) :
	bus(_bus), id(_id), vers(-1), role(-1), type(PT_Unknown), effectiveType(PT_Unknown)
{
	if ((id & NODE_ID_MASK) != id) {
		throw std::invalid_argument("Puck::Puck(): Invalid Node ID.");
	}

	updateRole();
	updateStatus();
}

Puck::~Puck()
{
}

void Puck::wake()
{
	wake(std::vector<Puck*>(1, this));
}

void Puck::updateRole()
{
	role = getProperty(ROLE);
	switch (role & ROLE_MASK) {
	case ROLE_SAFETY:
		type = PT_Safety;
		break;
	case ROLE_TATER:
	case ROLE_BHAND:
	case ROLE_WRAPTOR:
		type = PT_Motor;
		break;
	case ROLE_FORCE:
		type = PT_ForceTorque;
		break;
	default:
		type = PT_Unknown;
		break;
	}
}

void Puck::updateStatus()
{
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
		break;
	}
}

const char Puck::puckTypeStrs[][12] = { "Monitor", "Safety", "Motor", "ForceTorque", "Unknown" };


Puck::StandardParser::result_type Puck::StandardParser::parse(int id,
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
