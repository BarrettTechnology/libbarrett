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
#include <barrett/puck_group.h>


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


}
