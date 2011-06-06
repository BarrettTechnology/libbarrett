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
#include <native/task.h>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>


namespace barrett {


Puck::Puck(const bus::CommunicationsBus& _bus, int _id) :
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

void Puck::saveProperty(enum Property prop) const
{
	setProperty(SAVE, getPropertyId(prop), true);
}
void Puck::resetProperty(enum Property prop) const
{
	setProperty(DEF, getPropertyId(prop));
	setProperty(LOAD, getPropertyId(prop), true);
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


void Puck::wake(std::vector<Puck*> pucks)
{
	std::vector<Puck*>::iterator i;
	bool allPucksAwake;
	Puck* aNonNullPuck = NULL;

	// Find the Pucks that need to be woken up
	allPucksAwake = true;
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}
//		(*i)->updateStatus();
		if ((*i)->getEffectiveType() == PT_Monitor) {
			allPucksAwake = false;
			aNonNullPuck = *i;
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
		BARRETT_SCOPED_LOCK(aNonNullPuck->getBus().getMutex());

		for (i = pucks.begin(); i != pucks.end(); ++i) {
			if (*i == NULL) {
				continue;
			}
			(*i)->setProperty(STAT, STATUS_READY);

			// Talking on the CANbus when a transceiver goes offline can also cause
			// bus-off. Wait until this Puck drops off the bus before trying to
			// wake the next one.
			// TODO(dc): is there a more robust way of doing this?
			rt_task_sleep(TURN_OFF_TIME);
		}

		rt_task_sleep(WAKE_UP_TIME);
	}

	int ret;
	int stat;
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}

		// Pucks can take longer to respond when in the process of waking up, so wait for 50ms.
		ret = (*i)->tryGetProperty(STAT, &stat, 50000000);
		if (ret == 0  &&  stat == STATUS_READY) {
			(*i)->updateStatus();
		} else {
			if (ret == 0) {
				syslog(LOG_ERR, "%s: Failed to wake Puck ID=%d: STAT=%d", __func__, (*i)->getId(), stat);
			} else {
				syslog(LOG_ERR, "%s: Failed to wake Puck ID=%d: No response after waiting %.2fs.", __func__, (*i)->getId(), WAKE_UP_TIME/1e9);
			}
			throw std::runtime_error("Puck::wake(): Failed to wake Puck. Check /var/log/syslog for details.");
		}
	}
}


int Puck::StandardParser::parse(int id,
		int propId, result_type* result, const unsigned char* data, size_t len)
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
	if (data[1] != 0) {
		syslog(LOG_ERR,
				"%s: expected second data byte to be 0, got value of %d",
				__func__, (int) data[1]);
		err = true;
	}

	if (err) {
		return 1;
	}


	*result = (data[len - 1] & 0x80) ? -1 : 0;
	for (int i = len - 1; i >= 2; --i) {
		*result = (*result << 8) | data[i];
	}
	return 0;
}


}
