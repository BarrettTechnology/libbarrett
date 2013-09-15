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
 * puck.cpp
 *
 *  Created on: Aug 23, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <vector>

#include <barrett/os.h>
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
void Puck::saveAllProperties() const
{
	setProperty(SAVE, -1, true);
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
		(logMessage("Puck::%s(): Bad STAT value. "
				"ID=%d, STAT=%d.")
				% __func__ % id % stat).raise<std::runtime_error>();
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
			btsleepRT(TURN_OFF_TIME);
		}

		btsleepRT(WAKE_UP_TIME);
	}

	int ret;
	int stat;
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}

		// Pucks can take longer to respond when in the process of waking up, so wait for 50ms.
		Puck& p = **i;
		ret = Puck::tryGetProperty(p.getBus(), p.getId(), p.getPropertyId(Puck::STAT), &stat, 0.05);
		if (ret == 0  &&  stat == STATUS_READY) {
			(*i)->updateStatus();
		} else {
			if (ret == 0) {
				(logMessage("Puck::%s(): Failed to wake Puck ID=%d. "
						"STAT=%d.")
						% __func__ % p.getId() % stat).raise<std::runtime_error>();
			} else {
				const double wakeUpTime = WAKE_UP_TIME;
				(logMessage("Puck::%s(): Failed to wake Puck ID=%d. "
						"No response after waiting %.2fs.")
						% __func__ % p.getId() % wakeUpTime).raise<std::runtime_error>();
			}
		}
	}
}


int Puck::StandardParser::parse(int id,
		int propId, result_type* result, const unsigned char* data, size_t len)
{
	bool err = false;
	if (len != 4 && len != 6) {
		logMessage("%s: expected message length of 4 or 6, got message length of %d")
				% __func__ % len;
		err = true;
	}
	if (!(data[0] & Puck::SET_MASK)) {
		logMessage("%s: expected SET command, got GET request") % __func__;
		err = true;
	}
	if ((propId & Puck::PROPERTY_MASK) != (data[0] & Puck::PROPERTY_MASK)) {
		logMessage("%s: expected property = %d, got property %d")
				% __func__ % (propId & Puck::PROPERTY_MASK) % (data[0] & Puck::PROPERTY_MASK);
		err = true;
	}
	if (data[1] != 0) {
		logMessage("%s: expected second data byte to be 0, got value of %d")
				% __func__ % data[1];
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
