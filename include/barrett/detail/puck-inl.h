/*
 * puck-inl.h
 *
 *  Created on: Oct 5, 2010
 *      Author: dc
 */

namespace barrett {


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


}
