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

	allPucksAwake = true;
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}
		(*i)->updateStatus();
		if ((*i)->getEffectiveType() == PT_Monitor) {
			allPucksAwake = false;
			(*i)->setProperty(STAT, STATUS_READY);
		} else {
			*i = NULL;
		}
	}

	if (allPucksAwake) {
		return;
	}

	usleep(300000);

	int stat;
	bool successful;
	while (!allPucksAwake) {
		allPucksAwake = true;

		usleep(100000);
		for (i = pucks.begin(); i != pucks.end(); ++i) {
			if (*i == NULL) {
				continue;
			}

			allPucksAwake = false;
			stat = (*i)->tryGetProperty(STAT, &successful);
			if (successful) {
				if (stat == STATUS_READY) {
					(*i)->updateStatus();
					*i = NULL;
				} else {
					syslog(LOG_ERR, "%s: Failed to wake Puck ID=%d: STAT=%d", __func__, (*i)->getId(), stat);
					throw std::runtime_error("Puck::wake(): Failed to wake Puck. Check /var/log/syslog for details.");
				}
			}
		}
	}
}

inline int Puck::getPropertyId(enum Property prop, enum PuckType pt, int fwVers)
		throw(std::runtime_error)
{
	int propId = getPropertyIdNoThrow(prop, pt, fwVers);
	if (propId == -1) {
		syslog(LOG_ERR, "Puck::getPropertyId(): Pucks with type %d and firmware version %d do not respond to property %d.", pt, fwVers, prop);
		throw std::runtime_error("Puck::getPropertyId(): Invalid property. Check /var/log/syslog for details.");
	}
	return propId;
}


}
