/*
 * puck_group.cpp
 *
 *  Created on: Oct 7, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <vector>

#include <barrett/puck.h>
#include <barrett/puck_group.h>


namespace barrett {


PuckGroup::PuckGroup(int _id, const std::vector<Puck*>& _pucks) :
	id(_id), pucks(_pucks), bus(pucks[0]->getBus())
{
	if ( !(id & Puck::GROUP_MASK)  ||  (id & Puck::TO_MASK) != id) {
		throw std::invalid_argument("PuckGroup::PuckGroup(): Invalid Group ID.");
	}
}

PuckGroup::~PuckGroup()
{
}

bool PuckGroup::verifyProperty(enum Puck::Property prop) const
{
	int propId = getPropertyIdNoThrow(prop);

	if (propId == -1) {
		return false;
	}
	for (size_t i = 1; i < numPucks(); ++i) {
		if (pucks[i]->getPropertyIdNoThrow(prop) != propId) {
			return false;
		}
	}

	return true;
}

std::vector<int> PuckGroup::getProperty(enum Puck::Property prop) const
{
	int propId = getPropertyId(prop);

	int ret = Puck::sendGetPropertyRequest(bus, id, propId);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
		throw std::runtime_error("PuckGroup::getProperty(): Failed to send request. Check /var/log/syslog for details.");
	}

	std::vector<int> values(numPucks());
	bool successful;
	for (size_t i = 0; i < numPucks(); ++i) {
		values[i] = Puck::receiveGetPropertyReply(bus, pucks[i]->getId(), propId, true, &successful);
		if (!successful) {
			syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d while receiving message %d of %d for ID=%d.",
					__func__, values[i], i, numPucks(), pucks[i]->getId());
			throw std::runtime_error("PuckGroup::getProperty(): Failed to receive reply. Check /var/log/syslog for details.");
		}
	}

	return values;
}

void PuckGroup::setProperty(enum Puck::Property prop, int value) const
{
	Puck::setProperty(bus, id, getPropertyId(prop), value);
}


}
