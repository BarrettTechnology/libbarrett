/*
 * puck_group-inl.h
 *
 *  Created on: Nov 1, 2010
 *      Author: dc
 */

namespace barrett {


inline std::vector<int> PuckGroup::getProperty(enum Puck::Property prop) const
{
	return getProperty<Puck::StandardParser>(prop);
}
template<typename Parser> std::vector<typename Parser::result_type>
		PuckGroup::getProperty(enum Puck::Property prop) const
{
	int ret;
	int propId = getPropertyId(prop);

	ret = Puck::sendGetPropertyRequest(bus, id, propId);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
		throw std::runtime_error("PuckGroup::getProperty(): Failed to send request. Check /var/log/syslog for details.");
	}

	std::vector<typename Parser::result_type> values(numPucks());
	for (size_t i = 0; i < numPucks(); ++i) {
		values[i] = Puck::receiveGetPropertyReply<Parser>(bus, pucks[i]->getId(), propId, true, &ret);
		if (ret != 0) {
			syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d while receiving message %d of %d for ID=%d.",
					__func__, ret, i, numPucks(), pucks[i]->getId());
			throw std::runtime_error("PuckGroup::getProperty(): Failed to receive reply. Check /var/log/syslog for details.");
		}
	}

	return values;
}

inline void PuckGroup::setProperty(enum Puck::Property prop, int value) const
{
	Puck::setProperty(bus, id, getPropertyId(prop), value);
}


}
