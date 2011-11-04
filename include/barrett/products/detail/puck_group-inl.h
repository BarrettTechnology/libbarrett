/*
 * puck_group-inl.h
 *
 *  Created on: Nov 1, 2010
 *      Author: dc
 */

#include <boost/thread/locks.hpp>


namespace barrett {


inline void PuckGroup::getProperty(enum Puck::Property prop, int results[], bool realtime) const
{
	getProperty<Puck::StandardParser>(prop, results, realtime);
}
template<typename Parser> void PuckGroup::getProperty(enum Puck::Property prop, typename Parser::result_type results[], bool realtime) const
{
	boost::unique_lock<thread::Mutex> ul(bus.getMutex(), boost::defer_lock);
	if (realtime) {
		ul.lock();
	}

	int propId = getPropertyId(prop);
	sendGetPropertyRequest(propId);
	receiveGetPropertyReply<Parser>(propId, results, realtime);
}

inline void PuckGroup::setProperty(enum Puck::Property prop, int value) const
{
	Puck::setProperty(bus, id, getPropertyId(prop), value);
}

inline void PuckGroup::sendGetPropertyRequest(int propId) const
{
	int ret = Puck::sendGetPropertyRequest(bus, id, propId);
	if (ret != 0) {
		syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
		throw std::runtime_error("PuckGroup::sendGetPropertyRequest(): Failed to send request. Check /var/log/syslog for details.");
	}
}
template<typename Parser>
void PuckGroup::receiveGetPropertyReply(int propId, typename Parser::result_type results[], bool realtime) const
{
	int ret;
	for (size_t i = 0; i < numPucks(); ++i) {
		ret = Puck::receiveGetPropertyReply<Parser>(bus, pucks[i]->getId(), propId, &results[i], true, realtime);
		if (ret != 0) {
			syslog(LOG_ERR, "%s: Puck::receiveGetPropertyReply() returned error %d while receiving message %d of %d for ID=%d.",
					__func__, ret, i+1, numPucks(), pucks[i]->getId());
			throw std::runtime_error("PuckGroup::receiveGetPropertyReply(): Failed to receive reply. Check /var/log/syslog for details.");
		}
	}
}


}
