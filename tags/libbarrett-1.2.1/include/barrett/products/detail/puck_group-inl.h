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
 * puck_group-inl.h
 *
 *  Created on: Nov 1, 2010
 *      Author: dc
 */

#include <barrett/os.h>
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
		(logMessage("PuckGroup::%s(): Failed to send request. "
				"Puck::sendGetPropertyRequest() returned error %d.")
				% __func__ % ret).raise<std::runtime_error>();
	}
}
template<typename Parser>
void PuckGroup::receiveGetPropertyReply(int propId, typename Parser::result_type results[], bool realtime) const
{
	int ret;
	for (size_t i = 0; i < numPucks(); ++i) {
		ret = Puck::receiveGetPropertyReply<Parser>(bus, pucks[i]->getId(), propId, &results[i], true, realtime);
		if (ret != 0) {
			(logMessage("PuckGroup::%s(): Failed to receive reply. "
					"Puck::receiveGetPropertyReply() returned error %d while receiving message from ID=%d (group reply %d of %d).")
					% __func__ % ret % pucks[i]->getId() % (i+1) % numPucks()).template raise<std::runtime_error>();
		}
	}
}


}
