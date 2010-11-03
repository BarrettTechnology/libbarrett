/*
 * bus_manager.cpp
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <vector>
#include <algorithm>

#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <libgen.h>

#include <boost/thread/locks.hpp>
#include <boost/circular_buffer.hpp>

#include <libconfig.h++>

#include <barrett/detail/stl_utils.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/puck.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/bus_manager.h>


namespace barrett {


BusManager::BusManager(const char* configFile) :
	config(), bus(actualBus), pucks(), actualBus(), messageBuffers()
{
	char* cf1 = strdup(configFile);
	if (cf1 == NULL) {
		throw std::runtime_error("Out of memory.");
	}
	char* cf2 = strdup(configFile);
	if (cf2 == NULL) {
		free(cf1);
		throw std::runtime_error("Out of memory.");
	}
	char* origWd = get_current_dir_name();
	if (origWd == NULL) {
		free(cf1);
		free(cf2);
		throw std::runtime_error("Out of memory.");
	}

	// these functions require copies of the string because they sometimes modify their argument
	char* configDir = dirname(cf1);
	char* configBase = basename(cf2);

	// make @include paths in configFile relative to the containing directory
	chdir(configDir);

	try {
		config.readFile(configBase);
		bus.open(config.lookup("bus.port"));
	} catch (libconfig::ParseException pe) {
		printf("(%s:%d) %s\n", configFile, pe.getLine(), pe.getError());

		chdir(origWd);
		free(cf1);
		free(cf2);
		free(origWd);
		throw;
	} catch (...) {
		chdir(origWd);
		free(cf1);
		free(cf2);
		free(origWd);
		throw;
	}

	chdir(origWd);
	free(cf1);
	free(cf2);
	free(origWd);
}

BusManager::~BusManager()
{
	detail::purge(pucks);
}

void BusManager::enumerate()
{
	bool successful;
	int propId = Puck::getPropertyId(Puck::STAT, Puck::PT_Unknown, 0);
	Puck* p = NULL;
	int lastId = -1;

	syslog(LOG_ERR, "BusManager::enumerate()");
	for (int id = Puck::MIN_ID; id <= Puck::MAX_ID; ++id) {
		Puck::tryGetProperty(*this, id, propId, &successful);
		p = getPuck(id);

		if (successful) {
			// if the Puck doesn't exist, make it
			if (p == NULL) {
				p = new Puck(*this, id);
				pucks.push_back(p);
			} else {
				// if the Puck already exists (from a previous enumeration), update it
				p->updateRole();
				p->updateStatus();
			}

			if (lastId != id - 1  &&  lastId != -1) {
				syslog(LOG_ERR, "  --");  // marker to indicate that the listed IDs are not contiguous
			}
			syslog(LOG_ERR, "  ID=%2d VERS=%3d ROLE=0x%04x TYPE=%s%s",
					p->getId(), p->getVers(), p->getRole(),
					Puck::getPuckTypeStr(p->getType()),
					(p->getEffectiveType() == Puck::PT_Monitor) ? " (Monitor)" : "");
			lastId = id;
		} else if (p != NULL) {
			// if the Puck has disappeared since the last enumeration, remove it
			deletePuck(p);
		}
	}
}

Puck* BusManager::getPuck(int id)
{
	for (size_t i = 0; i < pucks.size(); ++i) {
		if (pucks[i]->getId() == id) {
			return pucks[i];
		}
	}
	return NULL;
}

void BusManager::deletePuck(Puck* p)
{
	std::vector<Puck*>::iterator i;
	i = std::find(pucks.begin(), pucks.end(), p);

	if (i == pucks.end()) {
		throw std::invalid_argument("BusManager::deletePuck(): Puck is not being managed by this BusManager.");
	}

	*i = pucks.back();
	pucks.pop_back();

	delete p;
}

int BusManager::receive(int expectedBusId, unsigned char* data, size_t& len, bool blocking, bool realtime) const
{
	boost::unique_lock<thread::Mutex> ul(getMutex());

	if (retrieveMessage(expectedBusId, data, len)) {
		return 0;
	}

	while (true) {
		updateBuffers();
		if (retrieveMessage(expectedBusId, data, len)) {
			return 0;
		} else if (!blocking) {
			return 1;
		}

		if (!realtime) {
			ul.unlock();
			usleep(100);
			ul.lock();
		}
	}
}

void BusManager::updateBuffers() const
{
	SCOPED_LOCK(getMutex());

	int busId;
	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
	size_t len;
	int ret;

	// empty the bus' receive buffer
	while (true) {
		ret = receiveRaw(busId, data, len, false);  // non-blocking read
		if (ret == 0) {  // successfully received a message
			storeMessage(busId, data, len);
		} else if (ret == 1) {  // would block
			return;
		} else {  // error
			syslog(LOG_ERR, "%s: BusManager::receiveRaw(): %d", __func__, ret);
			throw std::runtime_error("BusManager::updateBuffers(): receiveRaw() failed. Check /var/log/syslog for details.");
		}
	}
}

void BusManager::storeMessage(int busId, const unsigned char* data, size_t len) const
{
	if (messageBuffers[busId].full()) {
		syslog(LOG_ERR, "BusManager::storeMessage(): Buffer overflow. ID = %d", busId);
		throw std::runtime_error("BusManager::storeMessage(): Buffer overflow. Check /var/log/syslog for details.");
	}
	messageBuffers[busId].push_back(Message(data, len));
}

bool BusManager::retrieveMessage(int busId, unsigned char* data, size_t& len) const
{
	if (messageBuffers[busId].empty()) {
		return false;
	}

	messageBuffers[busId].front().copyTo(data, len);
	messageBuffers[busId].pop_front();

	return true;
}


}
