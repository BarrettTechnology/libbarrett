/*
 * bus_manager.cpp
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <libgen.h>

#include <boost/thread/locks.hpp>
#include <boost/circular_buffer.hpp>

#include <libconfig.h++>

#include <barrett/thread/abstract/mutex.h>
#include <barrett/puck.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/bus_manager.h>


#include <cstdio>
namespace barrett {


BusManager::BusManager(const char* configFile) :
	bus(actualBus), actualBus(), messageBuffers()
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
		libconfig::Config config;
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
}

void BusManager::enumerate()
{
	int ret;
	bool successful;
	int propId = Puck::getPropertyId(Puck::STAT, Puck::PT_Unknown, 0);

	for (int id = Puck::MIN_ID; id <= Puck::MAX_ID; ++id) {
		ret = Puck::sendGetPropertyRequest(*this, id, propId);
		if (ret != 0) {
			syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
			throw std::runtime_error("BusManager::enumerate(): Failed to send request. Check /var/log/syslog for details.");
		}

		usleep(1000);

		Puck::receiveGetPropertyReply(*this, id, propId, false, &successful);
		if (successful) {
			printf("Found ID=%d\n", id);
		}
	}
}

int BusManager::receive(int expectedBusId, unsigned char* data, size_t& len, bool blocking, bool realtime) const
{
	boost::unique_lock<thread::Mutex> ul(getMutex());

	if (retrieveMessage(expectedBusId, data, len)) {
		return 0;
	}

	while (true) {
		updateBuffers(blocking);
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

int BusManager::updateBuffers(bool blocking) const
{
	SCOPED_LOCK(getMutex());

	int busId;
	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
	size_t len;
	int ret;

	bool messageReceived = false;

	// empty the bus' receive buffer
	while (true) {
		ret = receiveRaw(busId, data, len, false);  // non-blocking read
		if (ret == 0) {  // successfully received a message
			storeMessage(busId, data, len);
			messageReceived = true;
		} else if (ret == 1) {  // would block
			break;
		} else {  // error
			return ret;
		}
	}

	// if we're supposed to block but haven't received a message yet, block until we do
	if (blocking  &&  !messageReceived) {
		ret = receiveRaw(busId, data, len, true);  // blocking read
		if (ret != 0) {
			return ret;
		}
		storeMessage(busId, data, len);
	}

	return 0;
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
