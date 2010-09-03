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

#include <boost/circular_buffer.h>

#include <libconfig.h++>

#include "../../puck.h"
#include "../abstract/communications_bus.h"
#include "../can_socket.h"
#include "../bus_manager.h"


#include <cstdio>
namespace barrett {


BusManager::BusManager(const char* configFile) :
	bus(actualBus), actualBus()
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
	for (int id = 0; id <= Puck::MAX_ID; ++id) {
		ret = Puck::sendGetPropertyRequest(bus, id, 5);  // TODO(dc): fix magic number once property definitions exist
		if (ret != 0) {
			syslog(LOG_ERR, "%s: Puck::sendGetPropertyRequest() returned error %d.", __func__, ret);
			throw std::runtime_error("BusManager::enumerate(): Failed to send request. Check /var/log/syslog for details.");
		}

		usleep(1000);

		Puck::receiveGetPropertyReply(bus, id, 5, false, &successful);  // TODO(dc): fix magic number once property definitions exist
		if (successful) {
			printf("Found ID=%d\n", id);
		}
	}
}

int BusManager::receive(int expectedBusId, unsigned char* data, size_t& len, bool blocking) const
{
	if (retrieveMessage(expectedBusId, data, len)) {
		return 0;
	}
	do {
		updateBuffers(blocking);
		if (retrieveMessage(expectedBusId, data, len)) {
			return 0;
		}
	} while (blocking);

	return 1;
}

int BusManager::updateBuffers(bool blocking) const
{
	int busId;
	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
	size_t len;
	int ret;

	bool receivedMessage = false;

	// empty the bus' receive buffer
	while (true) {
		ret = receiveRaw(busId, data, len, false);  // non-blocking read
		if (ret == 0) {  // successfully received a message
			storeMessage(busId, data, len);
			receivedMessage = true;
		} else if (ret == 1) {  // would block
			break;
		} else {  // error
			return ret;
		}
	}

	// if we're supposed to block but haven't received a message yet, block until we do
	if (blocking  &&  !receivedMessage) {
		ret = receiveRaw(busId, data, len, true);  // blocking read
		if (ret != 0) {
			return ret;
		}
		storeMessage(busId, data, len);
	}

	return 0;
}

void BusManager::storeMessage(int busId, unsigned char* data, size_t len) const
{

}

bool BusManager::retrieveMessage(int busId, unsigned char* data, size_t& len) const
{

}


}
