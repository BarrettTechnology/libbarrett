/*
 * bus_manager.cpp
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <string.h>
#include <unistd.h>
#include <libgen.h>

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
	bool successful;
	for (int id = 0; id <= Puck::MAX_ID; ++id) {
		Puck::sendGetPropertyRequest(bus, id, 5);  // TODO(dc): fix magic number once property definitions exist

		usleep(1000);

		Puck::receiveGetPropertyReply(bus, id, 5, false, &successful);  // TODO(dc): fix magic number once property definitions exist
		if (successful) {
			printf("Found ID=%d\n", id);
		}
	}
}


}
