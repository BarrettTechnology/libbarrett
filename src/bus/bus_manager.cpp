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

#include <native/timer.h>

#include <boost/thread/locks.hpp>
#include <boost/circular_buffer.hpp>

#include <libconfig.h++>

#include <barrett/detail/stl_utils.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/products/puck.h>
#include <barrett/products/safety_module.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/real_time_execution_manager.h>
#include <barrett/systems/wam.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/bus_manager.h>


namespace barrett {


BusManager::BusManager(const char* configFile) :
	config(), bus(actualBus), pucks(), wamPucks(MAX_WAM_DOF), safetyModule(NULL), rtem(NULL), wam4(NULL), actualBus(), messageBuffers()
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

	enumerate();
}

BusManager::~BusManager()
{
	if (rtem != NULL) {
		if (rtem->isRunning()) {
			rtem->stop();
		}
		delete rtem;
	}
	delete wam4;
	delete safetyModule;
	detail::purge(pucks);
}

void BusManager::enumerate()
{
	bool successful;
	int propId = Puck::getPropertyId(Puck::STAT, Puck::PT_Unknown, 0);
	Puck* p = NULL;
	int lastId = -1;

	syslog(LOG_ERR, "BusManager::enumerate()");

	syslog(LOG_ERR, "  Pucks:");
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
				syslog(LOG_ERR, "    --");  // marker to indicate that the listed IDs are not contiguous
			}
			syslog(LOG_ERR, "    ID=%2d VERS=%3d ROLE=0x%04x TYPE=%s%s",
					p->getId(), p->getVers(), p->getRole(),
					Puck::getPuckTypeStr(p->getType()),
					(p->getEffectiveType() == Puck::PT_Monitor) ? " (Monitor)" : "");
			lastId = id;
		} else if (p != NULL) {
			// if the Puck has disappeared since the last enumeration, remove it
			deletePuck(p);
		}
	}


	// update wamPucks
	for (size_t i = 0; i < MAX_WAM_DOF; ++i) {
		wamPucks[i] = getPuck(i+1);
	}


	syslog(LOG_ERR, "  Products:");
	bool noProductsFound = true;
	bool wamFound = false;
	if (wam4Found()) {
		noProductsFound = false;
		wamFound = true;
		syslog(LOG_ERR, "    4-DOF WAM");
	}
	if (wam7Found()) {
		noProductsFound = false;
		wamFound = true;
		syslog(LOG_ERR, "    7-DOF WAM%s%s", wam7WristFound() ? " (Wrist)" : "", wam7GimbalsFound() ? " (Gimbals)" : "");
	}
	if (wamFound) {
		if (safetyModuleFound()) {
			syslog(LOG_ERR, "    Safety Module");
		} else {
			syslog(LOG_ERR, "    *** NO SAFETY MODULE ***");
		}
	}

	if (noProductsFound) {
		syslog(LOG_ERR, "    (none)");
	}
}

bool BusManager::safetyModuleFound() const
{
	return getPuck(SAFETY_PUCK_ID) != NULL;
}
SafetyModule* BusManager::getSafetyModule()
{
	if (safetyModule == NULL  &&  safetyModuleFound()) {
		safetyModule = new SafetyModule(getPuck(SAFETY_PUCK_ID));
	}
	return safetyModule;
}


const std::vector<Puck*>& BusManager::getWamPucks() const
{
	return wamPucks;
}

bool BusManager::wam4Found() const
{
	return verifyWamPucks(4);
}
bool BusManager::wam7Found() const
{
	return verifyWamPucks(7);
}
bool BusManager::wam7WristFound() const
{
	return wam7Found()  &&  getPuck(7)->getProperty(Puck::POLES) == 6;
}
bool BusManager::wam7GimbalsFound() const
{
	return wam7Found()  &&  getPuck(7)->getProperty(Puck::POLES) == 8;
}

void BusManager::waitForWam(bool promptOnZeroing)
{
	if ( !safetyModuleFound() ) {
		printf(">>> ERROR: No SafetyModule was found.\n");
		exit(1);
	}
	SafetyModule* sm = getSafetyModule();

	sm->waitForMode(SafetyModule::IDLE);
	if ( !wamFound() ) {
		enumerate();
		if ( !wamFound() ) {
			printf(">>> ERROR: No WAM was found.\n");
			exit(1);
		}
	}

	if (promptOnZeroing) {
		if ( !sm->wamIsZeroed() ) {
			printf(">>> The WAM needs to be zeroed. Please move it to its home position, then press [Enter].");
			detail::waitForEnter();
		}
	}

}

systems::Wam<4>* BusManager::getWam4(bool waitForShiftActivate, const char* configPath)
{
	if ( !wam4Found() ) {
		throw std::logic_error("BusManager::getWam4(): No WAM4 was found on the bus.");
	}

	getExecutionManager();  // Make an RTEM if one doesn't already exist
	if (wam4 == NULL) {
		std::vector<Puck*> wam4Pucks = wamPucks;
		wam4Pucks.resize(4);  // Discard all but the first 4 elements

		if (configPath == NULL) {
			configPath = "wam4";
		}
		wam4 = new systems::Wam<4>(wam4Pucks, getSafetyModule(), getConfig().lookup(configPath));
		if (rtem != NULL  &&  !rtem->isRunning()) {
			rtem->start();
		}
	}

	if (waitForShiftActivate) {
		if ( !safetyModuleFound() ) {
			throw std::logic_error("BusManager::getWam4(): No SafetyModule was found on the bus.");
		}
		getSafetyModule()->waitForMode(SafetyModule::ACTIVE, true, 50000);
	}

	return wam4;
}

systems::RealTimeExecutionManager* BusManager::getExecutionManager(double T_s)
{
	if (systems::System::defaultExecutionManager == NULL) {
		if (rtem == NULL) {
			rtem = new systems::RealTimeExecutionManager(T_s);
		}
		systems::System::defaultExecutionManager = rtem;
	}
	return rtem;
}

// TODO(dc): There has to be a better way of making both const/non-const versions of a member function.
Puck* BusManager::getPuck(int id)
{
	for (size_t i = 0; i < pucks.size(); ++i) {
		if (pucks[i]->getId() == id) {
			return pucks[i];
		}
	}
	return NULL;
}
const Puck* BusManager::getPuck(int id) const
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
	RTIME start = rt_timer_read();

	if (retrieveMessage(expectedBusId, data, len)) {
		return 0;
	}

	int ret;
	while (true) {
		ret = updateBuffers();
		if (ret != 0) {
			return ret;
		} else if (retrieveMessage(expectedBusId, data, len)) {
			return 0;
		} else if (!blocking) {
			return 1;
		}

		if ((rt_timer_read() - start) > CommunicationsBus::TIMEOUT) {
			syslog(LOG_ERR, "BusManager::receive(): timed out");
			return 2;
		}

		if (!realtime) {
			ul.unlock();
			usleep(100);
			ul.lock();
		}
	}
}

int BusManager::updateBuffers() const
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
			return 0;
		} else {  // error
			return ret;
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

bool BusManager::verifyWamPucks(const size_t dof) const
{
	if (dof > MAX_WAM_DOF) {
		return false;
	}

	for (size_t i = 0; i < MAX_WAM_DOF; ++i) {
		if ( (i < dof)  ^  (wamPucks[i] != NULL) ) {
			return false;
		}
	}

	return true;
}


}
