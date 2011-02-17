/*
 * product_manager.cpp
 *
 *  Created on: Jan 3, 2011
 *      Author: dc
 */

#include <stdexcept>
#include <vector>
#include <algorithm>

#include <string.h>
#include <syslog.h>
#include <libgen.h>

#include <boost/thread/locks.hpp>
#include <boost/circular_buffer.hpp>

#include <libconfig.h++>

#include <barrett/detail/stl_utils.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/products/puck.h>
#include <barrett/products/hand.h>
#include <barrett/products/gimbals_hand_controller.h>
#include <barrett/products/safety_module.h>
#include <barrett/products/force_torque_sensor.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/real_time_execution_manager.h>
#include <barrett/systems/wam.h>
#include <barrett/products/product_manager.h>


namespace barrett {


const char ProductManager::DEFAULT_CONFIG_FILE[] = "/etc/barrett/default.conf";

ProductManager::ProductManager(bus::CommunicationsBus* _bus, const char* configFile) :
	config(), bus(_bus), deleteBus(false),
	pucks(), wamPucks(MAX_WAM_DOF), handPucks(Hand::DOF),
	sm(NULL), rtem(NULL), wam4(NULL), wam7(NULL), fts(NULL), hand(NULL), ghc(NULL)
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

		if (bus == NULL) {
			bus = new bus::BusManager;
			deleteBus = true;
		}
		if ( !bus->isOpen() ) {
			bus->open(config.lookup("bus.port"));
		}
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

ProductManager::~ProductManager()
{
	destroyEstopProducts();
	delete sm;
	sm = NULL;
	detail::purge(pucks);
	if (deleteBus) {
		delete bus;
		bus = NULL;
	}
}

void ProductManager::enumerate()
{
	int ret, result;
	int propId = Puck::getPropertyId(Puck::STAT, Puck::PT_Unknown, 0);
	Puck* p = NULL;
	int lastId = -1;

	syslog(LOG_ERR, "ProductManager::enumerate()");

	syslog(LOG_ERR, "  Pucks:");
	for (int id = Puck::MIN_ID; id <= Puck::MAX_ID; ++id) {
		ret = Puck::tryGetProperty(*bus, id, propId, &result);
		p = getPuck(id);

		if (ret == 0) {
			// if the Puck doesn't exist, make it
			if (p == NULL) {
				p = new Puck(*bus, id);
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


	// update WAM/Hand Pucks
	for (size_t i = 0; i < MAX_WAM_DOF; ++i) {
		wamPucks[i] = getPuck(i + FIRST_WAM_ID);
	}
	for (size_t i = 0; i < Hand::DOF; ++i) {
		handPucks[i] = getPuck(i + FIRST_HAND_ID);
	}


	syslog(LOG_ERR, "  Products:");
	bool noProductsFound = true;
	bool wamFound = false;
	if (foundWam4()) {
		noProductsFound = false;
		wamFound = true;
		syslog(LOG_ERR, "    4-DOF WAM");
	}
	if (foundWam7()) {
		noProductsFound = false;
		wamFound = true;
		syslog(LOG_ERR, "    7-DOF WAM%s%s", foundWam7Wrist() ? " (Wrist)" : "", foundWam7Gimbals() ? " (Gimbals)" : "");
	}
	if (wamFound) {
		if (foundSafetyModule()) {
			syslog(LOG_ERR, "    Safety Module");
		} else {
			syslog(LOG_ERR, "    *** NO SAFETY MODULE ***");
		}
	}
	if (foundForceTorqueSensor()) {
		noProductsFound = false;
		syslog(LOG_ERR, "    Force-Torque Sensor");
	}
	if (foundHand()) {
		noProductsFound = false;
		syslog(LOG_ERR, "    BarrettHand");
	}
	// TODO(dc): Don't report GHC because we don't have a very selective test.

	if (noProductsFound) {
		syslog(LOG_ERR, "    (none)");
	}
}

void ProductManager::cleanUpAfterEstop()
{
	destroyEstopProducts();
	enumerate();
}
void ProductManager::destroyEstopProducts()
{
	if (rtem != NULL) {
		if (rtem->isRunning()) {
			rtem->stop();
		}
		if (systems::System::defaultExecutionManager == rtem) {
			systems::System::defaultExecutionManager = NULL;
		}
		delete rtem;
		rtem = NULL;
	}
	delete wam4;
	wam4 = NULL;
	delete wam7;
	wam7 = NULL;
	delete fts;
	fts = NULL;
	delete hand;
	hand = NULL;
	delete ghc;
	ghc = NULL;
}


bool ProductManager::foundSafetyModule() const
{
	return getPuck(SAFETY_MODULE_ID) != NULL;
}
SafetyModule* ProductManager::getSafetyModule()
{
	if (sm == NULL  &&  foundSafetyModule()) {
		sm = new SafetyModule(getPuck(SAFETY_MODULE_ID));
	}
	return sm;
}


const std::vector<Puck*>& ProductManager::getWamPucks() const
{
	return wamPucks;
}

bool ProductManager::foundWam4() const
{
	return verifyWamPucks(4);
}
bool ProductManager::foundWam7() const
{
	return verifyWamPucks(7);
}
bool ProductManager::foundWam7Wrist() const
{
	return wam7FoundHelper(6);
}
bool ProductManager::foundWam7Gimbals() const
{
	return wam7FoundHelper(8);
}
bool ProductManager::wam7FoundHelper(int poles) const
{
	if (foundWam7()) {
		Puck* p7 = getPuck(7);
		p7->wake();
		return p7->getProperty(Puck::POLES) == poles;
	} else {
		return false;
	}
}

void ProductManager::waitForWam(bool promptOnZeroing)
{
	if ( !foundSafetyModule() ) {
		printf(">>> ERROR: No SafetyModule was found.\n");
		exit(1);
	}
	SafetyModule* sm = getSafetyModule();

	sm->waitForMode(SafetyModule::IDLE);
	if ( !foundWam() ) {
		enumerate();
		if ( !foundWam() ) {
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
const char* ProductManager::getWamDefaultConfigPath()
{
	if (foundWam4()) {
		return "wam4";
	} else if (foundWam7Wrist()) {
		return "wam7w";
	} else if (foundWam7Gimbals()) {
		return "wam7g";
	} else {
		throw std::logic_error("ProductManager::getWamDefaultConfigPath(): No WAM found.");
	}
}

systems::Wam<4>* ProductManager::getWam4(bool waitForShiftActivate, const char* configPath)
{
	if ( !foundWam4() ) {
		throw std::logic_error("ProductManager::getWam4(): No WAM4 was found on the bus.");
	}

	getExecutionManager();  // Make an RTEM if one doesn't already exist
	if (wam4 == NULL) {
		std::vector<Puck*> wam4Pucks = wamPucks;
		wam4Pucks.resize(4);  // Discard all but the first 4 elements

		if (configPath == NULL) {
			configPath = getWamDefaultConfigPath();
		}
		wam4 = new systems::Wam<4>(wam4Pucks, getSafetyModule(), getConfig().lookup(configPath));
		if (rtem != NULL  &&  !rtem->isRunning()) {
			rtem->start();
		}
	}

	if (waitForShiftActivate) {
		if ( !foundSafetyModule() ) {
			throw std::logic_error("ProductManager::getWam4(): No SafetyModule was found on the bus.");
		}

		// Check rapidly in case the user wants to perform some action (like
		// enabling gravity compensation) immediately after Shift-activate.
		getSafetyModule()->waitForMode(SafetyModule::ACTIVE, true, 50000);
	}

	return wam4;
}

systems::Wam<7>* ProductManager::getWam7(bool waitForShiftActivate, const char* configPath)
{
	if ( !foundWam7() ) {
		throw std::logic_error("ProductManager::getWam7(): No WAM7 was found on the bus.");
	}

	getExecutionManager();  // Make an RTEM if one doesn't already exist
	if (wam7 == NULL) {
		std::vector<Puck*> wam7Pucks = wamPucks;
		wam7Pucks.resize(7);  // Discard all but the first 7 elements

		if (configPath == NULL) {
			configPath = getWamDefaultConfigPath();
		}
		wam7 = new systems::Wam<7>(wam7Pucks, getSafetyModule(), getConfig().lookup(configPath));
		if (rtem != NULL  &&  !rtem->isRunning()) {
			rtem->start();
		}
	}

	if (waitForShiftActivate) {
		if ( !foundSafetyModule() ) {
			throw std::logic_error("ProductManager::getWam7(): No SafetyModule was found on the bus.");
		}

		// Check rapidly in case the user wants to perform some action (like
		// enabling gravity compensation) immediately after Shift-activate.
		getSafetyModule()->waitForMode(SafetyModule::ACTIVE, true, 50000);
	}

	return wam7;
}

systems::RealTimeExecutionManager* ProductManager::getExecutionManager(double period_s, int rt_priority)
{
	if (systems::System::defaultExecutionManager == NULL) {
		if (rtem == NULL) {
			rtem = new systems::RealTimeExecutionManager(period_s, rt_priority);
		}
		systems::System::defaultExecutionManager = rtem;
	}
	return rtem;
}


bool ProductManager::foundForceTorqueSensor() const
{
	return getPuck(FORCE_TORQUE_SENSOR_ID) != NULL;
}
ForceTorqueSensor* ProductManager::getForceTorqueSensor()
{
	if (fts == NULL  &&  foundForceTorqueSensor()) {
		fts = new ForceTorqueSensor(getPuck(FORCE_TORQUE_SENSOR_ID));
	}
	return fts;
}


const std::vector<Puck*>& ProductManager::getHandPucks() const
{
	return handPucks;
}
bool ProductManager::foundHand() const
{
	return std::find(handPucks.begin(), handPucks.end(), (Puck*)NULL) == handPucks.end();
}
Hand* ProductManager::getHand()
{
	if (hand == NULL  &&  foundHand()) {
		hand = new Hand(handPucks);
	}
	return hand;
}


bool ProductManager::foundGimbalsHandController() const
{
	return foundWam7Gimbals();
}
GimbalsHandController* ProductManager::getGimbalsHandController()
{
	if (ghc == NULL  &&  foundGimbalsHandController()) {
		ghc = new GimbalsHandController(wamPucks[5], wamPucks[6]);
	}
	return ghc;
}


Puck* ProductManager::getPuck(int id) const
{
	for (size_t i = 0; i < pucks.size(); ++i) {
		if (pucks[i]->getId() == id) {
			return pucks[i];
		}
	}
	return NULL;
}

void ProductManager::deletePuck(Puck* p)
{
	std::vector<Puck*>::iterator i;
	i = std::find(pucks.begin(), pucks.end(), p);

	if (i == pucks.end()) {
		throw std::invalid_argument("ProductManager::deletePuck(): Puck is not being managed by this ProductManager.");
	}

	*i = pucks.back();
	pucks.pop_back();

	delete p;
}

bool ProductManager::verifyWamPucks(const size_t dof) const
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
