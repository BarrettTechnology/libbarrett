/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */

/*
 * @file product_manager.cpp
 *
 * @date Jan 3, 2011
 * @author D.Cody
 * @author J.Hagstrand
 */

#include <stdexcept>
#include <vector>
#include <algorithm>
#include <cstdlib>

#include <string.h>
#include <libgen.h>

#include <libconfig.h++>

#include <barrett/os.h>
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

#include <barrett/config.h>

namespace barrett {


const std::string ProductManager::DEFAULT_CONFIG_FILE = barrett::EtcPathRelative("default.conf");

ProductManager::ProductManager(const char* configFile, bus::CommunicationsBus* _bus) :
	config(), bus(_bus), deleteBus(false),
	pucks(), wamPucks(MAX_WAM_DOF), handPucks(Hand::DOF),
	sm(NULL), rtem(NULL), wam3(NULL), wam4(NULL), wam7(NULL), fts(NULL), hand(NULL), ghc(NULL)
{
	int ret;


	logMessage("ProductManager::%s()") % __func__;

	char cfSource[8] = "param";
	if (configFile == NULL  ||  configFile[0] == '\0') {
		configFile = std::getenv("BARRETT_CONFIG_FILE");
		if (configFile == NULL  ||  configFile[0] == '\0') {
			configFile = DEFAULT_CONFIG_FILE.c_str();
			strcpy(cfSource, "default");
		} else {
			strcpy(cfSource, "env");
		}
	}
	logMessage("  Config file (from %s): %s") % cfSource % configFile;


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
	ret = chdir(configDir);
	if (ret != 0) {
		free(cf1);
		free(cf2);
		free(origWd);
		throw std::runtime_error("Couldn't change to the config directory.");
	}

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
		printf("\n>>> CONFIG FILE ERROR on line %d of %s: \"%s\"\n\n", pe.getLine(), configFile, pe.getError());
		printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
		printf("This error usually means that a configuration file is corrupted or missing.\n");
		printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");

		ret = chdir(origWd);  // Ignore ret value
		free(cf1);
		free(cf2);
		free(origWd);
		throw;
	} catch (...) {
		ret = chdir(origWd);  // Ignore ret value
		free(cf1);
		free(cf2);
		free(origWd);
		throw;
	}

	ret = chdir(origWd);
	free(cf1);
	free(cf2);
	free(origWd);
	if (ret != 0) {
		throw std::runtime_error("Couldn't change back to the original working directory.");
	}

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

	logMessage("ProductManager::%s()") % __func__;

	logMessage("  Pucks:");
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
				logMessage("    --");  // marker to indicate that the listed IDs are not contiguous
			}
			logMessage("    ID=%2d VERS=%3d ROLE=0x%04x TYPE=%s%s")
					% p->getId() % p->getVers() % p->getRole()
					% Puck::getPuckTypeStr(p->getType())
					% ((p->getEffectiveType() == Puck::PT_Monitor) ? " (Monitor)" : "");
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


	logMessage("  Products:");
	bool noProductsFound = true;
	bool wamFound = false;
	// TODO(JH): Rehab Update implement and test
	if (foundWam3()) {
		noProductsFound = false;
		wamFound = true;
		logMessage("    3-DOF WAM");
	}
	if (foundWam4()) {
		noProductsFound = false;
		wamFound = true;
		logMessage("    4-DOF WAM");
	}
	if (foundWam7()) {
		noProductsFound = false;
		wamFound = true;
		logMessage("    7-DOF WAM%s%s") % (foundWam7Wrist() ? " (Wrist)" : "") % (foundWam7Gimbals() ? " (Gimbals)" : "");
	}
	if (wamFound) {
		if (foundSafetyModule()) {
			logMessage("    Safety Module");
		} else {
			logMessage("    *** NO SAFETY MODULE ***");
		}
	}
	if (foundForceTorqueSensor()) {
		noProductsFound = false;
		logMessage("    Force-Torque Sensor");
	}
	if (foundHand()) {
		noProductsFound = false;
		logMessage("    BarrettHand");
	}
	// TODO(dc): Don't report GHC because we don't have a very selective test.

	if (noProductsFound) {
		logMessage("    (none)");
	}
}

void ProductManager::cleanUpAfterEstop()
{
	destroyEstopProducts();
	enumerate();
}
void ProductManager::destroyEstopProducts()
{
	delete rtem;
	rtem = NULL;
	// TODO(JH): Rehab Update implement and test
	delete wam3;
	wam3 = NULL;
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
// TODO(JH): Rehab Update implement and test
bool ProductManager::foundWam3() const
{
	return verifyWamPucks(3);
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
	// TODO(JH): Rehab Update implement and test
	if (foundWam3()){
		return "wam3";
	} else if (foundWam4()) {
		return "wam4";
	} else if (foundWam7Wrist()) {
		return "wam7w";
	} else if (foundWam7Gimbals()) {
		return "wam7g";
	} else {
		throw std::logic_error("ProductManager::getWamDefaultConfigPath(): No WAM found.");
	}
}
// TODO(JH): Rehab Update implement and test
systems::Wam<3>* ProductManager::getWam3(bool waitForShiftActivate, const char* configPath)
{
	if ( !foundWam3() ) {
		throw std::logic_error("ProductManager::getWam3(): No WAM3 was found on the bus.");
	}

	if (wam3 == NULL) {
		std::vector<Puck*> wam3Pucks = wamPucks;
		wam3Pucks.resize(3);  // Discard all but the first 3 elements

		if (configPath == NULL) {
			configPath = getWamDefaultConfigPath();
		}
		try {
			wam3 = new systems::Wam<3>(getExecutionManager(), wam3Pucks, getSafetyModule(), getConfig().lookup(configPath));
		} catch (libconfig::FileIOException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: I/O while reading file\n\n", configPath);
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingNotFoundException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: could not find \"%s\"\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a specific setting is missing.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingTypeException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: \"%s\" is the wrong type\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a setting is improperly formatted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		}
		startExecutionManager();
	}

	if (waitForShiftActivate) {
		if ( !foundSafetyModule() ) {
			throw std::logic_error("ProductManager::getWam3(): No SafetyModule was found on the bus.");
		}

		// Check rapidly in case the user wants to perform some action (like
		// enabling gravity compensation) immediately after Shift-activate.
		getSafetyModule()->waitForMode(SafetyModule::ACTIVE, true, 0.05);
	}

	return wam3;
}
systems::Wam<4>* ProductManager::getWam4(bool waitForShiftActivate, const char* configPath)
{
	if ( !foundWam4() ) {
		throw std::logic_error("ProductManager::getWam4(): No WAM4 was found on the bus.");
	}

	if (wam4 == NULL) {
		std::vector<Puck*> wam4Pucks = wamPucks;
		wam4Pucks.resize(4);  // Discard all but the first 4 elements

		if (configPath == NULL) {
			configPath = getWamDefaultConfigPath();
		}
		try {
			wam4 = new systems::Wam<4>(getExecutionManager(), wam4Pucks, getSafetyModule(), getConfig().lookup(configPath));
		} catch (libconfig::FileIOException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: I/O while reading file\n\n", configPath);
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingNotFoundException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: could not find \"%s\"\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a specific setting is missing.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingTypeException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: \"%s\" is the wrong type\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a setting is improperly formatted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		}
		startExecutionManager();
	}

	if (waitForShiftActivate) {
		if ( !foundSafetyModule() ) {
			throw std::logic_error("ProductManager::getWam4(): No SafetyModule was found on the bus.");
		}

		// Check rapidly in case the user wants to perform some action (like
		// enabling gravity compensation) immediately after Shift-activate.
		getSafetyModule()->waitForMode(SafetyModule::ACTIVE, true, 0.05);
	}

	return wam4;
}

systems::Wam<7>* ProductManager::getWam7(bool waitForShiftActivate, const char* configPath)
{
	if ( !foundWam7() ) {
		throw std::logic_error("ProductManager::getWam7(): No WAM7 was found on the bus.");
	}

	if (wam7 == NULL) {
		std::vector<Puck*> wam7Pucks = wamPucks;
		wam7Pucks.resize(7);  // Discard all but the first 7 elements

		if (configPath == NULL) {
			configPath = getWamDefaultConfigPath();
		}
		try {
			wam7 = new systems::Wam<7>(getExecutionManager(), wam7Pucks, getSafetyModule(), getConfig().lookup(configPath));
		} catch (libconfig::FileIOException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: I/O while reading file\n\n", configPath);
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingNotFoundException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: could not find \"%s\"\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a specific setting is missing.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		} catch (libconfig::SettingTypeException e) {
			printf("\n>>> CONFIG FILE ERROR in %s: \"%s\" is the wrong type\n\n", configPath, e.getPath());
			printf("Check your configuration file directory to ensure that the proper configuration files are installed.\n");
			printf("This error usually means that a configuration file is corrupted, and a setting is improperly formatted.\n");
			printf("Note that if a ~/.barrett/ directory exists, this location will override the standard /etc/barrett/.\n\n");
			throw e;
		}
		startExecutionManager();
	}

	if (waitForShiftActivate) {
		if ( !foundSafetyModule() ) {
			throw std::logic_error("ProductManager::getWam7(): No SafetyModule was found on the bus.");
		}

		// Check rapidly in case the user wants to perform some action (like
		// enabling gravity compensation) immediately after Shift-activate.
		getSafetyModule()->waitForMode(SafetyModule::ACTIVE, true, 0.05);
	}

	return wam7;
}

systems::RealTimeExecutionManager* ProductManager::getExecutionManager(double period_s, int rt_priority)
{
	if (rtem == NULL) {
		rtem = new systems::RealTimeExecutionManager(period_s, rt_priority);
	}
	return rtem;
}
void ProductManager::startExecutionManager() {
	getExecutionManager();
	if ( !rtem->isRunning() ) {
		rtem->start();
	}
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
	return wamPucks[5] != NULL  &&  wamPucks[6] != NULL  &&  !foundWam7Wrist();
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
