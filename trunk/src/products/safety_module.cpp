/*
 * safety_module.cpp
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <cstdio>

#include <syslog.h>
#include <unistd.h>

#include <barrett/products/puck.h>
#include <barrett/products/safety_module.h>


namespace barrett {


enum SafetyModule::SafetyMode SafetyModule::getMode(bool realtime) const {
	int mode = p->getProperty(Puck::MODE, realtime);
	if (mode < 0  ||  mode > 2) {
		syslog(LOG_ERR, "SafetyModule::getMode(): Expected MODE value of 0, 1, or 2. Got value of %d.", mode);
		throw std::runtime_error("SafetyModule::getMode(): Bad MODE value. Check /var/log/syslog for details.");
	}
	return static_cast<enum SafetyMode>(mode);
}

void SafetyModule::waitForMode(enum SafetyMode mode, bool printMessage, int pollingPeriod_us)
{
	if (getMode() == mode) {
		return;
	}

	if (printMessage) {
		printf(">>> Please %s the WAM.\n", SafetyModule::getSafetyModeStr(mode));
	}
	do {
		usleep(pollingPeriod_us);
	} while (getMode() != mode);
}

enum SafetyModule::SafetyMode SafetyModule::waitForModeChange(int pollingPeriod_us)
{
	enum SafetyMode originalMode = getMode();
	enum SafetyMode currentMode;

	do {
		usleep(pollingPeriod_us);
	} while ( (currentMode = getMode()) == originalMode );

	return currentMode;
}


const char SafetyModule::safetyModeStrs[][15] = { "E-stop", "Shift-idle", "Shift-activate" };


}
