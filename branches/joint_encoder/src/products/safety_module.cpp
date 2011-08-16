/*
 * safety_module.cpp
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <cstdio>
#include <cassert>

#include <syslog.h>
#include <unistd.h>

#include <barrett/products/puck.h>
#include <barrett/products/safety_module.h>


namespace barrett {


SafetyModule::SafetyModule(Puck* puck) :
	SpecialPuck(Puck::PT_Safety)
{
	setPuck(puck);

	// Load safety parameters from EEPROM so they won't be affected by previous
	// programs that may have adjusted these values.
	setDefaultSafetyLimits();
}

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
	const enum SafetyMode originalMode = getMode();
	enum SafetyMode currentMode;

	do {
		usleep(pollingPeriod_us);
		currentMode = getMode();
	} while (currentMode == originalMode);

	return currentMode;
}

void SafetyModule::setDefaultSafetyLimits()
{
	p->resetProperty(Puck::VL1);
	p->resetProperty(Puck::VL2);
	p->resetProperty(Puck::TL1);
	p->resetProperty(Puck::TL2);
}

void SafetyModule::setTorqueLimit(double fault, double warning, int ipnm)
{
	assert(fault > 0.0);
	if (warning < 0.0) {
		warning = 0.9 * fault;
	}

	p->setProperty(Puck::TL2, (int)(fault*ipnm));
	p->setProperty(Puck::TL1, (int)(warning*ipnm));
}

void SafetyModule::setVelocityLimit(double fault, double warning)
{
	assert(fault >= 0.0);  // 0 is a special value meaning "never velocity fault"
	if (warning < 0.0) {
		warning = 0.9 * fault;
	}

	p->setProperty(Puck::VL2, (int)(fault*0x1000));
	p->setProperty(Puck::VL1, (int)(warning*0x1000));
}

void SafetyModule::ignoreNextVelocityFault()
{
	p->setProperty(Puck::IFAULT, SafetyModule::VELOCITY_FAULT_HISTORY_BUFFER_SIZE);
}


const char SafetyModule::safetyModeStrs[][15] = { "E-stop", "Shift-idle", "Shift-activate" };


}
