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
 * safety_module.cpp
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#include <bitset>
#include <string>
#include <algorithm>
#include <stdexcept>
#include <cstdio>
#include <cassert>

#include <boost/lexical_cast.hpp>

#include <barrett/os.h>
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
		(logMessage("SafetyModule::%s(): Bad MODE value. "
				"Expected MODE value of 0, 1, or 2. Got value of %d.")
				% __func__ % mode).raise<std::runtime_error>();
	}
	return static_cast<enum SafetyMode>(mode);
}

void SafetyModule::waitForMode(enum SafetyMode mode, bool printMessage, double pollingPeriod_s)
{
	if (getMode() == mode) {
		return;
	}

	if (printMessage) {
		printf(">>> Please %s the WAM.\n", SafetyModule::getSafetyModeStr(mode));
	}
	do {
		btsleep(pollingPeriod_s);
	} while (getMode() != mode);
}

enum SafetyModule::SafetyMode SafetyModule::waitForModeChange(double pollingPeriod_s)
{
	const enum SafetyMode originalMode = getMode();
	enum SafetyMode currentMode;

	do {
		btsleep(pollingPeriod_s);
		currentMode = getMode();
	} while (currentMode == originalMode);

	return currentMode;
}

void SafetyModule::ignoreNextVelocityFault()
{
	p->setProperty(Puck::IFAULT, SafetyModule::VELOCITY_FAULT_HISTORY_BUFFER_SIZE);
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

void SafetyModule::getPendantState(PendantState* ps, bool realtime) const
{
	typedef const std::bitset<32> bits_type;

	assert(ps != NULL);
	int pen = p->getProperty(Puck::PEN);
	bits_type bits(pen);

	if (bits[27]) {
		if (bits[26]) {
			ps->pressedButton = PendantState::NONE;
		} else {
			ps->pressedButton = PendantState::IDLE;
		}
	} else {
		if (bits[26]) {
			ps->pressedButton = PendantState::ACTIVATE;
		} else {
			ps->pressedButton = PendantState::ESTOP;
		}
	}

	ps->activateLight = bits[25];
	ps->idleLight = bits[24];
	ps->displayedCharacter = (pen >> 16) & 0xff;  // Select bits 16 through 23

	for (int i = 0; i < PendantState::NUM_PARAMS; ++i) {
		bits_type paramBits((pen >> (3 * i)) & 0x7);  // Select three bits...
		if (paramBits.count() != 1) {  // exactly one of which should be set.
			(logMessage("SafetyModule::%s(): Bad PEN value for Parameter %d: %s")
					% __func__ % i % bits.to_string()).raise<std::runtime_error>();
		}

		if (paramBits[0]) {
			ps->safetyParameters[i] = PendantState::SAFE;
		} else if (paramBits[1]) {
			ps->safetyParameters[i] = PendantState::WARNING;
		} else {
			ps->safetyParameters[i] = PendantState::FAULT;
		}
	}
}


const char SafetyModule::safetyModeStrs[][15] = { "E-stop", "Shift-idle", "Shift-activate" };


bool SafetyModule::PendantState::allSafe() const
{
	return std::count(safetyParameters, safetyParameters + NUM_PARAMS, SAFE) == NUM_PARAMS;
}

bool SafetyModule::PendantState::hasFaults() const
{
	return std::find(safetyParameters, safetyParameters + NUM_PARAMS, FAULT) != safetyParameters + NUM_PARAMS;
}

std::string SafetyModule::PendantState::toString() const
{
	std::string out;

	switch (pressedButton) {
	case ESTOP:
		out += "E";
		break;
	case ACTIVATE:
		out += "A";
		break;
	case IDLE:
		out += "I";
		break;
	case NONE:
		out += "N";
		break;
	default:
		assert(false);
		break;
	}
	out += " ";

	out += activateLight ? "A":"a";
	out += idleLight ? "I":"i";
	out += " \"";
	out += decodeDisplayedCharacter();
	out += "\" (";
	out += boost::lexical_cast<std::string, int>(displayedCharacter);
	out += ")";

	for (int i = 0; i < NUM_PARAMS; ++i) {
		out += " ";
		switch (safetyParameters[i]) {
		case SAFE:
			out += "S";
			break;
		case WARNING:
			out += "W";
			break;
		case FAULT:
			out += "F";
			break;
		default:
			assert(false);
			break;
		}
	}

	return out;
}

char SafetyModule::PendantState::decodeDisplayedCharacter() const
{
	switch (displayedCharacter) {
	case 0x3f: return '0';
	case 0x06: return '1';
	case 0x5b: return '2';
	case 0x4f: return '3';
	case 0x66: return '4';
	case 0x6d: return '5';
	case 0x7d: return '6';
	case 0x07: return '7';
	case 0x77: return 'A';
	case 0x79: return 'E';
	case 0x76: return 'H';
	case 0x38: return 'L';
	default:   return '?';
	}
}


}
