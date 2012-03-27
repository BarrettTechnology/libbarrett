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
 * safety_module.h
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_SAFETY_MODULE_H_
#define BARRETT_PRODUCTS_SAFETY_MODULE_H_


#include <barrett/products/puck.h>
#include <barrett/products/abstract/special_puck.h>


namespace barrett {


class SafetyModule : public SpecialPuck {
public:
	enum SafetyMode {
		ESTOP, IDLE, ACTIVE
	};


	SafetyModule(Puck* puck = NULL);
	~SafetyModule() {}

	enum SafetyMode getMode(bool realtime = false) const;
	bool wamIsZeroed() const { return p->getProperty(Puck::ZERO) == 1; }
	void setWamZeroed(bool zeroed = true) const {
		p->setProperty(Puck::ZERO, zeroed);
	}

	void waitForMode(enum SafetyMode mode,
			bool printMessage = true, int pollingPeriod_us = 250000);
	enum SafetyMode waitForModeChange(int pollingPeriod_us = 250000);

	// For safety reasons, the SafetyModule will ignore requests to move from
	// ESTOP to IDLE, or from IDLE to ACTIVE.
	void setMode(enum SafetyMode mode) { p->setProperty(Puck::MODE, mode); }

	void setDefaultSafetyLimits();
	void ignoreNextVelocityFault();

	// Measured in Newton*meters at each motor, assuming equal IPNM.
	void setTorqueLimit(double fault,
			double warning = -1.0, int ipnm = Puck::DEFAULT_IPNM);

	// Measured in meters/second at the elbow and 4-DOF end-point.
	void setVelocityLimit(double fault, double warning = -1.0);


	static const char* getSafetyModeStr(enum SafetyMode mode) {
		return safetyModeStrs[mode];
	}

protected:
	static const int VELOCITY_FAULT_HISTORY_BUFFER_SIZE = 5;

private:
	static const char safetyModeStrs[][15];
};


}


#endif /* BARRETT_PRODUCTS_SAFETY_MODULE_H_ */
