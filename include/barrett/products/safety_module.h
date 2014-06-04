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
 */

/**
 * @file safety_module.h
 * @date 11/04/2010
 * @author Dan Cody
 * 
 */

#ifndef BARRETT_PRODUCTS_SAFETY_MODULE_H_
#define BARRETT_PRODUCTS_SAFETY_MODULE_H_


#include <string>

#include <barrett/products/puck.h>
#include <barrett/products/abstract/special_puck.h>


namespace barrett {


class SafetyModule : public SpecialPuck {
public:
	/** SafetyMode States
	 *
	 */
	enum SafetyMode {
		ESTOP, IDLE, ACTIVE
	};

	/** SafetyModule Constructor and Destructors
	 *
	 */
	SafetyModule(Puck* puck = NULL);
	~SafetyModule() {}
	/** getMode Method returns current state of Safety Pendant (E-Stopped/Idle/Active)
	 *
	 */
	enum SafetyMode getMode(bool realtime = false) const;
	/** waitForMode Method
	 *
	 */
	void waitForMode(enum SafetyMode mode,
			bool printMessage = true, double pollingPeriod_s = 0.25);
	/**
	 *
	 */
	enum SafetyMode waitForModeChange(double pollingPeriod_s = 0.25);
	/** getSafetyModeStr Method
	 *
	 */
	static const char* getSafetyModeStr(enum SafetyMode mode) {
		return safetyModeStrs[mode];
	}
	/** setMode Method
	 *
	 *	For safety reasons, the SafetyModule will ignore requests to move from
	 *	ESTOP to IDLE, or from IDLE to ACTIVE.
	 */
	void setMode(enum SafetyMode mode) { p->setProperty(Puck::MODE, mode); }
	/** wamIsZeroed Method returns true/false flag that wam has been zeroed by user.
	 *
	 */
	bool wamIsZeroed() const { return p->getProperty(Puck::ZERO) == 1; }
	/** setWamZeroed Method 
	 *
	 */
	void setWamZeroed(bool zeroed = true) const {
		p->setProperty(Puck::ZERO, zeroed);
	}
	/**	ignoreNextVelocityFault Method overrides next velocity fault. 
	 *
	 */
	void ignoreNextVelocityFault();
	/**	setDefaultSeafetyLimits Method resets all current safetly limits to default values.
	 *
	 */
	void setDefaultSafetyLimits();
	/** setTorqueLimit Method changes the absolute limit of the Torque Fault in Nm.
	 *
	 *	Measured in Newton*meters at each motor, assuming equal IPNM.
	 */
	void setTorqueLimit(double fault, double warning = -1.0, int ipnm = Puck::DEFAULT_IPNM);
	/** setVelocityLimit Method changes the absolute limit of the velocity fault in m/s.
	 *
	 *	Measured in meters/second at the elbow and 4-DOF end-point.
	 */
	void setVelocityLimit(double fault, double warning = -1.0);

	/**
	 *
	 */
	struct PendantState {
		enum Button { ESTOP, ACTIVATE, IDLE, NONE };
		enum Parameter { SAFE, WARNING, FAULT };
		enum ParameterNames {
			VELOCITY, TORQUE, VOLTAGE, HEARTBEAT, OTHER,
			NUM_PARAMS
		};

		enum Button pressedButton;
		bool activateLight;
		bool idleLight;
		char displayedCharacter;
		enum Parameter safetyParameters[NUM_PARAMS];

		bool allSafe() const;
		bool hasFaults() const;

		std::string toString() const;
		char decodeDisplayedCharacter() const;
	};
/** getPendantState Method updates PendantState Structure values
 *
 *	Method will update the current button, mode and parameter status values.
 */
	void getPendantState(PendantState* ps, bool realtime = false) const;

protected:
	static const int VELOCITY_FAULT_HISTORY_BUFFER_SIZE = 5;

private:
	static const char safetyModeStrs[][15];
};


}


#endif /* BARRETT_PRODUCTS_SAFETY_MODULE_H_ */
