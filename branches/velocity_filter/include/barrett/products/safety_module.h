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


	SafetyModule(Puck* puck = NULL) : SpecialPuck(Puck::PT_Safety) { setPuck(puck); }
	~SafetyModule() {}

	enum SafetyMode getMode(bool realtime = false) const;
	bool wamIsZeroed() const { return p->getProperty(Puck::ZERO) == 1; }
	void setWamZeroed(bool zeroed = true) const { p->setProperty(Puck::ZERO, zeroed); }

	void waitForMode(enum SafetyMode mode, bool printMessage = true, int pollingPeriod_us = 250000);
	enum SafetyMode waitForModeChange(int pollingPeriod_us = 250000);


	static const char* getSafetyModeStr(enum SafetyMode mode) { return safetyModeStrs[mode]; }


	static const int VELOCITY_FAULT_HISTORY_BUFFER_SIZE = 5;

private:
	static const char safetyModeStrs[][15];
};


}


#endif /* BARRETT_PRODUCTS_SAFETY_MODULE_H_ */
