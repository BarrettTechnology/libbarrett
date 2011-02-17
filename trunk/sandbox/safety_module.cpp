/*
 * safety_module.cpp
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#include <cstdio>

#include <barrett/products/product_manager.h>
#include <barrett/systems.h>


using namespace barrett;


void printCurrentMode(SafetyModule& sm) {
	printf("Current SafetyModule mode: %s\n", SafetyModule::getSafetyModeStr(sm.getMode()));
}

int main() {
	ProductManager pm;
	SafetyModule& sm = *pm.getSafetyModule();

	while (true) {
		printCurrentMode(sm);

		// Tell the ExecutionManager to do nothing when it detects an ESTOP. (The default
		// default behavior is to terminate the program.) This must be done after every ESTOP
		// because ProductManager::cleanUpAfterEstop() causes the ExecutionManager to be destroyed.
		// A new ExecutionManager is constructed in this call.
		pm.getExecutionManager()->clearErrorCallback();

		// An ESTOP that occurs in between a call to ProductManager::waitForWam() and a call to
		// ProductManager::getWam*() might cause Puck::wake() (which is called from the LowLevelWam
		// constructor) to throw a std::runtime_error exception.
		pm.waitForWam();

		// Instantiate a Wam object and start the realtime control loop
		if (pm.foundWam4()) {
			pm.getWam4();
		} else {
			pm.getWam7();
		}

		// An ESTOP that occurs while executing the following line will cause the program to hang
		// waiting for a Shift-activate that will never come...
		sm.waitForMode(SafetyModule::ACTIVE);

		while (true) {
			if (sm.waitForModeChange() == SafetyModule::ESTOP) {
				pm.cleanUpAfterEstop();
				break;
			}

			printCurrentMode(sm);
		}
	}

	return 0;
}
