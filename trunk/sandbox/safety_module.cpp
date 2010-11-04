/*
 * safety_module.cpp
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#include <cstdio>

#include <barrett/bus/bus_manager.h>
#include <barrett/products/safety_module.h>
#include <barrett/systems/wam.h>


using namespace barrett;


void printCurrentMode(SafetyModule& sm) {
	printf("Current SafetyModule mode: %s\n", SafetyModule::getSafetyModeStr(sm.getSafetyMode()));
}

int main() {
	BusManager bm;
	SafetyModule& sm = *bm.getSafetyModule();

	printCurrentMode(sm);

	sm.waitForMode(SafetyModule::IDLE);
	if ( !bm.wamFound() ) {
		bm.enumerate();
		if ( !bm.wamFound() ) {
			printf("ERROR: No WAM was found.\n");
			return 1;
		}
	}

	systems::Wam<4>& wam = *bm.getWam4();
	sm.waitForMode(SafetyModule::ACTIVE);

	while (true) {
		sm.waitForModeChange();
		printCurrentMode(sm);
	}

	return 0;
}
