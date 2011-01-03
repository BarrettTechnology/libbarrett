/*
 * safety_module.cpp
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#include <cstdio>

#include <barrett/products/product_manager.h>


using namespace barrett;


void printCurrentMode(SafetyModule& sm) {
	printf("Current SafetyModule mode: %s\n", SafetyModule::getSafetyModeStr(sm.getMode()));
}

int main() {
	ProductManager pm;
	SafetyModule& sm = *pm.getSafetyModule();

	printCurrentMode(sm);

	sm.waitForMode(SafetyModule::IDLE);
	if ( !pm.foundWam() ) {
		pm.enumerate();
		if ( !pm.foundWam() ) {
			printf("ERROR: No WAM was found.\n");
			return 1;
		}
	}

	systems::Wam<4>& wam = *pm.getWam4();
	sm.waitForMode(SafetyModule::ACTIVE);

	while (true) {
		sm.waitForModeChange();
		printCurrentMode(sm);
	}

	return 0;
}
