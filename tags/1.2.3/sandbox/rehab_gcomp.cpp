/*
 * rehab-gcomp.cpp
 *
 *  Created on: Jul 24, 2013
 *      Author: bz
 */

#include <iostream>
#include <vector>

#include <barrett/exception.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>


using namespace barrett;
using detail::waitForEnter;

const size_t DOF = 2;
const int FIRST_ID = 2;
const char CONFIG_FILE[] = "rehab2.conf";
const char CONFIG_PATH[] = "rehab2";


int main(int argc, char** argv) {
	BARRETT_UNITS_TYPEDEFS(DOF);

	installExceptionHandler();


	ProductManager pm(CONFIG_FILE);
	pm.wakeAllPucks();

	std::vector<Puck*> wamPucks;
	for (int i = FIRST_ID; i < FIRST_ID + (int)DOF; ++i) {
		wamPucks.push_back(pm.getPuck(i));
		if (wamPucks.back() == NULL) {
			printf(">>> ERROR: No rehab device found. Is the power on and the CAN cable connected?\n");
			return 1;
		}
	}

	/*
	// Take the other WAM Pucks off of the BGRP_WAM group so they won't respond
	// to position requests. This allows us to test this code while the Wrist is
	// mounted on a WAM. Power cycle the Pucks to return to the normal grouping.
	for (int i = ProductManager::FIRST_WAM_ID; i < FIRST_WRIST_ID; ++i) {
		Puck* puck = pm.getPuck(i);
		if (puck != NULL) {
			puck->setProperty(Puck::GRPC, 0);
		}
	}
	*/

	printf(">>> The Rehab Device needs to be zeroed. Please move it to its home position, then press [Enter].");
	waitForEnter();
	std::vector<int> torqueGroupIds(1, PuckGroup::BGRP_LOWER_WAM);
	systems::Wam<DOF> wam(pm.getExecutionManager(), wamPucks, NULL, pm.getConfig().lookup(CONFIG_PATH), torqueGroupIds);
	pm.startExecutionManager();

	printf(">>> Press [Enter] to Activate the Rehab Device. WARNING: This bypasses the Safety Module!");
	waitForEnter();
	wam.getLowLevelWam().getPuckGroup().setProperty(Puck::MODE, MotorPuck::MODE_TORQUE);


	printf(">>> Press [Enter] to hold position.");
	waitForEnter();
	wam.moveTo(wam.getJointPositions());


	printf(">>> Press [Enter] to Idle the Rehab Device.");
	waitForEnter();
	wam.getLowLevelWam().getPuckGroup().setProperty(Puck::MODE, MotorPuck::MODE_IDLE);
	return 0;
}
