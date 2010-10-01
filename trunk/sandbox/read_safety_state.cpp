/*
 * read_safety_state.cpp
 *
 *  Created on: Jun 3, 2010
 *      Author: dc
 */

#include <iostream>

#include <sys/mman.h>
#include <unistd.h>
#include <native/task.h>
#include <libconfig.h++>

#include <barrett/systems.h>
#include <barrett/wam.h>
#include <barrett/cdlbt/bus/bus.h>
#include <barrett/cdlbt/bus/bus_can.h>


using namespace barrett;

const size_t DOF = 7;
const double T_s = 0.002;


void printSafetyState(long safetyState) {
	switch (safetyState) {
	case 0:
		printf("E-stop\n");
		break;

	case 1:
		printf("Shift-idle\n");
		break;

	case 2:
		printf("Shift-activate\n");
		break;

	default:
		printf("UNKNOWN STATE: %ld\n", safetyState);
		break;
	}
}

int main() {
	long safetyState, lastSafetyState;

	libconfig::Config config;
	config.readFile("/etc/barrett/wam7.conf");

	mlockall(MCL_CURRENT|MCL_FUTURE);
	rt_task_shadow(new RT_TASK, NULL, 10, 0);

	{
		bt_bus_can* dev = NULL;
		if (bt_bus_can_create(&dev, (int)config.lookup("wam.low_level.bus.port"))) {
			printf("Couldn't open the CAN bus.\n");
			return -1;
		}


		bt_bus_can_get_property(dev, 10, 8, &safetyState, NULL, 1);
		printSafetyState(safetyState);

		// if E-stoped...
		if (safetyState == 0) {
			// wait until Shift-idled
			while (safetyState == 0) {
				usleep(100000);
				bt_bus_can_get_property(dev, 10, 8, &safetyState, NULL, 1);
			}

			printSafetyState(safetyState);
			usleep(500000);  // give the pucks time to wake up?
		}

		lastSafetyState = safetyState;

		bt_bus_can_destroy(dev);
	}

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	// instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));


	// start the main loop!
	rtem.start();

	while (true) {
		usleep(100000);

		bt_bus_get_property(wam.wam.wambot->bus, 10, 8, &safetyState);

		if (lastSafetyState != safetyState) {
			printSafetyState(safetyState);
			lastSafetyState = safetyState;
		}
	}

	rtem.stop();
	return 0;
}
