/*
 * hand_feedback.cpp
 *
 *  Created on: Apr 26, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <boost/thread.hpp>

#include <native/task.h>

#include <barrett/os/os.h>
#include <barrett/bus/bus.h>
#include <barrett/bus/bus_can.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/wam.h>


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

void handFeedback();


using namespace barrett;

const double T_s = 0.002;
const int WAM_DOF = 7;
const int HAND_DOF = 4;

typedef units::JointPositions<HAND_DOF>::type hjp_type;

bool going = true;
struct bt_bus* bus;
hjp_type hjp;


int main() {
	libconfig::Config config;
	config.readFile("/etc/wam/wam7-new.config");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	// instantiate Systems
	Wam<WAM_DOF> wam(config.lookup("wam"));
	bus = wam.wam.wambot->bus;


	// start the main loop!
	rtem.start();
	boost::thread t(handFeedback);

	printf("Press [Enter] to print hand positions.\n");
	while (true) {
		waitForEnter();
		std::cout << hjp << std::endl;
//		sleep(1);
	}

	rtem.stop();
	return 0;
}


void handFeedback() {
	int id, property;
	long value;

	rt_task_shadow(new RT_TASK, NULL, 10, 0);

	printf("Waking hand pucks ...\n");

	for(int i = 11; i <= 14; i++) {
		bt_bus_set_property(bus, i, 5, 2); // Set STAT to STATUS_READY
	}

	usleep((long)1e6);

	printf("Setting TSTOP ...\n");
	for(int i = 11; i <= 14; i++) {
		bt_bus_set_property(bus, i, 78, 50); // Set TSTOP to 50 ms
	}

	printf(" ... done.\n");


//	rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);
	while (going) {
//		waitForEnter();
//		printf("Acquiring mutex ...\n");
		bt_os_mutex_lock(bus->dev->async_mutex);

//		printf("Requesting positions ...\n");
		bt_bus_can_async_get_property(bus->dev, BT_BUS_CAN_GROUPID(HAND_GRP), 48);
//		printf(" ... done.\n");

		for (int i = 0; i < HAND_DOF; ++i) {
			bt_bus_can_async_read(bus->dev, &id, &property, &value, 1, 0);
			if (id >=11  &&  id <= 14) {
				hjp[id-11] = (double) value;
			} else {
				printf("%s: Spurious asynchronous CAN message (ID=%d, PROPERTY=%d, VALUE=%ld)\n", __func__, id, property, value);
			}
		}

		bt_os_mutex_unlock(bus->dev->async_mutex);
//		std::cout << hjp << std::endl;
	}
}
