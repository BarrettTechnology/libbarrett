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
#include <native/timer.h>

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
const RTIME FLIGHT_TIME = 75000;

typedef units::JointPositions<HAND_DOF>::type hjp_type;

bool going = true;
struct bt_bus* bus;
hjp_type hjp;
long tactTop10[HAND_DOF];
int ft[6];


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
		printf("%06lX %06lX %06lX %06lX\n", tactTop10[0], tactTop10[1], tactTop10[2], tactTop10[3]);
		printf("%d %d\n", ft[0], ft[1]);
//		sleep(1);
	}

	rtem.stop();
	return 0;
}


void handFeedback() {
	int id, property;
	long value1, value2;

	rt_task_shadow(new RT_TASK, NULL, 10, 0);

	printf("Waking hand pucks ...\n");

	for(int i = 11; i <= 14; i++) {
		bt_bus_set_property(bus, i, 5, 2); // Set STAT to STATUS_READY
	}
	usleep((long)1e6);

	printf("Setting TSTOP ...\n");
	bt_bus_set_property(bus, BT_BUS_CAN_GROUPID(HAND_GRP), 78, 50); // Set TSTOP to 50 ms
	usleep((long)1e3);

	printf("Initializing tactile sensors...\n");

	// first time, read garbage
	for(int i = 11; i <= 14; i++) {
		bt_bus_get_property(bus, i, 34, &value1);
	}

	// second time, read number of sensors (24)
	for(int i = 11; i <= 14; i++) {
		bt_bus_get_property(bus, i, 34, &value1);
		if (value1 != 24) {
			printf("Failed to init tactile sensors on ID=%d. Repored %ld sensors.\n", i, value1);
		}
	}
	bt_bus_set_property(bus, BT_BUS_CAN_GROUPID(HAND_GRP), 29, 23);
	usleep((long)1e3);



	printf(" ... done.\n");


	int osHandPos = HAND_DOF, osTact = HAND_DOF;

	bt_bus_can_async_get_property(bus->dev, BT_BUS_CAN_GROUPID(HAND_GRP), 48);
	rt_timer_spin(FLIGHT_TIME);
	bt_bus_can_set_property(bus->dev, BT_BUS_CAN_GROUPID(HAND_GRP), 106, 1);
	rt_timer_spin(FLIGHT_TIME);
	bt_bus_can_async_get_property(bus->dev, 8, 34);

	while (going) {
		bt_bus_can_async_read(bus->dev, &id, &property, &value1, &value2, 1, 0);
		switch (id) {
		// force-torque
		case 8:
			switch (property) {
			case 34:
				ft[0] = value1;
				bt_bus_can_async_get_property(bus->dev, 8, 35);
				break;
			case 35:
				ft[1] = value1;
				bt_bus_can_async_get_property(bus->dev, 8, 34);
				break;

			default:
				printf("%s: Spurious CAN message (ID=%d, PROPERTY=%d, VALUE=%ld)\n", __func__, id, property, value1);
			}
			break;

		// hand
		case 11:
		case 12:
		case 13:
		case 14:
			switch (property) {
			case 48:  // positions
				hjp[id-11] = (double) value1;

				if (--osHandPos == 0) {
					bt_bus_can_async_get_property(bus->dev, BT_BUS_CAN_GROUPID(HAND_GRP), 48);
					osHandPos = HAND_DOF;
				}
				break;

			case 106:  // tact
				tactTop10[id-11] = (value1 & 0xFFFFFF00) >> 8;

				if (--osTact == 0) {
					bt_bus_can_set_property(bus->dev, BT_BUS_CAN_GROUPID(HAND_GRP), 106, 1);
					osTact = HAND_DOF;
				}
				break;

			default:
				printf("%s: Spurious CAN message (ID=%d, PROPERTY=%d, VALUE=%ld)\n", __func__, id, property, value1);
			}
			break;

		default:
			printf("%s: Spurious CAN message (ID=%d, PROPERTY=%d, VALUE=%ld)\n", __func__, id, property, value1);
		}
	}

////	rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);
//	while (going) {
////		waitForEnter();
////		printf("Acquiring mutex ...\n");
//		bt_os_mutex_lock(bus->dev->async_mutex);
//
////		printf("Requesting positions ...\n");
//		bt_bus_can_async_get_property(bus->dev, BT_BUS_CAN_GROUPID(HAND_GRP), 48);
////		printf(" ... done.\n");
//
//		for (int i = 0; i < HAND_DOF; ++i) {
//			bt_bus_can_async_read(bus->dev, &id, &property, &value1, &value2, 1, 0);
//			if (id >=11  &&  id <= 14) {
//				hjp[id-11] = (double) value1;
//			} else {
//				printf("%s: Spurious asynchronous CAN message (ID=%d, PROPERTY=%d, VALUE=%ld)\n", __func__, id, property, value1);
//			}
//		}
//
//		bt_os_mutex_unlock(bus->dev->async_mutex);
////		std::cout << hjp << std::endl;
//	}
}
