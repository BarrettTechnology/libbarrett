/*
 * load_ft_cal.cpp
 *
 *  Created on: Jul 9, 2010
 *      Author: dc
 */

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>

#include <sys/mman.h>
#include <unistd.h>
#include <native/task.h>

#include <barrett/bus/bus.h>
#include <barrett/bus/bus_can.h>


using namespace std;


const int ID = BT_BUS_PUCK_ID_FT;
const int MIN_FW_VERS = 100;  // bogus value, but at least it rules out monitor
const int GM_SIZE = 36;  // the gain matrix is a 6x6 matrix with 36 elements


int main(int argc, char** argv) {
	mlockall(MCL_CURRENT|MCL_FUTURE);
	rt_task_shadow(new RT_TASK, NULL, 10, 0);


	int port = 0;
	switch (argc) {
	case 2:
		printf("No port argument given. Using default.\n");
		break;
	case 3:
		port = atoi(argv[2]);
		break;
	default:
		printf("ERROR: Expected 1 or 2 arguments.\n");
		return -1;
		break;
	}

	printf("Using CAN bus port %d.\n", port);
	bt_bus_can* dev = NULL;
	if (bt_bus_can_create(&dev, port)) {
		printf("Couldn't open the CAN bus.\n");
		return -1;
	}


	struct bt_bus_properties* plist;
	long value = 0;

	// init sensor
	bt_bus_can_set_property(dev, ID, 5, 2);
	usleep(1000000);

	bt_bus_can_get_property(dev, ID, 0, &value, NULL, 1);  // get VERS
	if (value < MIN_FW_VERS) {
		printf("The F/T Sensor does not have recent firmware. (Actual version = %ld. Required version >= %d.)\n", value, MIN_FW_VERS);
		exit(-1);
	}
	if (bt_bus_properties_create(&plist, value)) {
		printf("Couldn't create property list.\n");
		exit(-1);
	}


	// load cal
	int calValue = 0;
	ifstream cal(argv[1]);
	for (int i = 0; i < GM_SIZE; ++i) {
		if (cal.good()) {
			cal >> calValue;
			if (calValue < -32768  ||  calValue > 32767) {
				printf("Calibration file is poorly formated: value out of range: %d.\n", calValue);
				return -1;
			}
			bt_bus_can_set_property(dev, ID, plist->GM, calValue);
			printf(".");
			fflush(stdout);
			usleep(1000000);
		} else {
			printf("Calibration file is poorly formated: not enough values.\n");
			return -1;
		}
	}

	printf(" Done.\n");

	return 0;
}
