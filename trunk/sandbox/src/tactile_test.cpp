/*
 * tactile_test.cpp
 *
 *  Created on: Jun 15, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <cstdlib>

#include <sys/mman.h>
#include <unistd.h>
#include <native/task.h>

#include <barrett/bus/bus.h>
#include <barrett/bus/bus_can.h>


const int HAND_DOF = 4;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main(int argc, char** argv) {
	int osTact = 0, numPucks = 0;
	long tactTop10[HAND_DOF];
	int id, property;
	long value1, value2;
	bool going = true;


	mlockall(MCL_CURRENT|MCL_FUTURE);
	rt_task_shadow(new RT_TASK, NULL, 10, 0);

	int port = 0;
	switch (argc) {
	case 1:
		printf("No port argument given. Using default.\n");
		break;
	case 2:
		port = atoi(argv[1]);
		break;
	default:
		printf("ERROR: Expected 1 or 0 arguments.\n");
		return -1;
		break;
	}

	printf("Using CAN bus port %d.\n", port);
	bt_bus_can* dev = NULL;
	if (bt_bus_can_create(&dev, port)) {
		printf("Couldn't open the CAN bus.\n");
		return -1;
	}

	printf(">>> Attach and power the hand, then press [Enter]...\n");
	waitForEnter();

	printf("Waking hand pucks ...\n");

	for(int i = 11; i <= 14; i++) {
		bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
	}
	usleep((long)5e5);

	printf("Initializing tactile sensors...\n");
	for(int i = 11; i <= 14; i++) {
		bt_bus_can_async_get_property(dev, i, 5);
		usleep(1000);
		switch (bt_bus_can_async_read(dev, &id, &property, &value1, NULL, 0, 1)) {
		case 0:
			++numPucks;
			printf("Found Puck %d.\n", i);

			if (value1 == 0) {
				bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
				usleep((long)5e5);
			}

			bt_bus_can_get_property(dev, i, 34, &value1, NULL, 1);  // first time, read garbage

			// second time, read number of sensors (24)
			bt_bus_can_get_property(dev, i, 34, &value1, NULL, 1);
			if (value1 != 24) {
				printf("Failed to init tactile sensors on ID=%d. Reported %ld sensors.\n", i, value1);
			}

			break;

		case 3:  // the puck was not found
			continue;
			break;

		default:
			printf("CAN read error.\n");
			break;
		}
	}


//	printf("Initializing tactile sensors...\n");
//
//	// first time, read garbage
//	for(int i = 11; i <= 14; i++) {
//		bt_bus_can_get_property(dev, i, 34, &value1, NULL, 1);
//	}
//
//	// second time, read number of sensors (24)
//	for(int i = 11; i <= 14; i++) {
//		bt_bus_can_get_property(dev, i, 34, &value1, NULL, 1);
//		if (value1 != 24) {
//			printf("Failed to init tactile sensors on ID=%d. Reported %ld sensors.\n", i, value1);
//		}
//	}
	bt_bus_can_set_property(dev, BT_BUS_CAN_GROUPID(HAND_GRP), 29, 23);
	usleep((long)1e3);



	printf(" ... done.\n");


	osTact = numPucks;
	bt_bus_can_set_property(dev, BT_BUS_CAN_GROUPID(HAND_GRP), 106, 1);

	while (going) {
		bt_bus_can_async_read(dev, &id, &property, &value1, &value2, 1, 1);
		switch (id) {
		// hand
		case 11:
		case 12:
		case 13:
		case 14:
			switch (property) {
			case 106:  // tact
				tactTop10[id-11] = (value1 & 0xFFFFFF00) >> 8;

				if (--osTact == 0) {
					printf("%06lX %06lX %06lX %06lX\n", tactTop10[0], tactTop10[1], tactTop10[2], tactTop10[3]);

					waitForEnter();

					bt_bus_can_set_property(dev, BT_BUS_CAN_GROUPID(HAND_GRP), 106, 1);
					osTact = numPucks;
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


	return 0;
}
