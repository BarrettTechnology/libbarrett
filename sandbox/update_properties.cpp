/*
 * update_properties.cpp
 *
 *  Created on: Apr 21, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <cstdlib>

#include <sys/mman.h>
#include <unistd.h>
#include <native/task.h>

#include <barrett/cdlbt/bus.h>
#include <barrett/cdlbt/bus_can.h>


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main(int argc, char** argv) {
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

	printf(">>> With the hand attached, Shift-idle the WAM. Then press [Enter]...\n");
	waitForEnter();

	printf("Waking pucks...\n");
	for(int i = 1; i <= 7; i++) {
		bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
	}
	for(int i = 11; i <= 14; i++) {
		bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
	}
	usleep(500000);
	printf("Done.\n");

	printf("WAM: Setting GRPC to 4...\n");
	for(int i = 1; i <= 7; i++) {
		bt_bus_can_set_property(dev, i, 28, 4);
	}
	usleep(1000);
	printf("Done.\n");

	printf("HAND: Setting GRPC to 5...\n");
	for(int i = 11; i <= 14; i++) {
		bt_bus_can_set_property(dev, i, 28, 5);
	}
	usleep(1000);
	printf("Done.\n");

	printf("Saving changes...\n");
	for(int i = 1; i <= 7; i++) {
		bt_bus_can_set_property(dev, i, 30, -1);
	}
	for(int i = 11; i <= 14; i++) {
		bt_bus_can_set_property(dev, i, 30, -1);
	}
	usleep(1000);
	printf("Done.\n");


	printf(">>> E-stop and then Shift-idle the WAM. Then press [Enter]...\n");
	waitForEnter();

	printf("Waking pucks...\n");
	for(int i = 1; i <= 7; i++) {
		bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
	}
	for(int i = 11; i <= 14; i++) {
		bt_bus_can_set_property(dev, i, 5, 2); // Set STAT to STATUS_READY
	}
	usleep(500000);
	printf("Done.\n");

	printf("Verifying changes...\n");
	long reply;
	for(int i = 1; i <= 7; i++) {
		bt_bus_can_get_property(dev, i, 28, &reply, NULL, 1);
		printf("ID%d: ", i);
		if (reply == 4) {
			printf("PASS\n");
		} else {
			printf("FAIL (28 = %ld)\n", reply);
		}
	}
	for(int i = 11; i <= 14; i++) {
		bt_bus_can_get_property(dev, i, 28, &reply, NULL, 1);
		printf("ID%d: ", i);
		if (reply == 5) {
			printf("PASS\n");
		} else {
			printf("FAIL (28 = %ld)\n", reply);
		}
	}
	printf("Done.\n");

	return 0;
}
