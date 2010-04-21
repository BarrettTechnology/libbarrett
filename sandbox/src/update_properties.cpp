/*
 * update_properties.cpp
 *
 *  Created on: Apr 21, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <sys/mman.h>
#include <unistd.h>
#include <native/task.h>
#include <libconfig.h++>

#include <barrett/bus/bus.h>
#include <barrett/bus/bus_can.h>


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	libconfig::Config config;
	config.readFile("/etc/wam/wam7-new.config");

	mlockall(MCL_CURRENT|MCL_FUTURE);
	rt_task_shadow(new RT_TASK, NULL, 10, 0);

	struct bt_bus* bus;
	if (bt_bus_create(&bus, config.lookup("wam.low_level.bus").getCSetting(), bt_bus_UPDATE_POS_DIFF)) {
		printf("Couldn't create bus.\n");
		return 1;
	}

	printf(">>> With the hand attached, Shift-idle the WAM. Then press [Enter]...\n");
	waitForEnter();

	printf("Waking pucks...\n");
	for(int i = 1; i <= 7; i++) {
		bt_bus_set_property(bus, i, 5, 0, 2); // Set STAT to STATUS_READY
	}
	for(int i = 11; i <= 14; i++) {
		bt_bus_set_property(bus, i, 5, 0, 2); // Set STAT to STATUS_READY
	}
	usleep(500000);
	printf("Done.\n");

	printf("WAM: Setting GRPC to 4...\n");
	for(int i = 1; i <= 7; i++) {
		bt_bus_set_property(bus, i, 28, 0, 4);
	}
	usleep(1000);
	printf("Done.\n");

	printf("HAND: Setting GRPC to 5...\n");
	for(int i = 11; i <= 14; i++) {
		bt_bus_set_property(bus, i, 28, 0, 5);
	}
	usleep(1000);
	printf("Done.\n");

	printf("Saving changes...\n");
	for(int i = 1; i <= 7; i++) {
		bt_bus_set_property(bus, i, 30, 0, -1);
	}
	for(int i = 11; i <= 14; i++) {
		bt_bus_set_property(bus, i, 30, 0, -1);
	}
	usleep(1000);
	printf("Done.\n");


	printf(">>> E-stop and then Shift-idle the WAM. Then press [Enter]...\n");
	waitForEnter();

	printf("Waking pucks...\n");
	for(int i = 1; i <= 7; i++) {
		bt_bus_set_property(bus, i, 5, 0, 2); // Set STAT to STATUS_READY
	}
	for(int i = 11; i <= 14; i++) {
		bt_bus_set_property(bus, i, 5, 0, 2); // Set STAT to STATUS_READY
	}
	usleep(500000);
	printf("Done.\n");

	printf("Verifying changes...\n");
	bt_bus_can_clearmsg(bus->dev);
	long reply;
	for(int i = 1; i <= 7; i++) {
		bt_bus_get_property(bus, i, 28, &reply);
		printf("ID%d: ", i);
		if (reply == 4) {
			printf("PASS\n");
		} else {
			printf("FAIL (28 = %ld)\n", reply);
		}
	}
	for(int i = 11; i <= 14; i++) {
		bt_bus_get_property(bus, i, 28, &reply);
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
