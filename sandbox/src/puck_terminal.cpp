/*
 * puck_terminal.cpp
 *
 *  Created on: May 3, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <sys/mman.h>
#include <unistd.h>
#include <native/task.h>
#include <libconfig.h++>

#include <barrett/cdlbt/bus/bus.h>
#include <barrett/cdlbt/bus/bus_can.h>


using namespace std;


void waitForEnter() {
	static string line;
	getline(cin, line);
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


	string line;
	int id = 0;
	int property = 0;
	long value = 0;
	while (true) {
		cout << ">>> ";
		getline(cin, line);
		switch (line[0]) {
		case 'i':
			cout << "\tSet ID: ";
			cin >> id;
			break;

		case 's':
			cout << "\tProperty: ";
			cin >> property;
			cout << "\tValue: ";
			cin >> value;
			bt_bus_can_set_property(dev, id, property, value);
			printf("SET %d: %d = %ld\n", id, property, value);
			break;

		case 'g':
			cout << "\tProperty: ";
			cin >> property;
			bt_bus_can_get_property(dev, id, property, &value, NULL, 1);
			printf("GET %d: %d = %ld\n", id, property, value);

		default:
			cout << "\t'i' to setactive ID\n"
					"\t's' to set a property\n"
					"\t'g' to get a propety\n";
		}
	}

	return 0;
}
