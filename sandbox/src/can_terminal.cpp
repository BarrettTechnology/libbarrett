/*
 * can_terminal.cpp
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

#include <barrett/bus/bus.h>
#include <barrett/bus/bus_can.h>


using namespace std;


void waitForEnter() {
	static string line;
	getline(cin, line);
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
			bt_bus_set_property(bus, id, property, value);
			printf("SET %d: %d = %ld\n", id, property, value);
			break;

		case 'g':
			cout << "\tProperty: ";
			cin >> property;
			bt_bus_can_get_property(bus->dev, id, property, &value, NULL, 1);
			printf("GET %d: %d = %ld\n", id, property, value);

		default:
			cout << "\t'i' to setactive ID\n"
					"\t's' to set a property\n"
					"\t'g' to get a propety\n";
		}
	}

	return 0;
}
