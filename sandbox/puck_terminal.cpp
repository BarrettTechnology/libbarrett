/*
 * puck_terminal.cpp
 *
 *  Created on: May 3, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <cstdio>

#include <unistd.h>

#include <barrett/bus/bus_manager.h>
#include <barrett/products/puck.h>


using namespace std;
using namespace barrett;


void waitForEnter() {
	static string line;
	getline(cin, line);
}


int main(int argc, char** argv) {
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
	bus::BusManager bus(port);


	string line;
	int id = 0;
	int property = 0;
	int value = 0;
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
			Puck::setProperty(bus, id, property, value);
			printf("SET %d: %d = %d\n", id, property, value);
			break;

		case 'g':
			cout << "\tProperty: ";
			cin >> property;
			value = Puck::getProperty(bus, id, property);
			printf("GET %d: %d = %d\n", id, property, value);

		default:
			cout << "\t'i' to setactive ID\n"
					"\t's' to set a property\n"
					"\t'g' to get a propety\n";
		}
	}

	return 0;
}
