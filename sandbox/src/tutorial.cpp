/*
 * tutorial.cpp
 *
 *  Created on: Apr 7, 2010
 *      Author: arm_user
 */

#include <iostream>
#include <string>

#include <boost/thread.hpp>

#include <barrett/systems.h>
#include <barrett/wam.h>


using namespace barrett;
using systems::connect;

const size_t DOF = 7;
const double T_s = 0.002;


typedef Wam<DOF>::jp_type jp_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

void handThread(struct bt_bus *bus);

int main() {
	libconfig::Config config;
	config.readFile("/etc/wam/wam7-new.config");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	// instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));
	wam.jpController.setControlSignalLimit(jp_type(0.0));  // don't limit output torques
//	systems::ExposedOutput<jp_type> jpOutput;


	// connect systems
//	connect(***, wam.input);  // send a torque command
//	wam.trackReferenceSignal(jpOutput.output);


	// start the main loop!
	rtem.start();

	std::cout << "Shift-activate, then press enter.\n";
	waitForEnter();
	wam.gravityCompensate();

	boost::thread(handThread, wam.wam.wambot->bus);

	//////// DO STUFF
//	jpOutput.setValue(jp_type(0.0));

	jp_type jp(config.lookup("wam.low_level.home"));
//	std::string line;
//	bool going = true;
	int jIdx = 0;
	double angle = 0;

	while (true) {
		std::cout << "Joint: ";
		std::cin >> jIdx;
		jIdx--;

		if (jIdx < 0) {
			break;
		}

		std::cout << "Angle: ";
		std::cin >> angle;

		jp[jIdx] = angle;
		wam.moveTo(jp, false);

		std::cout << wam.getJointPositions() << std::endl;


//		std::cin >> line;
//		switch (line[0]) {
//		case '':
//
//		default:
//			std::cout <<"wrong.\n";
//		}
	}

	wam.moveHome();

	//////// END DO STUFF

	std::cout << "Shift-idle, then press enter.\n";
	waitForEnter();

	rtem.stop();
	return 0;
}


void handThread(struct bt_bus *bus) {
	int spreadspeed = 20;
	int i;

	printf("Initializing hand ...\n");
	printf("Waking hand pucks ...\n");

	for(i = 11; i <= 14; i++) {
		bt_bus_set_property(bus, i, 5, 0, 2); // Set STAT to STATUS_READY
	}

	usleep((long)1e6);

	for(i = 11; i <= 14; i++) {
		bt_bus_set_property(bus, i, 78, 0, 50); // Set TSTOP to 50 ms
	}

	printf(" ... done.\n");

	while (1) {
		sleep(1);
		spreadspeed = -spreadspeed;
		printf("Sending spread command(%d)\n",spreadspeed);
		bt_bus_set_property(bus, 14, 44, 0, spreadspeed);
	}
}
