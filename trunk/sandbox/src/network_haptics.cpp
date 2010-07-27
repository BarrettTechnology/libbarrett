/*
 * network_haptics.cpp
 *
 *  Created on: Apr 14, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <barrett/systems.h>
#include <barrett/wam.h>

#include "network_haptics.h"


using namespace barrett;
using systems::connect;

const size_t DOF = 4;
const double T_s = 0.002;

typedef Wam<DOF>::jt_type jt_type;
typedef Wam<DOF>::jp_type jp_type;
typedef Wam<DOF>::jv_type jv_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main(int argc, char** argv) {
	// check arguments
	if (argc != 2) {
		printf("Usage: %s <otherip>\n", argv[0]);
		return 0;
	}

	libconfig::Config config;
	config.readFile("/etc/barrett/wam4.conf");

	systems::RealTimeExecutionManager rtem(T_s, false);
	systems::System::defaultExecutionManager = &rtem;


	// instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));
	systems::ToolForceToJointTorques<DOF> tf2jt;
	NetworkHaptics nh(argv[1]);


	// connect systems
	connect(wam.toolPosition.output, nh.input);
	connect(nh.output, tf2jt.input);
	connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(tf2jt.output, wam.input);


	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to compensate for gravity.\n";
	waitForEnter();

	wam.gravityCompensate();

	while (true) {
		sleep(1);
	}

	rtem.stop();
	return 0;
}
