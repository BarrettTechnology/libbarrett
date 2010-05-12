/*
 * move_to.cpp
 *
 *  Created on: Apr 20, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <barrett/systems.h>
#include <barrett/wam.h>


using namespace barrett;

const size_t DOF = 4;
const double T_s = 0.002;


typedef Wam<DOF>::jp_type jp_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	libconfig::Config config;
	config.readFile("/etc/wam/wam4-new.config");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	// instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));


	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to compensate for gravity.\n";
	waitForEnter();
	wam.gravityCompensate();


//	jp_type jp1, jp2;
//	jp1 << 0, -M_PI/2.0, 0, 0;
//	jp2 << 0, -M_PI/4.0, 0, 0;
//
//	wam.moveTo(jp1);
//	while (true) {
//		sleep(2);
//		wam.moveTo(jp2, false);
//		sleep(2);
//		wam.moveTo(jp1, false);
//	}

	jp_type jp;
	jp << 0, -M_PI/2.0, 0, 0;

	wam.moveTo(jp);
	while (true) {
		usleep(30000);
		jp[1] += 0.03;
		wam.moveTo(jp, false, 0.5, 1.0);
	}

	rtem.stop();
	return 0;
}
