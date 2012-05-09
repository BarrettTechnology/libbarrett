/*
 * log_velocity.cpp
 *
 *  Created on: Apr 21, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/systems.h>


using namespace barrett;

const size_t DOF = 4;
const double T_s = 0.002;
BARRETT_UNITS_TYPEDEFS(DOF);

const char BIN_FILE[] = "/tmp/jv.bin";
const char CSV_FILE[] = "/tmp/jv.csv";


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	libconfig::Config config;
	config.readFile("/etc/barrett/wam4.conf");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;

	// instantiate Systems
	systems::Wam<DOF> wam(config.lookup("wam"));
	systems::PeriodicDataLogger<units::JointVelocities<DOF>::type> dl(
			new log::RealTimeWriter<units::JointVelocities<DOF>::type>(BIN_FILE, T_s),
			1);

	systems::connect(wam.jvOutput, dl.input);

	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to compensate for gravity.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::cout << "Press [Enter] to exit.\n";
	waitForEnter();

	rtem.stop();
	dl.closeLog();

	log::Reader<units::JointVelocities<DOF>::type> lr(BIN_FILE);
	lr.exportCSV(CSV_FILE);

	return 0;
}
