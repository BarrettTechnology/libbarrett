/*
 * low_level_wam.cpp
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */

#include <iostream>

#include <libconfig.h++>

#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/bus/bus_manager.h>


using namespace barrett;
using systems::connect;
using systems::reconnect;
using systems::disconnect;


const size_t DOF = 4;
const double T_s = 0.002;
BARRETT_UNITS_TYPEDEFS(DOF);


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	BusManager bm;
	bm.enumerate();
	std::vector<Puck*> wamPucks;
	wamPucks.push_back(bm.getPuck(1));
	wamPucks.push_back(bm.getPuck(2));
	wamPucks.push_back(bm.getPuck(3));
	wamPucks.push_back(bm.getPuck(4));
	const libconfig::Config& config = bm.getConfig();

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


    // instantiate Systems
	systems::LowLevelWamWrapper<DOF> llww(wamPucks, bm.getPuck(10), config.lookup("wam.low_level"));
	systems::PIDController<jp_type, jt_type> jpController(
			config.lookup("wam.joint_position_control"));
	systems::Constant<jp_type> point(
			config.lookup("wam.low_level.home"));


	// make connections between Systems
	connect(llww.jpOutput, jpController.feedbackInput);
	connect(jpController.controlOutput, llww.input);

	// initially, tie inputs together for zero torque
	connect(llww.jpOutput, jpController.referenceInput);


	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to move to home position.\n";
	waitForEnter();
	reconnect(point.output, jpController.referenceInput);

	std::cout << "Press [Enter] to idle.\n";
	waitForEnter();
	reconnect(llww.jpOutput, jpController.referenceInput);
	jpController.resetIntegrator();

	std::cout << "Shift-idle, then press [Enter].\n";
	waitForEnter();
	rtem.stop();

	return 0;
}
