/*
 * low_level_wam.cpp
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <unistd.h>  // usleep

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <libconfig.h++>

#include <barrett/exception.h>
#include <barrett/detail/debug.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/systems.h>
#include <barrett/systems/low_level_wam.h>


//namespace log = barrett::log;
namespace math = barrett::math;
namespace systems = barrett::systems;
namespace units = barrett::units;
using barrett::systems::LowLevelWam;
using systems::connect;
using systems::reconnect;
using systems::disconnect;


using boost::bind;
using boost::ref;


const size_t DOF = 7;
const double T_s = 0.002;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	libconfig::Config config;
	config.readFile("/etc/wam/wamg.config");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	LowLevelWam<DOF>::jp_type setPoint(config.lookup("wam.wambot.home"));
	math::Array<DOF> tmp;


    // instantiate Systems
	LowLevelWam<DOF> llw(config.lookup("wam.wambot"));
	systems::PIDController<LowLevelWam<DOF>::jp_type> jpController;
	systems::Constant<LowLevelWam<DOF>::jp_type> point(setPoint);


    // configure Systems
	tmp << 900, 2500, 600, 500, 40, 20, 5;
	jpController.setKp(tmp);
	tmp << 2, 2, 0.5, 0.8, 0.8, 0.1, 0.1;
	jpController.setKd(tmp);
	tmp << 25.0, 20.0, 15.0, 15.0, 5, 5, 5;
	jpController.setControlSignalLimit(tmp);


	// make connections between Systems
	connect(llw.jpOutput, jpController.feedbackInput);
	connect(jpController.controlOutput, llw.input);

	// initially, tie inputs together for zero torque
	connect(llw.jpOutput, jpController.referenceInput);


	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to move to set point.\n";
	waitForEnter();
	reconnect(point.output, jpController.referenceInput);

	std::cout << "Press [Enter] to idle.\n";
	waitForEnter();
	reconnect(llw.jpOutput, jpController.referenceInput);

	std::cout << "Shift-idle, then press [Enter].\n";
	waitForEnter();
	rtem.stop();

	return 0;
}
