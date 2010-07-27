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

typedef LowLevelWam<DOF>::jt_type jt_type;
typedef LowLevelWam<DOF>::jp_type jp_type;
typedef LowLevelWam<DOF>::jv_type jv_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	libconfig::Config config;
	config.readFile("/etc/barrett/wam7.conf");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


    // instantiate Systems
	LowLevelWam<DOF> llw(config.lookup("wam.low_level"));
	systems::PIDController<jp_type, jt_type> jpController(
			config.lookup("wam.joint_position_control"));
	systems::Constant<jp_type> point(
			config.lookup("wam.low_level.home"));


	// make connections between Systems
	connect(llw.jpOutput, jpController.feedbackInput);
	connect(jpController.controlOutput, llw.input);

	// initially, tie inputs together for zero torque
	connect(llw.jpOutput, jpController.referenceInput);


	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to move to home position.\n";
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
