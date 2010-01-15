/*
 * cartesian_hold.cpp
 *
 *  Created on: Jan 14, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <unistd.h>  // usleep

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <libconfig.h>

#include <barrett/exception.h>
#include <barrett/detail/debug.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/systems.h>
#include <barrett/wam.h>


//namespace log = barrett::log;
namespace math = barrett::math;
namespace systems = barrett::systems;
namespace units = barrett::units;
using barrett::Wam;
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

	math::Array<DOF> tmp;


	struct config_t config;
	char filename[] = "/etc/wam/wamg.config";
	config_init(&config);
	syslog(LOG_ERR,"Open '%s' ...",filename);
	int err = config_read_file(&config,filename);
	if (err != CONFIG_TRUE) {
		syslog(LOG_ERR,"libconfig error: %s, line %d\n",
		config_error_text(&config), config_error_line(&config));
		config_destroy(&config);
		/* Failure */
		return -1;
	}
	config_setting_t* wamconfig = config_lookup(&config, "wam");
	math::Kinematics<DOF> kin(config_setting_get_member(wamconfig, "kinematics"));
	config_destroy(&config);


	systems::RealTimeExecutionManager rtem;
	systems::System::defaultExecutionManager = &rtem;


	Wam<DOF> wam;
	systems::PIDController<Wam<DOF>::jp_type> pid;

	systems::TupleGrouper<Wam<DOF>::jp_type, Wam<DOF>::jv_type> kinTg;
	systems::Callback<boost::tuple<Wam<DOF>::jp_type, Wam<DOF>::jv_type>, units::CartesianPosition> kinSys(ref(kin), true);
//	systems::PrintToStream<boost::tuple<Wam<DOF>::jp_type, Wam<DOF>::jv_type> > printTuple;
//	systems::PrintToStream<units::CartesianPosition> printCpos;

	tmp << 900, 2500, 600, 500, 40, 20, 5;
	pid.setKp(tmp);
	tmp << 2, 2, 0.5, 0.8, 0.8, 0.1, 0.1;
	pid.setKd(tmp);
	tmp << 25.0, 20.0, 15.0, 15.0, 5, 5, 5;
	pid.setControlSignalLimit(tmp);

	connect(wam.jpOutput, pid.feedbackInput);
	connect(pid.controlOutput, wam.input);

	// tie inputs together for zero torque
	systems::forceConnect(wam.jpOutput, pid.referenceInput);

	connect(wam.jpOutput, kinTg.getInput<0>());
	connect(wam.jvOutput, kinTg.getInput<1>());
	connect(kinTg.output, kinSys.input);
//	connect(kinTg.output, printTuple.input);
//	connect(kinSys.output, printCpos.input);


	// start the main loop!
	rtem.start();

	std::cout << "Enter to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();

	{
		SCOPED_LOCK(rtem.getMutex());
		std::cout << units::CartesianPosition(kin.impl->tool->origin_pos);
	}

	std::cout << "Enter to move home.\n";
	waitForEnter();
	{
		SCOPED_LOCK(rtem.getMutex());
		std::cout << units::CartesianPosition(kin.impl->tool->origin_pos);
	}
	wam.moveHome();
	while ( !wam.moveIsDone() ) {
		usleep(static_cast<int>(1e5));
	}

	wam.gravityCompensate(false);
	wam.idle();

	std::cout << "Shift-idle, then press enter.\n";
	waitForEnter();
	rtem.stop();

	return 0;
}
