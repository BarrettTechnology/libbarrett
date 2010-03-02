/*
 * haptics.cpp
 *
 *  Created on: Feb 19, 2010
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
#include <Eigen/Geometry>

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

typedef Wam<DOF>::jt_type jt_type;
typedef Wam<DOF>::jp_type jp_type;
typedef Wam<DOF>::jv_type jv_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

units::CartesianForce scale(boost::tuple<units::CartesianForce, double> t) {
	return t.get<0>() * t.get<1>();
}

int main() {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	libconfig::Config config;
	config.readFile("/etc/wam/wamg-new.config");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


    // instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));

	units::CartesianPosition center;
	center << 0,0,1;
	systems::HapticBall hb(center, 0.5);
	systems::Gain<double> kp(3e3);
	systems::TupleGrouper<units::CartesianForce, double> tg;
	systems::Callback<boost::tuple<units::CartesianForce, double>, units::CartesianForce> mult(scale);
//	systems::Gain<units::CartesianForce, double> gain(10);
	systems::ToolForceToJointTorques<DOF> tf2jt;


	// connect systems
	connect(wam.toolPosition.output, hb.input);
	connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(hb.directionOutput, tg.getInput<0>());
	connect(hb.depthOutput, kp.input);
	connect(kp.output, tg.getInput<1>());
	connect(tg.output, mult.input);
	connect(mult.output, tf2jt.input);
//	connect(hb.directionOutput, tf2jt.input);
//	connect(hb.directionOutput, gain.input);
//	connect(gain.output, tf2jt.input);
	connect(tf2jt.output, wam.input);


	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to compensate for gravity.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::string line;
	bool going = true;
	units::CartesianPosition cp;
	units::CartesianForce cf;
	while (going) {
		std::cout << ">>> ";
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'q':
		case 'x':
			going = false;
			break;

		default:
			{
				SCOPED_LOCK(rtem.getMutex());
				cp = wam.tpController.feedbackInput.getValue();
				cf = tf2jt.input.getValue();
			}
			std::cout << cp << " " << cf << std::endl;

			break;
		}
	}


	std::cout << "Shift-idle, then press [Enter].\n";
	waitForEnter();
	wam.idle();
	wam.gravityCompensate(false);
	rtem.stop();

	return 0;
}
