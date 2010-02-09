/*
 * high_level_wam.cpp
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


int main() {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	libconfig::Config config;
	config.readFile("/etc/wam/wamg-new.config");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


    // instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));

	systems::Constant<jp_type> jpPoint(config.lookup("wam.low_level.home"));

	math::Kinematics<DOF> kin(config.lookup("wam.kinematics"));
	kin.eval(config.lookup("wam.low_level.home"), jv_type());
	systems::Constant<units::CartesianPosition> tpPoint(
			units::CartesianPosition(kin.impl->tool->origin_pos));

	Eigen::Matrix3d rot;
	for (size_t r = 0; r < 3; ++r) {
		for (size_t c = 0; c < 3; ++c) {
			rot(c,r) = gsl_matrix_get(kin.impl->tool->rot_to_world, r,c);  // transpose to get tool to world transform
		}
	}
	Eigen::Quaterniond q(rot);
	systems::Constant<Eigen::Quaterniond> toPoint(q);


	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to compensate for gravity.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::cout << "Press [Enter] to move to joint-space home position.\n";
	waitForEnter();
	wam.trackReferenceSignal(jpPoint.output);

	std::cout << "Press [Enter] to move to Cartesian home position.\n";
	waitForEnter();
	wam.trackReferenceSignal(tpPoint.output);

	std::cout << "Press [Enter] to move to Cartesian home orientation.\n";
	waitForEnter();
	wam.trackReferenceSignal(toPoint.output);

	std::cout << "Press [Enter] to idle.\n";
	waitForEnter();
	wam.idle();
	wam.gravityCompensate(false);

	std::cout << "Shift-idle, then press [Enter].\n";
	waitForEnter();
	rtem.stop();

	return 0;
}
