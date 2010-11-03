/*
 * high_level_wam.cpp
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <vector>

#include <libconfig.h++>
#include <Eigen/Geometry>

#include <barrett/exception.h>
#include <barrett/math.h>
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
	systems::Wam<DOF> wam(wamPucks, bm.getPuck(10), config.lookup("wam"));

	systems::Constant<jp_type> jpPoint(wam.getHomePosition());

	math::Kinematics<DOF> kin(config.lookup("wam.kinematics"));
	kin.eval(wam.getHomePosition(), jv_type(0.0));
	systems::Constant<units::CartesianPosition::type> tpPoint(
			units::CartesianPosition::type(kin.impl->tool->origin_pos));

	Eigen::Matrix3d rot;
	for (size_t r = 0; r < 3; ++r) {
		for (size_t c = 0; c < 3; ++c) {
			rot(c,r) = gsl_matrix_get(kin.impl->tool->rot_to_world, r,c);  // transpose to get tool-to-world transform
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
