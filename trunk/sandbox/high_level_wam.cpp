/*
 * high_level_wam.cpp
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */

#include <iostream>
#include <vector>

#include <libconfig.h++>
#include <Eigen/Geometry>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/bus/bus_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::reconnect;
using systems::disconnect;


template<size_t DOF>
int wam_main(BusManager& bm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	wam.gravityCompensate();

    // instantiate Systems
	systems::Constant<jp_type> jpPoint(wam.getHomePosition());

	math::Kinematics<DOF> kin(bm.getConfig().lookup("wam4.kinematics"));
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


	// interact with user
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

	bm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
