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
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	wam.gravityCompensate();

    // instantiate Systems
	systems::Constant<jp_type> jpPoint(wam.getHomePosition());

	math::Kinematics<DOF> kin(pm.getConfig().lookup(pm.getWamDefaultConfigPath())["kinematics"]);
	kin.eval(wam.getHomePosition(), jv_type(0.0));
	systems::Constant<cp_type> tpPoint(cp_type(kin.impl->tool->origin_pos));

	math::Matrix<3,3> rot(kin.impl->tool->rot_to_world);
	Eigen::Quaterniond q(rot.transpose());  // transpose to get world-to-tool transform
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

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
