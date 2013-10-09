/*
 * moveToPose.cpp
 *
 *  Created on: Nov 17, 2011
 *      Author: dc, bz
 */

#include <iostream>
#include <cassert>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <Eigen/Geometry>

#include <barrett/systems.h>
#include <barrett/math.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>

#include <barrett/detail/stl_utils.h>


using namespace barrett;
using detail::waitForEnter;


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF); 
	pose_type p;

	wam.gravityCompensate();


	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		printf("Press [Enter] to record pose.");
		waitForEnter();
		p = wam.getToolPose();

		printf("Now alter the WAM's pose, then press [Enter] to move back to the recorded pose.");
		waitForEnter();
		wam.moveTo(p);

		printf("Press [Enter] to idle.");
		waitForEnter();
		wam.idle();
	}

	return 0;
}
