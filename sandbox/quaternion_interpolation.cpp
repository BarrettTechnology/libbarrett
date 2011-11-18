/*
 * quaternion_interpolation.cpp
 *
 *  Created on: Nov 17, 2011
 *      Author: dc
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

	Eigen::Quaterniond q;

	wam.gravityCompensate();


	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		printf("Press [Enter] to record first orientation.");
		waitForEnter();
		q = wam.getToolOrientation();

		printf("Press [Enter] to move back to first orientation.");
		waitForEnter();
		wam.moveTo(q);

		printf("Press [Enter] to idle.");
		waitForEnter();
		wam.idle();
	}

	return 0;
}
