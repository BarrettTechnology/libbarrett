/*
 * move_to.cpp
 *
 *  Created on: Apr 20, 2010
 *      Author: dc
 */

#include <iostream>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/products/safety_module.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;


template<size_t DOF>
int wam_main(int argc, char** argv, BusManager& bm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();


	jp_type jp(0.0);

	std::cout << "Press [Enter] to move to Joint position: " << jp << "\n";
	waitForEnter();
	wam.moveTo(jp);

	std::cout << "Press [Enter] to move to Home position: " << wam.getHomePosition() << "\n";
	waitForEnter();
	wam.moveHome();


	bm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
