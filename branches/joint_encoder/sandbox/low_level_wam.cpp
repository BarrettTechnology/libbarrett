/*
 * low_level_wam.cpp
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */

#include <iostream>

#include <libconfig.h++>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::reconnect;
using systems::disconnect;


template<size_t DOF> int wam_main(ProductManager& pm);
int main() {
	// Give us pretty stack traces when things die
	barrett::installExceptionHandler();

	ProductManager pm;
	pm.waitForWam();

	if (pm.foundWam4()) {
		return wam_main<4>(pm);
	} else if (pm.foundWam7()) {
		return wam_main<7>(pm);
	}
}

template<size_t DOF> int wam_main(ProductManager& pm) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	const libconfig::Setting& setting = pm.getConfig().lookup(pm.getWamDefaultConfigPath());

    // instantiate Systems
	std::vector<Puck*> wamPucks = pm.getWamPucks();
	wamPucks.resize(DOF);
	systems::LowLevelWamWrapper<DOF> llww(pm.getExecutionManager(), wamPucks, pm.getSafetyModule(), setting["low_level"]);
	systems::PIDController<jp_type, jt_type> jpController(setting["joint_position_control"]);
	systems::Constant<jp_type> point(llww.getLowLevelWam().getHomePosition());


	// make connections between Systems
	connect(llww.jpOutput, jpController.feedbackInput);
	connect(jpController.controlOutput, llww.input);

	// initially, tie inputs together for zero torque
	connect(llww.jpOutput, jpController.referenceInput);


	// start the main loop!
	pm.getExecutionManager()->start();
	pm.getSafetyModule()->waitForMode(SafetyModule::ACTIVE);

	std::cout << "Press [Enter] to move to home position.\n";
	waitForEnter();
	reconnect(point.output, jpController.referenceInput);

	std::cout << "Press [Enter] to idle.\n";
	waitForEnter();
	reconnect(llww.jpOutput, jpController.referenceInput);
	jpController.resetIntegrator();

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	pm.getExecutionManager()->stop();

	return 0;
}
