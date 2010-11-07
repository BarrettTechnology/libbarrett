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
#include <barrett/bus/bus_manager.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::reconnect;
using systems::disconnect;


template<size_t DOF> int wam_main(BusManager& bm);
int main() {
	// Give us pretty stack traces when things die
	barrett::installExceptionHandler();

	BusManager bm;
	bm.waitForWam();

	if (bm.wam4Found()) {
		return wam_main<4>(bm);
	} else if (bm.wam7Found()) {
		return wam_main<7>(bm);
	}
}

template<size_t DOF> int wam_main(BusManager& bm) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	const libconfig::Setting& setting = bm.getConfig().lookup(bm.getWamDefaultConfigPath());
	systems::RealTimeExecutionManager& rtem = *bm.getExecutionManager();

    // instantiate Systems
	std::vector<Puck*> wamPucks = bm.getWamPucks();
	wamPucks.resize(DOF);
	systems::LowLevelWamWrapper<DOF> llww(wamPucks, bm.getSafetyModule(), setting["low_level"]);
	systems::PIDController<jp_type, jt_type> jpController(setting["joint_position_control"]);
	systems::Constant<jp_type> point(llww.getLowLevelWam().getHomePosition());


	// make connections between Systems
	connect(llww.jpOutput, jpController.feedbackInput);
	connect(jpController.controlOutput, llww.input);

	// initially, tie inputs together for zero torque
	connect(llww.jpOutput, jpController.referenceInput);


	// start the main loop!
	rtem.start();
	bm.getSafetyModule()->waitForMode(SafetyModule::ACTIVE);

	std::cout << "Press [Enter] to move to home position.\n";
	waitForEnter();
	reconnect(point.output, jpController.referenceInput);

	std::cout << "Press [Enter] to idle.\n";
	waitForEnter();
	reconnect(llww.jpOutput, jpController.referenceInput);
	jpController.resetIntegrator();

	bm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	rtem.stop();

	return 0;
}
