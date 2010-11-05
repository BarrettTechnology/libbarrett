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


const size_t DOF = 4;
const double T_s = 0.002;
BARRETT_UNITS_TYPEDEFS(DOF);


int main() {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	BusManager bm;
	const libconfig::Setting& setting = bm.getConfig().lookup("wam4");
	systems::RealTimeExecutionManager& rtem = *bm.getExecutionManager();


    // instantiate Systems
	bm.waitForWam();
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
