/*
 * calibrate_control_gains.cpp
 *
 *  Created on: Jul 29, 2012
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <libconfig.h++>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include "control_mode_switcher.h"

#include <barrett/standard_main_function.h>

using namespace barrett;

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();

	std::string calibration_file;
	if(DOF == 4)
		calibration_file = "calibration4.conf";
	else
		calibration_file = "calibration7.conf";

	libconfig::Config config;
	config.readFile(calibration_file.c_str());
	ControlModeSwitcher<DOF> cms(pm, wam, config.lookup("control_mode_switcher"));

	printf("\nCalibration Routine Started.\n\n");
	printf("Current Control Calibration.\n");
	cms.currentControl();
	cms.calculateTorqueGain();
	printf("Voltage Control Calibration.\n");
	cms.voltageControl();
	cms.calculateTorqueGain();

	config.lookup("control_mode_switcher.current_gain") = cms.getCurrentGain();
	config.lookup("control_mode_switcher.voltage_gain") = cms.getVoltageGain();
	config.writeFile(calibration_file.c_str());
	printf("Calibration Completed Successfully.\n");
	printf("Saved to Calibration File: %s\n", calibration_file.c_str());

	wam.moveHome();
	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
