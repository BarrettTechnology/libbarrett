/*
 * tuning.cpp
 *
 *  Created on: Jan 17, 2012
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <libconfig.h++>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();
	bool gComp = true;
	pm.getSafetyModule()->setVelocityLimit(0.3);

	jp_type jp;
	size_t joint = 0;
	systems::ExposedOutput<jp_type> setPoint;

	std::string line;
	bool going = true;
	while (going) {
		printf(">>> ");
		std::getline(std::cin, line);

		if (line.size() == 0) {
			wam.moveTo(jp);

			libconfig::Config config;
			config.readFile("tuning.conf");
			systems::PIDController<jp_type, jt_type> jpc(config.lookup(pm.getWamDefaultConfigPath())["joint_position_control"]);
			systems::connect(setPoint.output, jpc.referenceInput);
			systems::connect(wam.jpOutput, jpc.feedbackInput);

			wam.idle();
			usleep(100000);
			systems::connect(jpc.controlOutput, wam.input);
			printf("Controller updated.\n");

			printf("Press enter for step.\n");
			std::getline(std::cin, line);
			jp[joint] += 0.02;
			setPoint.setValue(jp);

			std::getline(std::cin, line);
			systems::disconnect(wam.input);
			wam.moveTo(jp);
			printf("Original controller now in use.\n");

			continue;
		}

		switch (line[0]) {
		case 'h':
			printf("Set point updated.\n");
			jp = wam.getJointPositions();
			wam.moveTo(jp);
			setPoint.setValue(jp);
			break;

		case 'i':
			printf("WAM idled.\n");
			wam.idle();
			break;

		case 'j':
			joint = (joint + 1) % DOF;
			printf("Step joint %zu.\n", joint+1);
			break;

		case 'g':
			gComp = ! gComp;
			wam.gravityCompensate(gComp);
			break;

		case 'q':
		case 'x':
			printf("Quitting.\n");
			going = false;
			break;

		default:
			printf("Unrecognized option.\n");
			break;
		}
	}


	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
