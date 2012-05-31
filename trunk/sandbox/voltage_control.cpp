/*
 * voltage_control.cpp
 *
 *  Created on: May 15, 2012
 *      Author: dc
 */

#include <vector>
#include <cassert>
#include <iostream>
#include <string>
#include <cstdlib>  // For strtod()

#include <libconfig.h++>

#include <barrett/os.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;


template<size_t DOF>
void switchToVoltageControl(ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.llww.getLowLevelWam().getPuckGroup().setProperty(Puck::IKI, 0);
	wam.llww.getLowLevelWam().getPuckGroup().setProperty(Puck::IKP, 8000);
	wam.llww.getLowLevelWam().getPuckGroup().setProperty(Puck::IKCOR, 0);

	wam.jpController.setKi(v_type(0.0));
	wam.jpController.setKd(v_type(0.0));
	wam.jpController.resetIntegrator();
}

template<size_t DOF>
void switchToCurrentControl(ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	for (size_t i = 0; i < DOF; ++i) {
		wam.llww.getLowLevelWam().getPucks()[i]->resetProperty(Puck::IKCOR);
		wam.llww.getLowLevelWam().getPucks()[i]->resetProperty(Puck::IKP);
		wam.llww.getLowLevelWam().getPucks()[i]->resetProperty(Puck::IKI);
	}

	wam.jpController.setFromConfig(pm.getConfig().lookup(pm.getWamDefaultConfigPath())["joint_position_control"]);
}

template<size_t DOF>
void calculateTorqueGain(ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	libconfig::Config config;
	config.readFile("/etc/barrett/calibration.conf");
	libconfig::Setting& setting = config.lookup("gravitycal")[pm.getWamDefaultConfigPath()];


	const double MAX_SCALE = 4.0;
	const double MIN_SCALE = 0.25;

	int scaleCount = 0;
	double scaleSum = 0.0;
	double maxScale = MIN_SCALE;
	double minScale = MAX_SCALE;

	for (int i = 0; i < setting.getLength(); ++i) {
		wam.moveTo(jp_type(setting[i]));
		btsleep(1.0);

		jt_type gravity = wam.jtSum.getInput(systems::Wam<DOF>::GRAVITY_INPUT).getValue();
		jt_type supporting = wam.llww.input.getValue();
		double scale = (gravity.dot(supporting) / gravity.norm()) / gravity.norm();
		jt_type error = supporting - scale*gravity;

		std::cout << "Gravity: " << gravity << ", " << gravity.norm() << "\n";
		std::cout << "Supporting: " << supporting << ", " << supporting.norm() << "\n";
		std::cout << "Scale: " << scale << "\n";
		std::cout << "Error: " << error << ", " << error.norm() << "\n";

		bool good = true;
		if (scale < MIN_SCALE  ||  scale > MAX_SCALE) {
			printf("Scale is out of range.\n");
			good = false;
		}
		if (error.norm() > 0.2 * supporting.norm()) {
			printf("Error is too big.\n");
			good = false;
		}

		if (good) {
			scaleCount++;
			scaleSum += scale;
			maxScale = math::max(maxScale, scale);
			minScale = math::min(minScale, scale);
		}

		printf("\n");
	}

	double meanScale = scaleSum/scaleCount;
	printf("SCALE: %f (+%f, -%f)\n", meanScale, maxScale - meanScale, meanScale - minScale);
}


template<int R, int C, typename Units>
bool parseDoubles(math::Matrix<R,C, Units>* dest, const std::string& str) {
	const char* cur = str.c_str();
	const char* next = cur;

	for (int i = 0; i < dest->size(); ++i) {
		(*dest)[i] = strtod(cur, (char**) &next);
		if (cur == next) {
			return false;
		} else {
			cur = next;
		}
	}

	// Make sure there are no extra numbers in the string.
	double ignore = strtod(cur, (char**) &next);
	(void)ignore;  // Prevent unused variable warnings

	if (cur != next) {
		return false;
	}

	return true;
}

template<size_t DOF, int R, int C, typename Units>
void moveToStr(systems::Wam<DOF>& wam, math::Matrix<R,C, Units>* dest,
		const std::string& description, const std::string& str)
{
	if (parseDoubles(dest, str)) {
		std::cout << "Moving to " << description << ": " << *dest << std::endl;
		wam.moveTo(*dest);
	} else {
		printf("ERROR: Please enter exactly %d numbers separated by "
				"whitespace.\n", dest->size());
	}
}

void printMenu() {
	printf("Commands:\n");
	printf("  j  Enter a joint position destination\n");
	printf("  p  Enter a tool position destination\n");
	printf("  h  Move to the home position\n");
	printf("  i  Idle (release position/orientation constraints)\n");
	printf("  q  Quit\n");
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	jp_type jp;
	cp_type cp;


	for (size_t i = 0; i < DOF; ++i) {
		assert(wam.llww.getLowLevelWam().getPucks()[i]->getVers() >= 200);
	}


	wam.gravityCompensate();
	printMenu();

	std::string line;
	bool going = true;
	while (going) {
		printf(">>> ");
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'j':
			moveToStr(wam, &jp, "joint positions", line.substr(1));
			break;

		case 'p':
			moveToStr(wam, &cp, "tool position", line.substr(1));
			break;

		case 'h':
			std::cout << "Moving to home position: "
					<< wam.getHomePosition() << std::endl;
			wam.moveHome();
			break;

		case 'i':
			printf("WAM idled.\n");
			wam.idle();
			break;

		case 'v':
			printf("Switching to voltage control!\n");
			switchToVoltageControl(pm, wam);
			break;

		case 'c':
			printf("Switching to current control!\n");
			switchToCurrentControl(pm, wam);
			break;

		case 'g':
			printf("Calculating torque gain...\n");
			calculateTorqueGain(pm, wam);
			break;

		case 'q':
		case 'x':
			printf("Quitting.\n");
			going = false;
			break;

		default:
			if (line.size() != 0) {
				printf("Unrecognized option.\n");
				printMenu();
			}
			break;
		}
	}


	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
