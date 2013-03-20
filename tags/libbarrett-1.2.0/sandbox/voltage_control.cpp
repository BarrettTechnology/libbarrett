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
class ControlModeSwitcher {
public:
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	enum ControlMode { CURRENT, VOLTAGE };

	ControlModeSwitcher(ProductManager& pm_, systems::Wam<DOF>& wam_, double currentTorqueGain, double voltageTorqueGain) :
		pm(pm_), wam(wam_), mode(VOLTAGE), cGain(currentTorqueGain), vGain(voltageTorqueGain), torqueGainSys(vGain)
	{
		for (size_t i = 0; i < DOF; ++i) {
			assert(wam.getLowLevelWam().getPucks()[i]->getVers() >= 200);
		}

		systems::connect(wam.jtSum.output, torqueGainSys.input);
		systems::reconnect(torqueGainSys.output, wam.llww.input);
		currentControl();
	}
	~ControlModeSwitcher() {
		currentControl();  // Revert back to current control.
		systems::reconnect(wam.jtSum.output, wam.llww.input);
	}

	enum ControlMode getMode() const { return mode; }
	void currentControl() {
		if (getMode() == CURRENT) {
			return;
		}

		for (size_t i = 0; i < DOF; ++i) {
			wam.getLowLevelWam().getPucks()[i]->resetProperty(Puck::IKCOR);
			wam.getLowLevelWam().getPucks()[i]->resetProperty(Puck::IKP);
			wam.getLowLevelWam().getPucks()[i]->resetProperty(Puck::IKI);
		}

		wam.jpController.setFromConfig(pm.getConfig().lookup(pm.getWamDefaultConfigPath())["joint_position_control"]);
		torqueGainSys.setGain(cGain);

		mode = CURRENT;
	}
	void voltageControl() {
		if (getMode() == VOLTAGE) {
			return;
		}

		wam.getLowLevelWam().getPuckGroup().setProperty(Puck::IKI, 0);
		wam.getLowLevelWam().getPuckGroup().setProperty(Puck::IKP, 8000);
		wam.getLowLevelWam().getPuckGroup().setProperty(Puck::IKCOR, 0);

		wam.jpController.setKi(v_type(0.0));
		wam.jpController.setKd(v_type(0.0));
		wam.jpController.resetIntegrator();
		torqueGainSys.setGain(vGain);

		mode = VOLTAGE;
	}


	static const double MAX_SCALE = 4.0;
	static const double MIN_SCALE = 0.25;

	void calculateTorqueGain() {
		libconfig::Config config;
		config.readFile("/etc/barrett/calibration.conf");
		libconfig::Setting& setting = config.lookup("gravitycal")[pm.getWamDefaultConfigPath()];

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
			if (error.norm() > 0.4 * supporting.norm()) {
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

		assert(scaleCount >= 3);
		double meanScale = scaleSum/scaleCount;
		printf("SCALE: %f (+%f, -%f)\n", meanScale, maxScale - meanScale, meanScale - minScale);

		switch (getMode()) {
		case CURRENT:
			cGain = meanScale;
			torqueGainSys.setGain(cGain);
			break;
		case VOLTAGE:
			vGain = meanScale;
			torqueGainSys.setGain(vGain);
			break;
		default:
			assert(false);
			break;
		}
	}

protected:
	ProductManager& pm;
	systems::Wam<DOF>& wam;
	enum ControlMode mode;
	double cGain, vGain;
	systems::Gain<jt_type, double> torqueGainSys;
};


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


	ControlModeSwitcher<DOF> cms(pm, wam, 1.17, 0.82);

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
			cms.voltageControl();
			break;

		case 'c':
			printf("Switching to current control!\n");
			cms.currentControl();
			break;

		case 'g':
			printf("Calculating torque gain...\n");
			cms.calculateTorqueGain();
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
