/*
 * vibrate.cpp
 *
 *  Created on: Jul 25, 2011
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <cstdlib>

#include <Eigen/Geometry>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;


template <typename T>
class Random : public systems::System, public systems::SingleOutput<T> {
public:
	explicit Random(const std::string& sysName = "Random") : systems::System(sysName), systems::SingleOutput<T>(this) {}
	virtual ~Random() { mandatoryCleanUp(); }

protected:
	T data;

	virtual void operate() {
		data.setRandom();
		this->outputValue->setData(&data);
	}
};


void printMenu() {
	printf("Commands:\n");
	printf("  j  Hold joint positions\n");
	printf("  p  Hold tool position (in Cartesian space)\n");
	printf("  o  Hold tool orientation\n");
	printf("  i  Idle (release position/orientation constraints)\n");
	printf("  q  Quit\n");
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();


	systems::ExposedOutput<Eigen::Quaterniond> toSetPoint;

//	Random<cf_type> rand;
//	systems::FirstOrderFilter<cf_type> fof;
//	fof.setLowPass(cf_type(60.0));
//	systems::Gain<cf_type, double> gain(0.1);
//	systems::ToolForceToJointTorques<DOF> tf2jt;
//	systems::connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
//	systems::connect(rand.output, fof.input);
//	systems::connect(fof.output, gain.input);
//	systems::connect(gain.output, tf2jt.input);

	Random<cf_type> rand;
	systems::Gain<cf_type, double> gain(0.1);
	systems::ToolForceToJointTorques<DOF> tf2jt;
	systems::connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
	systems::connect(rand.output, gain.input);
	systems::connect(gain.output, tf2jt.input);

	printMenu();

	std::string line;
	bool going = true;
	while (going) {
		printf(">>> ");
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'j':
			printf("Holding joint positions.\n");
			wam.moveTo(wam.getJointPositions());
			break;

		case 'p':
			printf("Holding tool position.\n");
			wam.moveTo(wam.getToolPosition());
			break;

		case 'o':
			printf("Holding tool orientation.\n");
			toSetPoint.setValue(wam.getToolOrientation());
			wam.trackReferenceSignal(toSetPoint.output);
			break;

		case 'i':
			printf("WAM idled.\n");
			wam.idle();
			break;

		case 'v':
			if (tf2jt.output.isConnected()) {
				printf("Vibration off.\n");
				systems::disconnect(tf2jt.output);
			} else {
				printf("Vibration on");
				if (line.size() > 1  &&  line[1] == ' ') {
					double amp = strtod(line.c_str() + 2, NULL);
					gain.setGain(amp);
					printf(" with amplitude %f", amp);
				}
				printf(".\n");
				systems::connect(tf2jt.output, wam.input);
			}
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


	wam.idle();  // Release the WAM if we're holding

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
