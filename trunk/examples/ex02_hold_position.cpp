#include <iostream>
#include <string>

#include <Eigen/Geometry>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();


	systems::ExposedOutput<Eigen::Quaterniond> toSetPoint;

	printf("Commands:\n");
	printf("  j  Hold joint positions\n");
	printf("  p  Hold tool position (in Cartesian space)\n");
	printf("  o  Hold tool orientation\n");
	printf("  i  Idle (release position/orientation constraints)\n");
	printf("  q  Quit\n");

	std::string line;
	bool going = true;
	while (going) {
		printf(">>> ");
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'j':
			wam.moveTo(wam.getJointPositions());
			break;

		case 'p':
			wam.moveTo(wam.getToolPosition());
			break;

		case 'o':
			toSetPoint.setValue(wam.getToolOrientation());
			wam.trackReferenceSignal(toSetPoint.output);
			break;

		case 'i':
			wam.idle();
			break;

		case 'q':
		case 'x':
			going = false;
			break;

		default:
			break;
		}
	}


	wam.idle();  // Release the WAM if we're holding

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
