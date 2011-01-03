#include <iostream>
#include <string>

#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();


	systems::ExposedOutput<jp_type> setPoint;

	std::string line;
	bool going = true, holding = false;
	std::cout << "Press 'h' to toggle holding position. Press 'q' to exit." << std::endl;
	while (going) {
		std::cout << ">>> ";
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'h':
			holding = !holding;
			if (holding) {
				BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());

				setPoint.setValue(wam.getJointPositions());
				wam.trackReferenceSignal(setPoint.output);
			} else {
				wam.idle();
			}
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
