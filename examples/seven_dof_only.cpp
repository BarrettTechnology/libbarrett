#include <cstdio>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/products/safety_module.h>


using namespace barrett;


int wam_main(int argc, char** argv, BusManager& bm, systems::Wam<4>& wam) {
	printf("ERROR: This program is designed to be used only with 7-DOF WAMs.\n");
	return 1;
}

const size_t DOF = 7;
int wam_main(int argc, char** argv, BusManager& bm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TYPEDEFS(DOF);

	wam.gravityCompensate();


	// ...


	// Wait for the user to press Shift-idle
	bm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}


#define BARRETT_SMF_NO_DECLARE
#include <barrett/standard_main_function.h>
