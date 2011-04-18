#include <cstdio>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>


using namespace barrett;


int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<4>& wam) {
	printf("ERROR: This program is designed to be used with 7-DOF WAMs only.\n");
	return 1;
}

const size_t DOF = 7;
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TYPEDEFS(DOF);

	wam.gravityCompensate();


	// ...


	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}


#define BARRETT_SMF_NO_DECLARE
#include <barrett/standard_main_function.h>
