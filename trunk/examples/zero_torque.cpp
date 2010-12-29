#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/products/safety_module.h>

#include <barrett/standard_main_function.h>


using namespace barrett;


template<size_t DOF>
int wam_main(int argc, char** argv, BusManager& bm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	// The WAM is now Shift-activated and applying zero torque with a loop-rate of 500Hz.


	// Wait for the user to press Shift-idle
	bm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
