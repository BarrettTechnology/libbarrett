/*
 * joint_encoder_init.cpp
 *
 *  Created on: Jun 17, 2012
 *      Author: dc
 */


#include <cstdio>
#include <cassert>
#include <cmath>

#include <barrett/math.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#include <barrett/standard_main_function.h>


using namespace barrett;


// Index pulses are 15 degrees away from the standard home position...
const double OFFSET = 15 * M_PI/180.0;
// ... plus or minus 10 degrees.
const double TOLERANCE = 10 * M_PI/180.0;


template<size_t DOF> void printEncoderStatus(const systems::Wam<DOF>& wam) {
	for (size_t i = 0; i < DOF; ++i) {
		printf("  Encoder %u ", (unsigned int)i);
		if (wam.getLowLevelWam().getMotorPucks()[i].foundIndexPulse()) {
			printf("initialized\n");
		} else {
			printf("not initialized\n");
		}
	}
}


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	assert(wam.getLowLevelWam().hasJointEncoders());


	// Otherwise the "farSideOfIndexes" position might cause self-collisions.
	assert(TOLERANCE < OFFSET);

	jp_type indexPositions(0.0);
	indexPositions[0] = 0.0 - OFFSET;
	indexPositions[1] = -2.0 + OFFSET;
	indexPositions[2] = 0.0 - OFFSET;
	indexPositions[3] = M_PI - OFFSET;

	v_type dir = math::sign(indexPositions - wam.getJointPositions());
	jp_type farSideOfIndexes = indexPositions + TOLERANCE * dir;


	printf("Before:\n");
	printEncoderStatus(wam);

	pm.getSafetyModule()->waitForMode(SafetyModule::ACTIVE);
	wam.gravityCompensate();
	wam.moveTo(farSideOfIndexes);
	wam.moveHome();

	printf("After:\n");
	printEncoderStatus(wam);

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	return 0;
}
