/*
 * hand_buttons_test.cpp
 *
 *  Created on: Jun 9, 2011
 *      Author: dc
 */

#include <iostream>
#include <unistd.h>

#include <parapin.h>

#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;


double velCommand(bool open, bool close, double speed = 1.25) {
	if (open  &&  !close) {
		return -speed;
	} else if (close  &&  !open) {
		return speed;
	} else {
		return 0.0;
	}
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	if ( !pm.foundHand() ) {
		printf("ERROR: No Hand found on bus!\n");
		return 1;
	}
	Hand& hand = *pm.getHand();

	int ret = pin_init_user(0x378);
	if (ret != 0) {
		printf("Could not initialize parallel port!\n");
		return -1;
	}

	wam.gravityCompensate();

	printf(">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
	waitForEnter();
	hand.initialize();

	// Set pin 1 to output, data pins (2-9) to input
	pin_output_mode( LP_PIN01);
	pin_input_mode( LP_DATA_PINS);

	// Set pin 1 to HIGH
	set_pin(LP_PIN01);


	unsigned char data = 0, data_1 = 0;
	Hand::jv_type hjv;

	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		data = ((pin_is_set(LP_PIN02) ? 0 : 1) << 0) +
			   ((pin_is_set(LP_PIN03) ? 0 : 1) << 1) +
			   ((pin_is_set(LP_PIN04) ? 0 : 1) << 2) +
			   ((pin_is_set(LP_PIN05) ? 0 : 1) << 3) +
			   ((pin_is_set(LP_PIN06) ? 0 : 1) << 4) +
			   ((pin_is_set(LP_PIN07) ? 0 : 1) << 5) +
			   ((pin_is_set(LP_PIN08) ? 0 : 1) << 6) +
			   ((pin_is_set(LP_PIN09) ? 0 : 1) << 7);

		// If things havn't changed, just wait and loop
		if (data != data_1) {
			printf("%x\n", data);
			hjv[0] = velCommand(data & (1<<6), data & (1<<7));
			hjv[1] = velCommand(data & (1<<6), data & (1<<7));
			hjv[2] = velCommand(data & (1<<6), data & (1<<7));
			hjv[3] = velCommand(data & (1<<5), data & (1<<4));

			hand.setVelocity(hjv);
			std::cout << "Velocity: " << hjv << std::endl;

			data_1 = data;
		}

		usleep(10000);
	}


	hand.idle();
	return 0;
}
