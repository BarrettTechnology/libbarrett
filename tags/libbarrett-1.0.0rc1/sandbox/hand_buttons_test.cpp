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


const double T_s = 0.001;
const double USLEEP_TIME = 100;


double velCommand(bool open, bool close, double speed = 1.25) {
	if (open  &&  !close) {
		return -speed;
	} else if (close  &&  !open) {
		return speed;
	} else {
		return 0.0;
	}
}

class Button {
	enum STATE {
		UP,
		LONG_UP,
		DOWN,
		LONG_DOWN
	};


	static const double DEBOUNCE = 0.01;
	static const double HOLD = 0.08;

	bool pressed_1;
	double sinceLastTransition;

	STATE state/*, state_1*/;
	int transitions;

	bool dc;

public:
	Button() :
		pressed_1(false), sinceLastTransition(0.0), state(LONG_UP)/*, state_1(LONG_UP)*/, transitions(0), dc(false) {}

	bool update(bool pressed) {
		if (pressed != pressed_1) {
			sinceLastTransition = 0.0;
			pressed_1 = pressed;
		} else {
			sinceLastTransition += T_s;
		}

		if (sinceLastTransition >= DEBOUNCE) {
			switch (state) {
			case UP:
				if (pressed) {
					state = DOWN;
					++transitions;
				} else if (sinceLastTransition >= HOLD) {
					state = LONG_UP;
					transitions = 0;
				}
				break;

			case LONG_UP:
				if (pressed) {
					state = DOWN;
					++transitions;
				}
				break;

			case DOWN:
				if ( !pressed ) {
					state = UP;
					if (transitions != 0) {
						++transitions;
					}
				} else if (sinceLastTransition >= HOLD) {
					state = LONG_DOWN;
					transitions = 0;
				}
				break;

			case LONG_DOWN:
				if ( !pressed ) {
					state = UP;
				}

				break;
			}

//			if (state != state_1) {
//				printf("%d\n", state);
//				state_1 = state;
//			}
			if (transitions == 4) {
				dc = true;
				transitions = 0;
//				printf("double!\n");
			}
		}

		return state == DOWN  ||  state == LONG_DOWN;
	}

	bool doubleClick() {
		bool ret = dc;
		dc = false;
		return ret;
	}
};


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


	Button a,b,c,d;
	Hand::jp_type hjp;
	Hand::jv_type hjv, hjv_1(0.0);

	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
//		unsigned char data = ((pin_is_set(LP_PIN02) ? 0 : 1) << 0) +
//			   ((pin_is_set(LP_PIN03) ? 0 : 1) << 1) +
//			   ((pin_is_set(LP_PIN04) ? 0 : 1) << 2) +
//			   ((pin_is_set(LP_PIN05) ? 0 : 1) << 3) +
//			   ((pin_is_set(LP_PIN06) ? 0 : 1) << 4) +
//			   ((pin_is_set(LP_PIN07) ? 0 : 1) << 5) +
//			   ((pin_is_set(LP_PIN08) ? 0 : 1) << 6) +
//			   ((pin_is_set(LP_PIN09) ? 0 : 1) << 7);
//		printf("%x\n", data);

		hjv[0] = velCommand(a.update( !pin_is_set(LP_PIN09) ), d.update( !pin_is_set(LP_PIN06) ));
		hjv[1] = hjv[0];
		hjv[2] = hjv[0];
		hjv[3] = velCommand(b.update( !pin_is_set(LP_PIN08) ), c.update( !pin_is_set(LP_PIN07) ));

		if (c.doubleClick()) {
			hand.updatePosition();
			hjp = hand.getInnerLinkPosition();
			hjp[3] = M_PI_2;
			hand.trapezoidalMove(hjp);
		} else if (b.doubleClick()) {
			hand.updatePosition();
			hjp = hand.getInnerLinkPosition();
			hjp[3] = 0.0;
			hand.trapezoidalMove(hjp);
		} else if (d.doubleClick()) {
			hand.updatePosition();
			hjp = hand.getInnerLinkPosition();
			hjp[0] = hjp[1] = hjp[2] = 2 * M_PI;
			hand.trapezoidalMove(hjp);
		} else if (a.doubleClick()) {
			hand.updatePosition();
			hjp = hand.getInnerLinkPosition();
			hjp[0] = hjp[1] = hjp[2] = 0.0;
			hand.trapezoidalMove(hjp);
		} else if (hjv != hjv_1) {  // If things haven't changed, just wait and loop
			hand.setVelocity(hjv);
			std::cout << "Velocity: " << hjv << std::endl;
			hjv_1 = hjv;
		}

		usleep(USLEEP_TIME);
	}


	hand.idle();
	return 0;
}
