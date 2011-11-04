/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <iostream>
#include <unistd.h>
#include <native/timer.h>

#include <barrett/products/product_manager.h>


using namespace barrett;


int main() {
	ProductManager pm;
	if ( !pm.foundHand() ) {
		printf("ERROR: No Hand found on bus!\n");
		return 1;
	}
	Hand& hand = *pm.getHand();


	hand.initialize();
	RTIME start, end;
	const size_t COUNT = 1000;


#define PRE \
	start = rt_timer_read(); \
	for (size_t i = 0; i < COUNT; ++i) {

#define POST \
	} \
	end = rt_timer_read(); \
	std::cout << (end - start) * 1e-6 / COUNT << "\n";


	PRE
	hand.updatePosition(true);
	hand.updateStrain(true);
	hand.updateTactFull(true);
	POST

	PRE
	hand.updatePosition();
	hand.updateStrain();
	hand.updateTactFull();
	POST

	PRE
	hand.update(Hand::S_ALL, true);
	POST

	PRE
	hand.update();
	POST


	return 0;

	Hand::jp_type jp;
	jp.setConstant(0.5);
	hand.trapezoidalMove(jp);

	hand.updatePosition(true);
	hand.updateStrain(true);
	//hand.update(Hand::S_POSITION | Hand::S_FINGER_TIP_TORQUE, true);
	std::cout << hand.getInnerLinkPosition() << hand.getOuterLinkPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";

	Hand::jv_type jv(0.5);
	hand.setVelocity(jv);

	sleep(1);
	hand.updatePosition(true);
	hand.updateStrain(true);
	//hand.update(Hand::S_POSITION | Hand::S_FINGER_TIP_TORQUE, true);
	std::cout << hand.getInnerLinkPosition() << hand.getOuterLinkPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";

	sleep(1);
	hand.updatePosition(true);
	hand.updateStrain(true);
	//hand.update(Hand::S_POSITION | Hand::S_FINGER_TIP_TORQUE, true);
	std::cout << hand.getInnerLinkPosition() << hand.getOuterLinkPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";

	hand.idle();

	return 0;
}
