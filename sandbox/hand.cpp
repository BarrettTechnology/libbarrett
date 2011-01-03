/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <iostream>

#include <unistd.h>

#include <boost/thread.hpp>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/products/product_manager.h>


using namespace barrett;
using detail::waitForEnter;


void moveSpread(Puck* spread) {
	int pos = 0;
	while (true) {
		sleep(3);

		pos = pos ? 0:15000;
		spread->setProperty(Puck::E, pos);
		spread->setProperty(Puck::MODE, MotorPuck::MODE_TRAPEZOIDAL);
	}
}

int main() {
	installExceptionHandler();


	ProductManager pm;
	if ( !pm.foundHand() ) {
		printf("ERROR: No Hand found on bus!\n");
		return 1;
	}
	Hand& hand = *pm.getHand();


	hand.initialize();

	Hand::jp_type jp;
	jp << 30000, 30000, 30000, 10000;
	hand.trapezoidalMove(jp);

	hand.updatePosition();
	hand.updateStrain();
	std::cout << hand.getPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";

	Hand::jv_type jv(50.0);
	hand.setVelocity(jv);

	sleep(1);
	hand.updatePosition();
	hand.updateStrain();
	std::cout << hand.getPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";

	sleep(1);
	hand.updatePosition();
	hand.updateStrain();
	std::cout << hand.getPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";

	hand.idle();


//	Puck* spread = hand.getPucks()[3];
//	boost::thread t(moveSpread, spread);


//	printf("Press [Enter] to stream TACT data.");
//	waitForEnter();
//	while (true) {
//		printf("Update:\n");
//		hand.updateTactFull();
//		for (size_t i = 0; i < hand.getTactilePucks().size(); ++i) {
//			std::cout << hand.getTactilePucks()[i]->getFullData() << "\n";
//		}
//	}

	return 0;
}
