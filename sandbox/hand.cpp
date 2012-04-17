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


	{
		hand.close();
		hand.open();
		hand.close(Hand::SPREAD);
		hand.close(Hand::GRASP);
		hand.open(Hand::GRASP, false);
		sleep(1);
		hand.open();
	}

	{
		Hand::jp_type open(0.0);
		Hand::jp_type closed(2.4);
		closed[3] = M_PI;

		// Original interface preserved? Should move all 4 motors.
		hand.trapezoidalMove(closed);
		hand.trapezoidalMove(open, false);
		hand.waitUntilDoneMoving();

		// New interface
		hand.trapezoidalMove(closed, Hand::SPREAD);
		hand.trapezoidalMove(closed, Hand::F1);
		hand.trapezoidalMove(closed, Hand::F2);
		hand.trapezoidalMove(closed, Hand::F3);
		hand.trapezoidalMove(open, Hand::GRASP);
		hand.trapezoidalMove(open, Hand::SPREAD);
		hand.trapezoidalMove(closed, Hand::F3 | Hand::SPREAD);
		hand.trapezoidalMove(open, Hand::WHOLE_HAND);
	}

	{
		Hand::jv_type open(-0.5);
		Hand::jv_type close(0.5);

		// Original interface preserved? Should move all 4 motors.
		hand.velocityMove(close);
		sleep(1);

		// New interface
		hand.trapezoidalMove(open, Hand::GRASP);
		sleep(1);
		hand.trapezoidalMove(open, Hand::WHOLE_HAND);
		hand.waitUntilDoneMoving();
	}


	hand.idle();
	return 0;
}
