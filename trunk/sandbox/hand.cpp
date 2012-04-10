/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <iostream>
#include <native/timer.h>

#include <barrett/os.h>
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
		btsleep(0.5);
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
		Hand::jv_type open(-0.75);
		Hand::jv_type close(0.75);

		// Original interface preserved? Should move all 4 motors.
		hand.velocityMove(close);
		btsleep(1);

		// New interface
		hand.velocityMove(open, Hand::GRASP);
		btsleep(1);
		hand.velocityMove(open, Hand::WHOLE_HAND);
		hand.waitUntilDoneMoving();
	}

	{
		Hand::jp_type open(0.0);
		Hand::jv_type close(0.75);

		// Original interface preserved? Should move all 4 motors.
		hand.velocityMove(close);
		btsleep(1);

		// SPREAD should continue its velocity move
		hand.trapezoidalMove(open, Hand::F1);  // Only blocks for F1
		hand.open(Hand::GRASP);  // Only blocks for GRASP
		btsleep(1);
		hand.open();
	}


	hand.idle();
	return 0;
}
