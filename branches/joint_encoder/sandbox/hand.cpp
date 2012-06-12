/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <iostream>

#include <barrett/os.h>
#include <barrett/math.h>
#include <barrett/products/product_manager.h>


using namespace barrett;


void assertPosition(Hand& hand, Hand::jp_type innerLinkJp, double tolerance = 0.05) {
	hand.update();
	if (math::abs(hand.getInnerLinkPosition() - innerLinkJp).maxCoeff() > tolerance) {
		std::cout << hand.getInnerLinkPosition() << " is not close enough to " << innerLinkJp << "\n";
		exit(2);
	}
}


int main() {
	typedef Hand::jp_type hjp_t;

	ProductManager pm;
	if ( !pm.foundHand() ) {
		printf("ERROR: No Hand found on bus!\n");
		return 1;
	}
	Hand& hand = *pm.getHand();
	hand.initialize();


	double O = 0.0;
	double C = 2.4;
	double SC = M_PI;
	hjp_t open(O);
	hjp_t closed(C);
	closed[3] = SC;

	double OR = -0.75;
	double CR = 0.75;
	Hand::jv_type opening(OR);
	Hand::jv_type closing(CR);



	{
		assertPosition(hand, open);
		hand.close();
		assertPosition(hand, closed);
		hand.open();
		assertPosition(hand, open);
		hand.close(Hand::SPREAD);
		assertPosition(hand, hjp_t(O,O,O,SC));
		hand.close(Hand::GRASP);
		assertPosition(hand, closed);
		hand.open(Hand::GRASP, false);
		btsleep(0.5);
		assertPosition(hand, hjp_t(1.6,1.6,1.6,SC));
		hand.open();
		assertPosition(hand, open);
	}

	{
		// Original interface preserved? Should move all 4 motors.
		hand.trapezoidalMove(closed);
		assertPosition(hand, closed);
		hand.trapezoidalMove(open, false);
		hand.waitUntilDoneMoving();
		assertPosition(hand, open);

		// New interface
		hand.trapezoidalMove(closed, Hand::SPREAD);
		assertPosition(hand, hjp_t(O,O,O,SC));
		hand.trapezoidalMove(closed, Hand::F1);
		assertPosition(hand, hjp_t(C,O,O,SC));
		hand.trapezoidalMove(closed, Hand::F2);
		assertPosition(hand, hjp_t(C,C,O,SC));
		hand.trapezoidalMove(closed, Hand::F3);
		assertPosition(hand, closed);
		hand.trapezoidalMove(open, Hand::GRASP);
		assertPosition(hand, hjp_t(O,O,O,SC));
		hand.trapezoidalMove(open, Hand::SPREAD);
		assertPosition(hand, open);
		hand.trapezoidalMove(closed, Hand::F3 | Hand::SPREAD);
		assertPosition(hand, hjp_t(O,O,C,SC));
		hand.trapezoidalMove(open, Hand::WHOLE_HAND);
		assertPosition(hand, open);
	}

	{
		double t = 0.0;

		// Original interface preserved? Should move all 4 motors.
		hand.velocityMove(closing);
		btsleep(1);
		t = 1.0;
		assertPosition(hand, hjp_t(CR*t), 0.2);

		// New interface
		hand.velocityMove(opening, Hand::GRASP);
		btsleep(1);
		t = 2.0;
		assertPosition(hand, hjp_t(O,O,O,CR*t), 0.4);
		hand.velocityMove(opening, Hand::WHOLE_HAND);
		hand.waitUntilDoneMoving();
		assertPosition(hand, open);
	}

	{
		double t = 0.0;

		// Original interface preserved? Should move all 4 motors.
		hand.velocityMove(closing);
		btsleep(1);
		t = 1.0;
		assertPosition(hand, hjp_t(CR*t), 0.2);

		// SPREAD should continue its velocity move
		hand.trapezoidalMove(open, Hand::F1);  // Only blocks for F1
		t = 1.6;
		assertPosition(hand, hjp_t(O,CR*t,CR*t,CR*t), 0.4);
		hand.open(Hand::GRASP);  // Only blocks for GRASP
		t = 2.4;
		assertPosition(hand, hjp_t(O,O,O,CR*t), 0.4);
		btsleep(1);
		t = 3.4;
		assertPosition(hand, hjp_t(O,O,O,CR*t), 0.6);
		hand.open();
		assertPosition(hand, open);
	}


	hand.idle();
	return 0;
}
