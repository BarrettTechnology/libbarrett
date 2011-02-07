/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <iostream>
#include <unistd.h>
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

	Hand::jp_type jp;
	jp.setConstant(0.5);
	hand.trapezoidalMove(jp);

	hand.updatePosition();
	hand.updateStrain();
	std::cout << hand.getInnerLinkPosition() << hand.getOuterLinkPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";

	Hand::jv_type jv(0.5);
	hand.setVelocity(jv);

	sleep(1);
	hand.updatePosition();
	hand.updateStrain();
	std::cout << hand.getInnerLinkPosition() << hand.getOuterLinkPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";

	sleep(1);
	hand.updatePosition();
	hand.updateStrain();
	std::cout << hand.getInnerLinkPosition() << hand.getOuterLinkPosition() << " [" << hand.getStrain()[0] << "," << hand.getStrain()[1] << "," << hand.getStrain()[2] << "," << hand.getStrain()[3] << "]" << "\n";

	hand.idle();

	return 0;
}
