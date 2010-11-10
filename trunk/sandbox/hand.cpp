/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <iostream>

#include <barrett/bus/bus_manager.h>
#include <barrett/products/hand.h>


using namespace barrett;


int main() {
	BusManager bm;
	if ( !bm.foundHand() ) {
		printf("ERROR: No Hand found on bus!\n");
		return 1;
	}

	Hand& hand = *bm.getHand();
}
