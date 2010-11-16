/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <iostream>

#include <unistd.h>

#include <boost/thread.hpp>

#include <barrett/bus/bus_manager.h>
#include <barrett/products/hand.h>


using namespace barrett;


void moveSpread(Puck* spread) {
	int pos = 0;
	while (true) {
		sleep(3);

		pos = pos ? 0:15000;
		spread->setProperty(Puck::E, pos);
		spread->setProperty(Puck::MODE, 5);
	}
}

int main() {
	BusManager bm;
	if ( !bm.foundHand() ) {
		printf("ERROR: No Hand found on bus!\n");
		return 1;
	}
	Hand& hand = *bm.getHand();

//	Puck* spread = hand.getPucks()[3];
//	boost::thread t(moveSpread, spread);

	while (true) {
		printf("Update:\n");
		hand.updateTactFull();
		for (size_t i = 0; i < hand.getTactilePucks().size(); ++i) {
			std::cout << hand.getTactilePucks()[i]->getFullData() << "\n";
		}
	}

	return 0;
}
