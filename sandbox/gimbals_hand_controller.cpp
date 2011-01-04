/*
 * gimbals_hand_controller.cpp
 *
 *  Created on: Oct 15, 2010
 *      Author: dc
 */

#include <cstdio>
#include <unistd.h>

#include <barrett/products/product_manager.h>


using namespace barrett;


int main() {
	ProductManager pm;
	if ( !pm.foundGimbalsHandController() ) {
		printf("ERROR: No Gimbals Hand Controller found!\n");
		return 1;
	}
	GimbalsHandController& ghc = *pm.getGimbalsHandController();


	int loopCount = 0;
	while (true) {
		usleep(10000);

		ghc.update();

		loopCount = (loopCount+1) % 25;
		if (loopCount == 0) {
			printf("%d,%d  %d,%d  %d,%d  %d,%d  %f\n",
					ghc.getThumbOpen(), ghc.getThumbClose(),
					ghc.getPointerOpen(), ghc.getPointerClose(),
					ghc.getMiddleOpen(), ghc.getMiddleClose(),
					ghc.getRockerUp(), ghc.getRockerDown(),
					ghc.getKnob());
		}
	}
}
