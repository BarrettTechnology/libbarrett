/*
 * gimbals_hand_controller.cpp
 *
 *  Created on: Oct 15, 2010
 *      Author: dc
 */

#include <cstdio>
#include <iostream>
#include <algorithm>
#include <string>

#include <unistd.h>

#include <barrett/products/product_manager.h>


using namespace barrett;


const int KNOB_MIN_VALID = 500;
const int KNOB_MAX_VALID = 2800;

const int SWITCH_THRESH1 = 1000;
const int SWITCH_THRESH2 = 2450;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

int main() {
	ProductManager pm;
	pm.enumerate();

	Puck& p6 = *pm.getPuck(6);
	Puck& p7 = *pm.getPuck(7);

	p6.wake();
	p7.wake();

	enum Puck::Property ana0, ana1;

	// TODO(dc): check lower version number with BZ
	if (p6.getVers() >= 155  || p6.getVers() <= 148) {
		ana0 = Puck::ANA0;
		ana1 = Puck::ANA1;
	} else {
		ana0 = Puck::ANA1;
		ana1 = Puck::ANA0;
	}


	int tmp;

	// wire colors as labeled on Gimbals documentation
	int brown, dGreen, orange, yellow, lGreen, blue, violet, white;

	// inputs, as the user views them
	bool thumbOpen, thumbClose, pointerOpen, pointerClose, middleOpen, middleClose, rockerUp, rockerDown;
	int knob = 0;  // CCW positive

	// knob state
	bool usingBrown = true;
	int brown_1, dGreen_1;


	int loopCount = 0;
	while (true) {
//		waitForEnter();
		usleep(10000);
		loopCount = (loopCount+1) % 25;


		// fetch raw data
		brown = p7.getProperty(ana0);
		dGreen = p7.getProperty(ana1);

		orange = p6.getProperty(ana0);
		yellow = p6.getProperty(ana1);

		tmp = p6.getProperty(Puck::HALLS);
		lGreen = tmp & 0x1;
		blue = tmp & 0x2;

		tmp = p7.getProperty(Puck::HALLS);
		violet = tmp & 0x1;
		white = tmp & 0x2;

//		printf("%d,\t%d,\t%d,\t%d,\t%d,\t%d,\t%d,\t%d\n", brown, dGreen, orange, yellow, lGreen, blue, violet, white);


		// convert to inputs
		thumbOpen = !blue;
		thumbClose = !lGreen;

		pointerOpen = !white;
		pointerClose = !violet;

		if (yellow > SWITCH_THRESH2) {
			middleClose = true;
			middleOpen = false;
		} else if (yellow > SWITCH_THRESH1) {
			middleClose = false;
			middleOpen = true;
		} else {
			middleClose = false;
			middleOpen = false;
		}

		if (orange > SWITCH_THRESH2) {
			rockerUp = true;
			rockerDown = false;
		} else if (orange > SWITCH_THRESH1) {
			rockerUp = false;
			rockerDown = true;
		} else {
			rockerUp = false;
			rockerDown = false;
		}

		tmp = usingBrown ? brown : dGreen;
		if (tmp < KNOB_MIN_VALID  ||  tmp > KNOB_MAX_VALID) {
			usingBrown = !usingBrown;
		}
		if (usingBrown) {
			knob -= brown - brown_1;
		} else {
			knob += dGreen - dGreen_1;
		}
		brown_1 = brown;
		dGreen_1 = dGreen;


		if (loopCount == 0) {
			printf("%d,%d  %d,%d  %d,%d  %d,%d  %d\n",
					thumbClose, thumbOpen,
					pointerClose, pointerOpen,
					middleClose, middleOpen,
					rockerDown, rockerUp,
					knob);
		}
	}
}
