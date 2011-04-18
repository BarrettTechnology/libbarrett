/*
 * gimbals_hand_controller.cpp
 *
 *  Created on: Jan 4, 2011
 *      Author: dc
 */

#include <barrett/products/puck.h>
#include <barrett/products/gimbals_hand_controller.h>


namespace barrett {


GimbalsHandController::GimbalsHandController(Puck* _p6, Puck* _p7) :
	p6(*_p6), p7(*_p7),
	thumbOpen(false), thumbClose(false),
	pointerOpen(false), pointerClose(false),
	middleOpen(false), middleClose(false),
	rockerUp(false), rockerDown(false),
	knob(0), usingBrown(true), brown_1(0), dGreen_1(0)

{
	p6.wake();
	p7.wake();

	// TODO(dc): check lower version number with BZ
	if (p6.getVers() >= 155  || p6.getVers() <= 148) {
		ana0 = Puck::ANA0;
		ana1 = Puck::ANA1;
	} else {
		ana0 = Puck::ANA1;
		ana1 = Puck::ANA0;
	}
}

void GimbalsHandController::update()
{
	int tmp;

	// wire colors as labeled on Gimbals documentation
	int brown, dGreen, orange, yellow, lGreen, blue, violet, white;


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

//	printf("%d,\t%d,\t%d,\t%d,\t%d,\t%d,\t%d,\t%d\n", brown, dGreen, orange, yellow, lGreen, blue, violet, white);


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

//	printf("%d,%d  %d,%d  %d,%d  %d,%d  %d\n",
//			thumbClose, thumbOpen,
//			pointerClose, pointerOpen,
//			middleClose, middleOpen,
//			rockerDown, rockerUp,
//			knob);
}


}
