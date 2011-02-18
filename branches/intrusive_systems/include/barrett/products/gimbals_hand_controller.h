/*
 * gimbals_hand_controller.h
 *
 *  Created on: Jan 4, 2011
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_GIMBALS_HAND_CONTROLLER_H_
#define BARRETT_PRODUCTS_GIMBALS_HAND_CONTROLLER_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/products/puck.h>


namespace barrett {


class GimbalsHandController {
public:
	GimbalsHandController(Puck* p6, Puck* p7);
	~GimbalsHandController() {}

	void update();

	bool getThumbOpen() const { return thumbOpen; }
	bool getThumbClose() const { return thumbClose; }

	bool getPointerOpen() const { return pointerOpen; }
	bool getPointerClose() const { return pointerClose; }

	bool getMiddleOpen() const { return middleOpen; }
	bool getMiddleClose() const { return middleClose; }

	bool getRockerUp() const { return rockerUp; }
	bool getRockerDown() const { return rockerDown; }

	double getKnob() const { return knob / COUNTS_PER_RAD; }
	void setKnob(double value) { knob = value * COUNTS_PER_RAD; }

protected:
	static const int SWITCH_THRESH1 = 1000;
	static const int SWITCH_THRESH2 = 2450;

	static const int KNOB_MIN_VALID = 500;
	static const int KNOB_MAX_VALID = 2800;
	static const double COUNTS_PER_RAD = 4096.0 / (2*M_PI);


	Puck& p6;
	Puck& p7;

	enum Puck::Property ana0, ana1;

	bool thumbOpen, thumbClose, pointerOpen, pointerClose, middleOpen, middleClose, rockerUp, rockerDown;
	int knob;  // CCW positive

	// knob state
	bool usingBrown;
	int brown_1, dGreen_1;

private:
	DISALLOW_COPY_AND_ASSIGN(GimbalsHandController);
};


}


#endif /* BARRETT_PRODUCTS_GIMBALS_HAND_CONTROLLER_H_ */
