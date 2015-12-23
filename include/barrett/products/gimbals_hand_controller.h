/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 */
/*
 * @file gimbals_hand_controller.h
 * @date 1/4/2011
 * @author Dan Cody
 * 
 */

#ifndef BARRETT_PRODUCTS_GIMBALS_HAND_CONTROLLER_H_
#define BARRETT_PRODUCTS_GIMBALS_HAND_CONTROLLER_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/products/puck.h>


namespace barrett {


class GimbalsHandController {
public:
	/** GimbalsHandController Constructor */
	GimbalsHandController(Puck* p6, Puck* p7);
	/** GimbalsHandController Destructor */
	~GimbalsHandController() {}
	/** update Method gets the latest status of the pointer, thumb, rocker, middle fingers */
	void update();
	/** getThumbOpen Method tells whether or not the thumb is open currently */
	bool getThumbOpen() const { return thumbOpen; }
	/** getThumbClose Method tells whether or not the thumb is closed currently */
	bool getThumbClose() const { return thumbClose; }
	/** getPointerOpen Method tells whether or not the pointer is open currently */
	bool getPointerOpen() const { return pointerOpen; }
	/** getPointerClose Method tells whether or not the pointer is closed currently */
	bool getPointerClose() const { return pointerClose; }
	/** getMiddleOpen method tells whether or not the middle is open currently */
	bool getMiddleOpen() const { return middleOpen; }
	/** getMiddleClose Method tells whether or not the middle is closed currently */
	bool getMiddleClose() const { return middleClose; }
	/** getRockerUp Method tells whether or not the rocker is up currently */
	bool getRockerUp() const { return rockerUp; }
	/** getRockerDown Method tells whether or not the rocker is down currently */
	bool getRockerDown() const { return rockerDown; }
	/** getKnob Method returns the current value of the knob position */
	double getKnob() const { return knob / COUNTS_PER_RAD; }
	/** setKnob Method changes the current knob value to desired value */
	void setKnob(double value) { knob = value * COUNTS_PER_RAD; }

protected:
	static const int SWITCH_THRESH1 = 1000;
	static const int SWITCH_THRESH2 = 2450;

	static const int KNOB_MIN_VALID = 500;
	static const int KNOB_MAX_VALID = 2800;
	static const double COUNTS_PER_RAD = 4096.0*M_1_PI/2.0;


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
