/*
	Copyright 2010, 2011, 2012 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * hand.h
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_HAND_H_
#define BARRETT_PRODUCTS_HAND_H_


#include <vector>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/products/abstract/multi_puck_product.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/tactile_puck.h>


namespace barrett {


class Hand : public MultiPuckProduct {
public:
	static const size_t DOF = 4;
	BARRETT_UNITS_TYPEDEFS(DOF);


	// Constants for referring to specific axes of the Hand
	static const unsigned int F1         = 1 << 0;
	static const unsigned int F2         = 1 << 1;
	static const unsigned int F3         = 1 << 2;
	static const unsigned int SPREAD     = 1 << 3;
	static const unsigned int GRASP      = F1 | F2 | F3;
	static const unsigned int WHOLE_HAND = GRASP | SPREAD;


	Hand(const std::vector<Puck*>& pucks);
	~Hand();

	void initialize() const;
	void idle() const { group.setProperty(Puck::MODE, MotorPuck::MODE_IDLE); }

	bool doneMoving(unsigned int whichDigits = WHOLE_HAND, bool realtime = false) const;
	void waitUntilDoneMoving(unsigned int whichDigits = WHOLE_HAND, double period_s = 0.1) const;


	// Basic moves
	void open(unsigned int whichDigits = WHOLE_HAND, bool blocking = true) const;
	void open(bool blocking) const { open(WHOLE_HAND, blocking); }
	void close(unsigned int whichDigits = WHOLE_HAND, bool blocking = true) const;
	void close(bool blocking) const { close(WHOLE_HAND, blocking); }

	// Preferred: low control-rate moves
	void trapezoidalMove(const jp_type& jp, unsigned int whichDigits = WHOLE_HAND, bool blocking = true) const;
	void trapezoidalMove(const jp_type& jp, bool blocking) const { trapezoidalMove(jp, WHOLE_HAND, blocking); }
	void velocityMove(const jv_type& jv, unsigned int whichDigits = WHOLE_HAND) const;

	// Advanced: high control-rate moves
	void setPositionMode(unsigned int whichDigits = WHOLE_HAND) const;
	void setPositionCommand(const jp_type& jp, unsigned int whichDigits = WHOLE_HAND) const;
	void setTorqueMode(unsigned int whichDigits = WHOLE_HAND) const;
	void setTorqueCommand(const jt_type& jt, unsigned int whichDigits = WHOLE_HAND) const;


	// Sensors
	static const unsigned int S_POSITION          = 1 << 0;
	static const unsigned int S_FINGERTIP_TORQUE = 1 << 1;
	static const unsigned int S_TACT_FULL         = 1 << 2;
	static const unsigned int S_ALL = S_POSITION | S_FINGERTIP_TORQUE | S_TACT_FULL;
	void update(unsigned int sensors = S_ALL, bool realtime = false);

	const jp_type& getInnerLinkPosition() const { return innerJp; }
	const jp_type& getOuterLinkPosition() const { return outerJp; }
	const std::vector<int>& getPrimaryEncoderPosition() const { return primaryEncoder; }
	const std::vector<int>& getSecondaryEncoderPosition() const { return secondaryEncoder; }
	void enableBreakawayEncoders(bool enable) { useSecondaryEncoders = enable; }  // Enabled by default.

	bool hasFingertipTorqueSensors() const { return hasFtt; }
	const std::vector<int>& getFingertipTorque() const { return ftt; }

	bool hasTactSensors() const { return hasTact; }
	const std::vector<TactilePuck*>& getTactilePucks() const { return tactilePucks; }


	static const size_t SPREAD_INDEX = 3;

protected:
	bool digitsInclude(unsigned int whichDigits, size_t index) const { return whichDigits & (1 << index); }
	void setProperty(unsigned int whichDigits, enum Puck::Property prop, int value) const;
	void setProperty(unsigned int whichDigits, enum Puck::Property prop, const v_type& values) const;
	void blockIf(bool blocking, unsigned int whichDigits) const;


	static const double J2_RATIO = 125.0;
	static const double J2_ENCODER_RATIO = 50.0;
	static const double J3_RATIO = 375.0;
	static const double SPREAD_RATIO = 17.5;


	bool hasFtt;
	bool hasTact;
	bool useSecondaryEncoders;

	int holds[DOF];
	v_type j2pp, j2pt;
	mutable v_type pt;

	std::vector<MotorPuck::CombinedPositionParser<int>::result_type> encoderTmp;
	std::vector<int> primaryEncoder, secondaryEncoder;
	jp_type innerJp, outerJp;
	std::vector<int> ftt;
	std::vector<TactilePuck*> tactilePucks;

private:
	static const int CMD_HI    = 13;
	static const int CMD_CLOSE = 18;
	static const int CMD_OPEN  = 20;
	static const enum Puck::Property props[];

	DISALLOW_COPY_AND_ASSIGN(Hand);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}


#endif /* BARRETT_PRODUCTS_HAND_H_ */
