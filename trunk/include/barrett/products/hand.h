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


	Hand(const std::vector<Puck*>& pucks);
	~Hand();

	void initialize() const;
	void idle() const { group.setProperty(Puck::MODE, MotorPuck::MODE_IDLE); }

	bool doneMoving(bool realtime = false) const;
	void waitUntilDoneMoving(int period_us = 100000) const;

	// preferred: low control-rate moves
	void trapezoidalMove(const jp_type& jp, bool blocking = true) const;
	void setVelocity(const jv_type& jv) const;

	// advanced: high control-rate moves
	void setPositionMode() const { group.setProperty(Puck::MODE, MotorPuck::MODE_PID); }
	void setPositionCommand(const jp_type& jp) const;
	void setTorqueMode() const { group.setProperty(Puck::MODE, MotorPuck::MODE_TORQUE); }
	void setTorqueCommand(const jt_type& jt) const;


	static const unsigned int S_POSITION          = 1 << 0;
	static const unsigned int S_FINGER_TIP_TORQUE = 1 << 1;
	static const unsigned int S_TACT_FULL         = 1 << 2;
	static const unsigned int S_ALL = S_POSITION | S_FINGER_TIP_TORQUE | S_TACT_FULL;
	void update(unsigned int sensors = S_ALL, bool realtime = false);

	void updatePosition(bool realtime = false);
	const jp_type& getInnerLinkPosition() const { return innerJp; }
	const jp_type& getOuterLinkPosition() const { return outerJp; }
	const std::vector<int>& getPrimaryEncoderPosition() const { return primaryEncoder; }
	const std::vector<int>& getSecondaryEncoderPosition() const { return secondaryEncoder; }
	void enableBreakawayEncoders(bool enable) { useSecondaryEncoders = enable; }

	bool hasStrainSensors() const { return hasSg; }
	void updateStrain(bool realtime = false);
	const std::vector<int>& getStrain() const { return sg; }

	bool hasTactSensors() const { return hasTact; }
	void updateTactFull(bool realtime = false);
	const std::vector<TactilePuck*>& getTactilePucks() const { return tactilePucks; }


	static const size_t SPREAD_INDEX = 3;

protected:
	static const double J2_RATIO = 125.0;
	static const double J2_ENCODER_RATIO = 50.0;
	static const double J3_RATIO = 375.0;
	static const double SPREAD_RATIO = 17.5;


	bool hasSg;
	bool hasTact;
	bool useSecondaryEncoders;

	int holds[DOF];
	v_type j2pp, j2pt;
	mutable v_type pt;

	std::vector<MotorPuck::CombinedPositionParser<int>::result_type> encoderTmp;
	std::vector<int> primaryEncoder, secondaryEncoder;
	jp_type innerJp, outerJp;
	std::vector<int> sg;
	std::vector<TactilePuck*> tactilePucks;

private:
	static const int CMD_HI = 13;
	static const enum Puck::Property props[];

	DISALLOW_COPY_AND_ASSIGN(Hand);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}


#endif /* BARRETT_PRODUCTS_HAND_H_ */
