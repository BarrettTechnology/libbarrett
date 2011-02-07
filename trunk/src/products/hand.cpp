/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <vector>
#include <algorithm>
#include <limits>

#include <syslog.h>
#include <unistd.h>

#include <barrett/detail/stl_utils.h>
#include <barrett/products/abstract/multi_puck_product.h>
#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/tactile_puck.h>
#include <barrett/products/hand.h>


namespace barrett {


const enum Puck::Property Hand::props[] = { Puck::HOLD, Puck::CMD, Puck::MODE, Puck::P, Puck::T, Puck::SG };


Hand::Hand(const std::vector<Puck*>& _pucks) :
	MultiPuckProduct(DOF, _pucks, PuckGroup::BGRP_HAND, props, sizeof(props)/sizeof(props[0]), "Hand::Hand()"),
	tactilePucks(), encoderTmp(DOF), primaryEncoder(DOF, 0), secondaryEncoder(DOF, 0), sg(DOF, 0)
{
	// Make TactilePucks
	int numTact = 0;
	for (size_t i = 0; i < DOF; ++i) {
		if (pucks[i]->hasOption(Puck::RO_Tact)) {
			++numTact;
			tactilePucks.push_back(new TactilePuck(pucks[i]));
		}
	}
	syslog(LOG_ERR, "  Found %d Tactile arrays", numTact);

	// record HOLD values
	group.getProperty(Puck::HOLD, holds);


	// For the fingers
	for (size_t i = 0; i < DOF - 1; ++i) {
		j2pp[i] = motorPucks[i].getCountsPerRad() * J2_RATIO;
		j2pt[i] = motorPucks[i].getIpnm() / J2_RATIO;
	}
	// For the spread
	j2pp[3] = motorPucks[3].getCountsPerRad() * SPREAD_RATIO;
	j2pt[3] = motorPucks[3].getIpnm() / SPREAD_RATIO;
}
Hand::~Hand()
{
	detail::purge(tactilePucks);
}

void Hand::initialize() const
{
	group.setProperty(Puck::CMD, CMD_HI);
	waitUntilDoneMoving();
}

bool Hand::doneMoving(bool realtime) const
{
	int modes[DOF];
	group.getProperty(Puck::MODE, modes, realtime);

	for (size_t i = 0; i < DOF; ++i) {
		if ((holds[i] != 0 && modes[i] != MotorPuck::MODE_PID)  ||  (holds[i] == 0 && modes[i] != MotorPuck::MODE_IDLE)) {
			return false;
		}
	}
	return true;
}
void Hand::waitUntilDoneMoving(int period_us) const
{
	while ( !doneMoving() ) {
		usleep(period_us);
	}
}

void Hand::trapezoidalMove(const jp_type& jp, bool blocking) const
{
	for (size_t i = 0; i < DOF; ++i) {
		pucks[i]->setProperty(Puck::E, j2pp[i] * jp[i]);
	}
	group.setProperty(Puck::MODE, MotorPuck::MODE_TRAPEZOIDAL);

	if (blocking) {
		waitUntilDoneMoving();
	}
}
void Hand::setVelocity(const jv_type& jv) const
{
	for (size_t i = 0; i < DOF; ++i) {
		// Convert to counts/millisecond
		pucks[i]->setProperty(Puck::V, (j2pp[i] * jv[i]) / 1000.0);
	}
	group.setProperty(Puck::MODE, MotorPuck::MODE_VELOCITY);
}

void Hand::setPositionCommand(const jp_type& jp) const
{
	for (size_t i = 0; i < DOF; ++i) {
		pucks[i]->setProperty(Puck::P, j2pp[i] * jp[i]);
	}
}
void Hand::setTorqueCommand(const jt_type& jt) const
{
	pt = j2pt.cwise() * jt;
	MotorPuck::sendPackedTorques(pucks[0]->getBus(), group.getId(), Puck::T, pt.data(), DOF);
}

void Hand::updatePosition(bool realtime)
{
	group.getProperty<MotorPuck::CombinedPositionParser<int> >(Puck::P, encoderTmp.data(), realtime);

	for (size_t i = 0; i < DOF; ++i) {
		primaryEncoder[i] = encoderTmp[i].get<0>();
		secondaryEncoder[i] = encoderTmp[i].get<1>();
	}

	// For the fingers
	for (size_t i = 0; i < DOF-1; ++i) {
		// If we got a reading from the secondary encoder...
		if (secondaryEncoder[i] != std::numeric_limits<int>::max()) {
			innerJp[i] = motorPucks[i].counts2rad(secondaryEncoder[i]) / J2_ENCODER_RATIO;
			outerJp[i] = motorPucks[i].counts2rad(primaryEncoder[i]) * (1.0/J2_RATIO + 1.0/J3_RATIO) - innerJp[i];
		} else {
			// These calculations are only valid before breakaway!
			innerJp[i] = motorPucks[i].counts2rad(primaryEncoder[i]) / J2_RATIO;
			outerJp[i] = innerJp[i] * J2_RATIO / J3_RATIO;
		}
	}

	// For the spread
	innerJp[3] = outerJp[3] = motorPucks[3].counts2rad(primaryEncoder[3]) / SPREAD_RATIO;
}
void Hand::updateStrain(bool realtime)
{
	group.getProperty(Puck::SG, sg.data(), realtime);
}
void Hand::updateTactFull(bool realtime)
{
	group.setProperty(Puck::TACT, TactilePuck::FULL_FORMAT);
	for (size_t i = 0; i < tactilePucks.size(); ++i) {
		tactilePucks[i]->receiveFull(realtime);
	}
}


}
