/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <vector>
#include <algorithm>

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
	MultiPuckProduct(DOF, _pucks, PuckGroup::BGRP_HAND, props, sizeof(props)/sizeof(props[0]), "Hand::Hand()"), tactilePucks(), sg(DOF, 0)
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
}
Hand::~Hand()
{
	detail::purge(tactilePucks);
}

void Hand::initialize(/*bool blocking*/) const
{
	group.setProperty(Puck::CMD, CMD_HI);

//	if (blocking) {
		int modes[DOF];
		std::vector<bool> seenMode4(DOF, false);
		std::vector<bool> seenNotMode4(DOF, false);

		while (true) {
			group.getProperty(Puck::MODE, modes);
			for (size_t i = 0; i < DOF; ++i) {
				if (seenMode4[i]) {
					if (modes[i] != MotorPuck::MODE_VELOCITY) {
						seenNotMode4[i] = true;
					}
				} else if (modes[i] == MotorPuck::MODE_VELOCITY) {
						seenMode4[i] = true;
				}
			}

			if (std::count(seenNotMode4.begin(), seenNotMode4.end(), true) == (int)DOF) {
				break;
			}
			usleep(100000);  // Poll at < 10Hz
		}
//	}
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

void Hand::trapezoidalMove(const jp_type& jpc, bool blocking) const
{
	for (size_t i = 0; i < DOF; ++i) {
		pucks[i]->setProperty(Puck::E, jpc[i]);
	}
	group.setProperty(Puck::MODE, MotorPuck::MODE_TRAPEZOIDAL);

	if (blocking) {
		waitUntilDoneMoving();
	}
}
void Hand::setVelocity(const jv_type& jv) const
{
	for (size_t i = 0; i < DOF; ++i) {
		pucks[i]->setProperty(Puck::V, jv[i]);
	}
	group.setProperty(Puck::MODE, MotorPuck::MODE_VELOCITY);
}

void Hand::setPositionCommand(const jp_type& jpc) const
{
	for (size_t i = 0; i < DOF; ++i) {
		pucks[i]->setProperty(Puck::P, jpc[i]);
	}
}
void Hand::setTorqueCommand(const jt_type& jt) const
{
	MotorPuck::sendPackedTorques(pucks[0]->getBus(), group.getId(), Puck::T, jt.data(), DOF);
}

void Hand::updatePosition(bool realtime)
{
	group.getProperty<MotorPuck::MotorPositionParser>(Puck::P, jp.data(), realtime);
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
