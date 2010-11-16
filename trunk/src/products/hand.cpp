/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <vector>

#include <syslog.h>

#include <barrett/detail/stl_utils.h>
#include <barrett/products/abstract/multi_puck_product.h>
#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/tactile_puck.h>
#include <barrett/products/hand.h>


namespace barrett {


const enum Puck::Property Hand::props[] = { Puck::P, Puck::T };


Hand::Hand(const std::vector<Puck*>& _pucks) :
	MultiPuckProduct(DOF, _pucks, PuckGroup::BGRP_HAND, props, sizeof(props)/sizeof(props[0]), "Hand::Hand()"), tactilePucks()
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
}
Hand::~Hand()
{
	detail::purge(tactilePucks);
}

void Hand::trapezoidalMove(const jp_type& jp) const
{
	for (size_t i = 0; i < DOF; ++i) {
		pucks[i]->setProperty(Puck::E, jp[i]);
	}
	group.setProperty(Puck::MODE, MotorPuck::MODE_TRAPEZOIDAL);
}
void Hand::setVelocity(const jv_type& jv) const
{
	for (size_t i = 0; i < DOF; ++i) {
		pucks[i]->setProperty(Puck::V, jv[i]);
	}
	group.setProperty(Puck::MODE, MotorPuck::MODE_VELOCITY);
}

void Hand::setPositionCommand(const jp_type& jp) const
{
	for (size_t i = 0; i < DOF; ++i) {
		pucks[i]->setProperty(Puck::P, jp[i]);
	}
}
void Hand::setTorqueCommand(const jt_type& jt) const
{
	MotorPuck::sendPackedTorques(pucks[0]->getBus(), group.getId(), Puck::T, jt.data(), DOF);
}

void Hand::updateTactFull(bool realtime) const {
	group.setProperty(Puck::TACT, TactilePuck::FULL_FORMAT);
	for (size_t i = 0; i < tactilePucks.size(); ++i) {
		tactilePucks[i]->receiveFull(realtime);
	}
}


}
