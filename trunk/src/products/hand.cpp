/*
 * hand.cpp
 *
 *  Created on: Nov 9, 2010
 *      Author: dc
 */

#include <vector>

#include <syslog.h>

#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/puck_group.h>
#include <barrett/products/hand.h>


namespace barrett {


Hand::Hand(const std::vector<Puck*>& genericPucks) :
		pucks(DOF), handGroup(PuckGroup::BGRP_HAND, genericPucks)
{
	syslog(LOG_ERR, "Hand::Hand()");

	// Check number of Pucks
	if (genericPucks.size() != DOF) {
		syslog(LOG_ERR, "  Expected a vector of %d Pucks, got %d", DOF, pucks.size());
		throw std::invalid_argument("Hand::Hand(): Wrong number of Pucks. Check /var/log/syslog for details.");
	}

	// Initialize MotorPucks
	Puck::wake(genericPucks);  // Make sure Pucks are awake
	for (size_t i = 0; i < DOF; ++i) {
		pucks[i].setPuck(genericPucks[i]);
	}

	// Verify properties
	bool err = false;
	std::vector<enum Puck::Property> props;
	props.push_back(Puck::P);
	props.push_back(Puck::T);
	for (size_t i = 0; i < props.size(); ++i) {
		if ( !handGroup.verifyProperty(props[i]) ) {
			err = true;
			syslog(LOG_ERR, "  Incompatible property: %s", Puck::getPropertyStr(props[i]));
		}
	}
	if (err) {
		syslog(LOG_ERR, "  Some Pucks might...");
		syslog(LOG_ERR, "    a) still be in Monitor");
		syslog(LOG_ERR, "    b) have incompatible firmware versions");
		syslog(LOG_ERR, "    c) have incompatible ROLEs");
		throw std::runtime_error("Hand::Hand(): Hand Pucks have incompatible property lists. Check /var/log/syslog for details.");
	}

}
Hand::~Hand()
{
}


}
