/*
 * multi_puck_product.cpp
 *
 *  Created on: Nov 11, 2010
 *      Author: dc
 */

#include <vector>
#include <stdexcept>

#include <syslog.h>

#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/abstract/multi_puck_product.h>


namespace barrett {


MultiPuckProduct::MultiPuckProduct(size_t DOF, const std::vector<Puck*>& _pucks, int groupId, const enum Puck::Property props[], size_t numProps, const char* syslogStr) :
	bus(_pucks.at(0)->getBus()), pucks(_pucks), motorPucks(DOF), group(groupId, pucks)
{
	if (syslogStr != NULL) {
		syslog(LOG_ERR, "%s", syslogStr);
	}

	// Check number of Pucks
	if (pucks.size() != DOF) {
		syslog(LOG_ERR, "  Expected a vector of %d Pucks, got %d", DOF, pucks.size());
		throw std::invalid_argument("MultiPuckProduct::MultiPuckProduct(): Wrong number of Pucks. Check /var/log/syslog for details.");
	}

	// Initialize MotorPucks
	Puck::wake(pucks);  // Make sure Pucks are awake
	for (size_t i = 0; i < DOF; ++i) {
		motorPucks[i].setPuck(pucks[i]);
	}

	// Verify properties
	bool err = false;
	for (size_t i = 0; i < numProps; ++i) {
		if ( !group.verifyProperty(props[i]) ) {
			err = true;
			syslog(LOG_ERR, "  Incompatible property: %s", Puck::getPropertyStr(props[i]));
		}
	}
	if (err) {
		syslog(LOG_ERR, "  Some Pucks might...");
		syslog(LOG_ERR, "    a) still be in Monitor");
		syslog(LOG_ERR, "    b) have incompatible firmware versions");
		syslog(LOG_ERR, "    c) have incompatible ROLEs");
		throw std::runtime_error("MultiPuckProduct::MultiPuckProduct(): Pucks have incompatible property lists. Check /var/log/syslog for details.");
	}
}
MultiPuckProduct::~MultiPuckProduct()
{
}


}
