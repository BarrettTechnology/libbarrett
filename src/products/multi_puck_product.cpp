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
 * multi_puck_product.cpp
 *
 *  Created on: Nov 11, 2010
 *      Author: dc
 */

#include <vector>
#include <stdexcept>

#include <barrett/os.h>
#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/abstract/multi_puck_product.h>


namespace barrett {


MultiPuckProduct::MultiPuckProduct(size_t DOF, const std::vector<Puck*>& _pucks, int groupId, const enum Puck::Property props[], size_t numProps, const char* syslogStr) :
	bus(_pucks.at(0)->getBus()), pucks(_pucks), motorPucks(DOF), group(groupId, pucks)
{
	if (syslogStr != NULL) {
		logMessage("%s") % syslogStr;
	}

	// Check number of Pucks
	if (pucks.size() != DOF) {
		(logMessage("MultiPuckProduct::MultiPuckProduct(): Wrong number of Pucks. "
				"Expected a vector of %d Pucks, got %d.")
				% DOF % pucks.size()).raise<std::invalid_argument>();
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
			logMessage("  Incompatible property: %s") % Puck::getPropertyStr(props[i]);
		}
	}
	if (err) {
		logMessage("  Some Pucks might...");
		logMessage("    a) still be in Monitor");
		logMessage("    b) have incompatible firmware versions");
		logMessage("    c) have incompatible ROLEs");
		logMessage("MultiPuckProduct::MultiPuckProduct(): Pucks have incompatible property lists. "
				"Check /var/log/syslog for details.").raise<std::runtime_error>();
	}
}
MultiPuckProduct::~MultiPuckProduct()
{
}


}
