/*
	Copyright 2012 Barrett Technology <support@barrett.com>

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
 * read_pendant_state.cpp
 *
 *  Created on: Mar 26, 2012
 *      Author: dc
 */


#include <iostream>
#include <unistd.h>

#include <barrett/products/product_manager.h>


using namespace barrett;


int main() {
	ProductManager pm;
	SafetyModule& sm = *pm.getSafetyModule();
	SafetyModule::PendantState ps;

	// Optional: Instantiate a Wam object and start the realtime control loop
	if (pm.foundWam4()) {
		pm.getWam4(false);
	} else {
		pm.getWam7(false);
	}

	while (true) {
		sm.getPendantState(&ps);
		std::cout << ps.toString() << " " << ps.allSafe() << " " << ps.hasFaults() << "\n";
		usleep(1000000);
	}

	return 0;
}
