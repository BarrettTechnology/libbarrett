/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

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

/** Defines a boilerplate main() function that initializes a WAM.
 *
 * This "standard" main() function simply initializes the WAM in the normal way, by:
 *   - Waiting for the WAM to be Shift-idled, if it's not already
 *   - Waking all Pucks found on the bus
 *   - Waiting for the WAM to be Shift-activated
 *   - Calling the wam_main() function that the user is responsible for defining
 *   .
 * The behavior is appropriate for many applications, but there is no issue with writing a custom main() function.
 *
 * The wam_main() function must be able to accept a reference to both a 4-DOF and a 7-DOF barrett::systems::Wam. If your
 * code treats these two cases similarly, then define a templated function:
 * @include zero_torque.cpp
 *
 * If your code handles these two cases very differently, then define two overloaded functions:
 * @include seven_dof_only.cpp
 *
 * @file standard_main_function.h
 * @date Nov 5, 2010
 * @author Dan Cody
 */


#ifndef BARRETT_STANDARD_MAIN_FUNCTION_H_
#define BARRETT_STANDARD_MAIN_FUNCTION_H_


#include <barrett/exception.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/systems/wam.h>


#ifndef BARRETT_SMF_NO_DECLARE
	template<size_t DOF> int wam_main(int argc, char** argv, ::barrett::BusManager& bm, ::barrett::systems::Wam<DOF>& wam);
#endif

#ifndef BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#  define BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE true
#else
#  define BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE false
#endif

#ifndef BARRETT_SMF_WAM_CONFIG_PATH
#  define BARRETT_SMF_WAM_CONFIG_PATH NULL
#endif


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	::barrett::installExceptionHandler();

	::barrett::BusManager bm;
	bm.waitForWam();
	bm.wakeAllPucks();

	if (bm.foundWam4()) {
		return wam_main(argc, argv, bm, *bm.getWam4(BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE, BARRETT_SMF_WAM_CONFIG_PATH));
	} else if (bm.foundWam7()) {
		return wam_main(argc, argv, bm, *bm.getWam7(BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE, BARRETT_SMF_WAM_CONFIG_PATH));
	} else {
		printf(">>> ERROR: No WAM was found. Perhaps you have found a bug in BusManager::waitForWam().\n");
		return 1;
	}
}


#endif /* BARRETT_STANDARD_MAIN_FUNCTION_H_ */
