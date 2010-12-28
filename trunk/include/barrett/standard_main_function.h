/*
 * standard_main_function.h
 *
 *  Created on: Nov 5, 2010
 *      Author: dc
 */

#ifndef BARRETT_STANDARD_MAIN_FUNCTION_H_
#define BARRETT_STANDARD_MAIN_FUNCTION_H_


#include <barrett/exception.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/systems/wam.h>


#ifndef BARRETT_SMF_NO_DECLARE
	template<size_t DOF> int wam_main(int argc, char** argv, ::barrett::BusManager& bm, ::barrett::systems::Wam<DOF>& wam);
#endif

#ifndef BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE
#  define BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE true
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
