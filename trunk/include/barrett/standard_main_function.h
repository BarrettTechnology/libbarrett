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
	template<size_t DOF> int wam_main(::barrett::BusManager& bm, ::barrett::systems::Wam<DOF>& wam);
#endif


int main() {
	// Give us pretty stack-traces when things die
	::barrett::installExceptionHandler();

	::barrett::BusManager bm;
	bm.waitForWam();

	if (bm.wam4Found()) {
		return wam_main(bm, *bm.getWam4());
//	} else {
//		return wam_main(bm, bm.getWam7());
	}
}


#endif /* BARRETT_STANDARD_MAIN_FUNCTION_H_ */
