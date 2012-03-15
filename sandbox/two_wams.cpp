/*
 * two_wams.cpp
 *
 *  Created on: Feb 15, 2011
 *      Author: dc
 */

#include <iostream>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>


using namespace barrett;


boost::thread startWam(ProductManager& pm,
		boost::function<void (ProductManager&, systems::Wam<4>&)> wt4,
		boost::function<void (ProductManager&, systems::Wam<7>&)> wt7);
template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0);
template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1);


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

	ProductManager pm0;
	ProductManager pm1("/etc/barrett/bus1/default.conf");

	printf("Starting the WAM on Bus 0...\n");
	boost::thread wt0 = startWam(pm0, wamThread0<4>, wamThread0<7>);
	printf("Starting the WAM on Bus 1...\n");
	boost::thread wt1 = startWam(pm1, wamThread1<4>, wamThread1<7>);

	wt0.join();
	wt1.join();

	return 0;
}


boost::thread startWam(ProductManager& pm,
		boost::function<void (ProductManager&, systems::Wam<4>&)> wt4,
		boost::function<void (ProductManager&, systems::Wam<7>&)> wt7)
{
	pm.waitForWam();
	pm.wakeAllPucks();

	if (pm.foundWam4()) {
		return boost::thread(wt4, boost::ref(pm), boost::ref(*pm.getWam4()));
	} else {
		return boost::thread(wt7, boost::ref(pm), boost::ref(*pm.getWam7()));
	}
}


template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam0.gravityCompensate();

	jp_type jp(0.0);
	while (pm0.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		wam0.moveTo(jp);
		sleep(1);
		wam0.moveHome();
		sleep(1);
	}
}

template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1) {
	wamThread0(pm1, wam1);
}

