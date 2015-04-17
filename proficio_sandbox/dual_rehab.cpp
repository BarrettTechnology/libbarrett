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
		boost::function<void (ProductManager&, systems::Wam<3>&)> wt3);
template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0);
template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1);


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

	//ProductManager pm0("/etc/barrett/bus0/bus0.conf");
	ProductManager pm1("/etc/barrett/bus1/bus1.conf");

	//printf("Starting the WAM on Bus 0...\n");
	//boost::thread wt0 = startWam(pm0, wamThread0<3>);
	printf("Starting the WAM on Bus 1...\n");
	boost::thread wt1 = startWam(pm1, wamThread1<3>);

	//wt0.join();
	wt1.join();

	return 0;
}


boost::thread startWam(ProductManager& pm,
		boost::function<void (ProductManager&, systems::Wam<3>&)> wt3)
{
	pm.waitForWam();
	pm.wakeAllPucks();

	if (pm.foundWam3()) {
		return boost::thread(wt3, boost::ref(pm), boost::ref(*pm.getWam3()));
	}
}


template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam0.gravityCompensate();

	jp_type jp(0.0, 0.0, 1.57);
	wam0.moveTo(jp);
	wam0.idle();
	while (pm0.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		sleep(1);
		//wam0.moveHome();
		//sleep(1);
	}
}

template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam1.gravityCompensate();

	jp_type jp(0.0, 0.0, -1.57);
	wam1.moveTo(jp);
	wam1.idle();
	while (pm1.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		sleep(1);
		//wam1.moveHome();
		//sleep(1);
	}
}

