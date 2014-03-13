/*
 * @file dual_rehab_wams.cpp
 *
 *  @date March 12, 2014
 *  @author jh
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
template <size_t DOF> void wamThreadR(ProductManager& pmR, systems::Wam<DOF>& wamR);
template <size_t DOF> void wamThreadL(ProductManager& pmL, systems::Wam<DOF>& wamL);


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

  // Define where the non-standard configuration files are located.
	ProductManager pmR("/etc/barrett/bus0/bus0.conf");
	ProductManager pmL("/etc/barrett/bus1/bus1.conf");

  // Start up each individual arm
	printf("Starting the Right Rehab Arm ...\n");
	boost::thread wtr = startWam(pmR, wamThreadR<3>);
	printf("Starting the Left Rehab Arm ...\n");
	boost::thread wtl = startWam(pmL, wamThreadL<3>);

	wtr.join();
	wtl.join();

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


template <size_t DOF> void wamThreadR(ProductManager& pmR, systems::Wam<DOF>& wamR) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wamR.gravityCompensate();

	jp_type rsPos(0.0, 0.0, 1.57);
	wamR.moveTo(rsPos);
  printf("Hit Enter when Finished...\n");
  // Change from exit upon shift-idle to move home upon enter or q
	while (pmR.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
    // Add display thread here?		
    wamR.idle();
	}
  wamR.moveHome();
}

template <size_t DOF> void wamThreadL(ProductManager& pmL, systems::Wam<DOF>& wamL) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wamL.gravityCompensate();

	jp_type lsPos(0.0, 0.0, -1.57); // elbow did not open up at this point.
	wamL.moveTo(lsPos);
  printf("Hit Enter when Finished...\n");
  // Change from exit upon shift-idle to move home upon enter or q
  while (pmL.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
    // Add display Thread here?		
    wamL.idle();
	}
  wamL.moveHome();
}

