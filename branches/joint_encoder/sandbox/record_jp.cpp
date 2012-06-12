/*
 * record.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: dc
 */

#include <cstdio>

#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;



template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	typedef boost::tuple<double, jp_type> jp_sample_type;

	if (argc != 2) {
		printf("Usage:  %s <fileName>\n", argv[0]);
		exit(1);
	}

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	const double T_s = pm.getExecutionManager()->getPeriod();


	wam.gravityCompensate();
	pm.getSafetyModule()->setVelocityLimit(1.5);

	systems::Ramp time(pm.getExecutionManager());

	systems::TupleGrouper<double, jp_type> jpLogTg;

	// Record at the loop rate
	systems::PeriodicDataLogger<jp_sample_type> jpLogger(pm.getExecutionManager(),
			new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile, 1*T_s), 1);


	printf("Press [Enter] to start teaching.\n");
	waitForEnter();
	{
		// Make sure the Systems are connected on the same execution cycle
		// that the time is started. Otherwise we might record a bunch of
		// samples all having t=0; this is bad because the Spline requires time
		// to be monotonic.
		BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());

		connect(time.output, jpLogTg.template getInput<0>());
		connect(wam.jpOutput, jpLogTg.template getInput<1>());
		connect(jpLogTg.output, jpLogger.input);
		time.start();
	}

	printf("Press [Enter] to stop teaching.\n");
	waitForEnter();
	jpLogger.closeLog();
	disconnect(jpLogger.input);


	// Build spline between recorded points
	log::Reader<jp_sample_type> lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("File saved as: %s\n", argv[1]);
	std::remove(tmpFile);

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
