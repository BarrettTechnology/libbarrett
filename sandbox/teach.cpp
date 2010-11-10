/*
 * teach.cpp
 *
 *  Created on: Sep 29, 2009
 *      Author: dc
 */

#include <iostream>
#include <vector>
#include <string>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/products/safety_module.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;



template<size_t DOF>
int wam_main(int argc, char** argv, BusManager& bm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	typedef boost::tuple<double, jp_type> jp_sample_type;

	char tmpFile[L_tmpnam];
	if (std::tmpnam(tmpFile) == NULL) {
		printf("Couldn't create temporary file!\n");
		return 1;
	}

	const double T_s = bm.getExecutionManager()->getPeriod();


	wam.gravityCompensate();

	systems::Ramp time;
	time.setSamplePeriod(T_s);

	systems::TupleGrouper<double, jp_type> jpLogTg;

	// Record at 1/10th of the loop rate
	systems::PeriodicDataLogger<jp_sample_type> jpLogger(
			new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile, 10*T_s), 10);


	printf("Press [Enter] to start teaching.\n");
	waitForEnter();
	{
		// Make sure the Systems are connected on the same execution cycle
		// that the time is started. Otherwise we might record a bunch of
		// samples all having t=0; this is bad because the Spline requires time
		// to be monotonic.
		BARRETT_SCOPED_LOCK(bm.getExecutionManager()->getMutex());

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
	std::vector<jp_sample_type> vec;
	for (size_t i = 0; i < lr.numRecords(); ++i) {
		vec.push_back(lr.getRecord());
	}
	math::Spline<jp_type> spline(vec);

	printf("Press [Enter] to play back the recorded trajectory.\n");
	waitForEnter();

	// First, move to the starting position
	wam.moveTo(spline.eval(spline.initialX()));

	// Then play back the recorded motion
	time.stop();
	time.setOutput(spline.initialX());

	double (*saturatePtr)(double,double) = math::saturate;  // Cast needed because math::saturate is overloaded
	systems::Callback<double, jp_type> trajectory(boost::bind(boost::ref(spline), boost::bind(saturatePtr, _1, spline.finalX())));

	connect(time.output, trajectory.input);
	wam.trackReferenceSignal(trajectory.output);

	time.start();

	while (trajectory.input.getValue() < spline.finalX()) {
		usleep(100000);
	}


	printf("Press [Enter] to idle the WAM.\n");
	waitForEnter();
	wam.idle();


	std::remove(tmpFile);
	bm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	return 0;
}
