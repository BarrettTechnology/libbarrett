/*
 * teach.cpp
 *
 *  Created on: Sep 29, 2009
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <unistd.h>  // usleep

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/exception.h>
#include <barrett/detail/debug.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/systems.h>
#include <barrett/wam.h>


//namespace log = barrett::log;
namespace math = barrett::math;
namespace systems = barrett::systems;
namespace units = barrett::units;
using barrett::Wam;
using systems::connect;
using systems::reconnect;
using systems::disconnect;


using boost::bind;
using boost::ref;


const size_t DOF = 7;
const double T_s = 0.002;

typedef Wam<DOF>::jt_type jt_type;
typedef Wam<DOF>::jp_type jp_type;
typedef Wam<DOF>::jv_type jv_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

int main() {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	libconfig::Config config;
	config.readFile("/etc/wam/wamg-new.config");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	Wam<DOF> wam(config.lookup("wam"));


	// start the main loop!
	rtem.start();

	std::cout << "Enter to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::cout << "Enter to start teaching.\n";
	waitForEnter();

	typedef boost::tuple<double, Wam<DOF>::jp_type> jp_sample_type;

	systems::Ramp unitRamp;
	systems::TupleGrouper<double, Wam<DOF>::jp_type> jpLogTg;

	// TODO(dc): use TriggeredDataLogger to take samples with a spatial period instead of a temporal period
	systems::PeriodicDataLogger<jp_sample_type> jpLogger(
			new barrett::log::RealTimeWriter<jp_sample_type>("/tmp/test.bin", T_s),
			10);

	unitRamp.setSamplePeriod(T_s);
	systems::System::Output<double>& time = unitRamp.output;

	connect(time, jpLogTg.getInput<0>());
	connect(wam.jpOutput, jpLogTg.getInput<1>());
	connect(jpLogTg.output, jpLogger.input);

	unitRamp.start();


	std::cout << "Enter to stop teaching.\n";
	waitForEnter();

	jpLogger.closeLog();
	disconnect(jpLogger.input);

	// build spline between recorded points
	barrett::log::Reader<jp_sample_type> lr("/tmp/test.bin");
	std::vector<jp_sample_type> vec;
	for (size_t i = 0; i < lr.numRecords(); ++i) {
		vec.push_back(lr.getRecord());
	}

	math::Spline<Wam<DOF>::jp_type> spline(vec);

	std::cout << "Enter to play back.\n";
	waitForEnter();

	unitRamp.stop();
	unitRamp.setOutput(spline.initialX());

	double (*saturatePtr)(double,double) = math::saturate;  // cast needed because math::saturate is overloaded
	systems::Callback<double, jp_type> trajectory(bind(ref(spline), bind(saturatePtr, _1, spline.finalX())));

	connect(time, trajectory.input);
	wam.trackReferenceSignal(trajectory.output);
	unitRamp.start();


	std::cout << "Enter to idle.\n";
	waitForEnter();
	wam.idle();
	wam.gravityCompensate(false);

	std::cout << "Shift-idle, then press enter.\n";
	waitForEnter();
	rtem.stop();

	return 0;
}
