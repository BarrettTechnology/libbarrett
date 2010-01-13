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


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

int main() {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	units::Array<DOF> tmp;

	systems::RealTimeExecutionManager rtem;
	systems::System::defaultExecutionManager = &rtem;


	Wam<DOF> wam;

	systems::PIDController<Wam<DOF>::jp_type>* pid =
			new systems::PIDController<Wam<DOF>::jp_type>();

	tmp << 900, 2500, 600, 500, 40, 20, 5;
	pid->setKp(tmp);
	tmp << 2, 2, 0.5, 0.8, 0.8, 0.1, 0.1;
	pid->setKd(tmp);
	tmp << 25.0, 20.0, 15.0, 15.0, 5, 5, 5;
	pid->setControlSignalLimit(tmp);

	connect(wam.jpOutput, pid->feedbackInput);

	systems::Converter<Wam<DOF>::jt_type> supervisoryController;
	supervisoryController.registerConversion(pid);
	connect(supervisoryController.output, wam.input);

	// tie inputs together for zero torque
	supervisoryController.connectInputTo(wam.jpOutput);


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

	systems::Callback<double, Wam<DOF>::jp_type> trajectory(bind(ref(spline), bind(math::saturate<double>, _1, spline.finalX())));

	connect(time, trajectory.input);
	supervisoryController.connectInputTo(trajectory.output);
	unitRamp.start();


	std::cout << "Enter to move home.\n";
	waitForEnter();
	supervisoryController.connectInputTo(wam.jpOutput);
	wam.moveHome();
	while ( !wam.moveIsDone() ) {
		usleep(static_cast<int>(1e5));
	}

	wam.gravityCompensate(false);
	wam.idle();

	std::cout << "Shift-idle, then press enter.\n";
	waitForEnter();
	rtem.stop();

	return 0;
}
