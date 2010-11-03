/*
 * teach.cpp
 *
 *  Created on: Sep 29, 2009
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/log.h>
#include <barrett/systems.h>


//namespace log = barrett::log;
namespace math = barrett::math;
namespace systems = barrett::systems;
namespace units = barrett::units;
using systems::connect;
using systems::reconnect;
using systems::disconnect;


using boost::bind;
using boost::ref;


const size_t DOF = 7;
const double T_s = 0.002;
BARRETT_UNITS_TYPEDEFS(DOF);


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

int main(int argc, char** argv) {
	libconfig::Config config;
	config.readFile("/etc/barrett/wam7.conf");

	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	systems::Wam<DOF> wam(config.lookup("wam"));


	// start the main loop!
	rtem.start();

	std::cout << "Enter to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();

	while (true) {
		typedef boost::tuple<double, jp_type> jp_sample_type;

		systems::Ramp time;
		time.setSamplePeriod(T_s);

		if (argc == 1) {
			std::cout << "Enter to start teaching.\n";
			waitForEnter();

			systems::TupleGrouper<double, jp_type> jpLogTg;

			// TODO(dc): use TriggeredDataLogger to take samples with a spatial period instead of a temporal period
			systems::PeriodicDataLogger<jp_sample_type> jpLogger(
					new barrett::log::RealTimeWriter<jp_sample_type>("/tmp/test.bin", T_s),
					10);

			connect(time.output, jpLogTg.getInput<0>());
			connect(wam.jpOutput, jpLogTg.getInput<1>());
			connect(jpLogTg.output, jpLogger.input);

			time.start();


			std::cout << "Enter to stop teaching.\n";
			waitForEnter();

			jpLogger.closeLog();
			disconnect(jpLogger.input);
		}

		// build spline between recorded points
		barrett::log::Reader<jp_sample_type> lr("/tmp/test.bin");
		std::vector<jp_sample_type> vec;
		for (size_t i = 0; i < lr.numRecords(); ++i) {
			vec.push_back(lr.getRecord());
		}

		math::Spline<jp_type> spline(vec);

		std::cout << "Enter to play back.\n";
		waitForEnter();

		bool repeat = true;
		while (repeat) {
			// first move to the starting position
			wam.moveTo(spline.eval(spline.initialX()));

			// then play back the recorded motion
			time.stop();
			time.setOutput(spline.initialX());

			double (*saturatePtr)(double,double) = math::saturate;  // cast needed because math::saturate is overloaded
			systems::Callback<double, jp_type> trajectory(bind(ref(spline), bind(saturatePtr, _1, spline.finalX())));

			connect(time.output, trajectory.input);
			wam.trackReferenceSignal(trajectory.output);

			time.start();

			while (trajectory.input.getValue() < spline.finalX()) {
				usleep(10000);
			}

//			std::string line;
//			std::cin >> line;
//			if (line[0] == 'x') {
//				repeat = false;
//			}
		}

		std::cout << "Enter to repeat, 'x' Enter to teach again.\n";
//		waitForEnter();
		wam.idle();
	}

	std::cout << "Shift-idle, then press enter.\n";
	waitForEnter();
	rtem.stop();

	return 0;
}
