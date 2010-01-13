/*
 * main.cpp
 *
 *  Created on: Sep 29, 2009
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <unistd.h>  // usleep

#include <boost/ref.hpp>
#include <boost/bind.hpp>

#include <barrett/exception.h>
#include <barrett/detail/debug.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/wam.h>


using namespace barrett;
using boost::bind;
using boost::ref;


//#include <native/task.h>
//#include <barrett/thread/real_time_mutex.h>
//
//
//int blah(const int& ns) {
//	rt_task_sleep(ns);
//	return ns;
//}


const size_t DOF = 7;
const double T_s = 0.002;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

int main() {
//	systems::RealTimeExecutionManager rtem(1000000000);
//	systems::System::defaultExecutionManager = &rtem;
//
//	systems::Constant<int> five(500000000);
//	systems::Callback<int, int> blahSys(&blah);
//	systems::PrintToStream<int> pts("Five: ");
//
//	systems::connect(five.output, blahSys.input);
//	systems::connect(blahSys.output, pts.input);
//
//	rtem.start();
//
//	for (int i = 0; i < 25; ++i) {
//		usleep(200000);
////		rtem.getMutex().lock();
//		systems::Constant<int> stuff(8);
////		systems::forceConnect(five.output, blahSys.input);
//		std::cerr << "locked\n";
////		rtem.getMutex().unlock();
//	}
//	rtem.stop();
//	std::cerr << "stopped\n";
//	usleep(2000000);


//#if 0
	installExceptionHandler();  // give us pretty stack traces when things die

	math::Array<DOF> tmp;

	systems::RealTimeExecutionManager rtem;
	systems::System::defaultExecutionManager = &rtem;


	Wam<DOF> wam;

	systems::PIDController<Wam<DOF>::jp_type>* pid =
			new systems::PIDController<Wam<DOF>::jp_type>();

	tmp << 900, 2500, 600, 500, 40, 20, 5;
	pid->setKp(tmp);
//	tmp << 2.5, 5, 2.5, 0.5, 0, 0, 0;
//	pid->setKi(tmp);
	tmp << 2, 2, 0.5, 0.8, 0.8, 0.1, 0.1;
	pid->setKd(tmp);
	tmp << 25.0, 20.0, 15.0, 15.0, 5, 5, 5;
	pid->setControlSignalLimit(tmp);

	systems::connect(wam.jpOutput, pid->feedbackInput);

	systems::Converter<Wam<DOF>::jt_type> supervisoryController;
	supervisoryController.registerConversion(pid);
	systems::connect(supervisoryController.output, wam.input);

	// tie inputs together for zero torque
	supervisoryController.connectInputTo(wam.jpOutput);


	Wam<DOF>::jp_type setPoint;
	setPoint << 0.000, -1.57, 0.0, 1.57, 0.0, 1.605, 0.0;

	rtem.start();
	std::cout << wam.operateCount << std::endl;

	std::cout << "Enter to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::cout << wam.operateCount << std::endl;
	usleep(1000000);
	std::cout << wam.operateCount << std::endl;

	std::cout << "Enter to move to set point.\n";
	waitForEnter();

	// build spline to setPoint
	std::vector<Wam<DOF>::jp_type> vec;
	vec.push_back(wam.getJointPositions());
	vec.push_back(setPoint);
	math::Spline<Wam<DOF>::jp_type> spline(vec);
	math::TrapezoidalVelocityProfile profile(.5, 1.0, 0, spline.changeInX());

	systems::Constant<double> one(1.0);
	systems::FirstOrderFilter<double> integral(true);
	systems::Callback<double, Wam<DOF>::jp_type> trajectory(bind(ref(spline), bind(ref(profile), _1)));

	integral.setSamplePeriod(T_s);
	integral.setIntegrator(1.0);

	systems::connect(one.output, integral.input);
	systems::System::Output<double>& time = integral.output;
	systems::connect(time, trajectory.input);
	supervisoryController.connectInputTo(trajectory.output);

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

//#endif
	return 0;
}
