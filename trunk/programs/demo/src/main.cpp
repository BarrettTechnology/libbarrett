/*
 * main.cpp
 *
 *  Created on: Sep 29, 2009
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <unistd.h>  // usleep

#include <barrett/exception.h>
#include <barrett/detail/debug.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/wam.h>


using namespace barrett;


//#include <native/task.h>
//#include <barrett/threading/real_time_mutex.h>
//
//threading::RealTimeMutex rtm;
//
//
//int blah(const int& ns) {
//	SCOPED_LOCK(rtm);
//	rt_task_sleep(ns);
//	return ns;
//}


const size_t DOF = 7;


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
//		rtm.lock();
//		std::cerr << "locked\n";
//		rtm.unlock();
//	}
//	rtem.stop();
//	std::cerr << "stopped\n";
//	usleep(2000000);


//#if 0
	installExceptionHandler();  // give us pretty stack traces when things die

	units::Array<DOF> tmp;

	systems::RealTimeExecutionManager rtem;
	systems::System::defaultExecutionManager = &rtem;


	Wam<DOF> wam;

	systems::PIDController<Wam<DOF>::jp_type>* pid =
			new systems::PIDController<Wam<DOF>::jp_type>();

	tmp << 900, 2500, 600, 500, 40, 20, 5;
	pid->setKp(tmp);
	tmp << 2.5, 5, 2.5, 0.5, 0, 0, 0;
	pid->setKi(tmp);
	tmp << 2, 2, 0.5, 0.8, 0.8, 0.1, 0.1;
	pid->setKd(tmp);
	tmp << 25.0, 20.0, 15.0, 15.0, 5, 5, 5;
	pid->setControlSignalLimit(tmp);

//	tmp << 3e3, 1e3, 1e2, 1e2, 0.0, 0.0, 0.0;
//	pid->setKp(tmp);
//	tmp << 100, 0, 0, 0, 0.0, 0.0, 0.0;
//	pid->setKi(tmp);
//	tmp << 5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0;
//	pid->setIntegratorLimit(tmp);
//	tmp << 25.0, 20.0, 15.0, 15.0, 0.0, 0.0, 0.0;
//	pid->setControlSignalLimit(tmp);

	systems::connect(wam.jpOutput, pid->feedbackInput);

	systems::Converter<Wam<DOF>::jt_type> supervisoryController;
	supervisoryController.registerConversion(pid);
	systems::connect(supervisoryController.output, wam.input);

	// tie inputs together for zero torque
	supervisoryController.connectInputTo(wam.jpOutput);


	Wam<DOF>::jp_type setPoint;
	setPoint << 0.000, -1.57, 0.0, 1.57, 0.0, 1.605, 0.0;
	systems::Constant<Wam<DOF>::jp_type> point(setPoint);

//	systems::PrintToStream<Wam<DOF>::jt_type> pts("JT: ");
//	systems::connect(supervisoryController.output, pts.input);

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
//	systems::reconnect(point.output, pid.referenceInput);
	supervisoryController.connectInputTo(point.output);

	std::cout << "Enter to move home.\n";
	waitForEnter();
//	systems::reconnect(wam.output, pid.referenceInput);
	supervisoryController.connectInputTo(wam.jpOutput);
	wam.moveHome();

	std::cout << "Enter to idle.\n";
	waitForEnter();
	wam.gravityCompensate(false);
	wam.idle();

	std::cout << "Shift-idle, then press enter.\n";
	waitForEnter();

//#endif
	return 0;
}
