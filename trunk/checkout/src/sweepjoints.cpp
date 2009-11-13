/*
 * sweepjoints.c
 *
 *  Created on: Nov 12, 2009
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


const size_t DOF = 7;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

int main() {
	installExceptionHandler();  // give us pretty stack traces when things die

	units::Array<DOF> tmp;

	Wam<DOF> wam;

	systems::PIDController<Wam<DOF>::ja_type>* pid =
			new systems::PIDController<Wam<DOF>::ja_type>();

	tmp << 3e3, 1e3, 1e2, 1e2, 0.0, 0.0, 0.0;
	pid->setKp(tmp);
//	tmp << 100, 0, 0, 0, 0.0, 0.0, 0.0;
//	pid->setKi(tmp);
//	tmp << 5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0;
//	pid->setIntegratorLimit(tmp);
	tmp << 25.0, 20.0, 15.0, 15.0, 0.0, 0.0, 0.0;
	pid->setControlSignalLimit(tmp);

	systems::connect(wam.output, pid->feedbackInput);


	systems::ArrayEditor<Wam<DOF>::jt_type> ae;
	systems::Constant<double> zero(0.0);
	systems::connect(zero.output, ae.getElementInput(0));

	systems::connect(pid.controlOutput, ae.input);
	systems::connect(ae.output, wam.input);

	// tie inputs together for zero torque
	systems::connect(wam.output, pid.referenceInput);


	Wam<DOF>::ja_type setPoint;
	setPoint << 0.000, -1.57, 0.0, 1.57, 0.0, 1.605, 0.0;
	systems::Constant<Wam<DOF>::ja_type> point(setPoint);

	std::cout << "Enter to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();

	std::cout << "Enter to move to set point.\n";
	waitForEnter();
	systems::reconnect(point.output, pid.referenceInput);

	std::cout << "Enter to move home.\n";
	waitForEnter();
	systems::reconnect(wam.output, pid.referenceInput);
	wam.moveHome();

	std::cout << "Enter to idle.\n";
	waitForEnter();
	wam.gravityCompensate(false);
	wam.idle();

	std::cout << "Shift-idle, then press enter.\n";
	waitForEnter();

	return 0;
}
