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
#include <barrett/math_utils.h>
#include <barrett/wam.h>


using namespace barrett;
using systems::connect;
using systems::reconnect;


const size_t DOF = 7, JOINT = 0;

double sweepJoint(const Wam<DOF>::jv_type& jv) {
	return sign(deadband(jv[JOINT], 0.3)) * 2.0;
}

void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

int main() {
	installExceptionHandler();  // give us pretty stack traces when things die

	Wam<DOF> wam;
	systems::PIDController<Wam<DOF>::jp_type> pid;
	systems::ExposedOutput<Wam<DOF>::jp_type> setPoint;
	systems::Constant<double> zero(0.0);
	systems::Callback<Wam<DOF>::jv_type, double> sweepSys(&sweepJoint);
	systems::ArrayEditor<Wam<DOF>::jt_type> ae;
//	systems::PrintToStream<bool> switchOut("switch: ");


	units::Array<DOF> tmp;

	tmp << 3e3, 1e3, 1e2, 1e2, 0.0, 0.0, 0.0;
	pid.setKp(tmp);
//	tmp << 100, 0, 0, 0, 0.0, 0.0, 0.0;
//	pid.setKi(tmp);
//	tmp << 5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0;
//	pid.setIntegratorLimit(tmp);
	tmp << 25.0, 20.0, 15.0, 15.0, 0.0, 0.0, 0.0;
	pid.setControlSignalLimit(tmp);

	connect(wam.jpOutput, pid.feedbackInput);


	connect(zero.output, ae.getElementInput(JOINT));

	connect(pid.controlOutput, ae.input);
	connect(ae.output, wam.input);

	connect(wam.jvOutput, sweepSys.input);
//	connect(switchSys.output, switchOut.input);

	// tie inputs together for zero torque
	connect(wam.jpOutput, pid.referenceInput);


	std::cout << "Enter to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();
	reconnect(sweepSys.output, ae.getElementInput(JOINT));

	std::string line;
	bool going = true, holding = false;
	while (going) {
		std::cout << ">>> ";
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'h':
			holding = !holding;
			if (holding) {
				setPoint.setValue(wam.getJointPositions());
				reconnect(setPoint.output, pid.referenceInput);
			} else {
				reconnect(wam.jpOutput, pid.referenceInput);
			}
			break;

		case 'q':
		case 'x':
			going = false;
			break;

		default:
			std::cout << "Commands:\n"
					<< "\t'h': toggle holding position\n"
					<< "\t'q', 'x': home home, then idle\n";
		}
	}

	// move home, then idle
	reconnect(zero.output, ae.getElementInput(JOINT));
	reconnect(wam.jpOutput, pid.referenceInput);
	wam.moveHome();
	while ( !wam.moveIsDone() ) {
		usleep(static_cast<int>(1e5));
	}

	wam.gravityCompensate(false);
	wam.idle();

	std::cout << "Shift-idle, then press enter.\n";
	waitForEnter();

	return 0;
}
