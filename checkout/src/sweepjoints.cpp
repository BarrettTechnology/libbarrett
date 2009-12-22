/*
 * sweepjoints.c
 *
 *  Created on: Nov 12, 2009
 *      Author: dc
 */

#include <iostream>
#include <string>
#include <unistd.h>  // usleep

#include <boost/tuple/tuple.hpp>

#include <barrett/exception.h>
#include <barrett/detail/debug.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/math/utils.h>
#include <barrett/wam.h>


using namespace barrett;
using systems::connect;
using systems::reconnect;


const size_t DOF = 7, JOINT = 0;


const double SWEEP_VELOCITY = 1.0;

double negEnd, posEnd;
int signChangeCount = 0;

int vSign_0, vSign_1 = 0;
int stoppedCount = 0;

double sweepJoint(const boost::tuple<double, double>& tuple) {
	// unpack tuple
	double theta = tuple.get<0>();
	double omega = tuple.get<1>();

	vSign_0 = static_cast<int>( math::sign(math::deadband(omega, 0.2)) );

	if (vSign_0 != 0) {
		if (stoppedCount > 5) {
			if (vSign_0 != vSign_1) {
				if (signChangeCount < 2) {
					++signChangeCount;

					if (vSign_0 == 1) {
						negEnd = theta + 0.1;
					} else {  // vSign_0 == -1
						posEnd = theta - 0.1;
					}
				}
			} else {
				signChangeCount = 0;
			}

			vSign_1 = vSign_0;
		}
		stoppedCount = 0;
	} else {
		++stoppedCount;
	}

	if (signChangeCount == 2) {
		if (theta < negEnd) {
			return SWEEP_VELOCITY;
		} else if (theta > posEnd) {
			return -SWEEP_VELOCITY;
		}
	}
	return vSign_0 * SWEEP_VELOCITY;
}

void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


int main() {
	installExceptionHandler();  // give us pretty stack traces when things die

	Wam<DOF> wam;

	systems::ExposedOutput<Wam<DOF>::jp_type> setPoint;
	systems::PIDController<Wam<DOF>::jp_type> pid;

	systems::ArraySplitter<Wam<DOF>::jp_type> jpSplitter;
	systems::ArraySplitter<Wam<DOF>::jv_type> jvSplitter;
	systems::TupleGrouper<double, double> pvGrouper;
	systems::Callback<boost::tuple<double, double>, double> sweepSys(&sweepJoint);
	systems::Constant<double> zero(0.0);

	systems::Summer<double, 2> vError("+-");
	systems::FirstOrderFilter<double> velocityCompensator;
	systems::ArrayEditor<Wam<DOF>::jt_type> jtEditor;


	units::Array<DOF> tmp;

//	tmp << 900, 2500, 600, 500, 40, 20, 5;
//	pid.setKp(tmp);
//	tmp << 2.5, 5, 2.5, 0.5, 0, 0, 0;
//	pid.setKi(tmp);
//	tmp << 2, 2, 0.5, 0.8, 0.8, 0.1, 0.1;
//	pid.setKd(tmp);

	tmp << /*3e3*/0, 1e3, 1e2, 1e2, 0.0, 0.0, 0.0;
	pid.setKp(tmp);
	tmp << 25.0, 20.0, 15.0, 15.0, 0.0, 0.0, 0.0;
	pid.setControlSignalLimit(tmp);

	velocityCompensator.setSamplePeriod(0.002);
	velocityCompensator.setLowPass(3, 3);


	connect(wam.jpOutput, pid.feedbackInput);
	connect(pid.controlOutput, jtEditor.input);
	connect(jtEditor.output, wam.input);

	connect(zero.output, jtEditor.getElementInput(JOINT));

	connect(wam.jpOutput, jpSplitter.input);
	systems::System::Output<double>& theta = jpSplitter.getOutput(JOINT);
	connect(wam.jvOutput, jvSplitter.input);
	systems::System::Output<double>& omega = jvSplitter.getOutput(JOINT);

	connect(theta, pvGrouper.getInput<0>());
	connect(omega, pvGrouper.getInput<1>());
	connect(pvGrouper.output, sweepSys.input);

	connect(sweepSys.output, vError.getInput(0));
	connect(omega, vError.getInput(1));
	connect(vError.output, velocityCompensator.input);
	systems::System::Output<double>& tau = velocityCompensator.output;

	// tie inputs together for zero torque
	connect(wam.jpOutput, pid.referenceInput);


	std::cout << "Enter to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();
	reconnect(tau, jtEditor.getElementInput(JOINT));

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
			std::cout << negEnd << " " << posEnd << " " << signChangeCount << std::endl;
			std::cout << vSign_1 << " " << vSign_0 << " " << stoppedCount << std::endl;
//			std::cout << "Commands:\n"
//					<< "\t'h': toggle holding position\n"
//					<< "\t'q', 'x': home home, then idle\n";
		}
	}

	// move home, then idle
	reconnect(zero.output, jtEditor.getElementInput(JOINT));
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
