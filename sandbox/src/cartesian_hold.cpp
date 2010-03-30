/*
 * cartesian_hold.cpp
 *
 *  Created on: Jan 14, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <unistd.h>  // usleep

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <libconfig.h++>
#include <Eigen/Geometry>

#include <barrett/exception.h>
#include <barrett/detail/debug.h>
#include <barrett/thread/abstract/mutex.h>
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
const double T_s = 0.010;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

std::ostream& operator<< (std::ostream& os, const Eigen::Quaterniond& q) {
	os << "[" << q.w() << ", (" << q.x() << ", " << q.y() << ", " << q.z() << ")]";
	return os;
}


int main() {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die


	libconfig::Config config;
	config.readFile("/etc/wam/wamg-new.config");
	systems::KinematicsBase<DOF> kin(config.lookup("wam.kinematics"));


	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;


	Wam<DOF> wam(config.lookup("wam"));

	systems::ToolPosition<DOF> toolPos;
	systems::ToolForceToJointTorques<DOF> tf2jt;
	systems::PIDController<units::CartesianPosition::type, units::CartesianForce::type> pid(config.lookup("wam.tool_position_control"));

	systems::ToolOrientation<DOF> toolOrient;
	systems::ToolOrientationController<DOF> toolOrientController;

//	systems::Summer<Wam<DOF>::jt_type, 2> jtSum;


//	math::Array<3> tmp;
//	tmp.assign(2e3);
//	pid.setKp(tmp);
//	tmp.assign(2e1);
//	pid.setKd(tmp);
//	tmp.assign(1e2);
//	pid.setControlSignalLimit(tmp);


	connect(wam.jpOutput, kin.jpInput);
	connect(wam.jvOutput, kin.jvInput);
	connect(kin.kinOutput, toolPos.kinInput);
	connect(kin.kinOutput, tf2jt.kinInput);
	connect(kin.kinOutput, toolOrient.kinInput);
	connect(kin.kinOutput, toolOrientController.kinInput);

	connect(toolPos.output, pid.feedbackInput);
	connect(pid.controlOutput, tf2jt.input);
//	connect(tf2jt.output, jtSum.getInput(0));

	systems::PrintToStream<Wam<DOF>::jt_type> jtPrint;
	connect(toolOrient.output, toolOrientController.feedbackInput);
	connect(toolOrientController.controlOutput, wam.input);
//	connect(toolOrientController.controlOutput, jtSum.getInput(1));

//	systems::Constant<Wam<DOF>::jt_type> zero(Wam<DOF>::jt_type(0.0));
//	connect(zero.output, jtSum.getInput(0));

//	connect(jtSum.output, wam.input);


	// tie inputs together for zero torque
	connect(toolPos.output, pid.referenceInput);
	connect(toolOrient.output, toolOrientController.referenceInput);



	// start the main loop!
	rtem.start();

	std::cout << "Enter to gravity compensate.\n";
	waitForEnter();
	wam.gravityCompensate();

//	systems::ExposedOutput<units::CartesianPosition::type> setPointLoc;
	systems::ExposedOutput<Eigen::Quaterniond> setPointOrient;
	std::string line;
	bool going = true, holding = false, gravComp = true;
	Eigen::Quaterniond q;
	Wam<DOF>::jp_type jp;
	while (going) {
		std::cout << ">>> ";
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'h':
			holding = !holding;
			if (holding) {
				SCOPED_LOCK(rtem.getMutex());
//				setPointLoc.setValue(pid.feedbackInput.getValue());
				setPointOrient.setValue(toolOrientController.feedbackInput.getValue());
//				reconnect(setPointLoc.output, pid.referenceInput);
				reconnect(setPointOrient.output, toolOrientController.referenceInput);
			} else {
//				reconnect(toolPos.output, pid.referenceInput);  // zero torque
				reconnect(toolOrient.output, toolOrientController.referenceInput);
			}
			break;

		case 'g':
			gravComp = !gravComp;
			wam.gravityCompensate(gravComp);
			break;

		case 'q':
		case 'x':
			going = false;
			break;

		default:
			{
				SCOPED_LOCK(rtem.getMutex());
				q = toolOrientController.feedbackInput.getValue();
				jp = kin.jpInput.getValue();
			}
			std::cout << q << " " << jp << std::endl;

			break;
		}
	}


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
