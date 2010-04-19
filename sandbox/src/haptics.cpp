/*
 * haptics.cpp
 *
 *  Created on: Feb 19, 2010
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
#include <barrett/systems/haptic_ball.h>
#include <barrett/systems/haptic_box.h>
#include <barrett/wam.h>

#include "network_haptics.h"


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


const size_t DOF = 4;
const double T_s = 0.002;

typedef Wam<DOF>::jt_type jt_type;
typedef Wam<DOF>::jp_type jp_type;
typedef Wam<DOF>::jv_type jv_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}

units::CartesianForce::type scale(boost::tuple<units::CartesianForce::type, double> t) {
	return t.get<0>() * t.get<1>();
}

double maxVel = 0.0;
units::CartesianPosition::type cp_1(0.0);
math::FirstOrderFilter<units::CartesianPosition::type> cvFilt;
units::CartesianForce::type limitCv(const units::CartesianPosition::type& cp) {
	const double THRESH = 3.0;
	const double GAIN = 10;

	units::CartesianPosition::type cv = cvFilt.eval((cp - cp_1) / T_s);
	cp_1 = cp;
	double vel = cv.norm();

	if (math::abs(vel) > maxVel) {
		maxVel = vel;
		std::cerr << vel << std::endl;
	}
//	std::cerr << vel << std::endl;

//	if (math::abs(vel) > THRESH) {
	if (false) {
//		std::cerr << "over!";
		return GAIN * (THRESH - vel)/vel * cv;
	} else {
		return units::CartesianForce::type(0.0);
	}
}

jt_type saturateJt(const jt_type& x, const jt_type& limit) {
	int index;
	double minRatio;

	minRatio = (limit.cwise() / (x.cwise().abs())).minCoeff(&index);
	if (minRatio < 1.0) {
		return minRatio * x;
	} else {
		return x;
	}
}

int main(int argc, char** argv) {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	// check arguments
	if (argc != 2) {
		printf("Usage: %s <otherip>\n", argv[0]);
		return 0;
	}

	libconfig::Config config;
	config.readFile("/etc/wam/wam4-new.config");

	systems::RealTimeExecutionManager rtem(T_s, false);
	systems::System::defaultExecutionManager = &rtem;


	units::CartesianPosition::type center;

    // instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));
	NetworkHaptics nh(argv[1]);
	systems::Callback<units::CartesianPosition::type, units::CartesianForce::type> cvLimit(limitCv);
	systems::Summer<units::CartesianForce::type> tfSum;

	center << 0.5,-.3,0;
	systems::HapticBall ball(center, 0.2);
	center << 0.5,0.4,0;
	systems::HapticBox box(center, .4,.4,.4);

	systems::Summer<units::CartesianForce::type> dirSum;
	systems::Summer<double> depthSum;
//	systems::Gain<double> kp(1e4);
	systems::PIDController<double, double> comp;
	systems::Constant<double> zero(0.0);
	systems::TupleGrouper<units::CartesianForce::type, double> tg;
	systems::Callback<boost::tuple<units::CartesianForce::type, double>, units::CartesianForce::type> mult(scale);
	systems::ToolForceToJointTorques<DOF> tf2jt;

	jt_type jtLimits(25);
//	jtLimits << 25, 25, 25, 25;
	systems::Callback<jt_type> jtSat(bind(saturateJt, _1, jtLimits));


	cvFilt.setLowPass(units::CartesianPosition::type(30), units::CartesianPosition::type(1));
	comp.setKp(3e3);
	comp.setKd(3e1);

	// connect systems
	connect(wam.toolPosition.output, nh.input);

	connect(wam.toolPosition.output, ball.input);
	connect(ball.directionOutput, dirSum.getInput(0));
	connect(ball.depthOutput, depthSum.getInput(0));

	connect(wam.toolPosition.output, box.input);
	connect(box.directionOutput, dirSum.getInput(1));
	connect(box.depthOutput, depthSum.getInput(1));

	connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(dirSum.output, tg.getInput<0>());
//	connect(dirSum.output, kp.input);
//	connect(kp.output, tg.getInput<1>());

	connect(depthSum.output, comp.referenceInput);
	connect(zero.output, comp.feedbackInput);
	connect(comp.controlOutput, tg.getInput<1>());

	connect(tg.output, mult.input);
	connect(mult.output, tfSum.getInput(0));
	connect(wam.toolPosition.output, cvLimit.input);
	connect(cvLimit.output, tfSum.getInput(1));
	connect(tfSum.output, tf2jt.input);
	connect(tf2jt.output, jtSat.input);
	connect(jtSat.output, wam.input);


	// start the main loop!
	rtem.start();

	std::cout << "Press [Enter] to compensate for gravity.\n";
	waitForEnter();
	maxVel = 0.0;
	wam.gravityCompensate();

	std::string line;
	bool going = true;
	units::CartesianPosition::type cp;
	units::CartesianForce::type cf;
	while (going) {
		std::cout << ">>> ";
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'p':
			double p;
			std::cin >> p;
			comp.setKp(p);
			break;

		case 'd':
			double d;
			std::cin >> d;
			comp.setKd(d);
			break;

		case 'q':
		case 'x':
			going = false;
			break;

		default:
			{
				SCOPED_LOCK(rtem.getMutex());
				cp = wam.tpController.feedbackInput.getValue();
				cf = tf2jt.input.getValue();
			}
			std::cout << cp << " " << cf << std::endl;

			break;
		}
	}


	std::cout << "Shift-idle, then press [Enter].\n";
	waitForEnter();
	wam.idle();
	wam.gravityCompensate(false);
	rtem.stop();

	return 0;
}
