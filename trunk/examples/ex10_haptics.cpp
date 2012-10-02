/*
 * ex10_haptics.cpp
 *
 *  Created on: Feb 19, 2010
 *      Author: dc
 */

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "ex10_haptics.h"


using namespace barrett;
using systems::connect;
BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;


char* remoteHost = NULL;
double kp = 3e3;
double kd = 3e1;
bool validate_args(int argc, char** argv) {
	switch (argc) {
	case 4:
		kd = atof(argv[3]);
	case 3:
		kp = atof(argv[2]);
	case 2:
		remoteHost = argv[1];
		break;

	default:
		printf("Usage: %s <otherip> [<kp> [<kd>]]\n", argv[0]);
		return false;
		break;
	}

	printf("Gains: kp = %f; kd = %f\n", kp, kd);
	return true;
}


cf_type scale(boost::tuple<cf_type, double> t) {
	return t.get<0>() * t.get<1>();
}

template <size_t DOF>
typename units::JointTorques<DOF>::type saturateJt(const typename units::JointTorques<DOF>::type& x, const typename units::JointTorques<DOF>::type& limit) {
	int index;
	double minRatio;

	minRatio = (limit.cwise() / (x.cwise().abs())).minCoeff(&index);
	if (minRatio < 1.0) {
		return minRatio * x;
	} else {
		return x;
	}
}

template <size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


    // instantiate Systems
	NetworkHaptics nh(pm.getExecutionManager(), remoteHost);

	cp_type center;
	center << 0.4, -.3, 0.0;
	systems::HapticBall ball(center, 0.2);
	center << 0.35, 0.4, 0.0;
	math::Vector<3>::type size;
	size << 0.3, 0.3, 0.3;
	systems::HapticBox box(center, size);

	systems::Summer<cf_type> dirSum;
	systems::Summer<double> depthSum;
	systems::PIDController<double, double> comp;
	systems::Constant<double> zero(0.0);
	systems::TupleGrouper<cf_type, double> tg;
	systems::Callback<boost::tuple<cf_type, double>, cf_type> mult(scale);
	systems::ToolForceToJointTorques<DOF> tf2jt;

	jt_type jtLimits(35.0);
	systems::Callback<jt_type> jtSat(boost::bind(saturateJt<DOF>, _1, jtLimits));

	// configure Systems
	comp.setKp(kp);
	comp.setKd(kd);

	// connect Systems
	connect(wam.toolPosition.output, nh.input);

	connect(wam.toolPosition.output, ball.input);
	connect(ball.directionOutput, dirSum.getInput(0));
	connect(ball.depthOutput, depthSum.getInput(0));

	connect(wam.toolPosition.output, box.input);
	connect(box.directionOutput, dirSum.getInput(1));
	connect(box.depthOutput, depthSum.getInput(1));

	connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(dirSum.output, tg.getInput<0>());

	connect(depthSum.output, comp.referenceInput);
	connect(zero.output, comp.feedbackInput);
	connect(comp.controlOutput, tg.getInput<1>());

	connect(tg.output, mult.input);
	connect(mult.output, tf2jt.input);
	connect(tf2jt.output, jtSat.input);


	// adjust velocity fault limit
	pm.getSafetyModule()->setVelocityLimit(1.5);

	while (true) {
		wam.gravityCompensate();
		connect(jtSat.output, wam.input);

		// block until the user Shift-idles
		pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

		systems::disconnect(wam.input);
		wam.gravityCompensate(false);

		// block until the user Shift-activates
		pm.getSafetyModule()->waitForMode(SafetyModule::ACTIVE);
	}

	return 0;
}
