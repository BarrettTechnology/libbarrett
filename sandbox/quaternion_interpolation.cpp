/*
 * quaternion_interpolation.cpp
 *
 *  Created on: Nov 17, 2011
 *      Author: dc
 */

#include <iostream>
#include <cassert>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <Eigen/Geometry>

#include <barrett/systems.h>
#include <barrett/math.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>

#include <barrett/detail/stl_utils.h>


using namespace barrett;
using detail::waitForEnter;


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {

	Eigen::Quaterniond q1, q2;
	systems::ExposedOutput<Eigen::Quaterniond> orientationSetPoint;

	wam.gravityCompensate();


	printf("Press [Enter] to record first orientation.");
	waitForEnter();
	q1 = wam.getToolOrientation();
	printf("  q1 = %f, %f, %f, %f\n", q1.coeffs()[0], q1.coeffs()[1], q1.coeffs()[2], q1.coeffs()[3]);

	printf("Press [Enter] to record second orientation.");
	waitForEnter();
	q2 = wam.getToolOrientation();
	orientationSetPoint.setValue(q2);
	wam.trackReferenceSignal(orientationSetPoint.output);
	printf("  q1 = %f, %f, %f, %f\n", q2.coeffs()[0], q2.coeffs()[1], q2.coeffs()[2], q2.coeffs()[3]);

	printf("Press [Enter] to move back to first orientation.");
	waitForEnter();

	systems::Ramp time(pm.getExecutionManager(), 1.0);
	math::TrapezoidalVelocityProfile profile(0.2, 0.1, 0.0, 1.0);
	systems::Callback<double, Eigen::Quaterniond> trajectory(boost::bind(&Eigen::Quaterniond::slerp, boost::ref(q2), boost::bind(boost::ref(profile), _1), boost::ref(q1)));

	systems::connect(time.output, trajectory.input);
	wam.trackReferenceSignal(trajectory.output);
	time.start();

//	for (double t = 0.0; t <= 1.0; t += 0.002) {
//		usleep(30000);
//		orientationSetPoint.setValue(q2.slerp(t, q1));
//	}

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
