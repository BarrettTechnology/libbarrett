/** <b> Implementation file: do not include.\ </b> Defines barrett::Wam.
 *
 * @file wam-inl.h
 * @date Sep 25, 2009
 * @author Dan Cody
 *
 * @warning
 * This file is located in a \c detail directory. It is part of the
 * implementation and should not be directly included by the user.
 * @see barrett/wam.h
 */

/* Copyright 2009 Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */


#include <unistd.h>  // usleep

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <libconfig.h>

#include "../units.h"
#include "../thread/abstract/mutex.h"
#include "../math/spline.h"
#include "../math/trapezoidal_velocity_profile.h"
#include "../systems/abstract/system.h"
#include "../systems/io_conversion.h"


namespace barrett {

// TODO(dc): some of these members should be inline

template<size_t DOF>
Wam<DOF>::Wam(const libconfig::Setting& setting) :
	wam(setting["low_level"]),
	kinematicsBase(setting["kinematics"]),
	gravity(setting["gravity_compensation"]),
	jvFilter(setting["joint_velocity_filter"]),
	toolPosition(),
	toolOrientation(),

	supervisoryController(),
	jtPassthrough(1.0),
	jpController(setting["joint_position_control"]),
	jvController1(setting["joint_velocity_control"][0]),
	jvController2(setting["joint_velocity_control"][1]),
	tpController(setting["tool_position_control"]),
	tf2jt(),
	toController(),

	jtSum(true),

	input(jtSum.getInput(JT_INPUT)), jpOutput(wam.jpOutput), jvOutput(jvFilter.output),

	doneMoving(true)
{
	using systems::connect;
	using systems::makeIOConversion;

	connect(wam.jpOutput, kinematicsBase.jpInput);
	connect(jvOutput, kinematicsBase.jvInput);

	connect(kinematicsBase.kinOutput, gravity.kinInput);
	// Don't connect gravity.output here. Gravity compensation is off by default.

	connect(kinematicsBase.kinOutput, toolPosition.kinInput);
	connect(kinematicsBase.kinOutput, toolOrientation.kinInput);
	connect(kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(kinematicsBase.kinOutput, toController.kinInput);

	connect(wam.jvOutput, jvFilter.input);
//	jv_type corners;
//	corners.setConstant(180.0);
//	jvFilter.setLowPass(corners);

	supervisoryController.registerConversion(makeIOConversion(
			jtPassthrough.input, jtPassthrough.output));

	connect(wam.jpOutput, jpController.feedbackInput);
	supervisoryController.registerConversion(makeIOConversion(
			jpController.referenceInput, jpController.controlOutput));

	// TODO(dc): change to a FirstOrderFilterController
	connect(jvOutput, jvController1.feedbackInput);
	connect(jvController1.controlOutput, jvController2.input);
	supervisoryController.registerConversion(makeIOConversion(
			jvController1.referenceInput, jvController2.output));
//	corners << 180, 180, 56, 56, 10, 30, 3;
//	jvController2.setLowPass(corners);

	connect(toolPosition.output, tpController.feedbackInput);
	connect(tpController.controlOutput, tf2jt.input);
	supervisoryController.registerConversion(makeIOConversion(
			tpController.referenceInput, tf2jt.output));

	connect(toolOrientation.output, toController.feedbackInput);
	supervisoryController.registerConversion(makeIOConversion(
			toController.referenceInput, toController.controlOutput));

	connect(supervisoryController.output, jtSum.getInput(SC_INPUT));
	connect(jtSum.output, wam.input);
}

template<size_t DOF>
Wam<DOF>::~Wam()
{
}

template<size_t DOF>
template<typename T>
void Wam<DOF>::trackReferenceSignal(systems::System::Output<T>& referenceSignal)
{
	supervisoryController.connectInputTo(referenceSignal);
}

template<size_t DOF>
typename units::JointTorques<DOF>::type Wam<DOF>::getJointTorques()
{
	SCOPED_LOCK(gravity.getEmMutex());
	return wam.input.getValue();
}

template<size_t DOF>
typename units::JointPositions<DOF>::type Wam<DOF>::getJointPositions()
{
	SCOPED_LOCK(gravity.getEmMutex());
	return kinematicsBase.jpInput.getValue();
}

template<size_t DOF>
typename units::JointVelocities<DOF>::type Wam<DOF>::getJointVelocities()
{
	SCOPED_LOCK(gravity.getEmMutex());
	return kinematicsBase.jvInput.getValue();
}

template<size_t DOF>
units::CartesianPosition::type Wam<DOF>::getToolPosition()
{
	SCOPED_LOCK(gravity.getEmMutex());
	return tpController.feedbackInput.getValue();
}

template<size_t DOF>
Eigen::Quaterniond Wam<DOF>::getToolOrientation()
{
	SCOPED_LOCK(gravity.getEmMutex());
	return toController.feedbackInput.getValue();
}


template<size_t DOF>
void Wam<DOF>::gravityCompensate(bool compensate)
{
	if (compensate) {
		systems::forceConnect(gravity.output, jtSum.getInput(GRAVITY_INPUT));
	} else {
		systems::disconnect(jtSum.getInput(GRAVITY_INPUT));
	}
}

template<size_t DOF>
void Wam<DOF>::moveHome(bool blocking, double velocity, double acceleration)
{
	moveTo(jp_type(wam.wambot->base.home), blocking, velocity, acceleration);
}

template<size_t DOF>
inline void Wam<DOF>::moveTo(const jp_type& destination, bool blocking, double velocity, double acceleration)
{
//	moveTo(getJointPositions(), getJointVelocities(), destination, blocking, velocity, acceleration);
	moveTo(getJointPositions(), jv_type(0.0), destination, blocking, velocity, acceleration);
}

template<size_t DOF>
template<typename T>
void Wam<DOF>::moveTo(const T& currentPos, const typename T::unitless_type& currentVel, const T& destination, bool blocking, double velocity, double acceleration)
{
	boost::thread(&Wam<DOF>::moveToThread<T>, this, currentPos, currentVel, destination, velocity, acceleration);

	// wait until thread starts
	// TODO(dc): potential deadlock issue for very short trajectories
	while (moveIsDone()) {
		usleep(1000);
	}

	if (blocking) {
		while (!moveIsDone()) {
			usleep(10000);
		}
	}
}

template<size_t DOF>
bool Wam<DOF>::moveIsDone()
{
	return doneMoving;
}

template<size_t DOF>
void Wam<DOF>::idle()
{
	supervisoryController.disconnectInput();
}


template<size_t DOF>
template<typename T>
void Wam<DOF>::moveToThread(const T& currentPos, const typename T::unitless_type& currentVel, const T& destination, double velocity, double acceleration)
{
	std::vector<T> vec;
	vec.push_back(currentPos);
	vec.push_back(destination);
	math::Spline<T> spline(vec, currentVel);
	// TODO(dc): write a vel/acc traits class to give intelligent defaults for these values
	math::TrapezoidalVelocityProfile profile(velocity, acceleration, currentVel.norm(), spline.changeInX());
//	math::TrapezoidalVelocityProfile profile(.1, .2, 0, spline.changeInX());

	systems::Ramp time(1.0, false);
	systems::Callback<double, T> trajectory(boost::bind(boost::ref(spline), boost::bind(boost::ref(profile), _1)));

	// TODO(dc): Ramp should get this from the EM itself
	time.setSamplePeriod(tf2jt.getExecutionManager()->getPeriod());  // get the EM from one of the Wam's Systems...

	systems::connect(time.output, trajectory.input);
	trackReferenceSignal(trajectory.output);
	time.start();

	doneMoving = false;

	while (trajectory.input.getValue() < profile.finalT()) {
		// if the move is interrupted, clean up and end the thread
		if ( !trajectory.output.isConnected() ) {
			return;
		}
		usleep(10000);
	}

	doneMoving = true;

	// wait until the trajectory is no longer referenced by supervisoryController
	while (trajectory.output.isConnected()) {
		usleep(10000);
	}
}


}
