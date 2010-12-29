/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * wam-inl.h
 *
 *  Created on: Sep 25, 2009
 *      Author: dc
 */


#include <unistd.h>  // usleep

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <libconfig.h>

#include <barrett/units.h>
#include <barrett/products/puck.h>
#include <barrett/products/safety_module.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/math/spline.h>
#include <barrett/math/trapezoidal_velocity_profile.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/io_conversion.h>
#include <barrett/systems/ramp.h>
#include <barrett/systems/callback.h>


namespace barrett {
namespace systems {


// TODO(dc): some of these members should be inline

template<size_t DOF>
Wam<DOF>::Wam(const std::vector<Puck*>& genericPucks, SafetyModule* safetyModule,
		const libconfig::Setting& setting,
		std::vector<int> torqueGroupIds) :
	llww(genericPucks, safetyModule, setting["low_level"], torqueGroupIds),
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

	input(jtSum.getInput(JT_INPUT)), jpOutput(llww.jpOutput), jvOutput(jvFilter.output),

	doneMoving(true)
{
	connect(llww.jpOutput, kinematicsBase.jpInput);
	connect(jvOutput, kinematicsBase.jvInput);

	connect(kinematicsBase.kinOutput, gravity.kinInput);
	// Don't connect gravity.output here. Gravity compensation is off by default.

	connect(kinematicsBase.kinOutput, toolPosition.kinInput);
	connect(kinematicsBase.kinOutput, toolOrientation.kinInput);
	connect(kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(kinematicsBase.kinOutput, toController.kinInput);

	connect(llww.jvOutput, jvFilter.input);

	supervisoryController.registerConversion(makeIOConversion(
			jtPassthrough.input, jtPassthrough.output));

	connect(llww.jpOutput, jpController.feedbackInput);
	supervisoryController.registerConversion(makeIOConversion(
			jpController.referenceInput, jpController.controlOutput));

	// TODO(dc): combine into single controller
	connect(jvOutput, jvController1.feedbackInput);
	connect(jvController1.controlOutput, jvController2.input);
	supervisoryController.registerConversion(makeIOConversion(
			jvController1.referenceInput, jvController2.output));

	connect(toolPosition.output, tpController.feedbackInput);
	connect(tpController.controlOutput, tf2jt.input);
	supervisoryController.registerConversion(makeIOConversion(
			tpController.referenceInput, tf2jt.output));

	connect(toolOrientation.output, toController.feedbackInput);
	supervisoryController.registerConversion(makeIOConversion(
			toController.referenceInput, toController.controlOutput));

	connect(supervisoryController.output, jtSum.getInput(SC_INPUT));
	connect(jtSum.output, llww.input);
}

template<size_t DOF>
Wam<DOF>::~Wam()
{
}

template<size_t DOF>
template<typename T>
void Wam<DOF>::trackReferenceSignal(System::Output<T>& referenceSignal)
{
	supervisoryController.connectInputTo(referenceSignal);
}

template<size_t DOF>
inline const typename Wam<DOF>::jp_type& Wam<DOF>::getHomePosition() const
{
	return llww.getLowLevelWam().getHomePosition();
}

template<size_t DOF>
typename Wam<DOF>::jt_type Wam<DOF>::getJointTorques() const
{
	BARRETT_SCOPED_LOCK(gravity.getEmMutex());
	return llww.input.getValue();
}

template<size_t DOF>
inline typename Wam<DOF>::jp_type Wam<DOF>::getJointPositions() const
{
	return llww.getLowLevelWam().getJointPositions();
}

template<size_t DOF>
inline typename Wam<DOF>::jv_type Wam<DOF>::getJointVelocities() const
{
	return llww.getLowLevelWam().getJointVelocities();
}

template<size_t DOF>
typename Wam<DOF>::cp_type Wam<DOF>::getToolPosition() const
{
	BARRETT_SCOPED_LOCK(gravity.getEmMutex());
	return tpController.feedbackInput.getValue();
}

template<size_t DOF>
Eigen::Quaterniond Wam<DOF>::getToolOrientation() const
{
	BARRETT_SCOPED_LOCK(gravity.getEmMutex());
	return toController.feedbackInput.getValue();
}


template<size_t DOF>
void Wam<DOF>::gravityCompensate(bool compensate)
{
	if (compensate) {
		forceConnect(gravity.output, jtSum.getInput(GRAVITY_INPUT));
	} else {
		disconnect(jtSum.getInput(GRAVITY_INPUT));
	}
}
template<size_t DOF>
bool Wam<DOF>::isGravityCompensated()
{
	return jtSum.getInput(GRAVITY_INPUT).isConnected();
}

template<size_t DOF>
void Wam<DOF>::moveHome(bool blocking, double velocity, double acceleration)
{
	moveTo(getHomePosition(), blocking, velocity, acceleration);
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
bool Wam<DOF>::moveIsDone() const
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

	Ramp time(1.0, false);
	Callback<double, T> trajectory(boost::bind(boost::ref(spline), boost::bind(boost::ref(profile), _1)));

	// TODO(dc): Ramp should get this from the EM itself
	time.setSamplePeriod(tf2jt.getExecutionManager()->getPeriod());  // get the EM from one of the Wam's Systems...

	connect(time.output, trajectory.input);
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
}
