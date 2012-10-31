/*
	Copyright 2009, 2010, 2011, 2012 Barrett Technology <support@barrett.com>

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

#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <libconfig.h>

#include <barrett/os.h>
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
Wam<DOF>::Wam(ExecutionManager* em, const std::vector<Puck*>& genericPucks,
		SafetyModule* safetyModule, const libconfig::Setting& setting,
		std::vector<int> torqueGroupIds, const std::string& sysName) :
	llww(em, genericPucks, safetyModule, setting["low_level"], torqueGroupIds, sysName + "::LowLevel"),
	kinematicsBase(setting["kinematics"]),
	gravity(setting["gravity_compensation"]),
	jvFilter(setting["joint_velocity_filter"]),
	toolPosition(),
	toolVelocity(),
	toolOrientation(),
	toolPose(),

	supervisoryController(),
	jtPassthrough(1.0),
	jpController(setting["joint_position_control"]),
	jvController1(setting["joint_velocity_control"][0]),
	jvController2(setting["joint_velocity_control"][1]),
	tpController(setting["tool_position_control"]),
	tf2jt(),
	toController(setting["tool_orientation_control"]),
	tt2jt(),

	tpoSplitter(),
	tpoTpController(setting["tool_position_control"]),
	tpoTf2jt(),
	tpoToController(setting["tool_orientation_control"]),
	tpoTt2jt(),
	tpoSum(),

	jtSum(true),

	input(jtSum.getInput(JT_INPUT)), jpOutput(llww.jpOutput), jvOutput(jvFilter.output),

	doneMoving(true),
	kin(setting["kinematics"])
{
	connect(llww.jpOutput, kinematicsBase.jpInput);
	connect(jvOutput, kinematicsBase.jvInput);

	connect(kinematicsBase.kinOutput, gravity.kinInput);
	// Don't connect gravity.output here. Gravity compensation is off by default.

	connect(kinematicsBase.kinOutput, toolPosition.kinInput);
	connect(kinematicsBase.kinOutput, toolVelocity.kinInput);
	connect(kinematicsBase.kinOutput, toolOrientation.kinInput);
	connect(kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(kinematicsBase.kinOutput, toController.kinInput);
	connect(kinematicsBase.kinOutput, tt2jt.kinInput);
	connect(kinematicsBase.kinOutput, tpoTf2jt.kinInput);
	connect(kinematicsBase.kinOutput, tpoToController.kinInput);
	connect(kinematicsBase.kinOutput, tpoTt2jt.kinInput);

	connect(toolPosition.output, toolPose.getInput<0>());
	connect(toolOrientation.output, toolPose.getInput<1>());

	connect(llww.jvOutput, jvFilter.input);
	if (em != NULL) {
		// Keep the jvFilter updated so it will provide accurate values for
		// calls to Wam::getJointVelocities().
		em->startManaging(jvFilter);
	}

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
	connect(toController.controlOutput, tt2jt.input);
	supervisoryController.registerConversion(makeIOConversion(
			toController.referenceInput, tt2jt.output));

	connect(tpoSplitter.getOutput<0>(), tpoTpController.referenceInput);
	connect(toolPosition.output, tpoTpController.feedbackInput);
	connect(tpoTpController.controlOutput, tpoTf2jt.input);
	connect(tpoTf2jt.output, tpoSum.getInput(0));

	connect(tpoSplitter.getOutput<1>(), tpoToController.referenceInput);
	connect(toolOrientation.output, tpoToController.feedbackInput);
	connect(tpoToController.controlOutput, tpoTt2jt.input);
	connect(tpoTt2jt.output, tpoSum.getInput(1));
	supervisoryController.registerConversion(makeIOConversion(
			tpoSplitter.input, tpoSum.output));

	connect(supervisoryController.output, jtSum.getInput(SC_INPUT));
	connect(jtSum.output, llww.input);
}

template<size_t DOF>
Wam<DOF>::~Wam()
{
	// Make sure any outstanding moveTo() threads are cleaned up
	mtThreadGroup.interrupt_all();
	mtThreadGroup.join_all();
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
	return getLowLevelWam().getHomePosition();
}

template<size_t DOF>
typename Wam<DOF>::jt_type Wam<DOF>::getJointTorques() const
{
	{
		BARRETT_SCOPED_LOCK(getEmMutex());
		if (llww.input.valueDefined()) {
			return llww.input.getValue();
		}
	}

	return jt_type();
}

template<size_t DOF>
inline typename Wam<DOF>::jp_type Wam<DOF>::getJointPositions() const
{
	return getLowLevelWam().getJointPositions();
}

template<size_t DOF>
inline typename Wam<DOF>::jv_type Wam<DOF>::getJointVelocities() const
{
	{
		BARRETT_SCOPED_LOCK(getEmMutex());

		// Return the filtered velocity, if available.
		if (jvController1.feedbackInput.valueDefined()) {
			return jvController1.feedbackInput.getValue();
		}
	}

	// Otherwise just return differentiated positions.
	return getLowLevelWam().getJointVelocities();
}

template<size_t DOF>
typename Wam<DOF>::cp_type Wam<DOF>::getToolPosition() const
{
	{
		BARRETT_SCOPED_LOCK(getEmMutex());
		if (tpController.feedbackInput.valueDefined()) {
			return tpController.feedbackInput.getValue();
		}
	}

	kin.eval(getJointPositions(), getJointVelocities());
	return cp_type(kin.impl->tool->origin_pos);
}

template<size_t DOF>
typename Wam<DOF>::cv_type Wam<DOF>::getToolVelocity() const
{
	kin.eval(getJointPositions(), getJointVelocities());
	return cv_type(kin.impl->tool_velocity);
}

template<size_t DOF>
Eigen::Quaterniond Wam<DOF>::getToolOrientation() const
{
	{
		BARRETT_SCOPED_LOCK(getEmMutex());
		if (toController.feedbackInput.valueDefined()) {
			return toController.feedbackInput.getValue();
		}
	}

	kin.eval(getJointPositions(), getJointVelocities());
	math::Matrix<3,3> rot(kin.impl->tool->rot_to_world);
	return Eigen::Quaterniond(rot.transpose());
}

template<size_t DOF>
inline typename Wam<DOF>::pose_type Wam<DOF>::getToolPose() const
{
	return boost::make_tuple(getToolPosition(), getToolOrientation());
}

template<size_t DOF>
inline math::Matrix<6,DOF> Wam<DOF>::getToolJacobian() const
{
	kin.eval(getJointPositions(), getJointVelocities());
	return math::Matrix<6,DOF>(kin.impl->tool_jacobian);
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
inline bool Wam<DOF>::isGravityCompensated()
{
	return jtSum.getInput(GRAVITY_INPUT).isConnected();
}

template<size_t DOF>
inline void Wam<DOF>::moveHome(bool blocking)
{
	moveTo(getHomePosition(), blocking);
}
template<size_t DOF>
inline void Wam<DOF>::moveHome(bool blocking, double velocity)
{
	moveTo(getHomePosition(), blocking, velocity);
}
template<size_t DOF>
inline void Wam<DOF>::moveHome(bool blocking, double velocity, double acceleration)
{
	moveTo(getHomePosition(), blocking, velocity, acceleration);
}

template<size_t DOF>
inline void Wam<DOF>::moveTo(const jp_type& destination, bool blocking, double velocity, double acceleration)
{
//	moveTo(currentPosHelper(getJointPositions()), getJointVelocities(), destination, blocking, velocity, acceleration);
	moveTo(currentPosHelper(getJointPositions()), /*jv_type(0.0),*/ destination, blocking, velocity, acceleration);
}

template<size_t DOF>
inline void Wam<DOF>::moveTo(const cp_type& destination, bool blocking, double velocity, double acceleration)
{
	moveTo(currentPosHelper(getToolPosition()), /*cv_type(0.0),*/ destination, blocking, velocity, acceleration);
}

template<size_t DOF>
inline void Wam<DOF>::moveTo(const Eigen::Quaterniond& destination, bool blocking, double velocity, double acceleration)
{
	moveTo(currentPosHelper(getToolOrientation()), destination, blocking, velocity, acceleration);
}

template<size_t DOF>
inline void Wam<DOF>::moveTo(const pose_type& destination, bool blocking, double velocity, double acceleration)
{
	moveTo(currentPosHelper(getToolPose()), destination, blocking, velocity, acceleration);
}

template<size_t DOF>
template<typename T>
void Wam<DOF>::moveTo(const T& currentPos, /*const typename T::unitless_type& currentVel,*/ const T& destination, bool blocking, double velocity, double acceleration)
{
	bool started = false;
	boost::promise<boost::thread*> threadPtrPromise;
	boost::shared_future<boost::thread*> threadPtrFuture(threadPtrPromise.get_future());
	boost::thread* threadPtr = new boost::thread(&Wam<DOF>::moveToThread<T>, this, boost::ref(currentPos), /*currentVel,*/ boost::ref(destination), velocity, acceleration, &started, threadPtrFuture);
	mtThreadGroup.add_thread(threadPtr);
	threadPtrPromise.set_value(threadPtr);
	

	// wait until move starts
	while ( !started ) {
		btsleep(0.001);
	}

	if (blocking) {
		while (!moveIsDone()) {
			btsleep(0.01);
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
T Wam<DOF>::currentPosHelper(const T& currentPos)
{
	System::Input<T>* input = NULL;
	supervisoryController.getInput(&input);
	if (input != NULL  &&  input->valueDefined()) {
		// If we're already using the controller we're about to use in the
		// moveTo() call, it's best to treat the current *set point* as the
		// "current position". That way the reference signal is continuous as it
		// transitions from whatever is happening now to the moveTo() Spline.
		return input->getValue();
	}

	// If we're not using the appropriate controller already, then use our
	// *actual* current position.
	return currentPos;
}

template<size_t DOF>
template<typename T>
void Wam<DOF>::moveToThread(const T& currentPos, /*const typename T::unitless_type& currentVel,*/ const T& destination, double velocity, double acceleration, bool* started, boost::shared_future<boost::thread*> threadPtrFuture)
{
	// Only remove this thread from mtThreadGroup on orderly exit. (Don't remove on exception.)
	bool removeThread = false;
	
	try {
		std::vector<T, Eigen::aligned_allocator<T> > vec;
		vec.push_back(currentPos);
		vec.push_back(destination);

		// TODO(dc): Use currentVel. Requires changes to math::spline<Eigen::Quaternion<T> > specialization.
		//math::Spline<T> spline(vec, currentVel);
		//math::TrapezoidalVelocityProfile profile(velocity, acceleration, currentVel.norm(), spline.changeInS());
		math::Spline<T> spline(vec);
		math::TrapezoidalVelocityProfile profile(velocity, acceleration, 0.0, spline.changeInS());

		Ramp time(NULL, 1.0);
		Callback<double, T> trajectory(boost::bind(boost::ref(spline), boost::bind(boost::ref(profile), _1)));

		connect(time.output, trajectory.input);
		trackReferenceSignal(trajectory.output);
		time.start();

		doneMoving = false;
		*started = true;

		// The value of trajectory.input will be undefined until the next execution cycle.
		// It may become undefined again if trajectory.output is disconnected and this chain
		// of Systems loses its ExecutionManager. We must check that the value is defined
		// before calling getValue().
		while ( !trajectory.input.valueDefined()  ||  trajectory.input.getValue() < profile.finalT()) {
			// if the move is interrupted, clean up and end the thread
			if ( !trajectory.output.isConnected() ||  boost::this_thread::interruption_requested() ) {
				removeThread = true;
				break;
			}
			btsleep(0.01);  // Interruption point. May throw boost::thread_interrupted
		}

		if ( !removeThread ) {
			doneMoving = true;

			// Wait until the trajectory is no longer referenced by supervisoryController
			while (trajectory.hasExecutionManager()  &&  !boost::this_thread::interruption_requested()) {
				btsleep(0.01);  // Interruption point. May throw boost::thread_interrupted
			}
			removeThread = true;
		}
	} catch (const boost::thread_interrupted& e) {}
	
	if (removeThread) {
		mtThreadGroup.remove_thread(threadPtrFuture.get());
		delete threadPtrFuture.get();
	}
}


}
}
