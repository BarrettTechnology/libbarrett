/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */

/** Defines systems::Wam.
 *
 * @file systems/wam.h
 * @date 09/25/2009
 * @author Dan Cody
 */


#ifndef BARRETT_SYSTEMS_WAM_H_
#define BARRETT_SYSTEMS_WAM_H_


#include <vector>

#include <boost/thread.hpp>
#include <Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/products/low_level_wam.h>
#include <barrett/products/puck.h>
#include <barrett/products/safety_module.h>
#include <barrett/math/kinematics.h>

#include <barrett/systems/low_level_wam_wrapper.h>
#include <barrett/systems/first_order_filter.h>
#include <barrett/systems/converter.h>
#include <barrett/systems/summer.h>
#include <barrett/systems/gain.h>
#include <barrett/systems/tuple_grouper.h>
#include <barrett/systems/tuple_splitter.h>

#include <barrett/systems/kinematics_base.h>
#include <barrett/systems/gravity_compensator.h>
#include <barrett/systems/tool_position.h>
#include <barrett/systems/tool_velocity.h>
#include <barrett/systems/tool_orientation.h>

#include <barrett/systems/pid_controller.h>
#include <barrett/systems/tool_orientation_controller.h>
#include <barrett/systems/tool_force_to_joint_torques.h>
#include <barrett/systems/tool_torque_to_joint_torques.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class Wam {
public:
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	// these need to be before the IO references
	LowLevelWamWrapper<DOF> llww;
	KinematicsBase<DOF> kinematicsBase;
	GravityCompensator<DOF> gravity;
	FirstOrderFilter<jv_type> jvFilter;
	ToolPosition<DOF> toolPosition;
	ToolVelocity<DOF> toolVelocity;
	ToolOrientation<DOF> toolOrientation;
	TupleGrouper<cp_type, Eigen::Quaterniond> toolPose;

	Converter<jt_type> supervisoryController;
	Gain<jt_type, double> jtPassthrough;
	PIDController<jp_type, jt_type> jpController;
	PIDController<jv_type, jt_type> jvController1;
	FirstOrderFilter<jt_type> jvController2;
	PIDController<cp_type, cf_type> tpController;
	ToolForceToJointTorques<DOF> tf2jt;
	ToolOrientationController<DOF> toController;
	ToolTorqueToJointTorques<DOF> tt2jt;

	// tool orientation + tool position control
	TupleSplitter<cp_type, Eigen::Quaterniond> tpoSplitter;
	PIDController<cp_type, cf_type> tpoTpController;
	ToolForceToJointTorques<DOF> tpoTf2jt;
	ToolOrientationController<DOF> tpoToController;
	ToolTorqueToJointTorques<DOF> tpoTt2jt;
	Summer<jt_type, 2> tpoSum;

	Summer<jt_type, 3> jtSum;
	enum {JT_INPUT = 0, GRAVITY_INPUT, SC_INPUT};


/** Input/Output Interface definitions available to developers.
 *
 *  Joint Torques are passed into the WAM, while providing the output joint positions and joint vectors. 
 */
public:		System::Input<jt_type>& input;
public:		System::Output<jp_type>& jpOutput;
public:		System::Output<jv_type>& jvOutput;


public:
   /** Constructor for Wam
    *	
    *	GenericPucks must be ordered by joint and must break into torque groups as arranged.
    */
	Wam(ExecutionManager* em, const std::vector<Puck*>& genericPucks,
			SafetyModule* safetyModule, const libconfig::Setting& setting,
			std::vector<int> torqueGroupIds = std::vector<int>(),
			const std::string& sysName = "Wam");
	/** Destructor for Wam
	 */
	~Wam();
	/** trackReferenceSignal() used for following updating input. (any barrett input units for control)
     */
	template<typename T>
	void trackReferenceSignal(System::Output<T>& referenceSignal);  //NOLINT: non-const reference for syntax
	/** getHomePosition() returns home postion of individual joints in Radians
     */
	const jp_type& getHomePosition() const;
	/** getJointTorques() returns joint torques in Newtons per meter
     */
	jt_type getJointTorques() const;
	/** getJointPositions() returns joint Position values in Radians
     */
	jp_type getJointPositions() const;
	/** getJointVelocities() returns joint velocity values in meters per second
     */
	jv_type getJointVelocities() const;
	/** getToolPosition() returns Tool Position in Cartesian Space meters
     */
	cp_type getToolPosition() const;
	/** getToolVelocity() returns Tool velocity in meters per second
     */
	cv_type getToolVelocity() const;
	/** getToolOrientation() returns Tool Orientation in Quaternions
     */
	Eigen::Quaterniond getToolOrientation() const;
	/** getToolPose() returns Tool Pose as a combination of 
     */
	pose_type getToolPose() const;
    /** getToolJacobian() returns matrix of first order partial derivatives.
     */
	math::Matrix<6,DOF> getToolJacobian() const;

    /** gravityCompensate() method activates Gravity Compensation for WAM
     */
	void gravityCompensate(bool compensate = true);
    /** updateGravity() method updates the value of Gravity applied for WAM
     */
	bool updateGravity(double val = -9.8);
    /** isGravityCompensated() returns flag as to the status of Gravity Compensation
     */
	bool isGravityCompensated();
    /** moveHome(bool blocking, double velocity, double acceleration) method sends WAM to stored Home Position 
     *	
     *	Blocking Allows/Disallows other functions to happen while Movement is under way
     *	Velocity Speed value in m/s
     *	Acceleration Acceleration value in m/s^2
	 *   Function is overloaded to allow multiple types of calls.
     */
	void moveHome(bool blocking = true);
	void moveHome(bool blocking, double velocity);
	void moveHome(bool blocking, double velocity, double acceleration);
    /** moveTo() method sends WAM to desired point based on input type.
     *	
 	 *  Function is overloaded to allow moves any type of Barrett Unit Types input.
     *	Destination Based on type cartesian(xyz), joint(radians), quaternoind(orientation), pose
     *	blocking Determines whether program should wait for move to finish before continuing
     *	velocity Speed at which to move
     *	acceleration value in radians per second
     */
	void moveTo(const jp_type& destination, bool blocking = true, double velocity = 0.5, double acceleration = 0.5);
	void moveTo(const cp_type& destination, bool blocking = true, double velocity = 0.1, double acceleration = 0.2);
	void moveTo(const Eigen::Quaterniond& destination, bool blocking = true, double velocity = 0.5, double acceleration = 0.5);
	void moveTo(const pose_type& destination, bool blocking = true, double velocity = 0.1, double acceleration = 0.2);
	template<typename T> void moveTo(const T& currentPos, /*const typename T::unitless_type& currentVel,*/ const T& destination, bool blocking, double velocity, double acceleration);
	/** moveIsDone() method returns false while the trajectory controller for the most recent moveTo() command is still active. 
	 *
	 *  Only useful if the moveTo() is non-blocking. 
	 */
	bool moveIsDone() const;
	/** idle() method Terminates the position controller (if active). 
	 *
	 * To prevent uncontrolled falling, you should ensure you have set gravityCompensate(true) before calling idle().
	 */
	void idle();

	/** getEmMutex() method allows access to ExecutionManagers Mutex in LowLevelWam Class.
	 */
	thread::Mutex& getEmMutex() const { return llww.getEmMutex(); }
	/** getLowLevelWam() method allows access to methods in LowLevelWam Class.
	 */
	LowLevelWam<DOF>& getLowLevelWam() { return llww.getLowLevelWam(); }
	const LowLevelWam<DOF>& getLowLevelWam() const { return llww.getLowLevelWam(); }

protected:
	template<typename T> T currentPosHelper(const T& currentPos);
	template<typename T> void moveToThread(const T& currentPos, const T& destination, double velocity, double acceleration, bool* started, boost::shared_future<boost::thread*> threadPtrFuture);

	bool doneMoving;
	boost::thread_group mtThreadGroup;

	// Used to calculate TP and TO if the values aren't already being calculated in the control loop.
	mutable math::Kinematics<DOF> kin;

private:
	DISALLOW_COPY_AND_ASSIGN(Wam);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


// include template definitions
#include <barrett/systems/detail/wam-inl.h>


#endif /* BARRETT_SYSTEMS_WAM_H_ */
