/** Defines barrett::Wam.
 *
 * @file barrett/wam.h
 * @date Sep 25, 2009
 * @author Dan Cody
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


#ifndef BARRETT_WAM_H_
#define BARRETT_WAM_H_


#include <libconfig.h++>

#include "./detail/ca_macro.h"
#include "./units.h"
#include "./systems/abstract/system.h"

#include "./systems/low_level_wam.h"
#include "./systems/converter.h"
#include "./systems/summer.h"

#include "./systems/kinematics_base.h"
#include "./systems/gravity_compensator.h"
#include "./systems/tool_position.h"
#include "./systems/tool_orientation.h"

#include "./systems/pid_controller.h"
#include "./systems/tool_orientation_controller.h"
#include "./systems/tool_force_to_joint_torques.h"


namespace barrett {


template<size_t DOF>
class Wam {
public:
	typedef typename units::JointTorques<DOF>::type jt_type;
	typedef typename units::JointPositions<DOF>::type jp_type;
	typedef typename units::JointVelocities<DOF>::type jv_type;


	// these need to be before the IO references
	systems::LowLevelWam<DOF> wam;
	systems::KinematicsBase<DOF> kinematicsBase;
	systems::GravityCompensator<DOF> gravity;
	systems::ToolPosition<DOF> toolPosition;
	systems::ToolOrientation<DOF> toolOrientation;

	systems::Converter<jt_type> supervisoryController;
	systems::PIDController<jp_type, jt_type> jpController;
	systems::PIDController<units::CartesianPosition::type, units::CartesianForce::type> tpController;
	systems::ToolForceToJointTorques<DOF> tf2jt;
	systems::ToolOrientationController<DOF> toController;

	systems::Summer<jt_type, 3> jtSum;
	enum {JT_INPUT = 0, GRAVITY_INPUT, SC_INPUT};


// IO
public:		systems::System::Input<jt_type>& input;
public:		systems::System::Output<jp_type>& jpOutput;
public:		systems::System::Output<jv_type>& jvOutput;


public:
	Wam(const libconfig::Setting& setting);
	virtual ~Wam();

	template<typename T>
	void trackReferenceSignal(systems::System::Output<T>& referenceSignal);  //NOLINT: non-const reference for syntax

	jp_type getJointPositions();
	jv_type getJointVelocities();

	void gravityCompensate(bool compensate = true);
	void moveHome(bool blocking = true, double velocity = 1.0, double acceleration = 1.0);
	void moveTo(const jp_type& destination, bool blocking = true, double velocity = 1.0, double acceleration = 1.0);
	template<typename T> void moveTo(const T& currentPos, const T& destination, bool blocking = true, double velocity = 1.0, double acceleration = 1.0);
	bool moveIsDone();
	void idle();

protected:
	template<typename T> void moveToThread(const T& currentPos, const T& destination, double velocity, double acceleration);

	bool doneMoving;

private:
	DISALLOW_COPY_AND_ASSIGN(Wam);
};


}


// include template definitions
#include "./detail/wam-inl.h"


#endif /* BARRETT_WAM_H_ */
