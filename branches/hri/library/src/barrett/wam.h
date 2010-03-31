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
	typedef units::JointTorques<DOF> jt_type;
	typedef units::JointPositions<DOF> jp_type;
	typedef units::JointVelocities<DOF> jv_type;


	// these need to be before the IO references
	systems::LowLevelWam<DOF> wam;
	systems::KinematicsBase<DOF> kinematicsBase;
	systems::GravityCompensator<DOF> gravity;
	systems::ToolPosition<DOF> toolPosition;
	systems::ToolOrientation<DOF> toolOrientation;

	systems::Converter<jt_type> supervisoryController;
	systems::PIDController<jp_type> jpController;
	systems::PIDController<units::CartesianPosition> tpController;
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
	void moveTo(jp_type point); // TODO(dc): make this generic
	void moveHome();
	bool moveIsDone();
	void idle();

private:
	DISALLOW_COPY_AND_ASSIGN(Wam);
};


}


// include template definitions
#include "./detail/wam-inl.h"


#endif /* BARRETT_WAM_H_ */