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


#include <libconfig.h>

#include "../units.h"
#include "../systems/abstract/system.h"
#include "../systems/io_conversion.h"


namespace barrett {

// TODO(dc): some of these members should be inline

template<size_t DOF>
Wam<DOF>::Wam(const libconfig::Setting& setting) :
	wam(setting["low_level"]),
	kinematicsBase(setting["kinematics"]),
	gravity(setting["gravity_compensation"]),
	toolPosition(),
	toolOrientation(),

	supervisoryController(),
	jpController(setting["joint_position_control"]),
	tpController(setting["tool_position_control"]),
	tf2jt(),
	toController(),

	jtSum(true),

	input(jtSum.getInput(JT_INPUT)), jpOutput(wam.jpOutput), jvOutput(wam.jvOutput)
{
	using systems::connect;
	using systems::makeIOConversion;

	connect(wam.jpOutput, kinematicsBase.jpInput);
	connect(wam.jvOutput, kinematicsBase.jvInput);

	connect(kinematicsBase.kinOutput, gravity.kinInput);
	// Don't connect gravity.output here. Gravity compensation is off by default.

	connect(kinematicsBase.kinOutput, toolPosition.kinInput);
	connect(kinematicsBase.kinOutput, toolOrientation.kinInput);
	connect(kinematicsBase.kinOutput, tf2jt.kinInput);
	connect(kinematicsBase.kinOutput, toController.kinInput);

	connect(wam.jpOutput, jpController.feedbackInput);
	supervisoryController.registerConversion(makeIOConversion(
			jpController.referenceInput, jpController.controlOutput));

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
typename units::JointPositions<DOF>::type Wam<DOF>::getJointPositions()
{
	return kinematicsBase.jpInput.getValue();
}

template<size_t DOF>
typename units::JointVelocities<DOF>::type Wam<DOF>::getJointVelocities()
{
	return kinematicsBase.jvInput.getValue();
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
void Wam<DOF>::moveHome()
{
}

template<size_t DOF>
bool Wam<DOF>::moveIsDone()
{
	// TODO(dc): stub
	return false;
}

template<size_t DOF>
void Wam<DOF>::idle()
{
	supervisoryController.disconnectInput();
}


}
