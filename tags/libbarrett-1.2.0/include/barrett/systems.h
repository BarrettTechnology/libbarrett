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

/** Includes all non-abstract members of the barrett::systems namespace.
 *
 * @file systems.h
 * @date Sep 4, 2009
 * @author Dan Cody
 * @see barrett::systems
 */

/** @namespace barrett::systems
 *
 * Provides chunks of functionality that can be composed to form a realtime
 * control loop.
 */


#ifndef BARRETT_SYSTEMS_H_
#define BARRETT_SYSTEMS_H_


#include <barrett/systems/wam.h>
#include <barrett/systems/low_level_wam_wrapper.h>

// other -- these operate on Systems, but are not Systems themselves
#include <barrett/systems/helpers.h>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/real_time_execution_manager.h>
#include <barrett/systems/converter.h>
#include <barrett/systems/io_conversion.h>

// sources
#include <barrett/systems/constant.h>
#include <barrett/systems/exposed_output.h>

#include <barrett/systems/ramp.h>

// sinks
#include <barrett/systems/print_to_stream.h>
#include <barrett/systems/periodic_data_logger.h>
#include <barrett/systems/triggered_data_logger.h>

// operators
#include <barrett/systems/kinematics_base.h>
#include <barrett/systems/inverse_dynamics.h>
#include <barrett/systems/gravity_compensator.h>
#include <barrett/systems/friction_compensator.h>
#include <barrett/systems/tool_position.h>
#include <barrett/systems/tool_orientation.h>
#include <barrett/systems/tool_force_to_joint_torques.h>
#include <barrett/systems/tool_torque_to_joint_torques.h>
#include <barrett/systems/tool_orientation_controller.h>

#include <barrett/systems/haptic_ball.h>
#include <barrett/systems/haptic_box.h>
#include <barrett/systems/haptic_path.h>

#include <barrett/systems/summer.h>
#include <barrett/systems/gain.h>

#include <barrett/systems/pid_controller.h>
#include <barrett/systems/first_order_filter.h>
#include <barrett/systems/rate_limiter.h>

#include <barrett/systems/callback.h>

#include <barrett/systems/array_splitter.h>
#include <barrett/systems/array_editor.h>
#include <barrett/systems/tuple_grouper.h>
#include <barrett/systems/tuple_splitter.h>


#endif /* BARRETT_SYSTEMS_H_ */
