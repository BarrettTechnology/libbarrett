/** Master header for the barrett::systems library.\ Includes all non-abstract
 * systems.
 *
 * @file systems.h
 * @date Sep 4, 2009
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


#ifndef BARRETT_SYSTEMS_H_
#define BARRETT_SYSTEMS_H_


// other -- these operate on Systems, but are not Systems themselves
#include "systems/helpers.h"
#include "systems/manual_execution_manager.h"
#include "systems/real_time_execution_manager.h"
#include "systems/converter.h"
#include "systems/io_conversion.h"

// sources
#include "systems/constant.h"
#include "systems/exposed_output.h"

#include "systems/ramp.h"

// sinks
#include "systems/print_to_stream.h"
#include "systems/data_logger.h"

// operators
#include "systems/summer.h"
#include "systems/gain.h"

#include "systems/pid_controller.h"
#include "systems/first_order_filter.h"

#include "systems/callback.h"

#include "systems/array_splitter.h"
#include "systems/array_editor.h"
#include "systems/tuple_grouper.h"


#endif /* BARRETT_SYSTEMS_H_ */
