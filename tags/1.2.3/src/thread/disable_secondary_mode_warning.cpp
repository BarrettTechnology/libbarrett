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
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */
/*
 * @file disable_secondary_mode_warning.cpp
 * @date 12/14/2010
 * @author Dan Cody
 * 
 */

#ifdef BARRETT_XENOMAI
#include <native/task.h>
#endif

#include <barrett/thread/disable_secondary_mode_warning.h>

namespace barrett {
namespace thread {


DisableSecondaryModeWarning::DisableSecondaryModeWarning()
{
#ifdef BARRETT_XENOMAI
	int oldMode;
	rt_task_set_mode(T_WARNSW, 0, &oldMode);
	leaveWarnSwitchOn = oldMode & T_WARNSW;
#endif
}

DisableSecondaryModeWarning::~DisableSecondaryModeWarning()
{
#ifdef BARRETT_XENOMAI
	if (leaveWarnSwitchOn) {
		rt_task_set_mode(0, T_WARNSW, NULL);
	}
#endif
}


}
}
