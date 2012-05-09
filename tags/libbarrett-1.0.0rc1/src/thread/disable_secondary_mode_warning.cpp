/*
 * disable_secondary_mode_warning.cpp
 *
 *  Created on: Dec 14, 2010
 *      Author: dc
 */

#include <native/task.h>

#include <barrett/thread/disable_secondary_mode_warning.h>

namespace barrett {
namespace thread {


DisableSecondaryModeWarning::DisableSecondaryModeWarning()
{
	int oldMode;
	rt_task_set_mode(T_WARNSW, 0, &oldMode);
	leaveWarnSwitchOn = oldMode & T_WARNSW;
}

DisableSecondaryModeWarning::~DisableSecondaryModeWarning()
{
	if (leaveWarnSwitchOn) {
		rt_task_set_mode(0, T_WARNSW, NULL);
	}
}


}
}
