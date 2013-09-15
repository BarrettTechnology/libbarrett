/*
 * disable_secondary_mode_warning.cpp
 *
 *  Created on: Dec 14, 2010
 *      Author: dc
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
