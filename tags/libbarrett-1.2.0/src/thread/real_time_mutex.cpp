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
 * real_time_mutex.cpp
 *
 *  Created on: Dec 15, 2009
 *      Author: dc
 */

#include <iostream>
#include <stdexcept>

#include <signal.h>

#include <native/task.h>
#include <native/mutex.h>

#include <barrett/os.h>
#include <barrett/detail/stacktrace.h>
#include <barrett/thread/real_time_mutex.h>


namespace barrett {
namespace thread {

namespace detail {
extern "C" {

void warnOnSwitchToSecondaryMode(int)
{
	logMessage("WARNING: Switched out of RealTime. Stack-trace:",true);
	barrett::detail::syslog_stacktrace();
}


}
}


RealTimeMutex::RealTimeMutex() :
	mutex(NULL), lockCount(0), leaveWarnSwitchOn(false)
{
	mutex = new RT_MUTEX;
	int ret = rt_mutex_create(mutex, NULL);
	if (ret != 0) {
		(logMessage("thread::RealTimeMutex::%s:  Could not create RT_MUTEX: (%d) %s") %__func__ %-ret %strerror(-ret)).raise<std::logic_error>();
	}

	// Handler for warnings about falling out of real time mode
	signal(SIGXCPU, &detail::warnOnSwitchToSecondaryMode);
}

RealTimeMutex::~RealTimeMutex()
{
	int ret = rt_mutex_delete(mutex);
	delete mutex;
	mutex = NULL;

	if (ret != 0) {
		(logMessage("thread::RealTimeMutex::%s:  Could not delete RT_MUTEX: (%d) %s") %__func__ %-ret %strerror(-ret)).raise<std::logic_error>();
	}
}

void RealTimeMutex::lock()
{
    int ret = acquireWrapper(TM_INFINITE);
    if (ret != 0) {
	logMessage("RealTimeMutex::lock(): %s returned %d") %__func__ % ret;
	throw boost::thread_resource_error(ret);
    }
}

bool RealTimeMutex::try_lock()
{
	return acquireWrapper(TM_NONBLOCK) == 0;
}

void RealTimeMutex::unlock()
{
	--lockCount;
	bool changeMode = lockCount == 0 && !leaveWarnSwitchOn;

	int ret = rt_mutex_release(mutex);
	if (ret != 0) {
		(logMessage("thread::RealTimeMutex::%s:  Could not release RT_MUTEX: (%d) %s") %__func__ %-ret %strerror(-ret)).raise<std::logic_error>();
	}

	if (changeMode) {
		ret = rt_task_set_mode(T_WARNSW, 0, NULL);
		if (ret != 0) {
			throw std::runtime_error("thread::RealTimeMutex::unlock(): Could not clear T_WARNSW mode.");
		}
	}
}

int RealTimeMutex::fullUnlock()
{
	int lc = lockCount;
	if (lc <= 0) {
		(logMessage("thread::RealTimeMutex::%s Bad lockCount value.  lockCount = %d") %__func__ %lc).raise<std::logic_error>();
	}

	while (lockCount > 1) {
		unlock();
	}
	unlock();

	return lc;
}

void RealTimeMutex::relock(int lc)
{
	lock();
	while (lockCount != lc) {
		lock();
	}
}


int RealTimeMutex::acquireWrapper(RTIME timeout)
{
	int ret;

	ret = rt_mutex_acquire(mutex, timeout);
	if (ret == -EPERM) {
		// become real-time, then try again

		// Allocate a new RT_TASK struct, and then forget the pointer. This
		// leak allows us to avoid ownership issues for the RT_TASK, which
		// shouldn't necessarily be deleted when the mutex is released, or when
		// the mutex is deleted, etc.. It is a small overhead that happens (at
		// most) once per thread. If needed, we can always get the pointer back
		// by calling rt_task_self().
		rt_task_shadow(new RT_TASK, NULL, 10, 0);

		ret = rt_mutex_acquire(mutex, timeout);
	}
	if (ret != 0) {
		return ret;
	}

	if (lockCount == 0) {
		int oldMode;
		ret = rt_task_set_mode(0, T_WARNSW, &oldMode);
		if (ret != 0) {
			throw std::runtime_error("thread::RealTimeMutex::acquireWrapper(): Could not set T_WARNSW mode.");
		}
		leaveWarnSwitchOn = oldMode & T_WARNSW;
	}
	++lockCount;

	return ret;
}


}
}
