/*
 * real_time_mutex.cpp
 *
 *  Created on: Dec 15, 2009
 *      Author: dc
 */

#include <iostream>
#include <stdexcept>

#include <syslog.h>
#include <signal.h>

#include <native/task.h>
#include <native/mutex.h>

#include <barrett/detail/stacktrace.h>
#include <barrett/thread/real_time_mutex.h>


namespace barrett {
namespace thread {

namespace detail {
extern "C" {

void warnOnSwitchToSecondaryMode(int)
{
	syslog(LOG_ERR, "WARNING: Switched out of RealTime. Stack-trace:");
	barrett::detail::syslog_stacktrace();
	std::cerr << "WARNING: Switched out of RealTime. Stack-trace in syslog.\n";
}


}
}


RealTimeMutex::RealTimeMutex() :
	mutex(NULL), lockCount(0), leaveWarnSwitchOn(false)
{
	mutex = new RT_MUTEX;
	int ret = rt_mutex_create(mutex, NULL);
	if (ret) {
		syslog(LOG_ERR, "Could not create RT_MUTEX: (%d) %s", -ret, strerror(-ret));
		throw std::logic_error("thread::RealTimeMutex::RealTimeMutex(): Could not create RT_MUTEX.");
	}

	// Handler for warnings about falling out of real time mode
	signal(SIGXCPU, &detail::warnOnSwitchToSecondaryMode);
}

RealTimeMutex::~RealTimeMutex()
{
	int ret = rt_mutex_delete(mutex);
	delete mutex;
	mutex = NULL;

	if (ret) {
		syslog(LOG_ERR, "Could not delete RT_MUTEX: (%d) %s", -ret, strerror(-ret));
		throw std::logic_error("thread::RealTimeMutex::~RealTimeMutex(): Could not delete RT_MUTEX.");
	}
}

void RealTimeMutex::lock()
{
    int ret = acquireWrapper(TM_INFINITE);
    if (ret != 0) {
            syslog(LOG_ERR, "RealTimeMutex::lock(): acquireWrapper() returned %d", ret);
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
	if (ret) {
		syslog(LOG_ERR, "Could not release RT_MUTEX: (%d) %s", -ret, strerror(-ret));
		throw std::logic_error("thread::RealTimeMutex::unlock(): Could not release RT_MUTEX.");
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
		syslog(LOG_ERR, "thread::RealTimeMutex::fullUnlock() called when lockCount = %d", lc);
		throw std::logic_error("thread::RealTimeMutex::fullUnlock(): Bad lockCount value.");
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
