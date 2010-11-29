/*
 * real_time_mutex.cpp
 *
 *  Created on: Dec 15, 2009
 *      Author: dc
 */

#include <stdexcept>
#include <syslog.h>

#include <native/task.h>
#include <native/mutex.h>

#include <barrett/thread/real_time_mutex.h>


namespace barrett {
namespace thread {


RealTimeMutex::RealTimeMutex() :
	mutex(NULL)
{
	mutex = new RT_MUTEX;
	int ret = rt_mutex_create(mutex, NULL);
	if (ret) {
		syslog(LOG_ERR, "Could not create RT_MUTEX: (%d) %s", -ret, strerror(-ret));
		throw std::logic_error("thread::RealTimeMutex::RealTimeMutex(): Could not create RT_MUTEX.");
	}
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
    int ret = acquireWrapper(mutex, TM_INFINITE);
    if (ret != 0) {
            syslog(LOG_ERR, "RealTimeMutex::lock(): acquireWrapper() returned %d", ret);
            throw boost::thread_resource_error(ret);
    }
}

bool RealTimeMutex::try_lock()
{
	return acquireWrapper(mutex, TM_NONBLOCK) == 0;
}

void RealTimeMutex::unlock()
{
	int ret = rt_mutex_release(mutex);
	if (ret) {
		syslog(LOG_ERR, "Could not release RT_MUTEX: (%d) %s", -ret, strerror(-ret));
		throw std::logic_error("thread::RealTimeMutex::unlock(): Could not release RT_MUTEX.");
	}
}

int RealTimeMutex::acquireWrapper(RT_MUTEX* m, RTIME timeout)
{
	int ret = rt_mutex_acquire(m, timeout);
	if (ret == -EPERM) {
		// become real-time, then try again

		// Allocate a new RT_TASK struct, and then forget the pointer. This
		// leak allows us to avoid ownership issues for the RT_TASK, which
		// shouldn't necessarily be deleted when the mutex is released, or when
		// the mutex is deleted, etc.. It is a small overhead that happens (at
		// most) once per thread. If needed, we can always get the pointer back
		// by calling rt_task_self().
		rt_task_shadow(new RT_TASK, NULL, 10, 0);

		ret = rt_mutex_acquire(m, timeout);
	}

	return ret;
}


}
}
