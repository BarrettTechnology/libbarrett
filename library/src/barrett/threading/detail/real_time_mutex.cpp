/*
 * real_time_mutex.cpp
 *
 *  Created on: Dec 15, 2009
 *      Author: dc
 */


#include <native/task.h>
#include <native/mutex.h>

#include "../real_time_mutex.h"


namespace barrett {
namespace threading {


// TODO(dc): check return codes for errors!


RealTimeMutex::RealTimeMutex() :
	mutex(NULL)
{
	mutex = new RT_MUTEX;
	rt_mutex_create(mutex, NULL);
}

RealTimeMutex::~RealTimeMutex()
{
	rt_mutex_delete(mutex);
	delete mutex;
	mutex = NULL;
}

void RealTimeMutex::lock()
{
	acquireWrapper(mutex, TM_INFINITE);
}

bool RealTimeMutex::try_lock()
{
	return acquireWrapper(mutex, TM_NONBLOCK) == 0;
}

void RealTimeMutex::unlock()
{
	rt_mutex_release(mutex);
}

int RealTimeMutex::acquireWrapper(RT_MUTEX* m, RTIME timeout)
{
	int err;

	err = rt_mutex_acquire(m, timeout);
	if (err == -EPERM) {
		// become real-time, then try again

		// Allocate a new RT_TASK struct, and then forget the pointer. This
		// leak allows us to avoid ownership issues for the RT_TASK, which
		// shouldn't necessarily be deleted when the mutex is released, or when
		// the mutex is deleted, etc.. It is a small overhead that happens (at
		// most) once per thread. If needed, we can always get the pointer back
		// by calling rt_task_self().
		rt_task_shadow(new RT_TASK, NULL, 10, 0);

		err = rt_mutex_acquire(m, timeout);
	}

	return err;
}


}
}
