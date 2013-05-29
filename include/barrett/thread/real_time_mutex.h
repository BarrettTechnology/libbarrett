/*
 * real_time_mutex.h
 *
 *  Created on: Dec 15, 2009
 *      Author: dc
 */

#ifndef BARRETT_THREAD_REAL_TIME_MUTEX_H_
#define BARRETT_THREAD_REAL_TIME_MUTEX_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>


namespace barrett {
namespace thread {


// Forward declaration
namespace detail {
struct mutex_impl;
}


class RealTimeMutex : public Mutex {
public:
	RealTimeMutex();
	virtual ~RealTimeMutex();

	virtual void lock();
	virtual bool try_lock();
	virtual void unlock();

	virtual int fullUnlock();
	virtual void relock(int lc);

protected:
	int acquireWrapper(bool blocking);

	detail::mutex_impl* mutex;
	int lockCount;
	bool leaveWarnSwitchOn;

private:
	DISALLOW_COPY_AND_ASSIGN(RealTimeMutex);
};


}
}


#endif /* BARRETT_THREAD_REAL_TIME_MUTEX_H_ */
