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


namespace detail {
class mutex_impl;  // OS-dependent implementation
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
	detail::mutex_impl* mutex;
	int lockCount;

private:
	DISALLOW_COPY_AND_ASSIGN(RealTimeMutex);
};


}
}


#endif /* BARRETT_THREAD_REAL_TIME_MUTEX_H_ */
