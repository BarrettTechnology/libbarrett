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


// forward declarations from Xenomai's <native/mutex.h>
struct rt_mutex_placeholder;
typedef struct rt_mutex_placeholder RT_MUTEX;

typedef long long unsigned int RTIME;


namespace barrett {
namespace thread {


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
	int acquireWrapper(RTIME timeout);

	RT_MUTEX* mutex;
	int lockCount;
	bool leaveWarnSwitchOn;

private:
	DISALLOW_COPY_AND_ASSIGN(RealTimeMutex);
};


}
}


#endif /* BARRETT_THREAD_REAL_TIME_MUTEX_H_ */
