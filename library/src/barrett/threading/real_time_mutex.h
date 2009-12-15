/*
 * real_time_mutex.h
 *
 *  Created on: Dec 15, 2009
 *      Author: dc
 */

#ifndef REAL_TIME_MUTEX_H_
#define REAL_TIME_MUTEX_H_


#include "../detail/ca_macro.h"
#include "./abstract/mutex.h"


// forward declarations from Xenomai's <native/mutex.h>
struct rt_mutex_placeholder;
typedef struct rt_mutex_placeholder RT_MUTEX;

typedef long long unsigned int RTIME;


namespace barrett {
namespace threading {


class RealTimeMutex : public Mutex {
public:
	RealTimeMutex();
	virtual ~RealTimeMutex();

	virtual void lock();
	virtual bool try_lock();
	virtual void unlock();

protected:
	static int acquireWrapper(RT_MUTEX* m, RTIME timeout);

	RT_MUTEX* mutex;

private:
	DISALLOW_COPY_AND_ASSIGN(RealTimeMutex);
};


}
}


#endif /* REAL_TIME_MUTEX_H_ */
