/*
 * null_mutex.h
 *
 *  Created on: Dec 15, 2009
 *      Author: dc
 */

#ifndef BARRETT_THREAD_NULL_MUTEX_H_
#define BARRETT_THREAD_NULL_MUTEX_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>


namespace barrett {
namespace thread {


// provides no synchronization or mutual exclusion
class NullMutex : public Mutex {
public:
	// since all NullMutex's are the same, feel free to give out references to this object if you just need to return a NullMutex...
	static NullMutex aNullMutex;

	NullMutex() {}
	virtual ~NullMutex() {}

	virtual void lock() {}
	virtual bool try_lock() {  return true;  }
	virtual void unlock() {}

	virtual int fullUnlock() { return 0; }
	virtual void relock(int lc) {}

private:
	DISALLOW_COPY_AND_ASSIGN(NullMutex);
};


}
}


#endif /* BARRETT_THREAD_NULL_MUTEX_H_ */
