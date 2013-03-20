/*
 * mutex.h
 *
 *  Created on: Dec 15, 2009
 *      Author: dc
 */

#ifndef BARRETT_THREAD_ABSTRACT_MUTEX_H_
#define BARRETT_THREAD_ABSTRACT_MUTEX_H_


#include <boost/thread.hpp>
#include <barrett/detail/ca_macro.h>


#define BARRETT_SCOPED_LOCK(mutex)  \
	::boost::lock_guard< ::barrett::thread::Mutex> _barrett_scoped_lock_lg(mutex)


namespace barrett {
namespace thread {


// An abstract, recursive mutex object. Is a model of the boost::thread::Lockable concept.
class Mutex {
public:
	Mutex() {}
	virtual ~Mutex() {}

	virtual void lock() = 0;
	virtual bool try_lock() = 0;
	virtual void unlock() = 0;

	virtual int fullUnlock() = 0;
	virtual void relock(int lc) = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(Mutex);
};


}
}


#endif /* BARRETT_THREAD_ABSTRACT_MUTEX_H_ */
