/*
 * null_mutex.h
 *
 *  Created on: Dec 15, 2009
 *      Author: dc
 */

#ifndef NULL_MUTEX_H_
#define NULL_MUTEX_H_


#include "../detail/ca_macro.h"
#include "./abstract/mutex.h"


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

private:
	DISALLOW_COPY_AND_ASSIGN(NullMutex);
};


}
}


#endif /* NULL_MUTEX_H_ */
