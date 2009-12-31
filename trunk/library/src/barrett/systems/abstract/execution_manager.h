/*
 * execution_manager.h
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */

#ifndef EXECUTION_MANAGER_H_
#define EXECUTION_MANAGER_H_


#include <vector>

#include "../../detail/ca_macro.h"
#include "../../thread/abstract/mutex.h"
#include "../../thread/null_mutex.h"


namespace barrett {
namespace systems {


class System;

// TODO(dc): prevent Systems managed by different EMs from being connected

// this isn't technically abstract, but neither does it have all the elements of a useful interface...
class ExecutionManager {
public:
	ExecutionManager() :
		mutex(new thread::NullMutex), managedSystems(), alwaysUpdatedSystems(), updatedSystems() {}
	virtual ~ExecutionManager();

	virtual void startManaging(System* sys, bool alwaysUpdate = false);
	virtual void stopManaging(System* sys);

	virtual thread::Mutex& getMutex();

protected:
	void runExecutionCycle();

	void resetExecutionCycle();
	void update();
	template<template<typename T, typename = std::allocator<T> > class Container>
		void update(Container<System*> systems);
	void update(System* sys);
	virtual bool updateNeeded(System* sys);

	thread::Mutex* mutex;

	std::vector<System*> managedSystems;
	std::vector<System*> alwaysUpdatedSystems;
	std::vector<System*> updatedSystems;

private:
	friend class System;

	DISALLOW_COPY_AND_ASSIGN(ExecutionManager);
};


}
}


// include template definitions
#include "./detail/execution_manager-inl.h"


#endif /* EXECUTION_MANAGER_H_ */
