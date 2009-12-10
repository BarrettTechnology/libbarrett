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


namespace barrett {
namespace systems {


class System;


class ExecutionManager {
public:
	ExecutionManager() :
		managedSystems(), updatedSystems() {}
	virtual ~ExecutionManager();

	virtual void startManaging(System* sys);
	virtual void stopManaging(System* sys);

protected:
	void resetExecutionCycle();

	template<template<typename T, typename = std::allocator<T> > class Container>
	void update(Container<System*> systems);
	void update(System* sys);
	virtual bool updateNeeded(System* sys);

	std::vector<System*> managedSystems;
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
