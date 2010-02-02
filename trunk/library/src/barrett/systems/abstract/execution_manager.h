/*
 * execution_manager.h
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */

#ifndef EXECUTION_MANAGER_H_
#define EXECUTION_MANAGER_H_


#include <vector>
#include <libconfig.h++>

#include "../../detail/ca_macro.h"
#include "../../detail/libconfig_utils.h"
#include "../../thread/abstract/mutex.h"
#include "../../thread/null_mutex.h"


namespace barrett {
namespace systems {


class System;

// TODO(dc): prevent Systems managed by different EMs from being connected

// this isn't technically abstract, but neither does it have all the elements of a useful interface...
class ExecutionManager {
public:
	explicit ExecutionManager(double period_s) :
		mutex(new thread::NullMutex), period(period_s),
		managedSystems(), alwaysUpdatedSystems(), updatedSystems() {}
	explicit ExecutionManager(const libconfig::Setting& setting) :
		mutex(new thread::NullMutex), period(),
		managedSystems(), alwaysUpdatedSystems(), updatedSystems()
	{
		period = detail::numericToDouble(setting["control_loop_period"]);
	}
	virtual ~ExecutionManager();

	virtual void startManaging(System* sys, bool alwaysUpdate = false);
	virtual void stopManaging(System* sys);

	virtual thread::Mutex& getMutex();
	virtual double getPeriod() const {  return period;  }

protected:
	void runExecutionCycle();

	void resetExecutionCycle();
	void update();
	template<template<typename T, typename = std::allocator<T> > class Container>
		void update(const Container<System*>& systems);
	void update(System* sys);
	virtual bool updateNeeded(System* sys);

	thread::Mutex* mutex;

	double period;

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
