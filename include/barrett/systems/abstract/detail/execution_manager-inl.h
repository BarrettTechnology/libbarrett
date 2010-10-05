/*
 * execution_manager-inl.h
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */


#include <vector>

#include <barrett/detail/stl_utils.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


inline thread::Mutex& ExecutionManager::getMutex()
{
	return *mutex;
}

inline void ExecutionManager::startManaging(System* sys, bool alwaysUpdate) {
	SCOPED_LOCK(getMutex());

	if (sys->isExecutionManaged()) {
		sys->executionManager->stopManaging(sys);
	}
	managedSystems.push_back(sys);
	sys->executionManager = this;

	if (alwaysUpdate) {
		alwaysUpdatedSystems.push_back(sys);
	}
}

// this ExecutionManager must be currently managing sys, otherwise data is corrupted :(
inline void ExecutionManager::stopManaging(System* sys) {
	SCOPED_LOCK(getMutex());

	sys->executionManager = NULL;

	detail::replaceWithNull(managedSystems, sys);
	detail::replaceWithNull(alwaysUpdatedSystems, sys);
}


inline void ExecutionManager::runExecutionCycle() {
	SCOPED_LOCK(getMutex());

	resetExecutionCycle();
	update();
}


inline void ExecutionManager::resetExecutionCycle() {
	updatedSystems.clear();
}

inline void ExecutionManager::update() {
	update(alwaysUpdatedSystems);
}

template<template<typename T, typename = std::allocator<T> > class Container>
void ExecutionManager::update(const Container<System*>& systems) {
	typename Container<System*>::const_iterator i;
	for (i = systems.begin(); i != systems.end(); ++i) {
		if (*i != NULL) {
			update(*i);
		}
	}
}

inline void ExecutionManager::update(System* sys) {
	sys->update();
}

inline bool ExecutionManager::updateNeeded(System* sys) {
	if (std::find(updatedSystems.begin(), updatedSystems.end(), sys) == updatedSystems.end()) {
		updatedSystems.push_back(sys);
		return true;
	}
	return false;
}


}
}
