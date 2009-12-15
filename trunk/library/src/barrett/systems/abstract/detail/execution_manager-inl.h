/*
 * execution_manager-inl.h
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */


#include <vector>

#include "../../../detail/stl_utils.h"
#include "../../../threading/abstract/mutex.h"
#include "../system.h"


namespace barrett {
namespace systems {


inline void ExecutionManager::startManaging(System* sys, bool alwaysUpdate) {
//	boost::lock_guard<threading::Mutex> lg(getMutex());

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
	sys->executionManager = NULL;
	replaceWithNull(managedSystems, sys);
	replaceWithNull(alwaysUpdatedSystems, sys);
}


inline void ExecutionManager::runExecutionCycle() {
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
void ExecutionManager::update(Container<System*> systems) {
	typename Container<System*>::const_iterator i;
	for (i = systems.begin(); i != systems.end(); ++i) {
		if (*i != NULL) {
			update(*i);
		}
	}
}
//	void update(std::vector<System*> systems) {
//		std::vector<System*>::const_iterator i;
//		for (i = systems.begin(); i != systems.end(); ++i) {
//			if (*i != NULL) {
//				update(*i);
//			}
//		}
//	}


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
