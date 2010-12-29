/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

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
	BARRETT_SCOPED_LOCK(getMutex());

	if (sys->isExecutionManaged()) {
		sys->executionManager->stopManaging(sys);
	}
	managedSystems.push_back(sys);
	sys->executionManager = this;

	if (alwaysUpdate) {
		alwaysUpdatedSystems.push_back(sys);
	}

	// Prevent a possible malloc() in ExecutionManager::updateNeeded()
	updatedSystems.reserve(managedSystems.size());
}

// this ExecutionManager must be currently managing sys, otherwise data is corrupted :(
inline void ExecutionManager::stopManaging(System* sys) {
	BARRETT_SCOPED_LOCK(getMutex());

	sys->executionManager = NULL;

	detail::replaceWithNull(managedSystems, sys);
	detail::replaceWithNull(alwaysUpdatedSystems, sys);
}


inline void ExecutionManager::runExecutionCycle() {
	BARRETT_SCOPED_LOCK(getMutex());

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
