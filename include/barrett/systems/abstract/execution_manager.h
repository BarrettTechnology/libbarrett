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
 * execution_manager.h
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_ABSTRACT_EXECUTION_MANAGER_H_
#define BARRETT_SYSTEMS_ABSTRACT_EXECUTION_MANAGER_H_


#include <vector>
#include <stdexcept>
#include <string>

#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/detail/libconfig_utils.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/thread/null_mutex.h>


namespace barrett {
namespace systems {


class System;


// Thrown to indicate that the ExecutionManager should stop executing.
class ExecutionManagerException : public std::runtime_error {
public:
	explicit ExecutionManagerException(const std::string& str) : std::runtime_error(str) {}
};


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
		period = barrett::detail::numericToDouble(setting["control_loop_period"]);
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
#include <barrett/systems/abstract/detail/execution_manager-inl.h>


#endif /* BARRETT_SYSTEMS_ABSTRACT_EXECUTION_MANAGER_H_ */
