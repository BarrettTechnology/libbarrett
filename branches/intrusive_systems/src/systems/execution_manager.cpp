/*
 * execution_manager.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */

#include <cassert>

#include <barrett/thread/abstract/mutex.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/execution_manager.h>


namespace barrett {
namespace systems {


ExecutionManager::~ExecutionManager()
{
	{
		BARRETT_SCOPED_LOCK(getMutex());
		managedSystems.clear_and_dispose(System::StopManagingDisposer());
	}

	delete mutex;
}

void ExecutionManager::startManaging(System& sys)
{
	BARRETT_SCOPED_LOCK(getMutex());

	if (sys.hasExecutionManager()) {
		sys.getExecutionManager()->stopManaging(sys);
	}
	managedSystems.push_back(sys);
	sys.em = this;
}

// this ExecutionManager must be currently managing sys, otherwise data is corrupted :(
void ExecutionManager::stopManaging(System& sys)
{
	BARRETT_SCOPED_LOCK(getMutex());

	assert(sys.getExecutionManager() == this);

	sys.em = NULL;
	managedSystems.erase(ExecutionManager::managed_system_list_type::s_iterator_to(sys));
}

void ExecutionManager::runExecutionCycle() {
	BARRETT_SCOPED_LOCK(getMutex());

	++ut;

	managed_system_list_type::iterator i(managedSystems.begin()), iEnd(managedSystems.end());
	for (; i != iEnd; ++i) {
		i->update(ut);
	}
}


}
}
