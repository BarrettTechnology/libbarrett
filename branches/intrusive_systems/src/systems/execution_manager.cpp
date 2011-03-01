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

	if (sys.hasDirectExecutionManager()) {
		sys.getExecutionManager()->stopManaging(sys);
	}

	sys.setExecutionManager(this);
	sys.emDirect = true;
	managedSystems.push_back(sys);
}

// this ExecutionManager must be currently managing sys
void ExecutionManager::stopManaging(System& sys)
{
	BARRETT_SCOPED_LOCK(getMutex());

	assert(sys.hasDirectExecutionManager());
	assert(sys.getExecutionManager() == this);

	managedSystems.erase(ExecutionManager::managed_system_list_type::s_iterator_to(sys));
	sys.unsetDirectExecutionManager();
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
