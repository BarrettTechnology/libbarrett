/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */
/*
 * @file execution_manager.cpp
 * @date 12//09/2009
 * @author Dan Cody
 * 
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
