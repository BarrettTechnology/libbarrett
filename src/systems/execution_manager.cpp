/*
 * execution_manager.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */


#include <vector>
#include <algorithm>

#include <barrett/thread/abstract/mutex.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/execution_manager.h>


namespace barrett {
namespace systems {


ExecutionManager::~ExecutionManager() {
	if (System::defaultExecutionManager == this) {
		System::defaultExecutionManager = NULL;
	}

	std::vector<System*>::const_iterator i;
	for (i = managedSystems.begin(); i != managedSystems.end(); ++i) {
		if (*i != NULL) {
			(*i)->setExecutionManager(NULL);
		}
	}

	delete mutex;
}


}
}
