/*
 * system-inl.h
 *
 *  Created on: Nov 17, 2009
 *      Author: dc
 */


#include <vector>

#include <barrett/thread/null_mutex.h>
#include <barrett/systems/abstract/execution_manager.h>


namespace barrett {
namespace systems {


// TODO(dc): test!


inline System::System(bool updateEveryExecutionCycle) :
	inputs(), outputValues(),
	executionManager(NULL), alwaysUpdate(updateEveryExecutionCycle)
{
	setExecutionManager(System::defaultExecutionManager);
}

inline bool System::isExecutionManaged() const
{
	return executionManager != NULL;
}

// newEm can be NULL
inline void System::setExecutionManager(ExecutionManager* newEm) {
	if (newEm == NULL) {
		if (isExecutionManaged()) {
			executionManager->stopManaging(this);
		}
	} else {
		newEm->startManaging(this, updateEveryExecutionCycle());
	}
}

inline ExecutionManager* System::getExecutionManager() const
{
	return executionManager;
}

inline thread::Mutex& System::getEmMutex()
{
	if (isExecutionManaged()) {
		return executionManager->getMutex();
	} else {
		return thread::NullMutex::aNullMutex;
	}
}


inline void System::update()
{
	if (!isExecutionManaged()  ||  executionManager->updateNeeded(this)) {
		if (inputsValid()) {
			operate();
		} else {
			invalidateOutputs();
		}
	}
}

inline bool System::updateEveryExecutionCycle() {
	return alwaysUpdate;
}


}
}
