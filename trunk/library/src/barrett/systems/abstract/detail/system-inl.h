/*
 * system-inl.h
 *
 *  Created on: Nov 17, 2009
 *      Author: dc
 */


#include <vector>
#include "../execution_manager.h"


namespace barrett {
namespace systems {


// TODO(dc): test!


inline System::System() :
	inputs(), outputValues(), executionManager(NULL)
{
	setExecutionManager(System::defaultExecutionManager);
}

inline bool System::isExecutionManaged()
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
		newEm->startManaging(this);
	}
}


inline void System::update()
{
	if (inputsValid()) {
		operate();
	} else {
		invalidateOutputs();
	}
}


}
}
