/*
 * system.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */


#include <vector>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


ExecutionManager* System::defaultExecutionManager = NULL;

// TODO(dc): test!


System::~System()
{
	setExecutionManager(NULL);

	for (std::vector<AbstractInput*>::const_iterator i = inputs.begin();
		 i != inputs.end();
		 ++i) {
		if (*i != NULL) {
			(*i)->disconnectFromParentSystem();
		}
	}

	for (std::vector<AbstractOutput::AbstractValue*>::const_iterator i = outputValues.begin();
		 i != outputValues.end();
		 ++i) {
		if ((*i) != NULL) {
			(*i)->disconnectFromParentSystem();
		}
	}
}


bool System::inputsValid()
{
	std::vector<AbstractInput*>::const_iterator i;
	for (i = inputs.begin(); i != inputs.end(); ++i) {
		if ( (*i) != NULL  &&  !(*i)->valueDefined() ) {
			return false;
		}
	}

	return true;
}

void System::invalidateOutputs()
{
	std::vector<AbstractOutput::AbstractValue*>::const_iterator i;
	for (i = outputValues.begin(); i != outputValues.end(); ++i) {
		if ((*i) != NULL) {
			(*i)->setValueUndefined();
		}
	}
}


}
}
