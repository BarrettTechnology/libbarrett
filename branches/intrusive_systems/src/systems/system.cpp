/*
 * system.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */


#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


System::~System() {
	if (hasExecutionManager()) {
		getExecutionManager()->stopManaging(*this);
	}
}

void System::update(update_token_type updateToken)
{
	// Check if an update is needed
	if (updateToken == UT_NULL  ||  updateToken != ut) {
		ut = updateToken;
	} else {
		return;
	}

	if (inputsValid()) {
		operate();
	} else {
		invalidateOutputs();
	}
}

bool System::inputsValid() const
{
	child_input_list_type::const_iterator i(inputs.begin()), iEnd(inputs.end());
	for (; i != iEnd; ++i) {
		if ( !i->valueDefined() ) {
			return false;
		}
	}
	return true;
}

void System::invalidateOutputs()
{
	child_output_list_type::iterator i(outputs.begin()), iEnd(outputs.end());
	for (; i != iEnd; ++i) {
		i->setValueUndefined();
	}
}


}
}
