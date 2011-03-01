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
	if (hasDirectExecutionManager()) {
		getExecutionManager()->stopManaging(*this);
	}
}

void System::update(update_token_type updateToken)
{
	// Check if an update is needed
	if (hasExecutionManager()  &&  updateToken != ut) {
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


void System::setExecutionManager(ExecutionManager* newEm)
{
	if (hasExecutionManager()) {
		assert(getExecutionManager() == newEm);
	} else if (newEm != NULL) {
		em = newEm;

		child_input_list_type::iterator i(inputs.begin()), iEnd(inputs.end());
		for (; i != iEnd; ++i) {
			i->pushExecutionManager();
		}

		child_output_list_type::iterator o(outputs.begin()), oEnd(outputs.end());
		for (; o != oEnd; ++o) {
			o->pushExecutionManager();
		}
	}
}

void System::unsetDirectExecutionManager()
{
	emDirect = false;
	unsetExecutionManager();
}
void System::unsetExecutionManager()
{
	if (hasDirectExecutionManager()) {
		return;
	}

	child_output_list_type::const_iterator oc(outputs.begin()), ocEnd(outputs.end());
	for (; oc != ocEnd; ++oc) {
		em = oc->collectExecutionManager();

		if (hasExecutionManager()) {
			return;
		}
	}

	// if no EM found...
	child_input_list_type::iterator i(inputs.begin()), iEnd(inputs.end());
	for (; i != iEnd; ++i) {
		i->unsetExecutionManager();
	}

	child_output_list_type::iterator o(outputs.begin()), oEnd(outputs.end());
	for (; o != oEnd; ++o) {
		o->unsetExecutionManager();
	}
}


}
}
