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
 * @file system.cpp
 * @date 12/09/2009
 * @author Dan Cody
 * 
 */


#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


void System::mandatoryCleanUp()
{
	BARRETT_SCOPED_LOCK(getEmMutex());

	if (hasDirectExecutionManager()) {
		getExecutionManager()->stopManaging(*this);
	}

	while ( !outputs.empty() ) {
		outputs.back().mandatoryCleanUp();
	}
	while ( !inputs.empty() ) {
		inputs.back().mandatoryCleanUp();
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

bool System::inputsValid()
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
	if (newEm != NULL) {
		if (hasExecutionManager()) {
			assert(getExecutionManager() == newEm);
		} else {
			em = newEm;
			onExecutionManagerChanged();

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

#ifndef NDEBUG
	// This variable is only used in the assert() below.
	ExecutionManager* oldEm = getExecutionManager();
#endif

	em = NULL;  // If there are no outputs, we can't collect an EM
	child_output_list_type::const_iterator oc(outputs.begin()), ocEnd(outputs.end());
	for (; oc != ocEnd; ++oc) {
		em = oc->collectExecutionManager();

		if (hasExecutionManager()) {
			assert(oldEm == getExecutionManager());
			return;
		}
	}

	// if no EM found...
	onExecutionManagerChanged();

	child_input_list_type::iterator i(inputs.begin()), iEnd(inputs.end());
	for (; i != iEnd; ++i) {
		i->unsetExecutionManager();
	}

	child_output_list_type::iterator o(outputs.begin()), oEnd(outputs.end());
	for (; o != oEnd; ++o) {
		o->unsetExecutionManager();
	}
}


System::AbstractInput::AbstractInput(System* parent) : parentSys(parent)
{
	assert(parentSys != NULL);

	BARRETT_SCOPED_LOCK(getEmMutex());
	parentSys->inputs.push_back(*this);
}
System::AbstractInput::~AbstractInput()
{
	if (parentSys != NULL) {
		mandatoryCleanUp();
	}
}

void System::AbstractInput::mandatoryCleanUp()
{
	assert(parentSys != NULL);

	BARRETT_SCOPED_LOCK(getEmMutex());
	parentSys->inputs.erase(System::child_input_list_type::s_iterator_to(*this));
	parentSys = NULL;
}


System::AbstractOutput::AbstractOutput(System* parent) : parentSys(parent)
{
	assert(parentSys != NULL);

	BARRETT_SCOPED_LOCK(getEmMutex());
	parentSys->outputs.push_back(*this);
}
System::AbstractOutput::~AbstractOutput()
{
	if (parentSys != NULL) {
		mandatoryCleanUp();
	}
}

void System::AbstractOutput::mandatoryCleanUp()
{
	assert(parentSys != NULL);

	BARRETT_SCOPED_LOCK(getEmMutex());
	parentSys->outputs.erase(System::child_output_list_type::s_iterator_to(*this));
	parentSys = NULL;
}


}
}
