/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

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

inline thread::Mutex& System::getEmMutex() const
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
