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
 * manual_execution_manager.h
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_MANUAL_EXECUTION_MANAGER_H_
#define BARRETT_SYSTEMS_MANUAL_EXECUTION_MANAGER_H_


#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/execution_manager.h>


namespace barrett {
namespace systems {


class ManualExecutionManager : public ExecutionManager {
public:
	ManualExecutionManager() {}
	explicit ManualExecutionManager(double period_s) :
		ExecutionManager(period_s) {}
	explicit ManualExecutionManager(const libconfig::Setting& setting) :
		ExecutionManager(setting) {}

	~ManualExecutionManager() {}

	using ExecutionManager::runExecutionCycle;

private:
	DISALLOW_COPY_AND_ASSIGN(ManualExecutionManager);
};


}
}


#endif /* BARRETT_SYSTEMS_MANUAL_EXECUTION_MANAGER_H_ */
