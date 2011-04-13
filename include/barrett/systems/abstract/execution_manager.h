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
 * execution_manager.h
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */


// Include system.h before the include-guard to resolve include order dependency.
#include <barrett/systems/abstract/system.h>


#ifndef BARRETT_SYSTEMS_ABSTRACT_EXECUTION_MANAGER_H_
#define BARRETT_SYSTEMS_ABSTRACT_EXECUTION_MANAGER_H_


#include <boost/intrusive/list.hpp>

#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/detail/libconfig_utils.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/thread/null_mutex.h>


namespace barrett {
namespace systems {


// Thrown to indicate that the ExecutionManager should stop executing.
class ExecutionManagerException : public std::runtime_error {
public:
	explicit ExecutionManagerException(const std::string& str) : std::runtime_error(str) {}
};



// this isn't technically abstract, but neither does it have all the elements of a useful interface...
class ExecutionManager {
public:
	explicit ExecutionManager(double period_s = -1.0) :
		mutex(new thread::NullMutex), period(period_s), ut(System::UT_NULL) {}
	explicit ExecutionManager(const libconfig::Setting& setting) :
		mutex(new thread::NullMutex), period(), ut(System::UT_NULL)
	{
		period = barrett::detail::numericToDouble(setting["control_loop_period"]);
	}
	~ExecutionManager();

	void startManaging(System& sys);
	void stopManaging(System& sys);

	thread::Mutex& getMutex() const { return *mutex; }
	double getPeriod() const {  return period;  }

protected:
	void runExecutionCycle();

	thread::Mutex* mutex;
	double period;
	System::update_token_type ut;

private:
	typedef boost::intrusive::list<System, boost::intrusive::member_hook<System, System::managed_hook_type, &System::managedHook> > managed_system_list_type;
	managed_system_list_type managedSystems;

	DISALLOW_COPY_AND_ASSIGN(ExecutionManager);
};


}
}


#endif /* BARRETT_SYSTEMS_ABSTRACT_EXECUTION_MANAGER_H_ */
