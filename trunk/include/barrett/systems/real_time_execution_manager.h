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
 * real_time_execution_manager.h
 *
 *  Created on: Dec 10, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_REAL_TIME_EXECUTION_MANAGER_H_
#define BARRETT_SYSTEMS_REAL_TIME_EXECUTION_MANAGER_H_


#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/execution_manager.h>


// forward declarations from Xenomai's <native/task.h>
struct rt_task_placeholder;
typedef struct rt_task_placeholder RT_TASK;

//typedef long long unsigned int RTIME;


namespace barrett {
namespace systems {


namespace detail {
extern "C" {

void rtemEntryPoint(void* cookie);

}
}


// TODO(dc): add a configuration file interface

class RealTimeExecutionManager : public ExecutionManager {
public:
	explicit RealTimeExecutionManager(double period_s, int rt_priority = 50);
	explicit RealTimeExecutionManager(const libconfig::Setting& setting);  //TODO(dc): test!
	virtual ~RealTimeExecutionManager();

	void start();
	bool isRunning() const { return running; }
	void stop();

protected:
	RT_TASK* task;
	int priority;
	bool running, stopRunning;

private:
	void init();

	friend void detail::rtemEntryPoint(void* cookie);

	DISALLOW_COPY_AND_ASSIGN(RealTimeExecutionManager);
};


}
}


#endif /* BARRETT_SYSTEMS_REAL_TIME_EXECUTION_MANAGER_H_ */
