/*
 * real_time_execution_manager.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: dc
 */


#include <sys/mman.h>
#include <native/task.h>

#include "../abstract/execution_manager.h"
#include "../real_time_execution_manager.h"


namespace barrett {
namespace systems {


// TODO(dc): check return codes for errors!

namespace detail {
extern "C" {

void rtemEntry(void* cookie)
{
	RealTimeExecutionManager* rtem = reinterpret_cast<RealTimeExecutionManager*>(cookie);

	rt_task_set_periodic(NULL, TM_NOW, 1000000000);
	rt_task_set_mode(0, T_WARNSW, NULL);

	for (size_t i = 0; i < 5; ++i) {
		rt_task_wait_period(NULL);
		rtem->runExecutionCycle();
	}
}

}
}


RealTimeExecutionManager::RealTimeExecutionManager() :
	ExecutionManager(), task(NULL)
{
	// Avoids memory swapping for this program
	mlockall(MCL_CURRENT|MCL_FUTURE);

	task = new RT_TASK;
	rt_task_create(task, "libbarrett::RealTimeExecutionManager", 0, 90, T_JOINABLE);
}

RealTimeExecutionManager::~RealTimeExecutionManager()
{
	rt_task_join(task);
	rt_task_delete(task);
	delete task;
	task = NULL;
}

void RealTimeExecutionManager::start() {
	rt_task_start(task, &detail::rtemEntry, reinterpret_cast<void*>(this));
}


}
}
