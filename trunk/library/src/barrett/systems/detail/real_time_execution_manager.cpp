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


// TODO(dc): test!
// TODO(dc): check return codes for errors!


namespace detail {
extern "C" {

void rtemEntryPoint(void* cookie)
{
	RealTimeExecutionManager* rtem = reinterpret_cast<RealTimeExecutionManager*>(cookie);

	rt_task_set_periodic(NULL, TM_NOW, rtem->period);
//	rt_task_set_mode(0, T_WARNSW, NULL);

	while (rtem->running) {
		rt_task_wait_period(NULL);
		rtem->runExecutionCycle();
	}
}

}
}


RealTimeExecutionManager::RealTimeExecutionManager(unsigned long period_ns) :
	ExecutionManager(), task(NULL), period(period_ns), running(false)
{
	// Avoids memory swapping for this program
	mlockall(MCL_CURRENT|MCL_FUTURE);
}

RealTimeExecutionManager::~RealTimeExecutionManager()
{
	if (isRunning()) {
		stop();
	}
}


void RealTimeExecutionManager::start() {
	task = new RT_TASK;
	rt_task_create(task, "RTEM", 0, 90, T_JOINABLE);

	running = true;
	rt_task_start(task, &detail::rtemEntryPoint, reinterpret_cast<void*>(this));
}

bool RealTimeExecutionManager::isRunning() {
	return running;
}

void RealTimeExecutionManager::stop() {
	running = false;
	rt_task_join(task);

	rt_task_delete(task);
	delete task;
	task = NULL;
}


}
}
