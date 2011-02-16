/*
 * real_time_execution_manager.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: dc
 */

#include <stdexcept>

#include <syslog.h>
#include <sys/mman.h>
#include <native/task.h>

#include <barrett/thread/real_time_mutex.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/real_time_execution_manager.h>


namespace barrett {
namespace systems {


// TODO(dc): test!
// TODO(dc): check return codes for errors!


namespace detail {


inline RTIME secondsToRTIME(double s) {
	return static_cast<RTIME>(s * 1e9);
}


extern "C" {

void rtemEntryPoint(void* cookie)
{
	RealTimeExecutionManager* rtem = reinterpret_cast<RealTimeExecutionManager*>(cookie);

	rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(rtem->period));

	rtem->running = true;
	try {
		while ( !rtem->stopRunning ) {
			rt_task_wait_period(NULL);
			rtem->runExecutionCycle();
		}
	} catch (const ExecutionManagerException& e) {
		BARRETT_SCOPED_LOCK(rtem->getMutex());

		rtem->error = true;
		rtem->errorStr = e.what();
	}
	rtem->running = false;
}


}
}


RealTimeExecutionManager::RealTimeExecutionManager(double period_s, int rt_priority) :
	ExecutionManager(period_s),
	task(NULL), priority(rt_priority), running(false), stopRunning(false), error(false), errorStr()
{
	init();
}

RealTimeExecutionManager::RealTimeExecutionManager(const libconfig::Setting& setting) :
	ExecutionManager(setting),
	task(NULL), priority(), running(false), stopRunning(false), error(false), errorStr()
{
	priority = setting["thread_priority"];
	init();
}


RealTimeExecutionManager::~RealTimeExecutionManager()
{
	if (isRunning()) {
		stop();
	}
}


void RealTimeExecutionManager::start()
{
	BARRETT_SCOPED_LOCK(getMutex());

	if (getError()) {
		throw std::logic_error("systems::RealTimeExecutionManager::start(): Cannot start while in an error state. Call RealTimeExecutionManager::clearError() first.");
	}

	if ( !isRunning() ) {
		task = new RT_TASK;
		rt_task_create(task, "RTEM", 0, priority, T_JOINABLE);
		rt_task_start(task, &detail::rtemEntryPoint, reinterpret_cast<void*>(this));

		// block until the thread starts reporting its new state
		while ( !isRunning() ) {
			rt_task_sleep(detail::secondsToRTIME(period / 10.0));
		}
	}
	// TODO(dc): else, throw an exception?
}

void RealTimeExecutionManager::stop()
{
	stopRunning = true;
	rt_task_join(task);

	rt_task_delete(task);
	delete task;
	task = NULL;
}

void RealTimeExecutionManager::clearError()
{
	BARRETT_SCOPED_LOCK(getMutex());

	error = false;
	errorStr = "";
}

void RealTimeExecutionManager::init()
{
	// Avoids memory swapping for this program
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// install a more appropriate mutex
	delete mutex;
	mutex = new thread::RealTimeMutex;  // ~ExecutionManager() will delete this

	clearError();
}




}
}
