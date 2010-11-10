/*
 * real_time_execution_manager.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: dc
 */


#include <iostream>

#include <syslog.h>
#include <signal.h>
#include <sys/mman.h>
#include <native/task.h>

#include <barrett/detail/stacktrace.h>
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
	if (rtem->warnOnSwitch) {
		rt_task_set_mode(0, T_WARNSW, NULL);
	}

	rtem->running = true;
	while ( !rtem->stopRunning ) {
		rt_task_wait_period(NULL);
		rtem->runExecutionCycle();
	}
	rtem->running = false;

	rt_task_set_mode(T_WARNSW, 0, NULL);
}

void warnOnSwitchToSecondaryMode(int)
{
	syslog(LOG_ERR, "WARNING: Switched out of RealTime. Stack-trace:");
	barrett::detail::syslog_stacktrace();
	std::cerr << "WARNING: Switched out of RealTime. Stack-trace in syslog.\n";
}


}
}


RealTimeExecutionManager::RealTimeExecutionManager(double period_s, bool warnOnSwitchToSecondaryMode, int rt_priority) :
	ExecutionManager(period_s),
	task(NULL), priority(rt_priority), warnOnSwitch(warnOnSwitchToSecondaryMode), running(false), stopRunning(false)
{
	init();
}

RealTimeExecutionManager::RealTimeExecutionManager(const libconfig::Setting& setting) :
	ExecutionManager(setting),
	task(NULL), priority(), warnOnSwitch(), running(false), stopRunning(false)
{
	warnOnSwitch = setting["warn_on_switch_to_secondary_mode"];
	priority = setting["thread_priority"];
	init();
}


RealTimeExecutionManager::~RealTimeExecutionManager()
{
	if (isRunning()) {
		stop();
	}
}


void RealTimeExecutionManager::start() {
	BARRETT_SCOPED_LOCK(getMutex());

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

bool RealTimeExecutionManager::isRunning() {
	return running;
}

void RealTimeExecutionManager::stop() {
	stopRunning = true;
	rt_task_join(task);

	rt_task_delete(task);
	delete task;
	task = NULL;
}


void RealTimeExecutionManager::init()
{
	// Avoids memory swapping for this program
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// handler for warnings about falling out of real time mode
	signal(SIGXCPU, &detail::warnOnSwitchToSecondaryMode);

	// install a more appropriate mutex
	delete mutex;
	mutex = new thread::RealTimeMutex;  // ~ExecutionManager() will delete this
}




}
}
