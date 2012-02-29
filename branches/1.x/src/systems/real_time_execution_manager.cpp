/*
 * real_time_execution_manager.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: dc
 */

#include <stdexcept>
#include <limits>
#include <cmath>

#include <syslog.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>

#include <boost/bind.hpp>

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

	uint32_t period_us = rtem->period * 1e6;
    RTIME start;
    uint32_t duration;
	uint32_t min = std::numeric_limits<unsigned int>::max();
	uint32_t max = 0;
	uint64_t sum = 0;
	uint64_t sumSq = 0;
	uint32_t loopCount = 0;
	uint32_t missedCycleCount = 0;


	rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(rtem->period));

	rtem->running = true;
	try {
		while ( !rtem->stopRunning ) {
			rt_task_wait_period(NULL);


			start = rt_timer_read();

			rtem->runExecutionCycle();

            duration = (rt_timer_read() - start) / 1000;
			if (duration < min) {
				min = duration;
			}
			if (duration > max) {
				max = duration;
			}
			sum += duration;
			sumSq += duration * duration;
			++loopCount;
			if (duration > period_us) {
				++missedCycleCount;
			}
		}
	} catch (const ExecutionManagerException& e) {
		BARRETT_SCOPED_LOCK(rtem->getMutex());

		rtem->error = true;
		rtem->errorStr = e.what();

		if (rtem->errorCallback) {
			rtem->errorCallback(rtem, e);
		}
	}
	rtem->running = false;


	double mean = (double)sum / loopCount;
	double stdev = std::sqrt( ((double)sumSq/loopCount) - mean*mean );

    syslog(LOG_ERR, "RealTimeExecutionManager control-loop stats (microseconds):");
    syslog(LOG_ERR, "  num missed cycles = %u (%u cycles total)", missedCycleCount, loopCount);
    syslog(LOG_ERR, "  target period = %u", period_us);
    syslog(LOG_ERR, "  min = %u", min);
    syslog(LOG_ERR, "  ave = %.3f", mean);
    syslog(LOG_ERR, "  max = %u", max);
    syslog(LOG_ERR, "  stdev = %.3f", stdev);
}


}
}


RealTimeExecutionManager::RealTimeExecutionManager(double period_s, int rt_priority) :
	ExecutionManager(period_s),
	task(NULL), priority(rt_priority), running(false), stopRunning(false), error(false), errorStr(), errorCallback()
{
	init();
}

RealTimeExecutionManager::RealTimeExecutionManager(const libconfig::Setting& setting) :
	ExecutionManager(setting),
	task(NULL), priority(), running(false), stopRunning(false), error(false), errorStr(), errorCallback()
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
		stopRunning = false;
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

	stopRunning = true;
	rt_task_delete(task);
	delete task;
	task = NULL;
}

void RealTimeExecutionManager::setErrorCallback(callback_type callback)
{
	BARRETT_SCOPED_LOCK(getMutex());

	errorCallback = callback;
}

void RealTimeExecutionManager::clearErrorCallback()
{
	setErrorCallback(callback_type());
}

void RealTimeExecutionManager::init()
{
	// Avoids memory swapping for this program
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// install a more appropriate mutex
	delete mutex;
	mutex = new thread::RealTimeMutex;  // ~ExecutionManager() will delete this

	errorCallback = boost::bind(std::terminate);
}




}
}
