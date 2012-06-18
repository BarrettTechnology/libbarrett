/*
	Copyright 2009, 2010, 2011, 2012 Barrett Technology <support@barrett.com>

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
 * real_time_execution_manager.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: dc
 */

#include <stdexcept>
#include <string>
#include <limits>
#include <cmath>

#include <errno.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include <barrett/os.h>
#include <barrett/thread/real_time_mutex.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/real_time_execution_manager.h>


namespace barrett {
namespace systems {


// TODO(dc): test!


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
	uint32_t overruns = 0;
	uint32_t missedReleasePoints = 0;
	unsigned long newMissedReleasePoints = 0;

	int ret;

	ret = rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(rtem->period));
	if (ret != 0) {
		logMessage("%s: rt_task_set_periodic(): (%d) %s") %__func__ %-ret %strerror(-ret);
		exit(2);
	}

	rtem->running = true;
	try {
		while ( !rtem->stopRunning ) {
			ret = rt_task_wait_period(&newMissedReleasePoints);
			if (ret != 0  &&  ret != -ETIMEDOUT) {  // ETIMEDOUT means that we missed a release point
				logMessage("%s: rt_task_wait_period(): (%d) %s") %__func__ %-ret %strerror(-ret);
				exit(2);
			}
			missedReleasePoints += newMissedReleasePoints;
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
				++overruns;
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

    logMessage("RealTimeExecutionManager control-loop stats (microseconds):");
    logMessage("  target period = %u") %period_us;
    logMessage("  min = %u") %min;
    logMessage("  ave = %.3f") %mean;
    logMessage("  max = %u") %max;
    logMessage("  stdev = %.3f") %stdev;
    logMessage("  num total cycles = %u") %loopCount;
    logMessage("  num missed release points = %u") %missedReleasePoints;
    logMessage("  num overruns = %u") %overruns;
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

		std::string name = "RTEM: " + boost::lexical_cast<std::string>(this);
		int ret = 0;
		ret = rt_task_create(task, name.c_str(), 0, priority, T_JOINABLE);
		if (ret != 0) {
			(logMessage("systems::RealTimeExecutionManager::%s: Couldn't start the realtime task.  %s: rt_task_create(): (%d) %s") %__func__ %__func__ %-ret %strerror(-ret)).raise<std::runtime_error>();
		}

		ret = rt_task_start(task, &detail::rtemEntryPoint, reinterpret_cast<void*>(this));
		if (ret != 0) {
			(logMessage("systems::RealTimeExecutionManager::%s:  Couldn't start the realtime task.  %s: rt_task_start(): (%d) %s\n") %__func__  %__func__ %-ret %strerror(-ret)).raise<std::runtime_error>();
		}

		// block until the thread starts reporting its new state
		while ( !isRunning() ) {
			ret = rt_task_sleep(detail::secondsToRTIME(period / 10.0));
			if (ret != 0) {
				(logMessage("systems::RealTimeExecutionManager::%s:  Couldn't start the realtime task.  %s: rt_task_sleep(): (%d) %s\n") %__func__  %__func__ %-ret %strerror(-ret)).raise<std::runtime_error>();
			}
		}
	}
	// TODO(dc): else, throw an exception?
}

void RealTimeExecutionManager::stop()
{
	stopRunning = true;

	// According to Xenomai docs, rt_task_join() also performs the rt_task_delete() behaviors.
	int ret = rt_task_join(task);
	if (ret != 0) {
		(logMessage("systems::RealTimeExecutionManager::%s: Couldn't stop the realtime task.  %s: rt_task_join(): (%d) %s") %__func__ %__func__ %-ret %strerror(-ret)).raise<std::runtime_error>();
	}

	delete task;
	task = NULL;
}

void RealTimeExecutionManager::clearError()
{
	BARRETT_SCOPED_LOCK(getMutex());

	error = false;
	errorStr = "";

	stopRunning = true;
	int ret = rt_task_delete(task);
	if (ret != 0) {
		(logMessage("systems::RealTimeExecutionManager::%s: Couldn't delete the realtime task. %s: rt_task_delete(): (%d) %s\n") % __func__ % __func__ % -ret %strerror(-ret)).raise<std::runtime_error>();
	}
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
