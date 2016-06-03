/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 */

/**
 * @file os.cpp
 * @date 03/28/2012
 * @author Dan Cody
 *         Brad Millman (modified for real time pre-empt (May 2016)
 * 
 */


#include <stdexcept>
#include <iostream>
#include <cassert>

#include <syslog.h>
#include <signal.h>
#include <sys/mman.h>

#ifdef BARRETT_XENOMAI
#include <native/task.h>
#include <native/timer.h>
#else
//
//  special stuff for RT-Preempt
//
#include <time.h>
#include <sched.h>
#include <errno.h>
#endif

#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include <barrett/detail/stacktrace.h>
#include <barrett/detail/os.h>
#include <barrett/os.h>

//
//  something for the time calculations....
//
#define NANO_SEC_PER_SECOND (1000000000)                           //



#ifdef BARRETT_XENOMAI
// Xenomai helper function
inline RTIME secondsToRTIME(double s) {
	return static_cast<RTIME>(s * 1e9);
}

// Xenomai requires at least one call to mlockall() per process. Also, a signal
// handler is installed to trap transitions from primary execution mode to
// secondary execution mode; this aids in identifying code that breaks Xenomai's
// realtime guarantee.
namespace {  // Using an anonymous namespace because no other code needs to
			 // interact with these declarations. It is necessary to construct a
			 // single instance of InitXenomai.
	extern "C" {
	void warnOnSwitchToSecondaryMode(int)
	{
		barrett::logMessage("WARNING: Switched out of RealTime. Stack-trace:", true);
		barrett::detail::syslog_stacktrace();
	}
	}

	class InitXenomai {
	public:
		InitXenomai() {
			// Avoids memory swapping for this program
			mlockall(MCL_CURRENT|MCL_FUTURE);

			// Handler for warnings about falling out of primary mode
			signal(SIGXCPU, &warnOnSwitchToSecondaryMode);
		}
	};
	// Static variables are initialized when the module is loaded. This causes the
	// InitXenomai::InitXenomai() ctor to be called at module load time.
	static InitXenomai ignore;
}
#endif


namespace barrett {

//
//  here we intend to study the precision of the
//  call to sleep....hoping to understand the
//  shakey behaviour of the arm
//
//  all times are in nano-seconds
//
int                 max_delay_deviation;
int                 min_delay_deviation;
long long           total_delay_deviation;
unsigned int        loop_count;
unsigned long long  total_delay;
//
//  this function does not seem to be thread safe....
//  TODO(bim) either fix this or figure out if isn't used
//  from two threads
//
void btsleep(double duration_s)
{
  static bool     first_time_called = true;
  struct timespec start_of_sleep;
  struct timespec end_of_sleep;
  unsigned int    delta_time;
  int             deviation_time;


  if (first_time_called) {
    //
    //  if this is the first time in the call....
    //  ... init all variables
    max_delay_deviation   = 0;
    min_delay_deviation   = NANO_SEC_PER_SECOND;
    total_delay_deviation = 0;
    loop_count            = 0;
    total_delay           = 0;
  }
  
  
  assert(duration_s > 1e-6);  // Minimum duration is 1 us
  
  clock_gettime (CLOCK_MONOTONIC,  &start_of_sleep);
  boost::this_thread::sleep(boost::posix_time::microseconds(long(duration_s * 1e6)));
  clock_gettime (CLOCK_MONOTONIC,  &end_of_sleep);

  //
  // are we in the same second?
  //
  if ( start_of_sleep.tv_sec  == end_of_sleep.tv_sec) {
    //
    // yes, within the same second
    //
    delta_time = end_of_sleep.tv_nsec - start_of_sleep.tv_nsec;
  } else {
    //
    //
    //
    delta_time = (end_of_sleep.tv_nsec + NANO_SEC_PER_SECOND) - start_of_sleep.tv_nsec;
  }
  deviation_time = delta_time  - (unsigned long long)(duration_s * 1e9);
  total_delay    = total_delay + (unsigned long long)(duration_s * 1e9);

  //
  // now that we have delta time....
  // do some time accounting
  //
  total_delay_deviation = total_delay_deviation + deviation_time;
  loop_count++;
    
  if (deviation_time > max_delay_deviation) {
    max_delay_deviation = deviation_time;
  }
  if (deviation_time < min_delay_deviation) {
    min_delay_deviation = deviation_time;
  }

  //
  //  now print out some info...every 10 seconds....
  //  this is ugly...we should do this at the end of the program
  //  that will be the next step
  //

  //
  //  set the first_time_called variable
  //

  first_time_called = false;
}

void btPrintSleepStats (void) {

  printf ("Printing Sleep Statistics (all times in nano seconds)\n");
  if (0 != loop_count) {
    //
    // if loop count is zero...doing a divide would be bad
    //
    printf ("loop %d, average delay was %lld\n", loop_count, (total_delay / loop_count));
    printf ("average delay deviation %lld\n", (total_delay_deviation / loop_count));
    printf ("max loop time deviation %d, min loop time deviation %d \n", max_delay_deviation, min_delay_deviation);
  } else {
    printf ("loop count is zero....no statistics\n");
  }
}

void btsleepRT(double duration_s)
{
#ifdef BARRETT_XENOMAI
	assert(duration_s > 1e-9);  // Minimum duration is 1 ns
	int ret = rt_task_sleep(RTIME(duration_s * 1e9));
	if (ret != 0) {
		(logMessage("%s: rt_task_sleep() returned error %d.") % __func__ % ret).raise<std::runtime_error>();
	}
#else
	btsleep(duration_s);
#endif
}

void btsleep(double duration_s, bool realtime)
{
	if (realtime) {
		btsleepRT(duration_s);
	} else {
		btsleep(duration_s);
	}
}

#ifndef BARRETT_XENOMAI
// Record the time program execution began
const boost::posix_time::ptime START_OF_PROGRAM_TIME = boost::posix_time::microsec_clock::local_time();
#endif
double highResolutionSystemTime()
{
#ifdef BARRETT_XENOMAI
	return 1e-9 * rt_timer_read();
#else
	return (boost::posix_time::microsec_clock::local_time() - START_OF_PROGRAM_TIME).total_nanoseconds() * 1e-9;
#endif
}



PeriodicLoopTimer::PeriodicLoopTimer(double period_, int threadPriority) :
		firstRun(true), period(period_), releasePoint(-1.0)
{
#ifdef BARRETT_XENOMAI
	int ret;

	// Try to become a Xenomai task
	ret = rt_task_shadow(NULL, NULL, threadPriority, 0);
	// EBUSY indicates the current thread is already a Xenomai task
	if (ret != 0  &&  ret != -EBUSY) {
		(logMessage("PeriodicLoopTimer::%s: rt_task_shadow(): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}

	ret = rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(period));
	if (ret != 0) {
		(logMessage("PeriodicLoopTimer::%s: rt_task_set_periodic(): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}
#else
//
//  defines for the RT-Preempt experiment
//
//
//  we are using 49 as the priority cuz' RT-PREMPT uses
//  50 as the priority of the kernel tasklets....(this is the default)
//
#define MY_THREAD_PRIORITY  (49)                                   // max priority for user threads
#define MAX_SAFE_STACK_SIZE (8 * 1024)                             // this is a guess
#define LOOP_FREQUENCY      (500)                                  //
#define LOOP_TIME_NANO_SEC  (NANO_SEC_PER_SECOND / LOOP_FREQUENCY) //


  //
  // here we are trying the experiment to see if the RT-PREEMPT solution is viable
  //
  struct sched_param the_scheduler_param;
  
  the_scheduler_param.sched_priority = MY_THREAD_PRIORITY;

  if (-1 == sched_setscheduler(0, SCHED_FIFO, &the_scheduler_param)) {
    printf ("Yikes, the call to sched_setscheduler failed...\n");
    switch (errno) {
    case EINVAL:
      printf ("Error number is EINVAL\n");
      break;
    case EPERM:
      printf ("Error number is EPERM\n");
      break;
    case ESRCH:
      printf ("Error number is ESRCH\n");
      break;
    default:
      printf ("Error number is an unexpected value %d\n", errno);
      break;
    }
  }
#endif

}

unsigned long PeriodicLoopTimer::wait()
{
#ifdef BARRETT_XENOMAI
	unsigned long missedReleasePoints;

	int ret = rt_task_wait_period(&missedReleasePoints);
	if (ret != 0  &&  ret != -ETIMEDOUT) {  // ETIMEDOUT means that we missed a release point
		(logMessage("%s: rt_task_wait_period(): (%d) %s") % __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}

	return missedReleasePoints;
#else
	const double now = highResolutionSystemTime();
	const double remainder = releasePoint - now;
	if (remainder <= 0) {
		releasePoint = now + period;

		if (firstRun) {
			firstRun = false;
			return 0;
		} else {
			return ceil(-remainder / period);
		}
	} else {
		// Calculate the new releasePoint based on the old one.
		// This eliminates drift (on average) due to over/under sleeping.
		releasePoint += period;
		btsleep(remainder);
		return 0;
	}
#endif
}



detail::LogFormatter logMessage(const std::string& message, bool outputToStderr)
{
	return detail::LogFormatter(message, outputToStderr);
}


namespace detail {

void LogFormatter::print()
{
	// Make sure we only print once
	if (printed) {
		return;
	}
	printed = true;

	std::string message = str();
	if (ose) {
		std::cerr << message;
		// The message should always have one newline
		if (message[message.size() - 1] != '\n') {
			std::cerr << std::endl;
		}
	}
	// Use a trivial format string in case message contains '%'
	syslog(LOG_ERR, "%s", message.c_str());
}

}
}
