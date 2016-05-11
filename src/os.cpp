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
#endif

#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include <barrett/detail/stacktrace.h>
#include <barrett/detail/os.h>
#include <barrett/os.h>


#ifdef BARRETT_XENOMAI
// Xenomai helper function
inline RTIME secondsToRTIME(double s) {
	return static_cast<RTIME>(s * 1e9);
}

// Xenomai requires at least one call to mlockall() per process. Also, a signal
// handler is installed to trap transitions from primary execution mode to
// secondary execution mode; this aids in identifying code that breaks Xenomai's
// realtime guarantee.
//
#define MAX_LOG_MESSAGE_SIZE (100)
namespace {  // Using an anonymous namespace because no other code needs to
			 // interact with these declarations. It is necessary to construct a
			 // single instance of InitXenomai.
	extern "C" {
	void warnOnSwitchToSecondaryMode(int)
	{
	        char log_message [ MAX_LOG_MESSAGE_SIZE];
                RT_TASK_INFO     the_info;
                rt_task_inquire (NULL, &the_info);
                sprintf (log_message, "WARNING: Switched out of RealTime for thread %s", the_info.name);
		barrett::logMessage(log_message, true);
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
//  this static function inquires xenomai to get
//  the current tasks info....mostly used at 
//  present for fixing the btsleep functions
//
static int   btInquireAboutTask (RT_TASK_INFO *current_task_info)
{
  int inquire_results;

  inquire_results = rt_task_inquire (NULL, current_task_info);
  switch (inquire_results) {
  case 0:
    //
    //  this is the good case and we need do nothing
    //
    break;
  case (-EINVAL):
    //
    //  this indicates that the task argument does
    //  not point to a task discription. but
    //  we are using NULL for the pointer...so this
    //  should be impossible
    //
    syslog(LOG_ERR, "Yikes. In btInquireAboutTask and we got -EINVAL");
    break;
  case (-EPERM):
     //
     //  this is the case where the task argument is 
     //  NULL, but we are not in a task context
     //
     syslog(LOG_ERR, "Yikes. In btInquireAboutTask and we got -EPERM");
     break;
   case (-EIDRM):
     //
     //  this is the case where the task argument is
     //  pointing to a deleted task discriptor....so
     //  this should be impossible
     //
     syslog(LOG_ERR, "Yikes. In btInquireAboutTask and we got -EIDRM");
     break;
   case (-ENOMEM):
     //
     //  this is not discussed in the Xenomai documenation
     //  but might be returned......
     //  
     //
     syslog(LOG_ERR, "Yikes. In btInquireAboutTask and we got -ENOMEM - no memory");
     break;
   case (-EACCES):
     //
     //  this is not discussed in the Xenomai documenation
     //  but might be returned......
     //  
     //
     syslog(LOG_ERR, "Yikes. In btInquireAboutTask and we got -EACCES - permission denied");
     break;
   case (-EFAULT):
     //
     //  this is not discussed in the Xenomai documenation
     //  but might be returned......
     //  
     //
     syslog(LOG_ERR, "Yikes. In btInquireAboutTask and we got -ENFAULT - bad address");
     break;
   case (-ESRCH):
     //
     //  this is not discussed in the Xenomai documenation
     //  but might be returned......
     //  
     //
     //syslog(LOG_ERR, "Yikes. In btInquireAboutTask and we got -ESRCH - task does not exist");
     break;
   default:
      //
      //  this is really a bad situation... a xenomai
      //  function returned a bogus value
      //
      syslog(LOG_ERR, "Yikes. In btInquireAboutTask and we got a totally bogus value");
      syslog(LOG_ERR, "The value is %x\n", inquire_results);
      break;
    }
    return (inquire_results);
}


//
//   brad's improved btSleep function that insures we are a real time task
//   prior to calling the xenomai sleep function
//
void btsleepRT(double duration_s)
{
#ifdef BARRETT_XENOMAI
        RT_TASK_INFO  current_task_info;
        RTIME   start_time;
        RTIME   end_time;
        RTIME   diff_time;
        RTIME   delay_time;
 
	assert(duration_s > 1e-9);  // Minimum duration is 1 ns
        if ( 0 == btInquireAboutTask (&current_task_info)) {
          if ((current_task_info.status & T_STARTED) ) {
	    //
            //  i presume it is okay to sleep if were are ready
            //
            delay_time = RTIME(duration_s * 1e9);
 
            start_time = rt_timer_read ();
            int ret = rt_task_sleep(delay_time);
            end_time = rt_timer_read();
            diff_time = end_time - start_time;
            if (diff_time < delay_time) {
              //
              //  it seems the the actual measured delay is just slightly short of the 
              //  requested time....so we only print out a message if the time is 
              //  10% to short.....
              //
	      if ( (delay_time - diff_time) > (delay_time - (delay_time / 10))) {
  	        syslog(LOG_ERR, "Yikes. In btsleepRT and the sleep was too short!!\n");
                syslog(LOG_ERR, "Diff time is %llu, delay time is %llu\n",diff_time, delay_time);
              }
	    }

	    if (ret != 0) {
		(logMessage("%s: rt_task_sleep() returned error %d.") % __func__ % ret).raise<std::runtime_error>();
	    }
	  } else if ( (current_task_info.status & T_BLOCKED) || 
                      (current_task_info.status & T_DELAYED) || 
                      (current_task_info.status & T_DORMANT) || 
                      (current_task_info.status & T_LOCK) ) {
            //
            //   uh oh...this task is blocked delayed dormant or
            //   even worse it hold the scheduler lock....
            //   don't sleep
            //
	    syslog(LOG_ERR, "Yikes. In btsleepRT and the task status is questionable\n");
	  } else if (current_task_info.status & T_BOOST)  {
            //
            //  somebody call btsleepRT when the task is not RT
            //  call the normal, boost thread
            //
 	    syslog(LOG_ERR, "somebody is calling btsleepRT for a non-real-time thread\n");
            btsleep(duration_s);
	  }
	} else {
	    syslog(LOG_ERR, "Yikes.  In btsleepRT....btInquireAboutTask returned false\n");
	}
#else
        //
        //  BARRETT_XENOMAI is not defined...just use the boost sleep
        //
	btsleep(duration_s);
#endif

}

//
//
//  this is the new version of btsleep that
//  checks to insure it is only called from a 
//  non-real-time thread
//
void btsleep(double duration_s)
{
  RT_TASK_INFO  current_task_info;
  assert(duration_s > 1e-6);  // Minimum duration is 1 us

  //if (  (-ESRCH) == btInquireAboutTask (&current_task_info)) {
     //  TODO (bim) resolve this question....
     //  
     //printf (" here we are in btsleep and the current task info status is %4x\n", current_task_info.status);
     //
     //  this i don't fully understand....I would assume a relaxed bit means the thread is
     //  not real time....however we see for a typical task
     //  a status of 300180
     //              200000  - a shadow task
     //              100000  - uses the FPU
     //                 100  - maps to a linux task
     //                  80  - started.....
     //
     //     this seems to differ from the real time task that
     //                 200  -  relaxed is not set....
     //
     //  which seems strange to me cuz' I thought relaxed means
     //  'fallen out of real time mode'
     //   bim (5/10/16)
     //
     //if (!(current_task_info.status & XNRELAX))  {
     //
     //  i presume it is okay to sleep if were are a relaxed thread
     //
     boost::this_thread::sleep(boost::posix_time::microseconds(long(duration_s * 1e6)));
     //} else {
     //syslog (LOG_ERR, "Yikes. someone called btsleep in a real time thread\n");
     //} 
}
#if 0
void btsleep(double duration_s)
{
	assert(duration_s > 1e-6);  // Minimum duration is 1 us
	boost::this_thread::sleep(boost::posix_time::microseconds(long(duration_s * 1e6)));
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

#endif


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



PeriodicLoopTimer::PeriodicLoopTimer(double period_,const char *thread_name, int threadPriority) :
		firstRun(true), period(period_), releasePoint(-1.0)
{
#ifdef BARRETT_XENOMAI
	int ret;

	// Try to become a Xenomai task
	ret = rt_task_shadow(NULL, thread_name, threadPriority, 0);
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
