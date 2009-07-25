/** Definition of a set of operating-system wrappers for threads,
 *  mutexes, and timing real-time loops.
 *
 * \file os.h
 * \author Traveler Hauptman
 * \author Brian Zenowich
 * \author Christopher Dellin
 * \date 2005-2009
 */

/* Copyright 2005, 2006, 2007, 2008, 2009
 *           Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */

/** \file os.h
 *
 * \section sec_intro Introduction
 *
 * The bt_os module defines a set of abstractions of core,
 * implementation-specific functions that operate on threads and timing.
 * Specifically, this file includes objects that operate on timing
 * information (such as bt_os_rtime, bt_os_usleep(), and
 * bt_os_rt_get_time()), functions that create and manage realtime and
 * non-realtime threads, and functions that manipulate data protection
 * mutexes.
 */

#ifndef BT_OS_H
#define BT_OS_H
#ifdef __cplusplus
extern "C" {
#endif

/* For size_t, etc */
#include <stdlib.h>


/** Allow non-root users access to the real-time subsystem
 *  (only supported in Xenomai). */
int bt_os_rt_allow_nonroot(void);

/** A simple wrapper around the non-ansi usleep() function. */
int bt_os_usleep(int useconds);


/** Mutex/Thread type selector.
 *
 * This enum explicitly defines the two types of mutexes and threads --
 * realtime (BT_OS_RT) and non-realtime (BT_OS_NONRT).
 */
enum bt_os_rt_type
{
   BT_OS_RT,
   BT_OS_NONRT
};

/** A current time or elapsed time, in nanoseconds.
 *
 * This is always typedef'ed to an unsigned long long.
 */
typedef unsigned long long bt_os_rtime;


/** System mutex object. */
struct bt_os_mutex
{
   enum bt_os_rt_type type; /**< Type of the mutex */
   void * p; /**< Implementation-specific mutex object */
};


/** \name Mutex functions:
 *  \{ */

/** Create a bt_os_mutex.
 *
 * Simply supply the typf of mutex (either BT_OS_RT or BT_OS_NONRT).
 *
 * \param[in] type Type of the mutex
 * \returns The bt_os_mutex object on success, 0 on failure.
 */
struct bt_os_mutex * bt_os_mutex_create(enum bt_os_rt_type type);

/** Destroy a bt_os_mutex.
 *
 * \param[in] m The bt_os_mutex object to destroy
 * \retval 0 Success
 */
int bt_os_mutex_destroy(struct bt_os_mutex * m);

/** Lock a bt_os_mutex.
 *
 * \param[in] m The bt_os_mutex object to lock
 * \retval 0 Success
 * \retval ?? Failure
 */
int bt_os_mutex_lock(struct bt_os_mutex * m);

/** Unlock a bt_os_mutex.
 *
 * \param[in] m The bt_os_mutex object to unlock
 * \retval 0 Success
 * \retval ?? Failure
 */
int bt_os_mutex_unlock(struct bt_os_mutex * m);

/** \} */




/* Forward-declared so it can go in the function pointer below ... */
struct bt_os_thread;

/** Abstracted thread object, supporting realtime and non-realtime threads.
 *
 * This object represents a running thread.  It provides an abstraction for
 * the underlying real-time operating system, such as Xenomai or RTAI.
 */
struct bt_os_thread
{
   enum bt_os_rt_type type;
   int priority; /**< Thread priority */

/* Why can't I #ifdef RTSYS_NONE these two? */
   double period; /**< The period we want this thread to be (in case we're in non-real-time mode) */
   bt_os_rtime last_called;

   int done; /**< Used by bt_os_thread_stop() and bt_os_thread_isdone() */
   void (*function)(struct bt_os_thread * thd); /**< Pointer to the function this thread is running*/
   void * data;
   
   struct bt_os_mutex * mutex;

   bt_os_rtime actual_period, proc_time;
   
   void * sys; /**< Some implementation-specific thread data */
};

/** \name Thread object functions:
 *  \{ */

/** Create a bt_os_thread with the type, name, and priority provided.
 *
 * This function creates and "spins off" a new thread, either in real-time
 * or non-real-time mode, depending on the type.  The function pointed to by
 * funcptr is called in the new thread, and the data object is sated in the
 * bt_os_thread object for reference by called function.
 *
 * \param[in] type Type of the thread
 * \param[in] name The name of the thread
 * \param[in] priority The priority of the thread [1-99] (inclusive),
 *            with 99 highest priority
 */
struct bt_os_thread * bt_os_thread_create(
   enum bt_os_rt_type type, const char * name, int priority,
   void (*funcptr)(struct bt_os_thread *), void * data);

/** Destroy a bt_os_thread.
 *
 * This function destroys a thread.
 *
 * \param[in] thd The bt_os_thread to destroy
 * \retval 0 Success
 */
int bt_os_thread_destroy(struct bt_os_thread * thd);

/** Tell a looping thread to stop.
 *
 * This function instructs a looping thread to stop.
 *
 * \param[in] thd The bt_os_thread to stop
 */
int bt_os_thread_stop(struct bt_os_thread * thd); /*set done = 1;*/

/** Check to see if a thread has been told to stop.
 *
 * This function is intended to be called from inside the thread, and will
 * return true if bt_os_thread_stop() has been called.
 *
 * \param[in] thd The bt_os_thread to check
 * \retval 0 Not yet told to stop
 * \retval nonzero Told to stop
 */
int bt_os_thread_isdone(struct bt_os_thread * thd);

/** Destroy the current thread.
 *
 * This function is intended to be called from inside the thread, and will
 * properly remove the thread from the scheduler once the function is
 * finished.
 *
 * \param[in] thd The bt_os_thread to exit
 * \retval 0 Success
 */
int bt_os_thread_exit(struct bt_os_thread * thd);

/** \} */


/** \name Thread utility functions:
 *
 * These functions are designed to be called from inside a real-time thread.
 *  \{ */

/** Get the current time, in nano-seconds. */
bt_os_rtime bt_os_rt_get_time(void);

/** Set the current thread to soft real-time mode. */
int bt_os_rt_set_mode_soft(void);

/** Set the current thread to hard real-time mode. */
int bt_os_rt_set_mode_hard(void);

/** Set the current thread to warn (??). */
int bt_os_rt_set_mode_warn(void);

/** Set the current thread to periodic mode.
 *
 * \note Be sure to only call this once!
 * \note Note that sixname is a unique six-character string
 *       used only in RTAI mode in rt_task_init()
 */
int bt_os_rt_make_periodic(double period, char * sixname);

/** Sleep the current periodic thread until the next period. */
int bt_os_rt_task_wait_period(void);

/** \} */




/** This object holds the data required for collecting timing statistics.
 */
struct bt_os_timestat
{
   int numchans;
   int numsamps;
   bt_os_rtime * start;
   bt_os_rtime * times;
   bt_os_rtime * mins;
   bt_os_rtime * maxs;
   bt_os_rtime * sums;
   bt_os_rtime * sumsqs;
};

/** \name Time-statistics functions:
 *
 * These functions are used to generate a simple set of timing statistics for
 * a real-time loop.  For each channel, the timestat calculates the minimum,
 * maximum, mean, and variance over the life of the measurements.
 *  \{ */

/** Create a bt_os_timestat object.
 *
 * \param[in] numchans The number of sequential channels to measure
 * \returns The bt_os_timestat object on success, 0 on failure
 */
struct bt_os_timestat * bt_os_timestat_create(int numchans);

/** Destroy a bt_os_timestat object.
 *
 * \param[in] ts The bt_os_timestat object to destroy
 * \retval 0 Success
 */
int bt_os_timestat_destroy(struct bt_os_timestat * ts);

/** Start measuring a loop with a bt_os_timestat object.
 *
 * \param[in] ts The bt_os_timestat object to use
 * \retval 0 Success
 */
int bt_os_timestat_start(struct bt_os_timestat * ts);

/** Trigger a particular channel on a bt_os_timestat object.
 *
 * \param[in] ts The bt_os_timestat object to trigger
 * \param[in] chan The channer to trigger
 * \retval 0 Success
 */
int bt_os_timestat_trigger(struct bt_os_timestat * ts, int chan);

/** End measuring a loop with a bt_os_timestat object.
 *
 * \param[in] ts The bt_os_timestat object to use
 * \retval 0 Success
 */
int bt_os_timestat_end(struct bt_os_timestat * ts);

/** Retrieve summary statistics from a bt_os_timestat object.
 *
 * \param[in] ts The bt_os_timestat object to query
 * \param[out] means An array of (numchans) in which to save the
 *             channel means
 * \param[out] variances An array of (numchans) in which to save the
 *             channel variances
 * \param[out] mins An array of (numchans) in which to save the
 *             channel minimums
 * \param[out] maxs An array of (numchans) in which to save the
 *             channel maximums
 */
int bt_os_timestat_get(struct bt_os_timestat * ts,
                       bt_os_rtime * means, bt_os_rtime * variances,
                       bt_os_rtime * mins, bt_os_rtime * maxs);

/** \} */

#ifdef __cplusplus
}
#endif
#endif /* BT_OS_H */
