/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... os.h
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... Mar 28, 2005
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *    Operating system abstractions and helpers
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt
 *
 * ======================================================================== */

/* ########### IMPORTANT - NO NON-GLOBAL IFDEFS IN HEADER FILES! */


/** \file os.h
    \brief Operating system abstractions and helpers.

\section bt_os 
    
The bt_os module is a thin layer between barrett technologies code and the operating system.
Additionally, global defines can go here also. Virtually every Barrett library
source file will use this.
 
These are the submodules that bt_os implements:
 - Thread API: A simplified api for starting, stopping, and monitoring threads.
 - Mutex API: A wrapper around pthread_mutex_* that writes errors to syslog.
 - Malloc API: Wrappers for malloc & free that automate error handling.
 - Pointer & Array Sanity Checks: Used for debugging.
 - Convinience functions: Typing is such a chore. 
 
The mutex layer allows for ERRORCHECK mutexes to be compiled in if desired for 
debugging.
 
btmalloc() and btfree() provide error checking for memory access. Lack of memory 
is fatal. btfree() sets the calling variable to NULL 
 
*/
/**
\page  boc Barrett Object Conventions
 
Much of the btsystem and btwam libraries are written in an object oriented style.
OOP in C is less clean but easier to bind to other languages. 
To keep things sane, we follow certain conventions when working with the objects
used in our library. Since this is C and not C++; it's trickier. Treat the 
following functionname_obj() as reserved functions.
 
 - Pointers: Pointers to objects should be initialized to a NULL value.
             A NULL object pointer is treated as on empty pointer and is handled 
             gracefully. When an object is destroyed; It's referencing pointer is
             set to NULL.
             
 - Object memory allocation: 
   - malloc_obj() - Allocate memory for this object and sub-objects.
   - local_obj() - Macro for allocating this object on the local function stack. (It will be deallocated when the function exits)
   - sizeof_obj() - Size in bytes needed for an instance of this object
   - destroy_obj() - Free memory and set referencing pointer to NULL
   
 - Object initialization:
   - initval_obj() - Initialize the values in the object to sane values
   - initptr_obj() - Initialize the structure of the object (inter-object pointers)
   
 - Compound functions:
   - init_obj() - Initialize values and structure of the object.
   - new_obj() - Allocate memory for the object and initialization.
\internal
\todo Update all of btclient to use these conventions.
 
*/
#ifndef BT_OS_H
#define BT_OS_H



/* For size_t, etc */
#include <stdlib.h>


/* Do we ever get included in C++? */
#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

/* Assume inline */
#ifndef BTINLINE
#define BTINLINE __inline__
#endif

/* What the heck is this? */
/*
#ifndef S_SPLINT_S
#include <pthread.h>
#endif
*/



int bt_os_usleep(int useconds);


/* Mutexes (realtime and non-realtime) */

/* Abstract mutex type
 * We use the struct for type-checking.
 * Maybe there's a better way of doing that? */
enum bt_os_rt_type {
   BT_OS_RT,
   BT_OS_NONRT
};

typedef struct {
   enum bt_os_rt_type type;
   void * p;
} bt_os_mutex;

bt_os_mutex * bt_os_mutex_create(enum bt_os_rt_type type);
int bt_os_mutex_destroy( bt_os_mutex * m );
BTINLINE int bt_os_mutex_lock(bt_os_mutex * m);
BTINLINE int bt_os_mutex_unlock(bt_os_mutex * m);



/*@}*/

/** @name Convinience functions */
/*@{

BTINLINE int bt_os_test_and_log(int ret,const char *str);
*/



/** @name Malloc & Free wrappers */
/*Memory*/
#if 0
BTINLINE void * bt_os_malloc(size_t size);
BTINLINE void * bt_os_realloc(void * ptr, size_t size);
BTINLINE void bt_os_free(void * ptr);
#endif

typedef unsigned long long bt_os_rtime;


/** @name Threads: */
/** Convenience info for creation of threads
    This is for non-realtime threads.
See new_btthread()
*/

/* forward-declare the struct so it can go in the function pointer */
struct bt_os_thread;

/* This is for realtime threads
 * If not Xenomai, the structure will be the same as btthread */
struct bt_os_thread
{
   /* Some architecture-specific thread data */
   void * sys;
   /*
#ifdef XENOMAI
   RT_TASK task;
#else
   pthread_t thd_id;
   pthread_attr_t attr;
   struct sched_param param;
#endif
   */

   enum bt_os_rt_type type;
   int priority; /*Priority this thread was created at*/

/* Why can't I #ifdef RTSYS_NONOE these two? */
   double period; /* The period we want this thread to be (in case we're in non-real-time mode) */
   bt_os_rtime last_called;

   int done; /*!< See btthread_done()*/
   void (*function)(struct bt_os_thread * thd); /*Pointer to the function this thread is running*/
   void * data;
   
   bt_os_mutex * mutex;

   bt_os_rtime actual_period, proc_time;

};

/* Functions for operating on threads */
struct bt_os_thread * bt_os_thread_create(enum bt_os_rt_type type, const char * name,
                                 int priority, void (*funcptr)(struct bt_os_thread *), void * data);
                                 
int bt_os_thread_destroy(struct bt_os_thread * thd);
BTINLINE int bt_os_thread_done(struct bt_os_thread * thd); /*ret !0 when time to kill*/
BTINLINE void bt_os_thread_stop(struct bt_os_thread * thd); /*set done = 1;*/
BTINLINE void bt_os_thread_exit(struct bt_os_thread * thd);


/*RT Abstractions */

bt_os_rtime bt_os_rt_get_time(void);
void bt_os_rt_set_mode_soft(void);
void bt_os_rt_set_mode_hard(void);
void bt_os_rt_set_mode_warn(void);
void bt_os_rt_task_wait_period(void);
void bt_os_make_periodic(double period, char * sixname);
void bt_os_rt_allow_nonroot();


/* Time statistic stuff
 * For statistics, assume independence */
struct bt_os_timestat {
   int numchans;
   int numsamps;
   bt_os_rtime * start;
   bt_os_rtime * times;
   bt_os_rtime * mins;
   bt_os_rtime * maxs;
   bt_os_rtime * sums;
   bt_os_rtime * sumsqs;
};
struct bt_os_timestat * bt_os_timestat_create(int numchans);
void bt_os_timestat_start(struct bt_os_timestat * ts);
void bt_os_timestat_trigger(struct bt_os_timestat * ts, int chan);
void bt_os_timestat_end(struct bt_os_timestat * ts);
void bt_os_timestat_get(struct bt_os_timestat * ts,
                       bt_os_rtime * means, bt_os_rtime * variances,
                       bt_os_rtime * mins, bt_os_rtime * maxs);


#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* BT_OS_H */

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003-2008 Barrett Technology, Inc.            *
 *                        625 Mount Auburn St                           *
 *                    Cambridge, MA  02138,  USA                        *
 *                                                                      *
 *                        All rights reserved.                          *
 *                                                                      *
 *  ******************************************************************  *
 *                            DISCLAIMER                                *
 *                                                                      *
 *  This software and related documentation are provided to you on      *
 *  an as is basis and without warranty of any kind.  No warranties,    *
 *  express or implied, including, without limitation, any warranties   *
 *  of merchantability or fitness for a particular purpose are being    *
 *  provided by Barrett Technology, Inc.  In no event shall Barrett     *
 *  Technology, Inc. be liable for any lost development expenses, lost  *
 *  lost profits, or any incidental, special, or consequential damage.  *
 *======================================================================*/
 
