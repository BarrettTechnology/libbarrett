/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... os.c
 *  Author ............. Traveler Hauptman
 *                       Lauren White
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... Mar 28, 2005
 *                                                                          *
 *  **********************************************************************  *
 *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 * ======================================================================== */

#include "os.h"

#ifdef S_SPLINT_S
#include <err.h>
#endif

#include <syslog.h>
#include <stdlib.h>
#include <unistd.h> /* For usleep() */

#ifdef RTSYS_XENOMAI
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#endif

#ifdef RTSYS_RTAI
#include <rtai_lxrt.h>
#include <rtai_sem.h>
#endif

#include <pthread.h>

#ifdef RTSYS_NONE
#include <sys/time.h>
#endif


/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/

/* Huh? get rid of this crap at some point ... */
void *backtracearray[50];
char **backtracestrings;
int backtracesize;

/*==============================*
 * Internal use Functions       *
 *==============================*/
void syslog_backtrace(int size)
{
   int cnt;
   for (cnt = 0;cnt < size;cnt ++)
      syslog(LOG_ERR,"WAM:Backtrace:%s",backtracestrings[cnt]);
}

/*==============================*
 * Functions                    *
 *==============================*/


/** Provides a shorthand to replace return variable checks.
\internal chk'd TH 051101
*/
BTINLINE int test_and_log(int return_val, const char *str)
{
   if (return_val != 0)
   {
      syslog(LOG_ERR, "%s: %d", str, return_val);
      return return_val;
   }
   else
      return 0;
}


/* usleep Wrapper */
int bt_os_usleep(int useconds)
{
   return usleep(useconds);
}

/** Initialize a mutex.
 
If pthread_mutex_init() fails, an error message is printed to syslog.
 
\return Result of pthread_mutex_init().
\exception Undefined if btm does not point to memory block meant for a btmutex.
\internal chk'd TH 051101
\todo Error checking mutexes enabled by compiler switch.
*/
/* Note - create() used to not have the mattr stuff for RTAI
 * (it used NULL in pthread_mutex_init)
 * What's the difference? */
bt_os_mutex * bt_os_mutex_create(enum bt_os_rt_type type)
{
   int ret;
   bt_os_mutex * m; /* The return mutex abstraction */
   m = (bt_os_mutex *) malloc(sizeof(bt_os_mutex));
   
   /* Create the system mutex */
#ifdef RTSYS_XENOMAI
   if (type == BT_OS_RT)
   {
      RT_MUTEX * sys_mutex;
      sys_mutex = (RT_MUTEX *) malloc(sizeof(RT_MUTEX));
      bt_os_rt_set_mode_hard(); /* Why must we be in realtime mode? */
      ret = test_and_log(
            rt_mutex_create(sys_mutex,NULL),
            "Could not initialize mutex.");
      if (ret)
      {
         
         free(sys_mutex);
         free(m);
         return NULL;
      }
      m->p = (void *)sys_mutex;
   }
   else
#endif
   {
      pthread_mutexattr_t mattr;
      pthread_mutex_t * sys_mutex;         
      sys_mutex = (pthread_mutex_t *) malloc(sizeof(pthread_mutex_t));
      pthread_mutexattr_init(&mattr);
      pthread_mutexattr_settype(&mattr,PTHREAD_MUTEX_TIMED_NP);
      /*pthread_mutexattr_settype(&mattr,PTHREAD_MUTEX_ERRORCHECK_NP);*/
      ret = test_and_log(
               pthread_mutex_init(sys_mutex,&mattr),
               "Could not initialize mutex.");
      if (ret)
      {
         free(sys_mutex);
         free(m);
         return NULL;
      }
      m->p = (void *)sys_mutex;
   }

   /* Save mutex type */
   m->type = type;
   return m;
}

/* Destroy a mutex */
int bt_os_mutex_destroy( bt_os_mutex * m )
{

#ifdef RTSYS_XENOMAI
   if (m->type == BT_OS_RT)
   {
      /* I don't know what goes here.
       * How to I destroy an RT_MUTEX? */
   }
   else
#endif
   {
      pthread_mutex_destroy( (pthread_mutex_t *)(m->p) );
   }
   
   free( m->p );
   free( m );
   return 0;
}


/** Lock a btmutex.
See pthread_mutex_lock() in pthread.h for more info.
This function calls pthread_mutex_lock() and prints an error to syslog if it 
fails.
\return Result of pthread_mutex_lock().
\exception Undefined if btm does not point to an initialized btmutex object.
\internal chk'd TH 051101
*/
BTINLINE int bt_os_mutex_lock(bt_os_mutex * m)
{
   int ret;
   
   /* Attempt the lock */
#ifdef RTSYS_XENOMAI
   if (m->type == BT_OS_RT)
   {
      bt_os_rt_set_mode_hard();
      ret = rt_mutex_acquire((RT_MUTEX *)(m->p), TM_INFINITE);
   }
   else
#endif
   {
      ret = pthread_mutex_lock((pthread_mutex_t *)(m->p));
   }
   
   /* Log on error */
   if (ret != 0)
   {
      syslog(LOG_ERR, "Mutex lock failed (?): %d", ret);
   }
   return ret;
}



/** Unlock a btmutex.
See pthread_mutex_unlock() in pthread.h for more info.
This function calls pthread_mutex_unlock() and prints an error to syslog if it 
fails.
\return Result of pthread_mutex_unlock().
\exception Undefined if btm does not point to an initialized btmutex object.
\internal chk'd TH 051101
*/
BTINLINE int bt_os_mutex_unlock(bt_os_mutex * m)
{
   int ret;
   
   /* Attempt the lock */
#ifdef RTSYS_XENOMAI
   if (m->type == BT_OS_RT)
   {
      bt_os_rt_set_mode_hard();
      ret = rt_mutex_release((RT_MUTEX *)(m->p));
   }
   else
#endif
   {
      ret = pthread_mutex_unlock((pthread_mutex_t *)(m->p));
   }
   
   /* Log on error */
   if (ret != 0)
   {
      syslog(LOG_ERR, "Mutex unlock failed (?): %d", ret);
   }
   return ret;
}











/** Check pointer for a NULL value.
\retval 0 Pointer is NOT valid.
\retval 1 Pointer is valid.
 
\exception Undefined if str points to something not a string and ptr is not valid.
\internal chk'd TH 051101
\todo Eventually we should check for pointers outside of program heap instead of
just null pointers. See sbrk() [gnu libc] for finding end of data segment. 
 
*/
int btptr_ok(void *ptr,char *str)
{
   if (ptr == NULL)
   {
#if BT_BACKTRACE & BTDEBUG_BACKTRACE
      backtracesize = baktrace(backtracearray,50);
      backtracestrings = backtrace_symbols(backtracearray,backtracesize);
      syslog_backtrace(backtracesize);
#endif

      syslog(LOG_ERR,"bt ERROR: you tried to access a null pointer in %s",str);
      return 0;
   }
   return 1;
}

/** Pointer out of range check.
 
Presently only checks for NULL.
Has same (backwards) return values as btptr_ok().
\retval 0 Pointer is NOT valid.
\retval 1 Pointer is valid.
 
\exception Undefined if str points to something not a string and ptr is not valid.
\internal chk'd TH 051101
*/
int btptr_chk(void *ptr)
{
   if (ptr == NULL)
   {
#if BT_BACKTRACE & BTDEBUG_BACKTRACE
      backtracesize = baktrace(backtracearray,50);
      backtracestrings = backtrace_symbols(backtracearray,backtracesize);
      syslog_backtrace(backtracesize);
#endif

      return 0;
   }
   return 1;
}

/** Prints an error if array index is out of range.
 
\retval 0 Array index is NOT valid.
\retval 1 Array index is valid.
 
\exception Undefined if str points to something not a string and index is not valid.
\internal chk'd TH 051101
 
*/
int idx_bounds_ok(int idx,int max,char *str)
{
   if ((idx < 0) || (idx > max))
   {
#if BT_BACKTRACE & BTDEBUG_BACKTRACE
      backtracesize = baktrace(backtracearray,50);
      backtracestrings = backtrace_symbols(backtracearray,backtracesize);
      syslog_backtrace(backtracesize);
#endif

      syslog(LOG_ERR,"bt ERROR: Your index is %d with max %d in function %s",idx,max,str);
      return 0;
   }
   return 1;
}


#if 0
/**Memory allocation wrapper.
 
\return Pointer to allocated memory.
\exception If malloc returns NULL there is no memory left and we kill the process!
\internal chk'd TH 051101
 
*/
BTINLINE void * bt_os_malloc(size_t size)
{
   void * vmem;
   if ((vmem = malloc(size)) == NULL)
   {
      syslog(LOG_ERR,"btMalloc: memory allocation failed, size %d",size);
      exit(-1);
   }
   return vmem;
}

/**Memory deallocation wrapper.
  Free's memory at *ptr and then sets *ptr to NULL.
\exception If *ptr points to a block of memory that was not allocated with btmalloc[malloc]
or if that block was previosly freed the results are undefined.
\internal chk'd TH 051101
*/
BTINLINE void bt_os_free(void *ptr)
{
#ifdef BT_NULL_PTR_GUARD
   if(btptr_ok(ptr,"btfree"))
#endif

      free(ptr);
}

BTINLINE void * bt_os_realloc(void * ptr, size_t size)
{
   void * vmem;
   if ((vmem = realloc(ptr,size)) == NULL)
   {
      syslog(LOG_ERR,"btMalloc: memory allocation failed, size %d",size);
      exit(-1);
   }
   return vmem;
}
#endif


/* If we're using RTAI, these will also be used in real-time mode */
struct pthd {
   pthread_t thd_id;
   pthread_attr_t attr;
   struct sched_param param;
};

#ifdef RTSYS_NONE
bt_os_thread ** threads;
int threads_num = 0;
static int thread_list_add( bt_os_thread * thd )
{
   if (threads_num == 0)
       threads = (bt_os_thread **) malloc( sizeof(bt_os_thread *) );
   else
       threads = (bt_os_thread **) realloc( threads, (threads_num+1) * sizeof(bt_os_thread *) );
   threads[threads_num] = thd;
   threads_num++;
   return 0;
}
static int thread_list_remove( bt_os_thread * thd )
{
   int i;
   for (i=0; i<threads_num; i++) if (threads[i] == thd)
   {
      int j;
      for (j=i+1; j<threads_num; j++)
         threads[j-1] = threads[j];
      threads_num--;
      if (threads_num)
         threads = (bt_os_thread **) realloc( threads, threads_num * sizeof(bt_os_thread *) );
      else
         free(threads);
      return 0;
   }
   return -1;
}
static bt_os_thread * thread_list_current()
{
   int i;
   pthread_t thd_id;
   thd_id = pthread_self();
   for (i=0; i<threads_num; i++)
   {
      if ( pthread_equal( ((struct pthd *)threads[i]->sys)->thd_id, thd_id ) )
      {
         return threads[i];
      }
   }
   return 0;
}
#endif

/* Non-realtime stuff first! */

/** Allocate memory for a btthread object.

We create a new posix thread with a schedpolicy of SCHED_FIFO.
 
\return Pointer to a newly allocated btthread object.
\internal chk'd TH 051101
 
\param  thd The barrett thread structure; allocated before calling this function.
\param  priority The priority we wish to call this thread with. 0 = linux non-realtime priority. 99 = Max priority
\param  function Pointer to the function that represents the thread.
\param  args Pointer to the arguments you want to pass.
 
\internal chk'd TH 051101 
right now we kill the program if a thread create doesn't work. I'm not sure if this 
is reasonable.

name is only used by xenomai in realtime
 
*/


struct bt_os_thread * bt_os_thread_create(enum bt_os_rt_type type, const char * name,
                                 int priority, void (*funcptr)(struct bt_os_thread *), void * data)
/*bt_os_thread * bt_os_thread_create(int priority, void * function, void * args)*/
{
   struct bt_os_thread * thd;
   
   /* Allocate space for a new thread */
   thd = (struct bt_os_thread *) malloc(sizeof(struct bt_os_thread));
   
   /* Save the extra stuff for the thread ..
    * Note that we need to do this /before/ we spin off the thread,
    * or the thread might try to access a part of itself
    * that hasn't been defined yet! */
   thd->done = 0;
   thd->function = funcptr;
   thd->data = data;
   thd->priority = priority;
#ifdef RTSYS_NONE
   thd->period = 0.0; /* Here just in case we want to wait_period in non-real-time mode */
   thd->last_called = 0;
#endif
   thd->mutex = bt_os_mutex_create(type);
   thd->type = type;
   
   /* Initialize sys-dependent stuff */
#ifdef RTSYS_XENOMAI
   if (type == BT_OS_RT)
   {
      int ret;
      RT_TASK * sys;
      sys = (RT_TASK *) malloc(sizeof(RT_TASK));
      thd->sys = (void *)sys;
      ret = rt_task_create(sys, name, 0, priority, T_JOINABLE);
      if(ret)
      {
         syslog(LOG_ERR, "btthread_xenomai_create: Could not create task! %d", ret);
         exit(-1);
      }
      ret = rt_task_start(sys, (void (*)(void *))funcptr, (void *)thd);
      if(ret)
      {
         syslog(LOG_ERR, "btthread_xenomai_create: Could not start task! %d", ret);
         exit(-1);
      }
   }
   else
#endif
   {
      struct pthd * sys;
      sys = (struct pthd *) malloc(sizeof(struct pthd));
      pthread_attr_init(&( sys->attr ));
      pthread_attr_setschedpolicy( &(sys->attr), SCHED_FIFO);
      pthread_attr_getschedparam( &(sys->attr), &(sys->param));
      sys->param.sched_priority = priority;
      pthread_attr_setschedparam( &(sys->attr), &(sys->param) );
      thd->sys = (void *)sys;
      
      pthread_create(&(sys->thd_id), &(sys->attr),
                     (void * (*)(void *))funcptr, (void *)thd);
      syslog(LOG_ERR,"Made thread ID: %d",(int)sys->thd_id);
      if (sys->thd_id == -1)
      {
         syslog(LOG_ERR,"btthread_create:Couldn't start control thread!");
         exit(-1);
      }
   }
   
#ifdef RTSYS_NONE
   if (thread_list_add(thd))
   {
      syslog(LOG_ERR,"Drastic linked list (add) error.");
      exit(-1);
   }
#endif
   return thd;
}


/* All this does (for now!) is free memory */
int bt_os_thread_destroy(struct bt_os_thread * thd)
{
   bt_os_mutex_destroy( thd->mutex );
   
   free( thd->sys );
   free( thd );

#ifdef RTSYS_NONE
   if (thread_list_remove(thd))
   {
      syslog(LOG_ERR,"Drastic linked list (remove) error.");
      exit(-1);
   }
#endif
   
   return 0;
}


/* See btthread_stop() */
BTINLINE int bt_os_thread_done(struct bt_os_thread * thd)
{
   int done;
   bt_os_mutex_lock( thd->mutex );
   done = thd->done;
   bt_os_mutex_unlock( thd->mutex );
   return done;
}

/** Stop a thread that is using btthread_done().
 
This function should only be called from outside the thread you want to stop.
The thread monitors thd->done using btthread_done().
\code
void mythread(void* args)
{
  btthread *mythd;
  mythd = (btthread*)args;
  
  while(!btthread_done(mythd))
  {
    //do something
  }
  pthread_exit(NULL);
}
\endcode
\internal chk'd TH 051101 
*/


BTINLINE void bt_os_thread_stop(struct bt_os_thread * thd)
{
   bt_os_mutex_lock( thd->mutex );
   thd->done = 1;
   bt_os_mutex_unlock( thd->mutex );
#ifdef RTSYS_XENOMAI
   if (thd->type == BT_OS_RT)
   {
      bt_os_rt_set_mode_hard();
      rt_task_join(thd->sys);
   }
   else
#endif
   {
      pthread_join(((struct pthd *)(thd->sys))->thd_id,NULL);
   }
}

/** Call pthread_exit() on this btthread object.
\internal chk'd TH 051101
*/
BTINLINE void bt_os_thread_exit(struct bt_os_thread * thd)
{
#ifdef RTSYS_XENOMAI
   if (thd->type == BT_OS_RT)
   {
      bt_os_rt_set_mode_hard();
      rt_task_delete(thd->sys);
   }
   else
#endif
   {
      pthread_exit(NULL);
   }
}



/****************Real Time Calls***************************/



void bt_os_rt_set_mode_soft()
{
#ifdef RTSYS_XENOMAI
   rt_task_set_mode( T_PRIMARY, 0, NULL);
#endif
#ifdef RTSYS_RTAI
   rt_make_soft_real_time();
#endif
   return;
}

void bt_os_rt_set_mode_hard()
{
#ifdef RTSYS_XENOMAI
   rt_task_set_mode(0, T_PRIMARY, NULL);
#endif
#ifdef RTSYS_RTAI
   rt_make_hard_real_time();
#endif
   return;
}


void bt_os_rt_set_mode_warn()
{
#ifdef RTSYS_XENOMAI
   rt_task_set_mode(0, T_WARNSW, NULL);
#endif
   return;
}


bt_os_rtime bt_os_rt_get_time(void)
{
#ifdef RTSYS_XENOMAI
   RTIME time;
   bt_os_rt_set_mode_hard();
   time = rt_timer_read(); 
   return (bt_os_rtime)time;
#endif
#ifdef RTSYS_RTAI
   RTIME time;
   time = rt_get_cpu_time_ns();
   return (bt_os_rtime)time;
#endif
#ifdef RTSYS_NONE
   struct timeval tv;
   gettimeofday(&tv,0);
   return 1e9 * (bt_os_rtime)tv.tv_sec + 1e3 * (bt_os_rtime)tv.tv_usec;
#endif
   return (bt_os_rtime)0;
}

/* Be sure to only call this once!
 * Note that sixname is a unique six-character string
 * used only in RTAI mode in rt_task_init() */
void bt_os_make_periodic(double period, char * sixname)
{
#ifdef RTSYS_RTAI
   RT_TASK * tsk;
#endif
   bt_os_rtime rtime_period;

#ifdef RTSYS_NONE
   {
      /* Determine which thread this is */
      bt_os_thread * cur;
      cur = thread_list_current();
      if (!cur)
      {
         syslog(LOG_ERR,"Drastic error, couldn't find thread in make_periodic.");
         exit(-1);
      }
      cur->period = period;
      return;
   }
#endif
   
   rtime_period = (period * 1000000000.0);
#ifdef RTSYS_RTAI
   /* Start the rt timer */
   rt_set_periodic_mode();
   start_rt_timer(nano2count(rtime_period));
   /* Do some task stuff */
   tsk = rt_task_init(nam2num(sixname), 5, 0, 0);
   rt_task_make_periodic_relative_ns(tsk, rtime_period, rtime_period);
#endif
#ifdef RTSYS_XENOMAI
#if 0
   signal(SIGXCPU, warn_upon_switch); /* Catch the SIGXCPU signal*/
   btrt_set_mode_warn(); /* Enable the warning*/
#endif
   /*Make task periodic*/
   test_and_log(
      rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks((rtime_period))),"WAMControlThread: rt_task_set_periodic failed, code");
#endif
}

void bt_os_rt_task_wait_period()
{
#ifdef RTSYS_XENOMAI
   rt_task_wait_period(0);
   return;
#endif
#ifdef RTSYS_RTAI
   rt_task_wait_period();
   return;
#endif
#ifdef RTSYS_NONE
   {
      bt_os_rtime now;
      bt_os_thread * cur;
      cur = thread_list_current();
      if (!cur)
      {
         syslog(LOG_ERR,"Drastic error, couldn't find thread in wait_period.");
         exit(-1);
      }
      now = bt_os_rt_get_time();
      if (!cur->last_called)
      {
         cur->last_called = now;
      }
      /* lastcalled --- now ----------------- period */
      bt_os_usleep( 1e6 * cur->period - 1e-3 * (now - cur->last_called));
      cur->last_called = bt_os_rt_get_time();
      return;
   }
#endif
}

void bt_os_rt_allow_nonroot()
{
#ifdef RTSYS_RTAI   
   rt_allow_nonroot_hrt();
#endif
#ifdef RTSYS_XENOMAI
   /* Xenomai non-root scheduling is coming soon! */   
#endif
}

struct bt_os_timestat * bt_os_timestat_create(int numchans)
{
   int i;
   struct bt_os_timestat * ts;
   ts = (struct bt_os_timestat *)malloc(sizeof(struct bt_os_timestat));
   
   ts->numchans = numchans;
   ts->numsamps = 0;
   
   ts->start = (bt_os_rtime *)malloc((numchans+1)*sizeof(bt_os_rtime));
   ts->times = ts->start+1;
   ts->mins = (bt_os_rtime *)malloc(numchans*sizeof(bt_os_rtime));
   ts->maxs = (bt_os_rtime *)malloc(numchans*sizeof(bt_os_rtime));
   ts->sums = (bt_os_rtime *)malloc(numchans*sizeof(bt_os_rtime));
   ts->sumsqs = (bt_os_rtime *)malloc(numchans*sizeof(bt_os_rtime));
   
   *(ts->start) = 0;
   for (i=0; i<numchans; i++)
   {
      ts->times[i] = 0;
      ts->mins[i] = 99999999;
      ts->maxs[i] = 0;
      ts->sums[i] = 0;
      ts->sumsqs[i] = 0;
   }
   
   return ts;
}
void bt_os_timestat_start(struct bt_os_timestat * ts)
{
   *(ts->start) = bt_os_rt_get_time();
}
void bt_os_timestat_trigger(struct bt_os_timestat * ts, int chan)
{
   ts->times[chan] = bt_os_rt_get_time();
}
void bt_os_timestat_end(struct bt_os_timestat * ts)
{
   int i;
   for (i=0; i<ts->numchans; i++)
   {
      bt_os_rtime diff;
      diff = ts->times[i] - ts->times[i-1];
      if (diff < ts->mins[i])
         ts->mins[i] = diff;
      if (diff > ts->maxs[i])
         ts->maxs[i] = diff;
      ts->sums[i] += diff;
      ts->sumsqs[i] += diff * diff;
   }
   ts->numsamps++;
   return;
}
void bt_os_timestat_get(struct bt_os_timestat * ts,
                       bt_os_rtime * means, bt_os_rtime * variances,
                       bt_os_rtime * mins, bt_os_rtime * maxs)
{
   int i;
   for (i=0; i<ts->numchans; i++)
   {
      means[i] = ts->sums[i] / ts->numsamps;
      variances[i] = (ts->sumsqs[i] / ts->numsamps)
                          - (means[i] * means[i]);
      mins[i] = ts->mins[i];
      maxs[i] = ts->maxs[i];
   }
   return;
}






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
 
