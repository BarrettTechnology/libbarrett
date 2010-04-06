/** Implementation of a set of operating-system wrappers for threads,
 *  mutexes, and timing real-time loops.
 *
 * \file os.c
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

#define _GNU_SOURCE
#include <stdlib.h>
#include <unistd.h> /* For usleep() */
#include <syslog.h>
#include <pthread.h>

#ifdef RTSYS_XENOMAI
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#endif

#ifdef RTSYS_RTAI
#include <rtai_lxrt.h>
#include <rtai_sem.h>
#endif

#ifdef RTSYS_NONE
#include <sys/time.h>
#endif
#undef _GNU_SOURCE

#include "os.h"


int bt_os_rt_allow_nonroot()
{
#ifdef RTSYS_RTAI   
   rt_allow_nonroot_hrt();
#endif
#ifdef RTSYS_XENOMAI
   /* Xenomai non-root scheduling is coming soon! */   
#endif
   return 0;
}


int bt_os_usleep(int useconds)
{
   return usleep(useconds);
}


/* Note - create() used to not have the mattr stuff for RTAI
 * (it used NULL in pthread_mutex_init)
 * What's the difference? */
int bt_os_mutex_create(struct bt_os_mutex ** mutexptr,
                       enum bt_os_rt_type type)
{
   int err;
   (*mutexptr) = 0;
   struct bt_os_mutex * m; /* The return mutex abstraction */
   m = (struct bt_os_mutex *) malloc(sizeof(struct bt_os_mutex));
   
#ifdef RTSYS_XENOMAI
   if (type == BT_OS_RT)
   {
      RT_MUTEX * sys_mutex;
      sys_mutex = (RT_MUTEX *) malloc(sizeof(RT_MUTEX));
      bt_os_rt_set_mode_hard(); /* Why must we be in realtime mode? */
      err = rt_mutex_create(sys_mutex,NULL);
      if (err)
      {
         syslog(LOG_ERR,"%s: Could not initialize mutex.",__func__);
         free(sys_mutex);
         free(m);
         return -1;
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
      err = pthread_mutex_init(sys_mutex,&mattr);
      if (err)
      {
         syslog(LOG_ERR,"%s: Could not initialize mutex.",__func__);
         free(sys_mutex);
         free(m);
         return -1;
      }
      m->p = (void *)sys_mutex;
   }

   /* Save mutex type */
   m->type = type;
   (*mutexptr) = m;
   return 0;
}


int bt_os_mutex_destroy(struct bt_os_mutex * m)
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
      pthread_mutex_destroy((pthread_mutex_t *)(m->p));
   }
   
   free(m->p);
   free(m);
   return 0;
}


int bt_os_mutex_lock(struct bt_os_mutex * m)
{
   int err;
   
   /* Attempt the lock */
#ifdef RTSYS_XENOMAI
   if (m->type == BT_OS_RT)
   {
      bt_os_rt_set_mode_hard();
      err = rt_mutex_acquire((RT_MUTEX *)(m->p), TM_INFINITE);
   }
   else
#endif
   {
      err = pthread_mutex_lock((pthread_mutex_t *)(m->p));
   }
   
   /* Log on error */
   if (err)
      syslog(LOG_ERR,"%s: Mutex lock failed (?): %d",__func__,err);
   return err;
}


int bt_os_mutex_unlock(struct bt_os_mutex * m)
{
   int err;
   
   /* Attempt the lock */
#ifdef RTSYS_XENOMAI
   if (m->type == BT_OS_RT)
   {
      bt_os_rt_set_mode_hard();
      err = rt_mutex_release((RT_MUTEX *)(m->p));
   }
   else
#endif
   {
      err = pthread_mutex_unlock((pthread_mutex_t *)(m->p));
   }
   
   /* Log on error */
   if (err)
      syslog(LOG_ERR, "%s: Mutex unlock failed (?): %d", __func__,err);
   return err;
}


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


int bt_os_thread_create(struct bt_os_thread ** threadptr,
                        enum bt_os_rt_type type, const char * name,
                        int priority, void (*funcptr)(struct bt_os_thread *),
                        void * data)
{
   struct bt_os_thread * thd;
   
   /* Allocate space for a new thread */
   (*threadptr) = 0;
   thd = (struct bt_os_thread *) malloc(sizeof(struct bt_os_thread));
   if (!thd)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return -1;
   }
   
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
   thd->mutex = 0;
   thd->type = type;
   
   bt_os_mutex_create(&thd->mutex,type);
   if (!thd->mutex)
   {
      syslog(LOG_ERR,"%s: Could not create thread mutex.",__func__);
      bt_os_thread_destroy(thd);
      return -1;
   }
   
   /* Initialize sys-dependent stuff */
#ifdef RTSYS_XENOMAI
   if (type == BT_OS_RT)
   {
      int err;
      RT_TASK * sys;
      sys = (RT_TASK *) malloc(sizeof(RT_TASK));
      if (!sys)
      {
         syslog(LOG_ERR, "%s: Out of memory.", __func__);
         bt_os_thread_destroy(thd);
         return -1;
      } 
      thd->sys = (void *)sys;
      err = rt_task_create(sys, name, 0, priority, T_JOINABLE);
      if(err)
      {
         syslog(LOG_ERR, "%s: Could not create task! %d", __func__,err);
         bt_os_thread_destroy(thd);
         return -1;
      }
      err = rt_task_start(sys, (void (*)(void *))funcptr, (void *)thd);
      if(err)
      {
         syslog(LOG_ERR, "%s: Could not start task! %d", __func__,err);
         bt_os_thread_destroy(thd);
         return -1;
      }
   }
   else
#endif
   {
      struct pthd * sys;
      sys = (struct pthd *) malloc(sizeof(struct pthd));
      if (!sys)
      {
         syslog(LOG_ERR, "%s: Out of memory.", __func__);
         bt_os_thread_destroy(thd);
         return -1;
      }
      pthread_attr_init(&( sys->attr ));
      pthread_attr_setschedpolicy( &(sys->attr), SCHED_FIFO);
      pthread_attr_getschedparam( &(sys->attr), &(sys->param));
      sys->param.sched_priority = priority;
      pthread_attr_setschedparam( &(sys->attr), &(sys->param) );
      thd->sys = (void *)sys;
      
      pthread_create(&(sys->thd_id), &(sys->attr),
                     (void * (*)(void *))funcptr, (void *)thd);
      if ((int)(sys->thd_id) == -1)
      {
         syslog(LOG_ERR, "%s: Couldn't start thread.", __func__);
         bt_os_thread_destroy(thd);
         return -1;
      }
   }
   
#ifdef RTSYS_NONE
   if (thread_list_add(thd))
   {
      syslog(LOG_ERR,"Drastic linked list (add) error.");
      exit(-1);
   }
#endif

   (*threadptr) = thd;
   return 0;
}


/* All this does (for now!) is free memory */
int bt_os_thread_destroy(struct bt_os_thread * thd)
{
   if (thd->mutex)
      bt_os_mutex_destroy(thd->mutex);
   
   if (thd->sys)
      free(thd->sys);

#ifdef RTSYS_NONE
   if (thread_list_remove(thd))
   {
      syslog(LOG_ERR,"Drastic linked list (remove) error.");
      exit(-1);
   }
#endif
   
   free(thd);
   return 0;
}


int bt_os_thread_stop(struct bt_os_thread * thd)
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
   return 0;
}


int bt_os_thread_isdone(struct bt_os_thread * thd)
{
   int done;
   bt_os_mutex_lock( thd->mutex );
   done = thd->done;
   bt_os_mutex_unlock( thd->mutex );
   return done;
}


int bt_os_thread_exit(struct bt_os_thread * thd)
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
   return 0;
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


int bt_os_rt_set_mode_soft(void)
{
#ifdef RTSYS_XENOMAI
   rt_task_set_mode( T_PRIMARY, 0, NULL);
#endif
#ifdef RTSYS_RTAI
   rt_make_soft_real_time();
#endif
   return 0;
}


int bt_os_rt_set_mode_hard(void)
{
#ifdef RTSYS_XENOMAI
   rt_task_set_mode(0, T_PRIMARY, NULL);
#endif
#ifdef RTSYS_RTAI
   rt_make_hard_real_time();
#endif
   return 0;
}


int bt_os_rt_set_mode_warn(void)
{
#ifdef RTSYS_XENOMAI
   rt_task_set_mode(0, T_WARNSW, NULL);
#endif
   return 0;
}


int bt_os_rt_make_periodic(double period, char * sixname)
{
   int err;
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
      return 0;
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
   /* Make task periodic */
   err = rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks((rtime_period)));
   if (err)
   {
      syslog(LOG_ERR,"%s: rt_task_set_periodic failed!",__func__);
   }
#endif
   return 0;
}


int bt_os_rt_task_wait_period(void)
{
#ifdef RTSYS_XENOMAI
   rt_task_wait_period(0);
   return 0;
#endif
#ifdef RTSYS_RTAI
   rt_task_wait_period();
   return 0;
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
      return 0;
   }
#endif
}


int bt_os_timestat_create(struct bt_os_timestat ** tsptr, int numchans)
{
   int i;
   struct bt_os_timestat * ts;

   (*tsptr) = 0;
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

   (*tsptr) = ts;
   return 0;
}


int bt_os_timestat_destroy(struct bt_os_timestat * ts)
{
   free(ts->start);
   free(ts->mins);
   free(ts->maxs);
   free(ts->sums);
   free(ts->sumsqs);
   free(ts);
   return 0;
}


int bt_os_timestat_start(struct bt_os_timestat * ts)
{
   *(ts->start) = bt_os_rt_get_time();
   return 0;
}


int bt_os_timestat_trigger(struct bt_os_timestat * ts, int chan)
{
   ts->times[chan] = bt_os_rt_get_time();
   return 0;
}


int bt_os_timestat_end(struct bt_os_timestat * ts)
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
   return 0;
}


int bt_os_timestat_get(struct bt_os_timestat * ts,
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
   return 0;
}
