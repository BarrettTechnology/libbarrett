/*
 * Copyright (C) 2001,2002 IDEALX (http://www.idealx.com/).
 * Written by Gilles Chanteperdrix <gilles.chanteperdrix@laposte.net>.
 * Copyright (C) 2003 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * This file satisfies the references within the emulator code
 * mimicking a VxWorks-like API built upon the XENOMAI nanokernel.
 *
 * VxWorks is a registered trademark of Wind River Systems, Inc.
 */

#ifndef _XENO_VXWORKS_VXWORKS_H
#define _XENO_VXWORKS_VXWORKS_H

#include <nucleus/types.h>

#define VXWORKS_SKIN_VERSION_STRING  "4"
#define VXWORKS_SKIN_VERSION_CODE    0x00000004
#define VXWORKS_SKIN_MAGIC           0x57494E44

#undef STATUS
typedef int STATUS;
typedef int BOOL;

#define OK    (0)
#define ERROR (-1)

/* errno values in some functions */
#define WIND_TASK_ERR_BASE  0x00030000
#define WIND_MEM_ERR_BASE   0x00110000
#define WIND_SEM_ERR_BASE   0x00160000
#define WIND_OBJ_ERR_BASE   0x003d0000
#define WIND_MSGQ_ERR_BASE  0x00410000
#define WIND_INT_ERR_BASE   0x00430000

#define S_objLib_OBJ_ID_ERROR                   (WIND_OBJ_ERR_BASE + 0x0001)
#define S_objLib_OBJ_UNAVAILABLE                (WIND_OBJ_ERR_BASE + 0x0002)
#define S_objLib_OBJ_DELETED                    (WIND_OBJ_ERR_BASE + 0x0003)
#define S_objLib_OBJ_TIMEOUT                    (WIND_OBJ_ERR_BASE + 0x0004)

#define S_taskLib_NAME_NOT_FOUND                (WIND_TASK_ERR_BASE + 0x0065)
#define S_taskLib_TASK_HOOK_NOT_FOUND           (WIND_TASK_ERR_BASE + 0x0067)
#define S_taskLib_ILLEGAL_PRIORITY              (WIND_TASK_ERR_BASE + 0x006d)

#define S_taskLib_TASK_HOOK_TABLE_FULL (WIND_TASK_ERR_BASE + 4) /* FIXME */

#define S_semLib_INVALID_STATE                  (WIND_SEM_ERR_BASE + 0x0065)
#define S_semLib_INVALID_OPTION                 (WIND_SEM_ERR_BASE + 0x0066)
#define S_semLib_INVALID_QUEUE_TYPE             (WIND_SEM_ERR_BASE + 0x0067)
#define S_semLib_INVALID_OPERATION              (WIND_SEM_ERR_BASE + 0x0068)

#define S_msgQLib_INVALID_MSG_LENGTH            (WIND_MSGQ_ERR_BASE + 0x0001)
#define S_msgQLib_NON_ZERO_TIMEOUT_AT_INT_LEVEL (WIND_MSGQ_ERR_BASE + 0x0002)
#define S_msgQLib_INVALID_QUEUE_TYPE            (WIND_MSGQ_ERR_BASE + 0x0003)

#define S_intLib_NOT_ISR_CALLABLE               (WIND_INT_ERR_BASE + 0x0001)

#define S_memLib_NOT_ENOUGH_MEMORY              (WIND_MEM_ERR_BASE + 0x0001)

/* defines for basic tasks handling */

/* Task Options: */

/* do not allow breakpoint debugging. */
#define VX_UNBREAKABLE   0x0002
/* execute with floating-point coprocessor support. */
#define VX_FP_TASK       0x0008
/* include private environment support (see envLib). */
#define VX_PRIVATE_ENV   0x0080
/* do not fill the stack for use by checkStack(). */
#define VX_NO_STACK_FILL 0x0100

/* defines for all kinds of semaphores */
#define SEM_Q_FIFO           0x0
#define SEM_Q_PRIORITY       0x1
#define SEM_DELETE_SAFE      0x4
#define SEM_INVERSION_SAFE   0x8
#define SEM_OPTION_MASK     (SEM_Q_FIFO|SEM_Q_PRIORITY| \
                             SEM_DELETE_SAFE|SEM_INVERSION_SAFE)

/* timeouts when waiting for semaphores */
#define NO_WAIT (0)
#define WAIT_FOREVER (-1)

typedef __natural_word_type SEM_ID;

/*for binary semaphores */
typedef enum {
    SEM_EMPTY =0,
    SEM_FULL =1
} SEM_B_STATE;

typedef __natural_word_type WDOG_ID;

typedef __natural_word_type MSG_Q_ID;

typedef __natural_word_type TASK_ID;

#define MSG_PRI_NORMAL   0
#define MSG_PRI_URGENT   1

#define MSG_Q_FIFO       0x0
#define MSG_Q_PRIORITY   0x1
#define WIND_MSG_Q_OPTION_MASK (MSG_Q_FIFO|MSG_Q_PRIORITY)

typedef unsigned int UINT;

typedef unsigned long ULONG;

typedef void (*FUNCPTR)(long, long, long, long, long, long, long, long, long, long);

typedef struct WIND_TCB_PLACEHOLDER {
    TASK_ID handle;
} WIND_TCB_PLACEHOLDER;

typedef void (*wind_timer_t)(long);
    
typedef struct wind_wd_utarget {

    wind_timer_t handler;
    long arg;

} wind_wd_utarget_t;

#if defined(__KERNEL__) || defined(__XENO_SIM__)

#include <nucleus/pod.h>
#include <nucleus/synch.h>

typedef struct wind_tcb {

    unsigned int magic;                  /* Magic code - must be first */

    /* The WIND task internal control block (which tends to be rather
       public in pre-6.0 versions of the VxWorks kernel). errorStatus
       is missing since we must handle the error code at nucleus
       level; applications should use errnoOfTaskGet/Set to access
       this field. */

    char name[XNOBJECT_NAME_LEN];
    int flags;
    int status;
    int prio;
    FUNCPTR entry;

    /* Xenomai specific: used by taskLib */

    int auto_delete;

    unsigned long int flow_id;

    int safecnt;
    xnsynch_t safesync;

    xnthread_t threadbase;

    xnholder_t link;        /* Link in wind_taskq */

#define link2wind_task(ln)  container_of(ln, wind_task_t, link)

    long arg0;
    long arg1;
    long arg2;
    long arg3;
    long arg4;
    long arg5;
    long arg6;
    long arg7;
    long arg8;
    long arg9;

    /* Xenomai specific: used by message queues */
    char * rcv_buf;             /* A place to save the receive buffer when this
                                   task is pending on a msgQReceive */
    unsigned int rcv_bytes;     /* this is the size passed to msgQReceive */
    
}  WIND_TCB;

static inline WIND_TCB *thread2wind_task(xnthread_t *t)
{
    return t ? container_of(t, WIND_TCB, threadbase) : NULL;
}

typedef void (*wind_create_hook)(WIND_TCB *);

typedef void (*wind_switch_hook)(WIND_TCB *, WIND_TCB *);

typedef void (*wind_delete_hook)(WIND_TCB *);

typedef void (*wind_tick_handler_t)(long);
    
#ifdef errno
#undef errno
#endif
#define errno (*wind_current_context_errno())

#ifdef __cplusplus
extern "C" {
#endif

WIND_TCB *taskTcb(TASK_ID task_id);

STATUS taskRestart(TASK_ID task_id);

static inline void taskHookInit(void)
{
}
    
STATUS taskCreateHookAdd(wind_create_hook hook);

STATUS taskCreateHookDelete(wind_create_hook hook);

STATUS taskSwitchHookAdd(wind_switch_hook hook);

STATUS taskSwitchHookDelete(wind_switch_hook hook);

STATUS taskDeleteHookAdd(wind_delete_hook hook);

STATUS taskDeleteHookDelete(wind_delete_hook hook);

int intCount(void);
    
int intLevelSet(int level);

int intLock(void);

void intUnlock(int flags);

STATUS sysClkConnect(wind_tick_handler_t routine,
		     long arg);

void tickAnnounce(void);

int *wind_current_context_errno(void);

#ifdef __cplusplus
}
#endif

#else /* !(__KERNEL__ || __XENO_SIM__) */

#include <vxworks/syscall.h>

typedef WIND_TCB_PLACEHOLDER WIND_TCB;

#endif /* __KERNEL__ || __XENO_SIM__ */

#ifdef __cplusplus
extern "C" {
#endif

void printErrno(int status);
    
STATUS errnoSet(int status);

int errnoGet(void);

int errnoOfTaskGet(TASK_ID task_id);

STATUS errnoOfTaskSet(TASK_ID task_id, int status);

TASK_ID taskSpawn(const char *name,
		  int prio,
		  int flags,
		  int stacksize,
		  FUNCPTR entry,
		  long arg0, long arg1, long arg2, long arg3, long arg4,
		  long arg5, long arg6, long arg7, long arg8, long arg9);

STATUS taskInit(WIND_TCB *pTcb,
		const char *name,
		int prio,
		int flags,
		char * stack __attribute__ ((unused)),
		int stacksize,
		FUNCPTR entry,
		long arg0, long arg1, long arg2, long arg3, long arg4,
		long arg5, long arg6, long arg7, long arg8, long arg9);

STATUS taskActivate(TASK_ID task_id);

STATUS taskDelete(TASK_ID task_id);

STATUS taskDeleteForce(TASK_ID task_id);

STATUS taskSuspend(TASK_ID task_id);

STATUS taskResume(TASK_ID task_id);

STATUS taskPrioritySet(TASK_ID task_id,
		       int prio);

STATUS taskPriorityGet(TASK_ID task_id,
		       int *pprio);

void taskExit(int code);
    
STATUS taskLock(void);

STATUS taskUnlock(void);

TASK_ID taskIdSelf(void);
    
STATUS taskSafe(void);

STATUS taskUnsafe(void);
    
STATUS taskDelay(int ticks);

STATUS taskIdVerify(TASK_ID task_id);

const char *taskName(TASK_ID task_id);

TASK_ID taskNameToId(const char *name);

TASK_ID taskIdDefault(TASK_ID task_id);
    
BOOL taskIsReady(TASK_ID task_id);

BOOL taskIsSuspended (TASK_ID task_id);
         
STATUS semGive(SEM_ID sem_id);

STATUS semTake(SEM_ID sem_id,
	       int timeout);

STATUS semFlush(SEM_ID sem_id);

STATUS semDelete(SEM_ID sem_id);

SEM_ID semBCreate(int flags,
		  SEM_B_STATE state);

SEM_ID semMCreate(int flags);

SEM_ID semCCreate(int flags,
		  int count);

WDOG_ID wdCreate(void);

STATUS wdDelete(WDOG_ID wdog_id);

STATUS wdStart(WDOG_ID wdog_id,
	       int timeout,
	       wind_timer_t handler,
	       long arg);

STATUS wdCancel(WDOG_ID wdog_id);

MSG_Q_ID msgQCreate(int nb_msgs,
		    int length,
		    int flags);

STATUS msgQDelete(MSG_Q_ID msg);

int msgQNumMsgs(MSG_Q_ID msg);

int msgQReceive(MSG_Q_ID msg,
		char *buf,
		UINT bytes,
		int timeout);

STATUS msgQSend(MSG_Q_ID msg,
		const char *buf,
		UINT bytes,
		int timeout,
		int prio);

BOOL intContext(void);
    
void sysClkDisable(void);

void sysClkEnable(void);

int sysClkRateGet(void);
    
STATUS sysClkRateSet(int ticksPerSecond);
    
ULONG tickGet(void);

void tickSet(ULONG ticks);

STATUS kernelTimeSlice(int ticks);
      
const char *kernelVersion(void);
    
#ifdef __cplusplus
}
#endif

#endif /* !_XENO_VXWORKS_VXWORKS_H */
