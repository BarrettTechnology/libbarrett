//****************************************************************************
// Copyright (C) 2001-2006  PEAK System-Technik GmbH
//
// linux@peak-system.com 
// www.peak-system.com
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// Maintainer(s): Klaus Hitschler (klaus.hitschler@gmx.de)
//****************************************************************************

//****************************************************************************
// realtime part
// receivetest.c - a small program to test the receive features of pcan driver 
//                 and the supporting shared library
//
// $Id: receivetest_rt.c 517 2007-07-09 09:40:42Z edouard $
//
//****************************************************************************

//****************************************************************************
// INCLUDE

#ifdef XENOMAI
  #include <sys/mman.h>
  #include <native/task.h>
  #include <native/timer.h>
#endif

#ifdef RTAI
  #include <sys/poll.h>
  #include <sys/mman.h>
  #include <rtai_lxrt.h>
#endif

//****************************************************************************
// DEFINES

#define STATE_FILE_OPENED         1
#define STATE_TASK_CREATED        2

//****************************************************************************
// GLOBALS

unsigned int current_state = 0;
int shutdownnow = 0;

#ifdef XENOMAI
  RT_TASK reading_task;
#endif

#ifdef RTAI
  RT_TASK *reading_task;
  RT_TASK *mainr;
  static pthread_t reading_thr;
#endif

void install_signal_implement(void);
//****************************************************************************
// CODE

void exit_implement(int error)
{
  if (current_state & STATE_FILE_OPENED) {
    print_diag("receivetest");
    CAN_Close(h);
    current_state &= ~STATE_FILE_OPENED;
  }

  #ifdef XENOMAI
    if (current_state & STATE_TASK_CREATED) {
      rt_task_delete(&reading_task);
      current_state &= ~STATE_TASK_CREATED;
    }
  #endif

  #ifdef RTAI
    pthread_join(reading_thr, NULL);
    if (current_state & STATE_TASK_CREATED) {
      rt_task_delete(reading_task);
      current_state &= ~STATE_TASK_CREATED;
    }
    rt_task_delete(mainr);
    stop_rt_timer();
  #endif
}

void signal_handler(int signal)
{
  #ifdef RTAI
    rt_make_soft_real_time();
  #endif
  install_signal_implement();
  shutdownnow = 1;
  exit_implement(0);
  printf("receivetest: finished(0)\n");
}

void install_signal_implement(void)
{
  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);
}

// open the CAN port
int open_can_implement(bool bDevNodeGiven,bool bTypeGiven,char *szDevNode,int nType,__u32 dwPort,__u16 wIrq)
{
  int err = 0;
  if ((bDevNodeGiven) || (!bDevNodeGiven && !bTypeGiven)) {
    h = LINUX_CAN_Open(szDevNode, O_RDWR);
    if (h)
      current_state |= STATE_FILE_OPENED;
    else {
      printf("receivetest: can't open %s\n", szDevNode);
      err = 1;
    }
  }
  else {
    // please use what is appropriate
    // HW_DONGLE_SJA
    // HW_DONGLE_SJA_EPP
    // HW_ISA_SJA
    // HW_PCI
    h = CAN_Open(nType, dwPort, wIrq);
    if (h)
      current_state |= STATE_FILE_OPENED;
    else {
      printf("receivetest: can't open %s device.\n", getNameOfInterface(nType));
      err = 1;
    }
  }
  return err;
}

//----------------------------------------------------------------------------
// real time task
void reading_task_proc(void *arg)
{
  TPCANMsg m;
  __u32 status;
  
  #ifdef RTAI
    reading_task = rt_task_init_schmod(nam2num("RDTSK"), 1, 0, 0, SCHED_FIFO, 0xF);
    rt_make_hard_real_time();
  #endif
  
  while (1) {
    if ((errno = CAN_Read(h, &m)))
      shutdownnow = 1;
    else {
      // check if a CAN status is pending
      if (m.MSGTYPE & MSGTYPE_STATUS) {
        status = CAN_Status(h);
        if ((int)status < 0)
          shutdownnow = 1;
      }
    }
    if (shutdownnow == 1) break;
  }
}

int read_loop_implement(void)
{
  install_signal_implement();
  mlockall(MCL_CURRENT | MCL_FUTURE);

  #ifdef XENOMAI
    errno = rt_timer_set_mode(TM_ONESHOT);
    if (errno) {
      printf("receivetest: error while configuring timer\n");
      return errno;
    }
    errno = rt_task_create(&reading_task,NULL,0,50,0);
    if (errno) {
      printf("receivetest: Failed to create rt task, code %d\n",errno);
      return errno;
    }
    // start reading_task
    errno = rt_task_start(&reading_task,&reading_task_proc,NULL);
    if (errno) {
      printf("receivetest: Failed to start rt task, code %d\n",errno);
      return errno;
    }
  #endif

  #ifdef RTAI
    /* Init scheduler */
    rt_set_oneshot_mode();
    /* Start timer */
    start_rt_timer(0);
    mainr = rt_task_init_schmod(nam2num("MAINR"), 3, 0, 0, SCHED_FIFO, 0xF);
    errno = pthread_create(&reading_thr, NULL, (void *)reading_task_proc, NULL);
    if (errno) {
      printf("receivetest: Failed to create and start rt task, code %d\n",errno);
      return errno;
    }
  #endif

  current_state |= STATE_TASK_CREATED;
  printf("receivetest: reading data from CAN ... (press Ctrl-C to exit)\n");

  pause();

  return 0;
}
