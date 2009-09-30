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
// no-realtime part
// receivetest_linux.c - a small program to test the receive features of pcan driver 
//                 and the supporting shared library
//
// $Id: receivetest_linux.c $
//
//****************************************************************************

//****************************************************************************
// INCLUDE

//****************************************************************************
// GLOBALS

//****************************************************************************
// CODE

void exit_implement(int error)
{
  if (h) {
    print_diag("receivetest");
    CAN_Close(h);
  }
  printf("receivetest: finished (%d).\n\n", error);
  exit(error);
}

void signal_handler(int signal)
{
  exit_implement(0);
}

void install_signal_implement(void)
{
  signal(SIGINT, signal_handler);
}

// open the CAN port
int open_can_implement(bool bDevNodeGiven,bool bTypeGiven,char *szDevNode,int nType,__u32 dwPort,__u16 wIrq)
{
  int err = 0;
  if ((bDevNodeGiven) || (!bDevNodeGiven && !bTypeGiven)) {
    h = LINUX_CAN_Open(szDevNode, O_RDWR);
  }
  else {
		// please use what is appropriate
		// HW_DONGLE_SJA
		// HW_DONGLE_SJA_EPP
		// HW_ISA_SJA
		// HW_PCI
		// HW_USB
		h = CAN_Open(nType, dwPort, wIrq);
  }
  if (!h) {
    return 1;
  }
  return err;
}

int read_loop_implement()
{
  // read in endless loop until Ctrl-C
  while (1) {
    TPCANMsg m;
    __u32 status;
    if ((errno = CAN_Read(h, &m))) {
      perror("receivetest: CAN_Read()");
      return errno;
    }
    else {
      print_message(&m);
      // check if a CAN status is pending
      if (m.MSGTYPE & MSGTYPE_STATUS) {
        status = CAN_Status(h);
        if ((int)status < 0) {
          errno = nGetLastError();
          perror("receivetest: CAN_Status()");
          return errno;
        }
        else
          printf("receivetest: pending CAN status 0x%04x read.\n", (__u16)status);
      }
    }
  }

  return 0;
}
