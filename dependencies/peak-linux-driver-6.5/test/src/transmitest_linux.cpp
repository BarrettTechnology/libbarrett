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
// transmitest_linux.cpp - a simple program to test CAN transmits
//
// $Id: transmitest_linux.cpp $
//
//****************************************************************************
// INCLUDE

//****************************************************************************
// GLOBALS

//****************************************************************************
// CODE

void exit_implement(int error)
{
  if (h)
  {
    print_diag("transmitest");
    CAN_Close(h);
  }
  printf("transmitest: finished (%d).\n\n", error);
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

int write_loop_implement()
{
// write out endless loop until Ctrl-C
  while (1)
  {
    std::list<TPCANMsg>::iterator iter;
    int i;
    for (iter = List->begin(); iter != List->end(); iter++)
    {
      // test for standard frames only
      if ((nExtended == CAN_INIT_TYPE_EX) || !(iter->MSGTYPE & MSGTYPE_EXTENDED)){
        // send the message
        if ((errno = CAN_Write(h, &(*iter)))){
          perror("transmitest: CAN_Write()");
          return errno;
        }
      }
    }
  }

  return 0;
}
