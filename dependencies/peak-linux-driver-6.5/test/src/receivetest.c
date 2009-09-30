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
// common part between no-realtime and realtime
// receivetest_common.c - a small program to test the receive features of pcan driver 
//                 and the supporting shared library
//
// $Id: receivetest.c 518 2007-08-08 07:40:31Z edouard $
//
//****************************************************************************

// set here current release for this program
#define CURRENT_RELEASE "Release_20060501_a"

//****************************************************************************
// INCLUDE

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>   // exit
#include <signal.h>
#include <string.h>
#include <stdlib.h>   // strtoul
#include <fcntl.h>    // O_RDWR
#include <libpcan.h>
#include <src/common.h>
#include <ctype.h>

//****************************************************************************
// DEFINES

#define DEFAULT_NODE "/dev/pcan0"
#ifndef bool
  #define bool  int
  #define true  1
  #define false 0
#endif

//****************************************************************************
// GLOBALS

HANDLE h;
char *current_release;

//****************************************************************************
// CODE

#ifdef NO_RT
  #include "receivetest_linux.c"
#else
  #include "receivetest_rt.c"
#endif

static void hlpMsg(void)
{
  printf("receivetest - a small test program which receives and prints CAN messages.\n");
  printf("usage:   receivetest {[-f=devicenode] | {[-t=type] [-p=port [-i=irq]]}} [-b=BTR0BTR1] [-e] [-?]\n");
  printf("options: -f - devicenode - path to devicefile, default=%s\n", DEFAULT_NODE);
  printf("         -t - type of interface, e.g. 'pci', 'sp', 'epp', 'isa', 'pccard' or 'usb' (default: pci).\n");
  printf("         -p - port in hex notation if applicable, e.g. 0x378 (default: 1st port of type).\n");
  printf("         -i - irq in dec notation if applicable, e.g. 7 (default: irq of 1st port).\n");
  printf("         -b - BTR0BTR1 code in hex, e.g. 0x001C (default: 500 kbit).\n");
  printf("         -e - accept extended frames. (default: standard frames)\n");
  printf("         -? or --help - this help\n");
  printf("\n");
}

// here all is done
int main(int argc, char *argv[])
{
  char *ptr;
  int i;
  int nType = HW_PCI;
  __u32 dwPort = 0;
  __u16 wIrq = 0;
  __u16 wBTR0BTR1 = 0;
  int   nExtended = CAN_INIT_TYPE_ST;
  char  *szDevNode = DEFAULT_NODE;
  bool bDevNodeGiven = false;
  bool bTypeGiven    = false;
  char txt[VERSIONSTRING_LEN];

  errno = 0;

  current_release = CURRENT_RELEASE;
  disclaimer("receivetest");

  // decode command line arguments
  for (i = 1; i < argc; i++)
  {
    char c;

    ptr = argv[i];

    while (*ptr == '-')
      ptr++;

    c = *ptr;
    ptr++;

    if (*ptr == '=')
      ptr++;

    switch(tolower(c))
    {
      case 'f':
        szDevNode = ptr;
        bDevNodeGiven = true;
        break;
      case 't':
        nType = getTypeOfInterface(ptr);
        if (!nType){
          errno = EINVAL;
          printf("receivetest: unknown type of interface!\n");
          goto error;
        }
        bTypeGiven = true;
        break;
      case 'p':
        dwPort = strtoul(ptr, NULL, 16);
        break;
      case 'i':
        wIrq   = (__u16)strtoul(ptr, NULL, 10);
        break;
      case 'e':
        nExtended = CAN_INIT_TYPE_EX;
        break;
      case '?':
      case 'h':
        hlpMsg();
        goto error;
        break;
      case 'b':
        wBTR0BTR1 = (__u16)strtoul(ptr, NULL, 16);
        break;
      default:
        errno = EINVAL;
        perror("receivetest: unknown command line argument!\n");
        goto error;
        break;
    }
  }
  // simple command input check
  if (bDevNodeGiven && bTypeGiven){
    errno = EINVAL;
    perror("receivetest: device node and type together is useless");
    goto error;
  }

  // give some information back
  if (!bTypeGiven){
    printf("receivetest: device node=\"%s\"\n", szDevNode);
  }
  else{
    printf("receivetest: type=%s", getNameOfInterface(nType));
    if (nType == HW_USB){
      if (dwPort)
        printf(", %d. device\n", dwPort);
      else
        printf(", standard device\n");
    }
    else{
      if (dwPort){
        if (nType == HW_PCI)
          printf(", %d. PCI device", dwPort);
        else
          printf(", port=0x%08x", dwPort);
      }
      else
        printf(", port=default");
      if ((wIrq) && !(nType == HW_PCI))
        printf(" irq=0x%04x\n", wIrq);
      else
        printf(", irq=default\n");
    }
  }

  if (nExtended == CAN_INIT_TYPE_EX)
    printf("             Extended frames are accepted");
  else
    printf("             Only standard frames are accepted");
  if (wBTR0BTR1)
    printf(", init with BTR0BTR1=0x%04x\n", wBTR0BTR1);
  else
    printf(", init with 500 kbit/sec.\n");

  // install signal handler for the specific implementation
  install_signal_implement();

  /* open the CAN port */
  errno = open_can_implement(bDevNodeGiven, bTypeGiven, szDevNode, nType, dwPort, wIrq);
  if (errno) {
    errno = nGetLastError();
    perror("receivetest: CAN_Open() or LINUX_CAN_Open()");
    goto error;
  }

  /* clear status */
  //  CAN_Status(h);

 // get version info
  errno = CAN_VersionInfo(h, txt);
  if (!errno)
    printf("receivetest: driver version = %s\n", txt);
  else {
    perror("receivetest: CAN_VersionInfo()");
    goto error;
  }

  // init to a user defined bit rate
  if (wBTR0BTR1)
  {
    errno = CAN_Init(h, wBTR0BTR1, nExtended);
    if (errno) {
      perror("receivetest: CAN_Init()");
      goto error;
    }
  }
  errno = read_loop_implement();
  if (!errno)
    return 0;

  error:
    exit_implement(errno);
    return errno;
}
