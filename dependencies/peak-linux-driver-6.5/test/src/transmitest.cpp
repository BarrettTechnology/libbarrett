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
// transmitest_common.cpp - a simple program to test CAN transmits
//
// $Id: transmitest.cpp 518 2007-08-08 07:40:31Z edouard $
//
//****************************************************************************

//----------------------------------------------------------------------------
// set here current release for this program
#define CURRENT_RELEASE "Release_20060501_a"

//****************************************************************************
// INCLUDES

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
#include <src/parser.h>

//****************************************************************************
// DEFINES

#define DEFAULT_NODE "/dev/pcan0"

//****************************************************************************
// GLOBALS

HANDLE h;
char *current_release;
std::list<TPCANMsg> *List;
int nExtended = CAN_INIT_TYPE_ST;
//****************************************************************************
// CODE

#ifdef NO_RT
  #include "transmitest_linux.cpp"
#else
  #include "transmitest_rt.cpp"
#endif

static void hlpMsg(void)
{
  printf("transmitest - a small test program which sends CAN messages.\n");
  printf("usage:   transmitest filename {[-f=devicenode] | {[-t=type] [-p=port [-i=irq]]}} [-b=BTR0BTR1] [-e] [-?]\n");
  printf("options: filename - mandatory name of message description file.\n");
  printf("         -f - devicenode - path to devicefile, default=%s\n", DEFAULT_NODE);
  printf("         -t - type of interface, e.g. 'pci', 'sp', 'epp' ,'isa', 'pccard' or 'usb' (default: pci).\n");
  printf("         -p - port in hex notation if applicable, e.g. 0x378 (default: 1st port of type).\n");
  printf("         -i - irq in dec notation if applicable, e.g. 7 (default: irq of 1st port).\n");
  printf("         -b - BTR0BTR1 code in hex, e.g. 0x001C (default: 500 kbit).\n");
  printf("         -e - accept extended frames. (default: standard frames)\n");
  printf("         -? or --help - this help\n");
  printf("\n");
}

int main(int argc, char *argv[])
{
  char *ptr;
  int i;
  int nType = HW_PCI;
  __u32 dwPort = 0;
  __u16 wIrq = 0;
  __u16 wBTR0BTR1 = 0;
  char *filename = NULL;
  char *szDevNode = DEFAULT_NODE;
  bool bDevNodeGiven = false;
  bool bTypeGiven = false;
  parser MyParser;
  char txt[VERSIONSTRING_LEN];

  errno = 0;

  current_release = CURRENT_RELEASE;
  disclaimer("transmitest");

  // decode command line arguments
  for (i = 1; i < argc; i++) {
    char c;

    ptr = argv[i];

    if (*ptr == '-') {
      while (*ptr == '-')
        ptr++;

      c = *ptr;
      ptr++;

      if (*ptr == '=')
      ptr++;

      switch(tolower(c)) {
        case 'f':
          szDevNode = ptr;
          bDevNodeGiven = true;
          break;
        case 't':
          nType = getTypeOfInterface(ptr);
          if (!nType) {
            errno = EINVAL;
            printf("transmitest: unknown type of interface\n");
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
          printf("transmitest: unknown command line argument\n");
          goto error;
          break;
      }
    }
    else
      filename = ptr;
  }

  // test for filename
  if (filename == NULL) {
    errno = EINVAL;
    perror("transmitest: no filename given");
    goto error;
  }

  // test device node and type
  if (bDevNodeGiven && bTypeGiven) {
    errno = EINVAL;
    perror("transmitest: device node and type together is useless");
    goto error;
  }

  // give the filename to my parser
  MyParser.setFileName(filename);

  // tell some information to user
  if (!bTypeGiven) {
    printf("transmitest: device node=\"%s\"\n", szDevNode);
  }
  else {
    printf("transmitest: type=%s", getNameOfInterface(nType));
    if (nType == HW_USB) {
      printf(", Serial Number=default, Device Number=%d\n", dwPort); 
    }
    else {
      if (dwPort) {
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
    printf("             Extended frames are sent");
  else
    printf("             Only standard frames are sent");
  
  if (wBTR0BTR1)
    printf(", init with BTR0BTR1=0x%04x\n", wBTR0BTR1);
  else
    printf(", init with 500 kbit/sec.\n");
    printf("             Data will be read from \"%s\".\n", filename);

  /* install signal handler for manual break */
  install_signal_implement();

  /* get the list of data from parser */
  List = MyParser.Messages();
  if (!List)
  {
    errno = MyParser.nGetLastError();
    perror("transmitest: error at file read");
    goto error;
  }
  
  /* open CAN port */
  errno = open_can_implement(bDevNodeGiven, bTypeGiven, szDevNode, nType, dwPort, wIrq);
  if (errno) {
  	errno = nGetLastError();
    perror("transmitest: CAN_Open() or LINUX_CAN_Open()");
  	goto error;
  }
  
  /* clear status */
  CAN_Status(h);
  
  // get version info
  errno = CAN_VersionInfo(h, txt);
  if (!errno)
    printf("transmitest: driver version = %s\n", txt);
  else {
    perror("transmitest: CAN_VersionInfo()");
    goto error;
  }
  
  // init to a user defined bit rate
  if (wBTR0BTR1) {
    errno = CAN_Init(h, wBTR0BTR1, nExtended);
    if (errno) {
      perror("transmitest: CAN_Init()");
      goto error;
    }
  }
  // enter in the write loop
  errno = write_loop_implement();
  if (!errno)
    return 0;

  error:
    exit_implement(errno);
    return errno;
}
