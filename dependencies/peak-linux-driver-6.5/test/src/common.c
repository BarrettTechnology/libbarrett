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
//
// common.c - common parts for transmittest and receivetest
//
// $Id: common.c 454 2007-02-11 22:18:01Z khitschler $
//
//****************************************************************************

//****************************************************************************
// INCLUDES
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>   // exit
#include <signal.h>
#include <string.h>
#include <stdlib.h>   // strtoul
#include <src/common.h> 

//****************************************************************************
// DEFINES

//****************************************************************************
// GLOBALS

//****************************************************************************
// LOCALS

//****************************************************************************
// CODE 

// print GPL disclaimer
void disclaimer(char *prgName)
{
  printf("\n");
  printf("%s Version \"%s\"  (www.peak-system.com)\n", prgName, current_release);
  printf("------- Copyright (C) 2004-2007 PEAK System-Technik GmbH ------\n");
  printf("%s comes with ABSOLUTELY NO WARRANTY.     This is free\n", prgName);
  printf("software  and you are welcome  to redistribute it under certain\n");
  printf("conditions.   For   details   see    attached   COPYING   file.\n");
  printf("\n"); 
} 
 
// print out the contents of a CAN message  
void print_message(TPCANMsg *m)
{
  int i;
  
  // print RTR, 11 or 29, CAN-Id and datalength
  printf("receivetest: %c %c 0x%08x %1d  ", 
      (m->MSGTYPE & MSGTYPE_RTR)      ? 'r' : 'm',
      (m->MSGTYPE & MSGTYPE_EXTENDED) ? 'e' : 's',
       m->ID, 
       m->LEN); 

	// don't print any telegram contents for remote frames
  if (!(m->MSGTYPE & MSGTYPE_RTR))
  	for (i = 0; i < m->LEN; i++)
    	printf("0x%02x ", m->DATA[i]);
    
  printf("\n");
}

// lookup for HW_... constant out of device type string
int getTypeOfInterface(char *szTypeName)
{	  
  int nType = 0;
    
  if (!strcmp(szTypeName, "pci"))
	  nType = HW_PCI;
	else
	{
		if (!strcmp(szTypeName, "isa"))
	    nType = HW_ISA_SJA;
		else
		{
		  if (!strcmp(szTypeName, "sp"))
	      nType = HW_DONGLE_SJA;
		  else
      {
		    if (!strcmp(szTypeName, "epp"))
	        nType = HW_DONGLE_SJA_EPP;
        else
        {
          if (!strcmp(szTypeName, "usb"))
  	        nType = HW_USB;
          else
          {
            if (!strcmp(szTypeName, "pccard"))
  	          nType = HW_PCCARD;
          }
        }
      }
		}
	}
	
	return nType;
}

// the opposite: lookup for device string out of HW_.. constant
char *getNameOfInterface(int nType)
{
  switch (nType)
  {
    case HW_PCI:            return "pci";
    case HW_ISA_SJA:        return "isa";
    case HW_DONGLE_SJA:     return "sp";
    case HW_DONGLE_SJA_EPP: return "epp";
    case HW_USB:            return "usb";
    case HW_PCCARD:         return "pccard";
    
    default:                return "unknown";
  }
}

// print out device and channel diagnostics
void print_diag(char *prgName)
{
  int err;
  TPDIAG diag;
  
  err = LINUX_CAN_Statistics(h, &diag);
  if (err)
    printf("%s: can't read diagnostics, error %d!\n", prgName, err);
  else      
  {
    printf("%s: type            = %s\n",              prgName, getNameOfInterface(diag.wType));
    if (diag.wType == HW_USB)
    {
      printf("             Serial Number   = 0x%08x\n", diag.dwBase);
      printf("             Device Number   = %d\n",     diag.wIrqLevel);
    }
    else
    {
      printf("             io              = 0x%08x\n", diag.dwBase);
      printf("             irq             = %d\n",     diag.wIrqLevel);
    }
    printf("             count of reads  = %d\n",     diag.dwReadCounter);
    printf("             count of writes = %d\n",     diag.dwWriteCounter);
    printf("             count of errors = %d\n",     diag.dwErrorCounter);
    printf("             count of irqs   = %d\n",     diag.dwIRQcounter);
    printf("             last CAN status = 0x%04x\n", diag.wErrorFlag);
    printf("             last error      = %d\n",     diag.nLastError);
    printf("             open paths      = %d\n",     diag.nOpenPaths);
    printf("             driver version  = %s\n",     diag.szVersionString);    
  }  
}

