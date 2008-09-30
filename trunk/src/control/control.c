/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Sam Clanton
 *                       Christopher Dellin
 *  Creation Date ...... Nov 24, 2002
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2002 Nov 24 - TH
 *      File created.
 *    2004 Dec 16 - BZ, SC, TH
 *      Initial port to linux + RTAI
 *    2008 Sept 16 - CD
 *      Ported from btsystem to libbt; btstatecontrol and btcontrol merged
 *
 * ======================================================================== */

#include "control.h"

char * bt_control_mode_names[] =
{
   "IDLE",
   "IDLE_TEACH_DISCRETE",
   "IDLE_TEACH_CONTINUOUS",
   "HOLD",
   "TRAJ_PREP",
   "TRAJ_READY",
   "TRAJ_MOVING",
   "TRAJ_PAUSING",
   "TRAJ_PAUSED",
   "TRAJ_UNPAUSING",
   "OTHER"
};

char * bt_control_mode_name(int mode)
{
   return bt_control_mode_names[mode];
}



