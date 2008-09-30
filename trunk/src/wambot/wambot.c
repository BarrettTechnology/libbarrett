/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wambot.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2003 Feb 15
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2003-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *    wam-specific low-level functions
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt
 *
 * ======================================================================== */

#include "wambot.h"

int bt_wambot_update( struct bt_wambot * wambot )
{
   return wambot->update(wambot);
}

int bt_wambot_setjtor( struct bt_wambot * wambot )
{
   return wambot->setjtor(wambot);
}
