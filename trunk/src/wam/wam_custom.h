/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wam.h
 *  Author ............. Sam Clanton
 *                       Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2004 Q3
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2004-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *    a high-level asynchronous non-realtime interface to the WAM
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt, split from btwam into wam and wambot
 *
 * ======================================================================== */

#ifndef BT_WAM_CUSTOM_H
#define BT_WAM_CUSTOM_H

/* For custom refgens */
#include "refgen.h"
#if 0
int bt_wam_local_refgen_use(struct bt_wam * wam, struct bt_refgen * refgen);

/* These are if you have your own refgen that you want to use */
int bt_wam_local_teach_start_custom(struct bt_wam * wam, struct bt_refgen * refgen);
int bt_wam_local_teach_end_custom(struct bt_wam * wam);
#endif

#endif /* BT_WAM_CUSTOM_H */
