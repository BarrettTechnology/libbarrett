/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... can.h
 *  Author ............. Brian Zenowich
 *                       Traveler Hauptman
 *                       Sam Clanton
 *                       Christopher Dellin
 *  Creation Date ...... 24 Mar 2003
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2003-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2003 Mar 24 - BZ
 *      File created & documented.
 *    2004 Dec 16 - BZ, SC, TH
 *      Initial port to linux + RTAI
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt
 *                                                                          *
 * ======================================================================== */

/** \file can.h
Handles all communication with the robot over the CAN bus.
    Requires library files "libcan.a" and "libmitop.a".
*/

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif
/* #define D(x) <blank> ...for no debug info */
/* #define D(x) x       ...for debug info */
#define D(x)
#define MAX_NODES    (31)

#define L08       (1)
#define L16       (2)
#define L24       (3)
#define L32       (4)

#define EE        (0x0008)

#define mbxID               (0)
#define BASE_ID             (0)

#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define NODE2ADDR(x) (((mbxID + BASE_ID) << 5) | ((x) + BASE_ID))
#define GROUPID(n)   (((mbxID + BASE_ID) << 5) | (0x0400 + (n)))
#define BROADCAST    (GROUPID(0))

/* Public Data Structures */
/** CAN device information data structure
*/

struct can_device;

/* Public Functions */

struct can_device * can_create(int port);
void can_destroy(struct can_device * dev);
int can_clearmsg(struct can_device * dev);

void can_iterate_start(struct can_device * dev);
int can_iterate_next(struct can_device * dev,
                      int * nextid, int * nextstatus);

/* Do we need this? */
void can_set_max_property(int prop);
int can_get_property(struct can_device * dev, int who, int property, long *reply);
int can_set_property(struct can_device * dev, int who, int property, int verify, long value);

int can_get_packed(struct can_device * dev, int group, int howMany, long *pos, int pos_prop);
int can_set_torques(struct can_device * dev, int group, int *values, int torque_prop);

#ifdef __cplusplus
}
#endif/* __cplusplus */

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003-2008 Barrett Technology, Inc.            *
 *                        625 Mount Auburn St                           *
 *                    Cambridge, MA  02138,  USA                        *
 *                                                                      *
 *                        All rights reserved.                          *
 *                                                                      *
 *  ******************************************************************  *
 *                            DISCLAIMER                                *
 *                                                                      *
 *  This software and related documentation are provided to you on      *
 *  an as is basis and without warranty of any kind.  No warranties,    *
 *  express or implied, including, without limitation, any warranties   *
 *  of merchantability or fitness for a particular purpose are being    *
 *  provided by Barrett Technology, Inc.  In no event shall Barrett     *
 *  Technology, Inc. be liable for any lost development expenses, lost  *
 *  lost profits, or any incidental, special, or consequential damage.  *
 *======================================================================*/
 

