/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wambot.h
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
 *    wam-specific low-level functions
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt, split from btwam into wam and wambot
 *
 * ======================================================================== */

#ifndef BT_WAMBOT_H
#define BT_WAMBOT_H

/* bt_wambot uses gsl :-) */
#include <gsl/gsl_vector.h>

struct bt_wambot
{
   int dof;
   
   /* Communication with actuators, interface with bot */
   gsl_vector * jtorque; /* Nm */
   gsl_vector * jposition; /* rad */
   gsl_vector * jvelocity; /* rad/s */
   gsl_vector * jacceleration; /* rad/s/s */
   
   /* Constant stuff to be read from config file */
   gsl_vector * home; /* rad */
   
   /* wambot function pointers */
   /*struct bt_wambot * bt_wambot_create( config_setting_t * wamconfig );
   int bt_wambot_destroy( struct bt_wambot * wambot );*/

   int (*update)( struct bt_wambot * wambot );
   int (*setjtor)( struct bt_wambot * wambot );
};

/* Shortcut Functions */
int bt_wambot_update( struct bt_wambot * wambot );
int bt_wambot_setjtor( struct bt_wambot * wambot );

#endif /* BT_WAMBOT_H */
