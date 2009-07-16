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

#ifndef BT_WAMBOT_PHYS_H
#define BT_WAMBOT_PHYS_H

#include "wambot.h"

#include "bus.h"

/* btwam uses gsl :-) */
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/* bt_wambot_phys uses libconfig */
#include <libconfig.h>

struct bt_wambot_phys
{
   struct bt_wambot base;

   /* Constant stuff to be read from config file */
   struct bt_bus * bus; /* The bt_bus this WAM is on */
   gsl_vector * zeromag; /* enc counts */
   gsl_matrix * j2mp; /* ratios */
   
   /* Constant cache stuff computed from config file stuff above */
   gsl_matrix * m2jp; /* inverse of j2mp */
   gsl_matrix * j2mt; /* inverse of j2mp^T */
   
   /* temporary storage locations */
   gsl_vector * mposition; /* rad */
   gsl_vector * mvelocity; /* rad */
   gsl_vector * macceleration; /* rad */
   gsl_vector * mtorque; /* Nm */
   
};

struct bt_wambot_phys * bt_wambot_phys_create( config_setting_t * config );
int bt_wambot_phys_destroy( struct bt_wambot_phys * wambot );

#endif /* BT_WAMBOT_PHYS_H */
