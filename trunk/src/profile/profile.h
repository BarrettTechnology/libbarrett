/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... profile.h
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2005 Mar 30
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt; merged from btstatecontrol and btpath
 *
 * ======================================================================== */

#ifndef BT_PROFILE_H
#define BT_PROFILE_H

#include <gsl/gsl_vector.h>
#include <gsl/gsl_interp.h>

/* A profile is simply a function from time to arc length.
 * For now, it simply does trapezoidal profiles,
 * given a path length, max vel, and max acc. */
struct bt_profile {
   double vel; /* Note - we may want this to be different for different dimensions */
   double acc;
   double v_init;
   double time_endup;
   double time_startdown;
   double time_end;
   double s_endup;
   double s_startdown;
   double s_end;
};

struct bt_profile * bt_profile_create(double vel, double acc, double v_init, double length);
int bt_profile_destroy( struct bt_profile * profile );
int bt_profile_get( struct bt_profile * profile, double * s, double t );

#endif /* BT_PROFILE_H */
