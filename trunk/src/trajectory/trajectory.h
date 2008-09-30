/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... trajectory.h
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

#ifndef BT_TRAJECTORY_H
#define BT_TRAJECTORY_H

#include <gsl/gsl_vector.h>
#include <gsl/gsl_interp.h>

/* A spline built on a gsl interpolator,
 * and is a map from quasi-arc-length to n-dimensional position */
struct bt_trajectory_spline {
   int dimension;
   int npoints;
   double * ss;
   double length;
   double ** points;
   gsl_interp_accel * acc;
   gsl_interp ** interps;
};

struct bt_trajectory_spline * bt_trajectory_spline_create( gsl_vector * start );
int bt_trajectory_spline_add( struct bt_trajectory_spline * spline, gsl_vector * vec );
int bt_trajectory_spline_init( struct bt_trajectory_spline * spline,
                              gsl_vector * start, gsl_vector * direction );
int bt_trajectory_spline_destroy( struct bt_trajectory_spline * spline );
int bt_trajectory_spline_get( struct bt_trajectory_spline * spline, gsl_vector * result, double s );


/* A profile is simply a function from time to arc length.
 * For now, it simply does trapezoidal profiles,
 * given a path length, max vel, and max acc. */
struct bt_trajectory_profile {
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

struct bt_trajectory_profile * bt_trajectory_profile_create(double vel, double acc, double v_init, double length);
int bt_trajectory_profile_destroy( struct bt_trajectory_profile * profile );
int bt_trajectory_profile_get( struct bt_trajectory_profile * profile, double * s, double t );

#endif /* BT_TRAJECTORY_H */
