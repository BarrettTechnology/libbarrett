/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... spline.h
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

#ifndef BT_SPLINE_H
#define BT_SPLINE_H

#include <gsl/gsl_vector.h>
#include <gsl/gsl_interp.h>

/* A spline built on a gsl interpolator,
 * and is a map from quasi-arc-length to n-dimensional position */
struct bt_spline {
   int dimension;
   int npoints;
   double * ss;
   double length;
   double ** points;
   gsl_interp_accel * acc;
   gsl_interp ** interps;
};

struct bt_spline * bt_spline_create( gsl_vector * start );
int bt_spline_add( struct bt_spline * spline, gsl_vector * vec );
int bt_spline_init( struct bt_spline * spline,
                              gsl_vector * start, gsl_vector * direction );
int bt_spline_destroy( struct bt_spline * spline );
int bt_spline_get( struct bt_spline * spline, gsl_vector * result, double s );


#endif /* BT_SPLINE_H */
