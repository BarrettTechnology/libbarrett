/** Implementation of bt_spline, an n-dimensional vector interpolator.
 *
 * \file spline.c
 * \author Christopher Dellin
 * \date 2008-2009
 */

/* Copyright 2008, 2009
 *           Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */

#include <math.h> /* For sqrt() */
#include <syslog.h>

#include <gsl/gsl_blas.h>

#include <barrett/cdlbt/spline.h>
#include <barrett/cdlbt/interp.h>


int bt_spline_create(struct bt_spline ** splineptr, const gsl_vector * start,
                     enum bt_spline_mode mode)
{
   int i;
   struct bt_spline * spline;

   (*splineptr) = 0;
   spline = (struct bt_spline *) malloc(sizeof(struct bt_spline));
   spline->dimension = start->size;
   spline->mode = mode;
   
   /* Add the start point */
   spline->npoints = 1;
   spline->points = (double **) malloc((spline->dimension)*sizeof(double *));
   for (i=0; i<spline->dimension; i++)
   {
      spline->points[i] = (double *) malloc(sizeof(double));
      spline->points[i][0] = gsl_vector_get(start,i);
   }
   
   /* If the mode is EXTERNAL, compute the ss array in-place
    * (if it's ARCLEN, we'll compute it at the end) */
   if (spline->mode == BT_SPLINE_MODE_ARCLEN)
      spline->ss = 0;
   else if (spline->mode == BT_SPLINE_MODE_EXTERNAL)
   {
      spline->ss = (double *) malloc( 1 * sizeof(double) );
      spline->ss[0] = 0.0;
   }
   spline->length = 0.0;
   
   spline->interps = 0;
   spline->acc = 0;

   (*splineptr) = spline;
   return 0;
}


int bt_spline_add(struct bt_spline * spline, const gsl_vector * vec, double s)
{
   int i;
   
   /* Make sure we haven't already made the interps
    * (then it'd be too late) */
   if (spline->interps)
      return -1;
   
   /* Add to the point arrays */
   for (i=0; i<spline->dimension; i++)
   {
      spline->points[i] = (double *) realloc(spline->points[i],(spline->npoints+1)*sizeof(double));
      spline->points[i][spline->npoints] = gsl_vector_get(vec,i);
   }
   
   /* If the mode is EXTERNAL, save the ss array in place;
    * (if it's ARCLEN, we'll compute it at the end) */
   if (spline->mode == BT_SPLINE_MODE_EXTERNAL)
   {
      spline->ss = (double *) realloc( spline->ss, (spline->npoints+1)*sizeof(double) );
      spline->ss[spline->npoints] = s;
   }

   /* Woo, one more point! */
   spline->npoints++;
   
   return 0;
}


int bt_spline_init(struct bt_spline * spline, gsl_vector * start,
                   gsl_vector * direction)
{
   int i;
   int n;
   
   /* Reset the start position if we're asked to */
   if (start && (spline->mode == BT_SPLINE_MODE_ARCLEN))
      for (i=0; i<spline->dimension; i++)
         spline->points[i][0] = gsl_vector_get(start,i);
   
   /* Unitize the direction vector if it was passed */
   if (direction && (spline->mode == BT_SPLINE_MODE_ARCLEN))
   {
      if (gsl_blas_dnrm2(direction) == 0)
         direction = 0;
      else
         gsl_vector_scale( direction, 1.0 / gsl_blas_dnrm2(direction) );
   }
   
   /* Compute the arc-lengths array ss[] (if we're in ARCLEN mode) */
   if (spline->mode == BT_SPLINE_MODE_ARCLEN)
   {
      if (spline->ss) free(spline->ss);
      spline->ss = (double *) malloc( (spline->npoints) * sizeof(double) );
      spline->ss[0] = 0.0;
      for (n=1; n<spline->npoints; n++)
      {
         double diff;
         double len;
         len = 0.0;
         for (i=0; i<spline->dimension; i++)
         {
            diff = spline->points[i][n] - spline->points[i][n-1];
            len += diff * diff;
         }
         /* FIX ME THE RIGHT WAY! */
         if (sqrt(len) != 0.0)
            spline->ss[n] = spline->ss[n-1] + sqrt(len);
         else
            spline->ss[n] = spline->ss[n-1] + 0.00001;
         
      }
   }
   spline->length = spline->ss[spline->npoints-1];
   
   /* Create the interps (if they don't already exist)  */
   if (!spline->interps)
   {
      spline->interps = (gsl_interp **) malloc((spline->dimension)*sizeof(gsl_interp *));
      for (i=0; i<spline->dimension; i++) spline->interps[i] = 0;
   }
   for (i=0; i<spline->dimension; i++)
   {
      if (spline->interps[i]) gsl_interp_free(spline->interps[i]);
      spline->interps[i] = gsl_interp_alloc( bt_interp, spline->npoints );
      
      if (direction)
      {
         bt_interp_set_type( spline->interps[i], BT_INTERP_SLOPE, BT_INTERP_NATURAL );
         bt_interp_set_slopes( spline->interps[i], gsl_vector_get(direction,i), 0.0 );
      }
      else
         bt_interp_set_type( spline->interps[i], BT_INTERP_NATURAL, BT_INTERP_NATURAL );
      
      gsl_interp_init( spline->interps[i], spline->ss, spline->points[i], spline->npoints);
   }
   
   /* Create an accelerator */
   spline->acc = gsl_interp_accel_alloc();
   
   return 0;
}


int bt_spline_destroy(struct bt_spline * spline)
{
   int i;
   
   for (i=0; i<spline->dimension; i++)
      free(spline->points[i]);
   free(spline->points);
   
   if (spline->interps)
   {
      for (i=0; i<spline->dimension; i++)
         if (spline->interps[i]) gsl_interp_free(spline->interps[i]);
      free(spline->interps);
   }
   
   if (spline->ss) free(spline->ss);
   if (spline->acc) gsl_interp_accel_free(spline->acc);
   
   free(spline);
   return 0;
}


int bt_spline_get(struct bt_spline * spline, gsl_vector * result, double s)
{
   int i;
   for (i=0; i<spline->dimension; i++)
   {
      double coord;
      coord = gsl_interp_eval( spline->interps[i], spline->ss, spline->points[i], s, spline->acc );
      gsl_vector_set( result, i, coord );
   }
   return 0;
}
