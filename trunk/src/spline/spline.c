/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... spline.c
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

#include "spline.h"
#include "interp.h"

#include <gsl/gsl_blas.h>

#include <math.h> /* For sqrt() */

#include <syslog.h>

/* Public functions */

struct bt_spline * bt_spline_create( gsl_vector * start )
{
   int i;
   struct bt_spline * spline;
   
   spline = (struct bt_spline *) malloc(sizeof(struct bt_spline));
   spline->dimension = start->size;
   
   /* Add the start point */
   spline->npoints = 1;
   spline->points = (double **) malloc((spline->dimension)*sizeof(double *));
   for (i=0; i<spline->dimension; i++)
   {
      spline->points[i] = (double *) malloc(sizeof(double));
      spline->points[i][0] = gsl_vector_get(start,i);
   }
   
   spline->ss = 0;
   spline->length = 0.0;
   
   spline->interps = 0;
   spline->acc = 0;
      
   return spline;
}

int bt_spline_add( struct bt_spline * spline, gsl_vector * vec )
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

   /* Woo, one more point! */
   spline->npoints++;
   
   return 0;
}

int bt_spline_init( struct bt_spline * spline,
                    gsl_vector * start, gsl_vector * direction )
{
   int i;
   int n;
   
   /* Reset the start position if we're asked to */
   if (start)
      for (i=0; i<spline->dimension; i++)
         spline->points[i][0] = gsl_vector_get(start,i);
   
   /* Unitize the direction vector if it was passed */
   if (direction)
   {
      if (gsl_blas_dnrm2(direction) == 0)
         direction = 0;
      else
         gsl_vector_scale( direction, 1.0 / gsl_blas_dnrm2(direction) );
   }
   
   /* Compute the arc-lengths array ss[] */
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

int bt_spline_destroy( struct bt_spline * spline )
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

int bt_spline_get( struct bt_spline * spline, gsl_vector * result, double s )
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

