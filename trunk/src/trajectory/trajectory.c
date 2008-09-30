/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... trajectory.c
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

#include "trajectory.h"
#include "interp.h"

#include <gsl/gsl_blas.h>

#include <math.h> /* For sqrt() */

#include <syslog.h>

/* Public functions */

struct bt_trajectory_spline * bt_trajectory_spline_create( gsl_vector * start )
{
   int i;
   struct bt_trajectory_spline * spline;
   
   spline = (struct bt_trajectory_spline *) malloc(sizeof(struct bt_trajectory_spline));
   spline->dimension = start->size;
   
   /* Add the start poing */
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

int bt_trajectory_spline_add( struct bt_trajectory_spline * spline, gsl_vector * vec )
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

int bt_trajectory_spline_init( struct bt_trajectory_spline * spline,
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
      spline->ss[n] = spline->ss[n-1] + sqrt(len);
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

int bt_trajectory_spline_destroy( struct bt_trajectory_spline * spline )
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

int bt_trajectory_spline_get( struct bt_trajectory_spline * spline, gsl_vector * result, double s )
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



struct bt_trajectory_profile * bt_trajectory_profile_create(double vel, double acc, double v_init, double length)
{
   struct bt_trajectory_profile * profile;
   double v_diff;
   
   profile = (struct bt_trajectory_profile *) malloc(sizeof(struct bt_trajectory_profile));
   
   /* Save parameters */
   profile->vel = vel;
   profile->acc = acc;
   profile->v_init = v_init;
   
   /* First, is the length too short to decellerate to stop?
    * |\
    * | \
    * |  \__ */
   if ( length < 0.5 * v_init * v_init / acc )
   {
      /* There are no up or plateau phases */
      profile->time_endup = 0.0;
      profile->s_endup = 0.0;
      profile->time_startdown = 0.0;
      profile->s_startdown = 0.0;
      
      /* The rampdown phase is simple ... */
      profile->time_end = 2 * length / v_init;
      profile->s_end = length;
      
      /* We need to readjust the acceleration */
      profile->acc = v_init / profile->time_end;
      
      return profile;
   }
   
   
   /* OK, do we not have enough space to plateau?
    * |
    * |/\
    * |  \__ */
   v_diff = vel - v_init;
   if ( length < (0.5*vel*vel + 0.5*v_diff*v_diff + v_init*v_diff)/acc )
   {
      double v_top;
      v_top = sqrt( length * acc + 0.5 * v_init * v_init );
      
      profile->time_endup = (v_top - v_init) / acc;
      profile->s_endup = v_init * profile->time_endup + 0.5 * acc * profile->time_endup * profile->time_endup;
      
      profile->time_startdown = profile->time_endup;
      profile->s_startdown = profile->s_endup;
      
      profile->time_end = profile->time_startdown + v_top / acc;
      profile->s_end = profile->s_startdown + 0.5 * v_top * v_top / acc; /* Let's home this is length! */
      
      return profile;
   }
   
   /* OK, we're going to plateau, either up or down
    * | __        |\__
    * |/  \    or |   \
    * |    \__    |    \__ */
   profile->time_endup = fabs(v_diff) / acc;
   profile->s_endup = profile->time_endup * (vel + v_init) / 2;
   
   /* Compute the ramp down portion */
   profile->s_startdown = length - 0.5 * vel * vel / acc;
   profile->time_startdown = profile->time_endup + (profile->s_startdown - profile->s_endup) / vel;
   
   profile->s_end = length;
   profile->time_end = profile->time_startdown + vel / acc;

   return profile;
}

int bt_trajectory_profile_destroy( struct bt_trajectory_profile * profile )
{
   free(profile);
   return 0;
}

/* returns:
 *  1 : before beginning
 *  2 : after end */
int bt_trajectory_profile_get( struct bt_trajectory_profile * p, double * s, double t )
{
   if (t < 0.0)
   {
      *s = 0.0;
      return 1;
   }
   
   /* Are we ramping up? */
   if (t < p->time_endup)
   {
      *s = p->v_init * t;
      if (p->v_init < p->vel)
         *s += 0.5 * p->acc * t * t;
      else
         *s -= 0.5 * p->acc * t * t;
      return 0;
   }
   /* Are we plateau-ed? */
   if (t < p->time_startdown)
   {
      *s = p->s_endup + p->vel * (t - p->time_endup);
      return 0;
   }
   /* Are we ramping down? */
   if (t < p->time_end)
   {
      *s = p->s_end - 0.5 * p->acc * (t - p->time_end) * (t - p->time_end);
      return 0;
   }
   
   *s = p->s_end;
   return 2;
}


