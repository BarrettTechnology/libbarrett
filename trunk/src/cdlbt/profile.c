/** Implementation of bt_profile, a trapezoidal profile mapping from time
 *  to arc-length.
 *
 * \file profile.c
 * \author Christopher Dellin
 * \date 2008-2009
 */

/* Copyright 2008, 2009 Barrett Technology <support@barrett.com> */

/*
 * This file is part of libbarrett.
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

#include <barrett/cdlbt/profile.h>
#include <barrett/cdlbt/interp.h>

/* Public functions */

int bt_profile_create(struct bt_profile ** profileptr, double vel,
                      double acc, double v_init, double length)
{
   struct bt_profile * profile;
   double v_diff;

   (*profileptr) = 0;
   profile = (struct bt_profile *) malloc(sizeof(struct bt_profile));
   
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
      
      (*profileptr) = profile;
      return 0;
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
      
      (*profileptr) = profile;
      return 0;
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

   (*profileptr) = profile;
   return 0;
}

int bt_profile_destroy( struct bt_profile * profile )
{
   free(profile);
   return 0;
}

/* returns:
 *  1 : before beginning
 *  2 : after end */
int bt_profile_get( struct bt_profile * p, double * s, double t )
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


