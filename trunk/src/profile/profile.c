/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... profile.c
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

#include "profile.h"
#include "interp.h"

#include <gsl/gsl_blas.h>

#include <math.h> /* For sqrt() */

#include <syslog.h>

/* Public functions */

struct bt_profile * bt_profile_create(double vel, double acc, double v_init, double length)
{
   struct bt_profile * profile;
   double v_diff;
   
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


