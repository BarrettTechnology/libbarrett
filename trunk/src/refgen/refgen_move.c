#include "refgen.h"
#include "refgen_move.h"

#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>

/* Define the type (see refgen.h for details) */
static int destroy(struct bt_refgen * base);
static int get_start(struct bt_refgen * base, gsl_vector ** start);
static int get_total_time(struct bt_refgen * base, double * time);
static int get_num_points(struct bt_refgen * base, int * points);
static int start(struct bt_refgen * base);
static int eval(struct bt_refgen * base, gsl_vector * ref);
static const struct bt_refgen_type bt_refgen_move_type = {
   "move",
   &destroy,
   &get_start,
   &get_total_time,
   &get_num_points,
   &start,
   &eval
};
const struct bt_refgen_type * bt_refgen_move = &bt_refgen_move_type;

/* refgen-specific creation function */
struct bt_refgen_move * bt_refgen_move_create(
   double * elapsed_time,
   gsl_vector * cur_pos, gsl_vector * cur_vel, gsl_vector * dest,
   double vel, double acc)
{
   struct bt_refgen_move * t;
   t = (struct bt_refgen_move *) malloc( sizeof(struct bt_refgen_move) );
   if (!t) return 0;
   
   /* Set the type */
   t->base.type = bt_refgen_move;
   
   /* Save the elapsed time */
   t->elapsed_time = elapsed_time;
   
   /* Make a new spline, starting at the current position */
   t->spline = bt_spline_create(cur_pos,BT_SPLINE_MODE_ARCLEN);
   
   /* Add the destination as a second point */
   bt_spline_add( t->spline, dest, 0.0 ); /* The 0.0 is meaningless for ARCLEN type */
   
   /* Initialize the spline, using the velocity as the direction */
   bt_spline_init( t->spline, 0, cur_vel );
   
   /* Make a new profile, using the spline length */
   if (cur_vel)
      t->profile = bt_profile_create(vel, acc, gsl_blas_dnrm2(cur_vel), t->spline->length);
   else
      t->profile = bt_profile_create(vel, acc, 0, t->spline->length);
   
   return t;
}

/* Interface functions (from refgen.h) */
static int destroy(struct bt_refgen * base)
{
   struct bt_refgen_move * t;
   t = (struct bt_refgen_move *) base;
   bt_spline_destroy(t->spline);
   bt_profile_destroy(t->profile);
   free(t);
   return 0;
}

static int get_start(struct bt_refgen * base, gsl_vector ** start)
{
   return -1; /* Not Implemented */
}

static int get_total_time(struct bt_refgen * base, double * time)
{
   struct bt_refgen_move * t;
   t = (struct bt_refgen_move *) base;
   (*time) = t->profile->time_end;
   return 0;
}

static int get_num_points(struct bt_refgen * base, int * points)
{
   (*points) = 2;
   return 0;
}

static int start(struct bt_refgen * base)
{
   return -1; /* Not Implemented */
}

static int eval(struct bt_refgen * base, gsl_vector * ref)
{
   struct bt_refgen_move * t;
   double s;
   t = (struct bt_refgen_move *) base;
   
   if ( *(t->elapsed_time) > t->profile->time_end )
      return 1; /* finished */
   
   bt_profile_get( t->profile, &s, *(t->elapsed_time) );
   bt_spline_get( t->spline, ref, s );
   
   return 0;
}
