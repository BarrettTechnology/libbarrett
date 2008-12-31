#include "trajectory.h"
#include "trajectory_move.h"

#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>

/* Define the type */
static int destroy(struct bt_trajectory * base);
static int get_num_points(struct bt_trajectory * base);
static int get_total_time(struct bt_trajectory * base, double * time);
static int get_start(struct bt_trajectory * base, gsl_vector ** start);
static int get_reference(struct bt_trajectory * base, gsl_vector * ref, double time);
static const struct bt_trajectory_type bt_trajectory_move_type = {
   "move",
   &destroy,
   &get_num_points,
   &get_total_time,
   &get_start,
   &get_reference
};
const struct bt_trajectory_type * bt_trajectory_move = &bt_trajectory_move_type;

/* Trajectory-specific create function */
struct bt_trajectory_move * bt_trajectory_move_create(
   gsl_vector * cur_pos, gsl_vector * cur_vel, gsl_vector * dest,
   double vel, double acc)
{
   struct bt_trajectory_move * t;
   t = (struct bt_trajectory_move *) malloc( sizeof(struct bt_trajectory_move) );
   if (!t) return 0;
   
   /* Set the type */
   t->base.type = bt_trajectory_move;
   
   /* Make a new spline, starting at the current position */
   t->spline = bt_spline_create(cur_pos);
   
   /* Add the destination as a second point */
   bt_spline_add( t->spline, dest );
   
   /* Initialize the spline, using the velocity as the direction */
   bt_spline_init( t->spline, 0, cur_vel );
   
   /* Make a new profile, using the spline length */
   t->profile = bt_profile_create(vel, acc, gsl_blas_dnrm2(cur_vel), t->spline->length);
   
   return t;
}

/* Interface functions (from trajectory.h) */
static int destroy(struct bt_trajectory * base)
{
   struct bt_trajectory_move * t;
   t = (struct bt_trajectory_move *) base;
   bt_spline_destroy(t->spline);
   bt_profile_destroy(t->profile);
   free(t);
   return 0;
}

static int get_num_points(struct bt_trajectory * base)
{
   return 0;
}

static int get_total_time(struct bt_trajectory * base, double * time)
{
   struct bt_trajectory_move * t;
   t = (struct bt_trajectory_move *) base;
   (*time) = t->profile->time_end;
   return 0;
}

static int get_start(struct bt_trajectory * base, gsl_vector ** start)
{
   return 0;
}

static int get_reference(struct bt_trajectory * base, gsl_vector * ref, double time)
{
   struct bt_trajectory_move * t;
   double s;
   t = (struct bt_trajectory_move *) base;
   
   bt_profile_get( t->profile, &s, time );
   bt_spline_get( t->spline, ref, s );
   
   return 0;
}
