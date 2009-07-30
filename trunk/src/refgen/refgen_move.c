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
static int eval(struct bt_refgen * base, double time, gsl_vector * ref);
static const struct bt_refgen_type bt_refgen_move_type = {
   "move",
   0, /* create */
   &destroy,
   0, 0, 0, 0, 0, /* teach functions */
   0, 0, /* load/save functions */
   &get_start,
   &get_total_time,
   &get_num_points,
   &start,
   &eval
};
const struct bt_refgen_type * bt_refgen_move = &bt_refgen_move_type;


/* refgen-specific creation function */
struct bt_refgen * bt_refgen_move_create(
   gsl_vector * cur_pos, gsl_vector * cur_vel, gsl_vector * dest,
   double vel, double acc)
{
   struct bt_refgen_move * r;
   
   r = (struct bt_refgen_move *) malloc( sizeof(struct bt_refgen_move) );
   if (!r) return 0;
   
   /* Set the type */
   r->base.type = bt_refgen_move;
   
   /* Make a new spline, starting at the current position */
   r->spline = bt_spline_create(cur_pos,BT_SPLINE_MODE_ARCLEN);
   
   /* Add the destination as a second point */
   bt_spline_add( r->spline, dest, 0.0 ); /* The 0.0 is meaningless for ARCLEN type */
   
   /* Initialize the spline, using the velocity as the direction */
   bt_spline_init( r->spline, 0, cur_vel );
   
   /* Make a new profile, using the spline length */
   if (cur_vel)
      r->profile = bt_profile_create(vel, acc, gsl_blas_dnrm2(cur_vel), r->spline->length);
   else
      r->profile = bt_profile_create(vel, acc, 0, r->spline->length);
   
   return (struct bt_refgen *)r;
}


static int destroy(struct bt_refgen * base)
{
   struct bt_refgen_move * r;
   r = (struct bt_refgen_move *) base;
   bt_spline_destroy(r->spline);
   bt_profile_destroy(r->profile);
   free(r);
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

static int eval(struct bt_refgen * base, double time, gsl_vector * ref)
{
   struct bt_refgen_move * r;
   double s;
   r = (struct bt_refgen_move *) base;
   
   if ( time > r->profile->time_end )
      return 1; /* finished */
   
   bt_profile_get( r->profile, &s, time );
   bt_spline_get( r->spline, ref, s );
   
   return 0;
}
