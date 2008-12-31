#include "refgen.h"

#include "spline.h"
#include "profile.h"

struct bt_refgen_move
{
   /* Include the base */
   struct bt_refgen base;
   
   /* The location of the elapsed time */
   double * elapsed_time;
   
   /* a bt_refgen_move has a spline and a profile */
   struct bt_spline * spline;
   struct bt_profile * profile;
   
};

/* refgen-specific creation function */
struct bt_refgen_move * bt_refgen_move_create(
   double * elapsed_time,
   gsl_vector * cur_pos, gsl_vector * cur_vel, gsl_vector * dest,
   double vel, double acc);
