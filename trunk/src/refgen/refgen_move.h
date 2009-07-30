#include "refgen.h"

#include "spline.h"
#include "profile.h"

/* Publicize the type */
const struct bt_refgen_type * bt_refgen_move;

struct bt_refgen_move
{
   /* Include the base */
   struct bt_refgen base;
   
   /* a bt_refgen_move has a spline and a profile */
   struct bt_spline * spline;
   struct bt_profile * profile;
   
};

/* refgen-specific creation function */
struct bt_refgen * bt_refgen_move_create(
   gsl_vector * cur_pos, gsl_vector * cur_vel, gsl_vector * dest,
   double vel, double acc);
