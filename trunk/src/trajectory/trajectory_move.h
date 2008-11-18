#include "trajectory.h"

#include "spline.h"
#include "profile.h"

struct bt_trajectory_move
{
   /* Include the base */
   struct bt_trajectory base;
   
   /* a bt_trajectory_move has a spline and a profile */
   struct bt_spline * spline;
   struct bt_profile * profile;
   
};

/* Trajectory-specific create function */
struct bt_trajectory_move * bt_trajectory_move_create(
   gsl_vector * cur_pos, gsl_vector * cur_vel, gsl_vector * dest,
   double vel, double acc);
