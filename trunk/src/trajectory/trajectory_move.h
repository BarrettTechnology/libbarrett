#include "trajectory.h"

struct bt_trajectory_move
{
   /* Include the base */
   struct bt_control base;
   
   /* a bt_trajectory_move has a spline and a profile */
   bt_spline * spline;
   bt_profile * profile;
   
};

/* Trajectory-specific create/destroy functions */
struct bt_trajectory_move * bt_trajectory_move_create(gsl_vector * pos, gsl_vector * vel, gsl_vector * dest);
int bt_trajectory_move_destroy(struct bt_trajectory_move * t);
