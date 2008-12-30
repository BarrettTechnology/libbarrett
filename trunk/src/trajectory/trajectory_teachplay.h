#include "trajectory.h"

#include "log.h"

#include "spline.h"
#include "profile.h"

#include <gsl/gsl_vector.h>

/* Publicize the type */
const struct bt_trajectory_type * bt_trajectory_teachplay;

struct bt_trajectory_teachplay
{
   /* Include the base */
   struct bt_trajectory base;
   
   int n;
   gsl_vector * start;
   
   /* a bt_trajectory_teachplay has a log object */
   double time; /* Location used for teaching */
   char * filename;
   struct bt_log * log;
   
   struct bt_spline * spline;
   struct bt_profile * profile;
   
};

/* Trajectory-specific create function */
struct bt_trajectory_teachplay * bt_trajectory_teachplay_create( gsl_vector * cur_position, char * filename);
int bt_trajectory_teachplay_trigger(struct bt_trajectory_teachplay * t, double time);
int bt_trajectory_teachplay_flush(struct bt_trajectory_teachplay * t);
int bt_trajectory_teachplay_save(struct bt_trajectory_teachplay * t);
