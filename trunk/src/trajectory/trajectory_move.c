#include "trajectory.h"
#include "trajectory_move.h"

/* Define the type */
static int get_num_points(struct bt_path * base);
static int get_total_time(struct bt_path * base);
static int get_reference(struct bt_path * base, double time);
static const struct bt_trajectory_type bt_trajectory_move_type = {
   "move",
   &get_num_points,
   &get_total_time,
   &get_reference
};
const struct bt_trajectory_type * bt_trajectory_move = &bt_trajectory_move_type;

/* Trajectory-specific create/destroy functions */
struct bt_trajectory_move * bt_trajectory_move_create(gsl_vector * pos, gsl_vector * vel, gsl_vector * dest)
{
   struct bt_trajectory_move * t;
   t = (struct bt_trajectory_move *) malloc( sizeof(struct bt_trajectory_move) );
   if (!t) return 0;
   
   /* Set the type */
   t->base.type = bt_trajectory_move;
   
   return t;
}

int bt_trajectory_move_destroy(struct bt_trajectory_move * t)
{
   free(t);
   return 0;
}

/* Interface functions (from trajectory.h) */
static int get_num_points(struct bt_path * base)
{
   return 0;
}

static int get_total_time(struct bt_path * base)
{
   return 0;
}

static int get_reference(struct bt_path * base, double time)
{
   return 0;
}
