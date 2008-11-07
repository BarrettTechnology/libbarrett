#include "path.h"

struct bt_path_discrete {
   /* Include the base function pointers */
   struct bt_path base;
   
   /* A discrete path has a spline and a profile */
   struct bt_trajectory_spline * spline;
   struct bt_trajectory_profile * profile;
   
};

/* Asynchronous creation/destruction functions */
struct bt_path_discrete * bt_path_discrete_create();
int bt_path_discrete_destroy(struct bt_path_discrete * path);

/* Asynchronous interface */
/*int bt_path_discrete_get_current_point(struct bt_path_discrete * path);*/
