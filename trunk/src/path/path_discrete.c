#include <stdlib.h>

#include "path.h"
#include "path_discrete.h"

/* Define the type */
static int get_num_points(struct bt_path * base);
static int get_total_time(struct bt_path * base);
static int get_reference(struct bt_path * base, double time);
static const struct bt_path_type bt_path_discrete_type = {
   "discrete",
   &get_num_points,
   &get_total_time,
   &get_reference
};
const struct bt_path_type * bt_path_discrete = &bt_path_discrete_type;


/* Specific asynchronous creation/destruction functions */
struct bt_path_discrete * bt_path_discrete_create()
{
   struct bt_path_discrete * p;
   p = (struct bt_path_discrete *) malloc( sizeof(struct bt_path_discrete));
   if (!p) return 0;
   
   /* Set the type */
   p->base.type = bt_path_discrete;
   
   return p;
   
}

int bt_path_discrete_destroy(struct bt_path_discrete * p)
{
   free(p);
   return 0;
}

/* Generic asynchronous interface */
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


