#include "path.h"
#include "path_discrete.h"

/* Base function pointers */
static const char name[] = "discrete";
static int get_num_points(struct bt_path * base);
static int get_total_time(struct bt_path * base);

/* Specific asynchronous creation/destruction functions */
struct bt_path_discrete * bt_path_discrete_create()
{
   struct bt_path_discrete * p;
   p = (struct bt_path_discrete *) malloc( sizeof(struct bt_path_discrete));
   if (!p) return 0;
   
   p->base.name = name;
   p->base.get_num_points = &get_num_points;
   p->base.get_total_time = &get_total_time;
   
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



