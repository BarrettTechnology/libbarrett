#ifndef BT_PATH_H
#define BT_PATH_H

/* Note: paths are time-invariant */

struct bt_path;

struct bt_path_type
{
   char name[20];

   /* Define the asynchronous interface */
   int (*get_num_points)(struct bt_path * path);
   int (*get_total_time)(struct bt_path * path);

   /* Define the synchronous interface */
   int (*get_reference)(struct bt_path * path, double time);
};

/* A path */
struct bt_path {
   const struct bt_path_type * type;
};

#endif /* BT_PATH_H */
