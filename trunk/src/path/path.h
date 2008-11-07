#ifndef BT_PATH_H
#define BT_PATH_H

/* "Base Class" function pointers */
struct bt_path {
   const char * name; /* points to the same place for a given type */
   
   /* Asynchronous interface */
   int (*get_num_points)(struct bt_path * base);
   int (*get_total_time)(struct bt_path * base);
   
   /* Eventually, here will go the generic functions (interface),
    * used in synchronous mode (e.g. get_reference()) */
};

#endif /* BT_PATH_H */
