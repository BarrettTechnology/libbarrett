




#ifndef BT_WAM_LIST_H
#define BT_WAM_LIST_H

#include "wam.h"

#define WAMCONFIGDIR "/etc/wam/"
#define WAMLOCKDIR "/var/lock/"
#define WAMCONFIGDIRLEN (70)
#define WAMLOCKDIRLEN (70)
#define WAMNAMELEN (30)

struct bt_wam_list_entry
{
   char name[30];
   enum bt_wam_list_entry_status status;
   int programpid;
   char programname[30];
};

struct bt_wam_list_local
{
   struct bt_wam_list_entry ** entries;
   int num;
};

struct bt_wam_list_local * bt_wam_list_local_create();
int bt_wam_list_local_destroy(struct bt_wam_list_local * list);
int bt_wam_list_local_get_num(struct bt_wam_list_local * list);
char * bt_wam_list_local_get_name(struct bt_wam_list_local * list, int i, char * buf);
enum bt_wam_list_entry_status bt_wam_list_local_get_status(struct bt_wam_list_local * list, int i);
int bt_wam_list_local_get_pid(struct bt_wam_list_local * list, int i);
char * bt_wam_list_local_get_programname(struct bt_wam_list_local * list, int i, char * buf);

#endif /* BT_WAM_LIST_H */

