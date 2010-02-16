
#ifndef BT_WAM_THREAD_H
#define BT_WAM_THREAD_H

/* Here's the WAM realtime thread; it is only one thread, rt_wam(), which
 * calls rt_wam_create() and rm_wam_destroy() appropriately. */
void bt_wam_thread(struct bt_os_thread * thread);

/* Here's the WAM non-realtime thread, which takes care of logging,
 * and perhaps some other things. */
void bt_wam_thread_nonrt(struct bt_os_thread * thread);

/* The setup helper is for communication between the non-realtime create()
 * function and the realtime rt_wam() setup function. */
struct bt_wam_thread_helper
{
   struct bt_wam_local * wam;
   config_setting_t * config;
   int no_wambot_zeroangle;
   int is_setup;
   int setup_failed;
};
int bt_wam_thread_helper_create(struct bt_wam_thread_helper ** helperptr);
void bt_wam_thread_helper_destroy(struct bt_wam_thread_helper * helper);

#endif /* BT_WAM_THREAD_H */
