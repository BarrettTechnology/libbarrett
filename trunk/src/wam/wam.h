/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wam.h
 *  Author ............. Sam Clanton
 *                       Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2004 Q3
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2004-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *    a high-level asynchronous non-realtime interface to the WAM
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt, split from btwam into wam and wambot
 *
 * ======================================================================== */

#ifndef BT_WAM_H
#define BT_WAM_H
#ifdef __cplusplus
extern "C" {
#endif
   

/* A list of WAMs */
struct bt_wam_list;

enum bt_wam_list_entry_status
{
   BT_WAM_LIST_ENTRY_STATUS_FREE,
   BT_WAM_LIST_ENTRY_STATUS_INUSE,
   BT_WAM_LIST_ENTRY_STATUS_DEFUNCT
};

/* List the status of available WAMs at a location
 * (use 0 or "" for "locally") */
struct bt_wam_list * bt_wam_list_create(char * wamloc);
int bt_wam_list_destroy(struct bt_wam_list * list);

int bt_wam_list_get_num(struct bt_wam_list * list);
char * bt_wam_list_get_name(struct bt_wam_list * list, int i, char * buf);
enum bt_wam_list_entry_status bt_wam_list_get_status(struct bt_wam_list * list, int i);
int bt_wam_list_get_pid(struct bt_wam_list * list, int i);
char * bt_wam_list_get_programname(struct bt_wam_list * list, int i, char * buf);


/* The opaque WAM structure */
struct bt_wam;
struct bt_wam_local;

/* WAM options */
enum bt_wam_opt {
   BT_WAM_OPT_NO_LOOP_START = 1
};

/* This function sets up a new WAM,
 * and spins off a wam_thread to control it.
 *
 * return values:
 *   0 (null pointer) - config or memory failure */
struct bt_wam * bt_wam_create(char * wamname);
struct bt_wam * bt_wam_create_opt(char * wamname, enum bt_wam_opt opts);

/* Get the local WAM if it exists */
struct bt_wam_local * bt_wam_get_local(struct bt_wam * wam);

/* Close communication with a WAM */
int bt_wam_destroy(struct bt_wam * wam);

int bt_wam_loop_start(struct bt_wam * wam);
int bt_wam_loop_stop(struct bt_wam * wam);

int bt_wam_dof(struct bt_wam * wam);

/* Kinematics functions */
char * bt_wam_str_jposition(struct bt_wam * wam, char * buf);
char * bt_wam_str_jvelocity(struct bt_wam * wam, char * buf);
char * bt_wam_str_jtorque(struct bt_wam * wam, char * buf);
char * bt_wam_str_cposition(struct bt_wam * wam, char * buf);
char * bt_wam_str_crotation_r1(struct bt_wam * wam, char * buf);
char * bt_wam_str_crotation_r2(struct bt_wam * wam, char * buf);
char * bt_wam_str_crotation_r3(struct bt_wam * wam, char * buf);

/* Gravity comp functions */
int bt_wam_isgcomp(struct bt_wam * wam);
int bt_wam_setgcomp(struct bt_wam * wam, int onoff);

/* Controller functions */
int bt_wam_idle(struct bt_wam * wam);
int bt_wam_hold(struct bt_wam * wam);
int bt_wam_is_holding(struct bt_wam * wam);
char * bt_wam_str_con_position(struct bt_wam * wam, char * buf);
char * bt_wam_get_current_controller_name(struct bt_wam * wam, char * buf);
char * bt_wam_get_current_controller_space(struct bt_wam * wam, char * buf);
int bt_wam_controller_toggle(struct bt_wam * wam);
int bt_wam_control_use_name(struct bt_wam * wam, char * name);
int bt_wam_control_use_space(struct bt_wam * wam, char * space);

/* Refgen functions */
int bt_wam_refgen_clear(struct bt_wam * wam);
char * bt_wam_refgen_active_name(struct bt_wam * wam, char * buf);
char * bt_wam_refgen_loaded_name(struct bt_wam * wam, char * buf);
int bt_wam_refgen_save(struct bt_wam * wam, char * filename);
int bt_wam_refgen_load(struct bt_wam * wam, char * filename);

/* For moves */
int bt_wam_set_velocity(struct bt_wam * wam, double vel);
int bt_wam_set_acceleration(struct bt_wam * wam, double acc);
int bt_wam_moveto(struct bt_wam * wam, int n, double * dest);
int bt_wam_movehome(struct bt_wam * wam);
int bt_wam_moveisdone(struct bt_wam * wam);

/* For teaching */
int bt_wam_is_teaching(struct bt_wam * wam);
int bt_wam_teach_start(struct bt_wam * wam);
int bt_wam_teach_end(struct bt_wam * wam);

/* Play a loaded refgen */
int bt_wam_run(struct bt_wam * wam);

#ifdef __cplusplus
}
#endif
#endif /* BT_WAM_H */

