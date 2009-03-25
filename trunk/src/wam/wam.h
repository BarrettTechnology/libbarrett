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

/* We also use gsl as an interface language.  Maybe change this to
 * arrays of doubles? */
#include <gsl/gsl_vector.h>

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

/* Close communication with a WAM */
int bt_wam_destroy(struct bt_wam * wam);

int bt_wam_loop_start(struct bt_wam * wam);
int bt_wam_loop_stop(struct bt_wam * wam);

/* String formatting functions */
char * bt_wam_str_jposition(struct bt_wam * wam, char * buf);
char * bt_wam_str_jvelocity(struct bt_wam * wam, char * buf);
char * bt_wam_str_jtorque(struct bt_wam * wam, char * buf);
char * bt_wam_str_cposition(struct bt_wam * wam, char * buf);
char * bt_wam_str_crotation_r1(struct bt_wam * wam, char * buf);
char * bt_wam_str_crotation_r2(struct bt_wam * wam, char * buf);
char * bt_wam_str_crotation_r3(struct bt_wam * wam, char * buf);

int bt_wam_isgcomp(struct bt_wam * wam);
int bt_wam_setgcomp(struct bt_wam * wam, int onoff);

char * bt_wam_get_current_controller_name(struct bt_wam * wam, char * buf);
int bt_wam_controller_toggle(struct bt_wam * wam);

/* These are simple wrappers for the active controller */
int bt_wam_idle(struct bt_wam * wam);
int bt_wam_hold(struct bt_wam * wam);
int bt_wam_is_holding(struct bt_wam * wam);

/* Refgen commands */
char * bt_wam_get_current_refgen_name(struct bt_wam * wam, char * buf);

/* For moves */
int bt_wam_set_velocity(struct bt_wam * wam, double vel);
int bt_wam_set_acceleration(struct bt_wam * wam, double acc);

int bt_wam_moveto(struct bt_wam * wam, gsl_vector * dest);
int bt_wam_movehome(struct bt_wam * wam);
int bt_wam_moveisdone(struct bt_wam * wam);

int bt_wam_is_teaching(struct bt_wam * wam);
int bt_wam_teach_start(struct bt_wam * wam);
int bt_wam_teach_end(struct bt_wam * wam);
int bt_wam_playback(struct bt_wam * wam);

#endif /* BT_WAM_H */

