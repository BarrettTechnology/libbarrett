/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wam_rpc.h
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

#include "wam.h"
#include "rpc.h"

#include "wam_rpc.h"

static const struct bt_rpc_interface_func list[] =
{
   {(void (*)())&bt_wam_create,           "bt_wam_create",           BT_RPC_FUNC_OBJ_STR_CREATE},
   {(void (*)())&bt_wam_destroy,          "bt_wam_destroy",          BT_RPC_FUNC_INT_OBJ_DESTROY},
   
   {(void (*)())&bt_wam_str_jposition,    "bt_wam_str_jposition",    BT_RPC_FUNC_STR_OBJ},
   {(void (*)())&bt_wam_str_jvelocity,    "bt_wam_str_jvelocity",    BT_RPC_FUNC_STR_OBJ},
   {(void (*)())&bt_wam_str_jtorque,      "bt_wam_str_jtorque",      BT_RPC_FUNC_STR_OBJ},
   {(void (*)())&bt_wam_str_cposition,    "bt_wam_str_cposition",    BT_RPC_FUNC_STR_OBJ},
   {(void (*)())&bt_wam_str_crotation_r1, "bt_wam_str_crotation_r1", BT_RPC_FUNC_STR_OBJ},
   {(void (*)())&bt_wam_str_crotation_r2, "bt_wam_str_crotation_r2", BT_RPC_FUNC_STR_OBJ},
   {(void (*)())&bt_wam_str_crotation_r3, "bt_wam_str_crotation_r3", BT_RPC_FUNC_STR_OBJ},
   {(void (*)())&bt_wam_isgcomp,          "bt_wam_isgcomp",          BT_RPC_FUNC_INT_OBJ},
   {(void (*)())&bt_wam_setgcomp,         "bt_wam_setgcomp",         BT_RPC_FUNC_INT_OBJ_INT},
   
   {(void (*)())&bt_wam_idle,             "bt_wam_idle",             BT_RPC_FUNC_INT_OBJ},
   {(void (*)())&bt_wam_hold,             "bt_wam_hold",             BT_RPC_FUNC_INT_OBJ},
   
   {(void (*)())&bt_wam_movehome,         "bt_wam_movehome",         BT_RPC_FUNC_INT_OBJ},
   
   {(void (*)())&bt_wam_teach_start,      "bt_wam_teach_start",      BT_RPC_FUNC_INT_OBJ},
   {(void (*)())&bt_wam_teach_end,        "bt_wam_teach_end",        BT_RPC_FUNC_INT_OBJ},
   {(void (*)())&bt_wam_playback,         "bt_wam_playback",         BT_RPC_FUNC_INT_OBJ},
   {0,{0},0}
};
static const struct bt_rpc_interface_funcs interface =
{
   "bt_wam_",
   list
};
const struct bt_rpc_interface_funcs * bt_wam_rpc = &interface;

