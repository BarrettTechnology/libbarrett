/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... file_rpc.c
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
 *
 *  REVISION HISTORY:
 *
 * ======================================================================== */

#include "file.h"
#include "rpc.h"

#include "file_rpc.h"

static const struct bt_rpc_interface_func list[] =
{
   {(void (*)())&bt_file_create,  "bt_file_create",  BT_RPC_FUNC_OBJ_STR_CREATE},
   {(void (*)())&bt_file_destroy, "bt_file_destroy", BT_RPC_FUNC_INT_OBJ_DESTROY},
   {(void (*)())&bt_file_fputs,   "bt_file_fputs",   BT_RPC_FUNC_INT_OBJ_STR},
   {(void (*)())&bt_file_fseek,   "bt_file_fseek",   BT_RPC_FUNC_INT_OBJ_INT_INT},
   {(void (*)())&bt_file_getline, "bt_file_getline", BT_RPC_FUNC_INT_OBJ_GETLINE},
   {0,{0},0}
};
static const struct bt_rpc_interface_funcs interface =
{
   "bt_file_",
   list
};
const struct bt_rpc_interface_funcs * bt_file_rpc = &interface;
