/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... discover.h
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

/* Note - see SVN/internal/tools/wamdiscover/client for example
 *        to make this Windows-compatible */

struct bt_discover_client_entry {
   char mac[18];
   char ip[16];
};

struct bt_discover_client
{
   int sock;
   
   struct bt_discover_client_entry * list;
   int num;
};

struct bt_discover_client * bt_discover_client_create(void);
int bt_discover_client_destroy(struct bt_discover_client * client);

/* For now, this blocks for about 1 second */
int bt_discover_client_discover(struct bt_discover_client * client);

